from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


MODE_TOPICS = {
    'mono': ('image_topic',),
    'rgbd': ('rgb_topic', 'depth_topic'),
    'rgbd_inertial': ('rgb_topic', 'depth_topic', 'imu_topic'),
    'stereo': ('left_topic', 'right_topic'),
    'stereo_inertial': ('left_topic', 'right_topic', 'imu_topic'),
}


def find_vocabulary_file(package_share):
    candidates = [package_share / 'Vocabulary' / 'ORBvoc.txt']
    for parent in package_share.parents:
        candidates.extend((
            parent / 'Vocabulary' / 'ORBvoc.txt',
            parent / 'src' / 'ORB_SLAM3' / 'Vocabulary' / 'ORBvoc.txt',
        ))
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    raise FileNotFoundError('ORB_SLAM3 Vocabulary/ORBvoc.txt was not found')


def load_topics(settings_path):
    class OpenCVYamlLoader(yaml.SafeLoader):
        pass

    OpenCVYamlLoader.add_constructor(
        'tag:yaml.org,2002:opencv-matrix',
        lambda loader, node: loader.construct_mapping(node),
    )
    contents = settings_path.read_text()
    if contents.startswith('%YAML:1.0'):
        contents = contents.split('\n', 1)[1]
    return yaml.load(contents, Loader=OpenCVYamlLoader)['topics']


def make_launch_description(mode):
    package_share = Path(get_package_share_directory('orb_slam3')).resolve()
    settings_path = package_share / 'config' / 'realsense' / f'realsense_d435i_{mode}.yaml'
    topics = load_topics(settings_path)
    arguments = [
        DeclareLaunchArgument('vocabulary', default_value=find_vocabulary_file(package_share)),
        DeclareLaunchArgument('settings', default_value=str(settings_path)),
        DeclareLaunchArgument('use_pangolin', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='false' if mode.startswith('stereo') else 'true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([FindPackageShare('orb_slam3'), 'config', 'orbslam3.rviz']),
        ),
    ]
    parameters = {
        'vocabulary': LaunchConfiguration('vocabulary'),
        'settings': LaunchConfiguration('settings'),
        'use_pangolin': ParameterValue(LaunchConfiguration('use_pangolin'), value_type=bool),
    }
    for topic_name in MODE_TOPICS[mode]:
        arguments.append(DeclareLaunchArgument(topic_name, default_value=topics[topic_name]))
        parameters[topic_name] = LaunchConfiguration(topic_name)
    if mode in ('mono', 'rgbd', 'rgbd_inertial'):
        arguments.extend((
            DeclareLaunchArgument('publish_tf', default_value='false'),
        ))
        parameters['publish_tf'] = ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool)
    if mode.startswith('stereo'):
        arguments.append(DeclareLaunchArgument('do_rectify', default_value='false'))
        parameters['do_rectify'] = ParameterValue(LaunchConfiguration('do_rectify'), value_type=bool)
    if mode == 'stereo_inertial':
        arguments.append(DeclareLaunchArgument('do_equalize', default_value='false'))
        parameters['do_equalize'] = ParameterValue(LaunchConfiguration('do_equalize'), value_type=bool)

    arguments.append(Node(
        package='orb_slam3',
        executable=f'ros_{mode}',
        name=f'orbslam3_{mode}_d435i',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('orb_slam3'), 'config', 'orbslam3.yaml']),
            parameters,
        ],
    ))
    arguments.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        output='screen',
    ))
    return LaunchDescription(arguments)
