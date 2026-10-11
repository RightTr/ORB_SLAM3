import os
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


def find_vocabulary_file():
    package_share = Path(get_package_share_directory('orb_slam3')).resolve()
    candidates = [package_share / 'Vocabulary' / 'ORBvoc.txt']
    for parent in package_share.parents:
        candidates.extend([
            parent / 'Vocabulary' / 'ORBvoc.txt',
            parent / 'src' / 'ORB_SLAM3' / 'Vocabulary' / 'ORBvoc.txt',
        ])
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    raise FileNotFoundError(
        'Could not find Vocabulary/ORBvoc.txt in the orb_slam3 install or source workspace'
    )


def load_topic_parameters(yaml_file_path):
    class OpenCVYamlLoader(yaml.SafeLoader):
        pass

    OpenCVYamlLoader.add_constructor(
        'tag:yaml.org,2002:opencv-matrix',
        lambda loader, node: loader.construct_mapping(node),
    )
    with open(yaml_file_path, 'r') as file:
        contents = file.read()
    if contents.startswith('%YAML:1.0'):
        contents = contents.split('\n', 1)[1]
    config = yaml.load(contents, Loader=OpenCVYamlLoader)
    return config.get('topics', {})


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare('orb_slam3'), 'config'])
    config_share = get_package_share_directory('orb_slam3')
    settings_path = os.path.join(
        config_share, 'config', 'orbbec', 'orbbec_femto_bolt_rgbd_inertial.yaml'
    )
    topic_params = load_topic_parameters(settings_path)
    imu_topic = topic_params.get('imu_topic', '/camera/gyro_accel/sample')

    return LaunchDescription([
        DeclareLaunchArgument('vocabulary', default_value=find_vocabulary_file()),
        DeclareLaunchArgument(
            'settings',
            default_value=PathJoinSubstitution(
                [config_dir, 'orbbec', 'orbbec_femto_bolt_rgbd_inertial.yaml']
            ),
        ),
        DeclareLaunchArgument('imu_topic', default_value=imu_topic),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config', default_value=PathJoinSubstitution([config_dir, 'orbslam3.rviz'])
        ),
        Node(
            package='orb_slam3',
            executable='ros_rgbd_inertial',
            name='orbslam3_rgbd_inertial_femto_bolt',
            output='screen',
            parameters=[
                PathJoinSubstitution([config_dir, 'orbslam3.yaml']),
                topic_params,
                {
                    'vocabulary': LaunchConfiguration('vocabulary'),
                    'settings': LaunchConfiguration('settings'),
                    'imu_topic': LaunchConfiguration('imu_topic'),
                    'publish_tf': ParameterValue(
                        LaunchConfiguration('publish_tf'), value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
            output='screen',
        ),
    ])
