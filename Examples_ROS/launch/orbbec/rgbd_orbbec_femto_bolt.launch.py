import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def find_vocabulary_file():
    """Find ORBvoc.txt in the package install or its source workspace."""
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


def load_ros1_yaml_as_params(yaml_file_path):
    """Load a plain ROS1-style YAML mapping as ROS 2 node parameters."""
    with open(yaml_file_path, 'r') as file:
        contents = file.read()
    # OpenCV FileStorage requires its YAML version marker, which PyYAML does
    # not accept in OpenCV's colon form.
    if contents.startswith('%YAML:1.0'):
        contents = contents.split('\n', 1)[1]
    config = yaml.safe_load(contents)
    # ROS node topic parameters live under a descriptive YAML section; omit
    # that container level because rgbd.cpp declares the topic names directly.
    config = config.get('topics', config)

    def flatten_dict(data, parent_key='', sep='/'):
        items = []
        for key, value in data.items():
            new_key = f'{parent_key}{sep}{key}' if parent_key else key
            if isinstance(value, dict):
                items.extend(flatten_dict(value, new_key, sep=sep).items())
            else:
                items.append((new_key, value))
        return dict(items)

    return flatten_dict(config)


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare('orb_slam3'), 'config'])
    vocabulary_file = find_vocabulary_file()
    topics_file = os.path.join(
        get_package_share_directory('orb_slam3'),
        'config', 'orbbec', 'orbbec_femto_bolt_rgbd.yaml'
    )
    topic_params = load_ros1_yaml_as_params(topics_file)
    return LaunchDescription([
        DeclareLaunchArgument('vocabulary', default_value=vocabulary_file),
        DeclareLaunchArgument(
            'settings',
            default_value=PathJoinSubstitution([config_dir, 'orbbec', 'orbbec_femto_bolt_rgbd.yaml']),
        ),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([config_dir, 'orbslam3.rviz']),
        ),
        Node(
            package='orb_slam3',
            executable='ros_rgbd',
            name='orbslam3_rgbd_femto_bolt',
            output='screen',
            parameters=[
                PathJoinSubstitution([config_dir, 'orbslam3.yaml']),
                topic_params,
                {
                    'vocabulary': LaunchConfiguration('vocabulary'),
                    'settings': LaunchConfiguration('settings'),
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
