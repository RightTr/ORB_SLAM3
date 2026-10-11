from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def find_vocabulary_file():
    """Find ORBvoc.txt in the installed package's source workspace."""
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


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare('orb_slam3'), 'config'])
    vocabulary_file = find_vocabulary_file()
    return LaunchDescription([
        DeclareLaunchArgument('vocabulary', default_value=vocabulary_file),
        DeclareLaunchArgument(
            'settings',
            default_value=PathJoinSubstitution(
                [config_dir, 'orbbec', 'orbbec_femto_bolt_mono.yaml']
            ),
        ),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([config_dir, 'orbslam3.rviz']),
        ),
        Node(
            package='orb_slam3',
            executable='ros_mono',
            name='orbslam3_mono_femto_bolt',
            output='screen',
            parameters=[
                PathJoinSubstitution([config_dir, 'orbslam3.yaml']),
                {
                    'vocabulary': vocabulary_file,
                    'settings': LaunchConfiguration('settings'),
                    'image_topic': LaunchConfiguration('image_topic'),
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
