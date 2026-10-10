from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vocabulary'),
        DeclareLaunchArgument('settings'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/rgb/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth_registered/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/rgb/camera_info'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([FindPackageShare('orb_slam3'), 'config', 'orbslam3.rviz'])),
        Node(package='orb_slam3', executable='ros_rgbd', name='orbslam3_rgbd', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('orb_slam3'), 'config', 'orbslam3.yaml']),
                         {'vocabulary': LaunchConfiguration('vocabulary'),
                          'settings': LaunchConfiguration('settings'),
                          'rgb_topic': LaunchConfiguration('rgb_topic'),
                          'depth_topic': LaunchConfiguration('depth_topic'),
                          'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                          'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
                          'debug': ParameterValue(LaunchConfiguration('debug'), value_type=bool)}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_config')], condition=IfCondition(LaunchConfiguration('start_rviz')),
             output='screen'),
    ])
