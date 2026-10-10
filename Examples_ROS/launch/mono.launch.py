from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [DeclareLaunchArgument('vocabulary'), DeclareLaunchArgument('settings')]
    params = {'vocabulary': LaunchConfiguration('vocabulary'), 'settings': LaunchConfiguration('settings')}
    args.append(DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'))
    params['image_topic'] = LaunchConfiguration('image_topic')
    args.append(Node(package='orb_slam3', executable='ros_mono', name='orbslam3_mono', output='screen', parameters=[params]))
    return LaunchDescription(args)
