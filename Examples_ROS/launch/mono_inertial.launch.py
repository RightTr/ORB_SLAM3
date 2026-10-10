from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [DeclareLaunchArgument('vocabulary'), DeclareLaunchArgument('settings')]
    params = {'vocabulary': LaunchConfiguration('vocabulary'), 'settings': LaunchConfiguration('settings')}
    args.append(DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'))
    params['image_topic'] = LaunchConfiguration('image_topic')
    args.append(DeclareLaunchArgument('imu_topic', default_value='/imu'))
    params['imu_topic'] = LaunchConfiguration('imu_topic')
    args.append(DeclareLaunchArgument('do_equalize', default_value='false'))
    params['do_equalize'] = ParameterValue(LaunchConfiguration('do_equalize'), value_type=bool)
    args.append(Node(package='orb_slam3', executable='ros_mono_inertial', name='orbslam3_mono_inertial', output='screen', parameters=[params]))
    return LaunchDescription(args)
