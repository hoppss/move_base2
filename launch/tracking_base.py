import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    realsense2_camera_prefix = get_package_share_directory('realsense2_camera')
    rtabmap_ros_prefix = get_package_share_directory('rtabmap_ros')
    object_tracking_prefix = get_package_share_directory('object_tracking')
    #navigation_prefix = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        Node(package='chassis_node', executable='chassis_node', name='chassis_node', output='screen'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource([realsense2_camera_prefix, '/launch/on_dog.py'])),

        IncludeLaunchDescription(PythonLaunchDescriptionSource([rtabmap_ros_prefix, '/launch/rs_d455.launch.py'])),

        #IncludeLaunchDescription(PythonLaunchDescriptionSource([navigation_prefix, '/launch/navigation_launch.py'])),
        #IncludeLaunchDescription(PythonLaunchDescriptionSource([navigation_prefix, '/launch/tracking_launch.py'])),

        IncludeLaunchDescription(PythonLaunchDescriptionSource([object_tracking_prefix, '/launch/launch.py'])),
        Node(package='camera_live', executable='maincamera', name='camera_server', output={}),
    ])