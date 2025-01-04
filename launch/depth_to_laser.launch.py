from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    params = os.path.join(get_package_share_directory('nera_bot'),'config','depth_to_laser.yaml')

    depthimage_to_laserscan_node = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=[('/depth','/camera/d435i/depth/image_rect_raw'),
                        ('/depth_camera_info','/camera/d435i/depth/camera_info'),
                        ('/scan', '/camera/d435i/depth/scan')],
            output='screen'
         )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        LogInfo(msg=f"Parameters being loaded from: {params}"),
        depthimage_to_laserscan_node   
    ])