from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
        package_name = 'ydlidar_ros2_driver'
        launch_file_path = get_package_share_directory(package_name) + '/launch>

        lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path)
        )

        return LaunchDescription([
        lidar_launch,
        Node(
            package='lifty_nodes',
            namespace='',
            executable='hardware_interface_node',
            name='hardware_interface_node'
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='lifty_nodes',
                    namespace='',
                    executable='motor_controller_node',
                    name='motor_controller_node'
                )
            ]
        ),
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='lifty_nodes',
                    namespace='',
                    executable='lifty_services_node.py',
                    name='lifty_services_node', 
                )
            ]
        )
    ])

