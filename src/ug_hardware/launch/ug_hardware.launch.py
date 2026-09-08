from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Hardware interface
        Node(
            package="ug_hardware",
            executable="hardware_interface",
            name="hardware_interface",
            output="screen",
        ),

        # Graphical user interface
        Node(
            package="ug_ui",
            executable="ui",
            name="ug_ui",
            output="screen",
            on_exit=Shutdown(),
        ),
    ])
