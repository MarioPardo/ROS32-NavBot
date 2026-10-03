from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory








def generate_launch_description():

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("ros32bot_controller"), "config", "joy_teleop.yaml")]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("ros32bot_controller"), "config", "joy_config.yaml")]
    )

    
    return LaunchDescription([
        joy_teleop,
        joy_node
    ])