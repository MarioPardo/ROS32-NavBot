import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    hardware_interface = IncludeLaunchDescription(os.path.join(
        get_package_share_directory("ros32bot_middleware"),
        "launch",
        "hardware_interface.launch.py"
    ))

    controller = IncludeLaunchDescription(
        os.path.join(
        get_package_share_directory("ros32bot_controller"),
        "launch",
        "controller.launch.py"
        ),
        launch_arguments = {
            "use_simple_controller" : "False",
            "use_python" : "False",
            "use_sim_time" : "False"
        }.items()

    )

    joystick = IncludeLaunchDescription(os.path.join(
        get_package_share_directory("ros32bot_controller"),
        "launch",
        "joystick_teleop.launch.py"
    ))




    return LaunchDescription([
        hardware_interface,
        controller,
        joystick,
    ])