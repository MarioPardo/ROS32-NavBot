from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os

# Default of the matching xacro arg; keep the two in step.
SERIAL_PORT = "/dev/ttyAMA0"



def generate_launch_description():


    robot_description = ParameterValue(Command([
        "xacro ", 
        os.path.join(get_package_share_directory("ros32bot_description"), "urdf","ros32bot.urdf.xacro"),
        " is_sim:=False",
        " port:=", SERIAL_PORT,
        ]),
        value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description,
                        "use_sim_time": False}]
    )

    controller_manager_parameters = {
        "robot_description" : robot_description,
        "use_sim_time" : False
    }

    # A dev PC has no UART, so never activate: the control node aborts if the open fails.
    if not os.path.exists(SERIAL_PORT):
        controller_manager_parameters["hardware_components_initial_state.inactive"] = ["RobotSystem"]
        controller_manager_parameters["defaults.allow_controller_activation_with_inactive_hardware"] = True

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=
            [
                os.path.join(
                    get_package_share_directory("ros32bot_controller"),"config", "ros32bot_controllers.yaml" ),
                # must come after the yaml: later parameter sources win, and the yaml sets use_sim_time: true
                controller_manager_parameters
            ]
    )


    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
    ])