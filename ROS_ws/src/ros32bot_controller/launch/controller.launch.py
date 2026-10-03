from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition,UnlessCondition


def noisy_controller(context, *args, **kwargs):
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))
    use_python = LaunchConfiguration("use_python")
    use_sim_time = LaunchConfiguration("use_sim_time")

    noisy_controller_py = Node(
        package="ros32bot_controller",
        executable="noisy_controller.py",
        parameters = [
            {"wheel_radius" : wheel_radius + wheel_radius_error ,
            "wheel_separation" : wheel_separation + wheel_separation_error,
            "use_sim_time": use_sim_time}
        ],
        condition=IfCondition(use_python)
    )

    noisy_controller_cpp = Node(
        package="ros32bot_controller",
        executable="noisy_controller",
        parameters = [
            {"wheel_radius" : wheel_radius + wheel_radius_error ,
            "wheel_separation" : wheel_separation + wheel_separation_error,
            "use_sim_time": use_sim_time}
        ],
        condition=UnlessCondition(use_python)
    )

    return [
        noisy_controller_py,
        noisy_controller_cpp
    ]



def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True"
    )

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False"
    )

    use_noisy_controller_arg = DeclareLaunchArgument(
        "use_noisy_controller",
        default_value="False"
    )

    wheel_radius_argument = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033"
    )

    wheel_separation_argument = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17"
    )

    use_simple_controller_argument = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True"
    )

    wheel_radius_err_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005"
    )

    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02"
    )

    use_python = LaunchConfiguration("use_python")
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ros32bot_controller",
            "--controller-manager",
            "/controller_manager",
            # the sim and the Pi both take commands on the standard /cmd_vel
            "--controller-ros-args",
            "-r /ros32bot_controller/cmd_vel:=/cmd_vel"
        ],
         condition=UnlessCondition(use_simple_controller)
    )

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions = [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]),
            Node(
                package="ros32bot_controller",
                executable="simple_controller.py",
                parameters=[{"use_sim_time": use_sim_time,
                            "wheel_radius":wheel_radius, 
                            "wheel_separation":wheel_separation}],
                condition=IfCondition(use_python)),
            Node(
                package="ros32bot_controller",
                executable="simple_controller",
                parameters=[{"use_sim_time": use_sim_time,
                            "wheel_radius":wheel_radius, 
                            "wheel_separation":wheel_separation}],
                condition=UnlessCondition(use_python))
             ]
      )

    noisy_controller_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration("use_noisy_controller")),
        actions=[OpaqueFunction(function=noisy_controller)]
    )



    return LaunchDescription([
        use_python_arg,
        use_noisy_controller_arg,
        use_sim_time_arg,
        wheel_radius_argument,
        wheel_separation_argument,
        use_simple_controller_argument,
        wheel_radius_err_arg,
        wheel_separation_error_arg,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        simple_controller,
        noisy_controller_launch

    ])