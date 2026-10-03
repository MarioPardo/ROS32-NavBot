from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    uart_bridge = Node(
        package="ros32bot_middleware",
        executable="uart_bridge",
        name="uart_bridge",
        parameters=[{
            "port": "/dev/ttyAMA0",
            "wheel_radius": 0.033,
            "wheel_separation": 0.17,
            "cmd_vel_topic": "cmd_vel",
            "cmd_vel_timeout": 0.5,
            "publish_rate": 50.0,
        }],
        output="screen"
    )

    return LaunchDescription([
        uart_bridge
    ])
