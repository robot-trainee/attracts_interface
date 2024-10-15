from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux_node',
        output='screen',
    )

    gamepad_node = Node(
        package='attracts_interface',
        executable='gamepad_node',
        name='gamepad_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_trans_vel': 0.2,
            'max_rot_vel': 0.4,
        }],
    )

    ld.add_action(joy_node)
    ld.add_action(gamepad_node)

    return ld
