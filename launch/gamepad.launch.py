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
            'max_omni_vel': 0.4, # m/s
            'max_omni_rot_vel': 1.0, # rad/s
            'max_yaw_rot_vel': 2.0, # rad/s
            'max_pitch_rot_vel': 2.0, # rad/s
        }],
    )

    ld.add_action(joy_node)
    ld.add_action(gamepad_node)

    return ld
