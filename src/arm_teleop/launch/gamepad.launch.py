from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    end_effector = LaunchConfiguration('end_effector')

    game_controller_node = Node(
        package='joy',
        executable='game_controller_node',
        name='joy_node',
        namespace='arm',
        output='screen',
        parameters=[{
            'dev': joy_dev,
            # Must stay 0.0 — deadzone is applied in GamepadInputLoop instead.
            'deadzone': ParameterValue(0.0, value_type=float),
        }],
        respawn=True,
        respawn_delay=10.0,
    )

    gamepad_servo_node = Node(
        package='arm_teleop',
        executable='gamepad_servo_node',
        namespace='arm',
        output='screen',
        parameters=[{
            'end_effector': end_effector,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path',
        ),
        DeclareLaunchArgument(
            'end_effector',
            default_value='jaw',
            description="Which tool is physically mounted (e.g. 'jaw').",
        ),
        game_controller_node,
        gamepad_servo_node,
    ])
