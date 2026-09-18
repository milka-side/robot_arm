# LAUNCH THIS ON THE ARM'S ONBOARD/CONTROL COMPUTER

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    end_effector = LaunchConfiguration('end_effector')
    home_pose_name = LaunchConfiguration('home_pose_name')
    activity_indicator_pre_delay_sec = LaunchConfiguration('activity_indicator_pre_delay_sec')
    plan_execute_velocity_scaling = LaunchConfiguration('plan_execute_velocity_scaling')
    plan_execute_acceleration_scaling = LaunchConfiguration('plan_execute_acceleration_scaling')

    gamepad_servo_node = Node(
        package='arm_teleop',
        executable='gamepad_servo_node',
        namespace='arm',
        output='screen',
        parameters=[{
            'end_effector': end_effector,
            'home_pose_name': home_pose_name,
            'activity_indicator_pre_delay_sec': activity_indicator_pre_delay_sec,
            'plan_execute_velocity_scaling': plan_execute_velocity_scaling,
            'plan_execute_acceleration_scaling': plan_execute_acceleration_scaling,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'end_effector',
            default_value='jaw',
            description="Which tool is physically mounted (e.g. 'jaw').",
        ),
        DeclareLaunchArgument(
            'home_pose_name',
            default_value='',
            description=(
                "poses.json key that 'A' drives to. Leave empty to "
                "auto-pick '{end_effector}_home', else 'home'."
            ),
        ),
        DeclareLaunchArgument(
            'activity_indicator_pre_delay_sec',
            default_value='0.0',
            description='Seconds the activity indicator waits before moving.',
        ),
        DeclareLaunchArgument(
            'plan_execute_velocity_scaling',
            default_value='0.4',
            description="Fraction of each joint's max_velocity for home moves.",
        ),
        DeclareLaunchArgument(
            'plan_execute_acceleration_scaling',
            default_value='0.4',
            description="Fraction of each joint's max_acceleration for home moves.",
        ),
        gamepad_servo_node,
    ])
