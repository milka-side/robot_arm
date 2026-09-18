import sys

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch, generate_rsp_launch


def _arg_from_argv(name: str, default: str) -> str:
    prefix = f"{name}:="
    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg[len(prefix):]
    return default


def generate_launch_description():
    """Visualize the arm with no Gazebo/physics: robot_state_publisher +
    joint_state_publisher_gui (drag sliders to move joints) + RViz.
    """
    moveit_config = (
        MoveItConfigsBuilder("robot_arm", package_name="arm_moveit_config")
        .robot_description(mappings={
            "use_fake_hardware": _arg_from_argv("use_fake_hardware", "true"),
            "end_effector": _arg_from_argv("end_effector", "jaw"),
        })
        .to_moveit_configs()
    )

    rsp_launch = generate_rsp_launch(moveit_config)
    rviz_launch = generate_moveit_rviz_launch(moveit_config)

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    return LaunchDescription([
        *rsp_launch.entities,
        joint_state_publisher_gui,
        *rviz_launch.entities,
    ])
