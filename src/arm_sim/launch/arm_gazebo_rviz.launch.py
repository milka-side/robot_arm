import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description() -> LaunchDescription:
    """Gazebo sim plus RViz in one launch."""
    arm_sim_dir = get_package_share_directory("arm_sim")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_sim_dir, "launch", "arm_gazebo.launch.py")
        ),
    )

    moveit_config = MoveItConfigsBuilder(
        "robot_arm", package_name="arm_moveit_config"
    ).to_moveit_configs()
    rviz_launch = generate_moveit_rviz_launch(moveit_config)

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            gazebo_launch,
            *rviz_launch.entities,
        ]
    )
