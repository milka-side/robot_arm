import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description() -> LaunchDescription:
    arm_description_dir = get_package_share_directory("arm_description")
    arm_sim_dir = get_package_share_directory("arm_sim")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")

    xacro_file = os.path.join(arm_description_dir, "urdf", "arm_standalone.urdf.xacro")
    world_file = os.path.join(arm_sim_dir, "worlds", "empty.sdf")
    bridge_config = os.path.join(arm_sim_dir, "config", "gz_bridge.yaml")

    resource_path_root = os.path.dirname(arm_description_dir)
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    existing_ign_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    gz_resource_path = os.pathsep.join(filter(None, [resource_path_root, existing_gz_path]))
    ign_resource_path = os.pathsep.join(filter(None, [resource_path_root, existing_ign_path]))

    robot_description_content = ParameterValue(
        Command(["xacro ", xacro_file, " sim:=true"]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": True}],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "robot_arm", "-z", "0.3"],
        output="screen",
    )

    # /clock only — no camera sensor in this build.
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
        output="screen",
    )

    # Sim is single-host, but its motion-control clients still run as
    # separate processes here too — bring up the same lock server the
    # real cross-host deployment needs, so sim actually exercises the
    # real locking path instead of silently having none.
    arm_motion_lock_server = Node(
        package="arm_teleop",
        executable="arm_motion_lock_server",
        output="screen",
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "robot_arm_controller",
            "--controller-manager-timeout", "60",
            "--switch-timeout", "60",
            "--service-call-timeout", "70",
        ],
        output="screen",
    )

    # Streaming teleop controller, spawned inactive — JTC owns the joints
    # until arm_teleop switches controllers for Servo. Mirrors arm_bringup/arm.launch.py;
    # a separate spawner call because --inactive applies to the whole call.
    forward_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_arm_forward_position_controller", "--inactive"],
        output="screen",
    )

    # Each startup stage runs only if the previous one exited with code 0;
    # otherwise the whole launch shuts down instead of starting nodes
    # against a broken stack.
    def _after_spawn(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f"Entity spawn failed (exit code {event.returncode})."),
                Shutdown(reason="entity spawn failed"),
            ]
        return [controller_spawner, forward_spawner]

    delayed_controller_spawners = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=_after_spawn,
        )
    )

    # Spawned only after robot_arm_controller is confirmed loaded and
    # active (chained off controller_spawner's own exit, not run alongside
    # it).
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_right_controller",
            "gripper_left_controller",
            "--controller-manager-timeout", "60",
            "--switch-timeout", "60",
            "--service-call-timeout", "70",
        ],
        output="screen",
    )

    def _after_arm_controller(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f"Controller activation failed (exit code {event.returncode})."),
                Shutdown(reason="controller activation failed"),
            ]
        return [gripper_spawner]

    delayed_gripper_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_spawner,
            on_exit=_after_arm_controller,
        )
    )

    # planning_pipelines restricted to ompl on purpose: with none of the
    # config yaml files it discovers for chomp/pilz_industrial_motion_planner
    # actually present in this package (only ompl_planning.yaml exists),
    # letting MoveItConfigsBuilder auto-load its bundled default configs
    # for all three still left move_group picking an ambiguous
    # "planning_plugin" between them ("Multiple planning plugins
    # available... Using 'chomp_interface/CHOMPPlanner' for now" — even
    # when the request explicitly asked for the 'ompl' pipeline_id).
    moveit_config = MoveItConfigsBuilder(
        "robot_arm", package_name="arm_moveit_config"
    ).planning_pipelines(pipelines=["ompl"]).to_moveit_configs()
    move_group_launch = generate_move_group_launch(moveit_config)

    moveit_config_dir = get_package_share_directory("arm_moveit_config")
    with open(os.path.join(moveit_config_dir, "config", "servo.yaml")) as f:
        servo_yaml = yaml.safe_load(f)
    servo_params = {"moveit_servo": servo_yaml["moveit_servo"]["ros__parameters"]}

    # Inverse Jacobian only — see arm_bringup/arm.launch.py (KDL searchPositionIK
    # from home makes +X teleop freeze while -X still works).
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.joint_limits,
            servo_params,
        ],
    )

    def _after_controllers(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f"Controller activation failed (exit code {event.returncode})."),
                Shutdown(reason="controller activation failed"),
            ]
        return list(move_group_launch.entities) + [servo_node]

    delayed_move_group = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_spawner,
            on_exit=_after_controllers,
        )
    )

    ld = LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ign_resource_path),
            arm_motion_lock_server,
            gz_sim,
            robot_state_publisher,
            spawn_entity,
            ros_gz_bridge,
            delayed_controller_spawners,
            delayed_move_group,
            delayed_gripper_spawner,
        ]
    )

    return ld
