#!/usr/bin/env python3
"""Keyboard/gamepad control node for MoveIt Servo.

Reads input and sends Cartesian velocity commands to MoveIt Servo.
See README.md for key bindings and usage.
"""

import sys
import os
import fcntl
import threading
import termios
import time
import json
import math
import select
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Quaternion, TwistStamped
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Int8, Float64MultiArray, ColorRGBA
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers, SwitchController
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from arm_interfaces.srv import AcquireArmMotionLock, ReleaseArmMotionLock
import evdev
from evdev import ecodes
import socket

from arm_teleop.arm_motion_lock import ArmMotionBusy, arm_motion_lock


DEFAULT_LINEAR_SPEED  = 0.6
DEFAULT_ANGULAR_SPEED = 1.8
DEFAULT_PUBLISH_RATE  = 100.0
# Holds TCP attitude during translation without scaling XYZ.
HOLD_ANGULAR_GAIN = 6.0
HOLD_ANGULAR_MAX = 0.8
HOLD_CMD_EPS = 1e-4

DEFAULT_LINEAR_FRAME  = 'arm_mount_link'
DEFAULT_EE_FRAME      = 'arm_tcp_link'

DEFAULT_VIEW_FRAME    = 'arm_camera_link'
# Fallback if poses.json "home" cannot be loaded (matches SRDF group_state home).
DEFAULT_HOME_POSE     = [-1.5461, 0.1734, 0.8292, 1.4197, 0.0086, -1.5314]
# 'auto' picks a USB/external keyboard over the laptop's built-in one.
DEFAULT_KEYBOARD_DEVICE_PATH = 'auto'
DEFAULT_GAMEPAD_SHIFT_BUTTON = 10
DEFAULT_SAFE_POSE_TIMEOUT = 60.0
# Fraction of each joint's max_velocity/max_acceleration
# (arm_moveit_config/config/joint_limits.yaml) used for home moves.
DEFAULT_PLAN_EXECUTE_VELOCITY_SCALING = 1.0
DEFAULT_PLAN_EXECUTE_ACCELERATION_SCALING = 1.0

DEFAULT_GRIPPER_SPEED = 0.006   # m/s
DEFAULT_GRIPPER_STROKE = 0.012  # m — matches finger_stroke in arm_macro.xacro

GRIPPER_JOINT_NAME = 'arm_jaw_gripper_finger_right_joint'

# Minimum delay (seconds) before an activity's action runs, so the
# activity-indicator light is visibly on first.
REQUIRED_ACTIVITY_INDICATOR_PRE_DELAY_SEC = 5.0
DEFAULT_ACTIVITY_INDICATOR_PRE_DELAY_SEC = 0.0
DEFAULT_ACTIVITY_INDICATOR_TOPIC = 'activity_indicator'
ACTIVITY_INDICATOR_COLOR_ACTIVE = (0.0, 0.0, 1.0, 1.0)  # blue, a=1 (lit)
ACTIVITY_INDICATOR_COLOR_IDLE = (0.0, 0.0, 0.0, 0.0)    # off

JTC_CONTROLLER_NAME = 'robot_arm_controller'
FORWARD_CONTROLLER_NAME = 'robot_arm_forward_position_controller'
MOVEIT_GROUP_NAME = 'robot_arm'

HOME_POSE_JOINTS = [
    'arm_mount_base_joint',
    'arm_base_shoulder_joint',
    'arm_shoulder_forearm_joint',
    'arm_forearm_wrist_1_joint',
    'arm_wrist_1_wrist_2_joint',
    'arm_wrist_2_end_effector_joint',
]
def _load_home_pose_from_json(pose_name='home'):
    """Return ``pose_name`` joint positions from poses.json, or None if unavailable."""
    candidates = [
        Path('/opt/ws/src/arm/arm_teleop/poses.json'),
        Path(__file__).resolve().parent.parent / 'poses.json',
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('arm_teleop')) / 'poses.json'
        candidates.insert(0, share)
    except Exception:
        pass

    for path in candidates:
        if not path.is_file():
            continue
        try:
            data = json.loads(path.read_text())
            pose = data.get(pose_name) or {}
            values = [float(pose[name]) for name in HOME_POSE_JOINTS]
        except (KeyError, TypeError, ValueError, OSError, json.JSONDecodeError):
            continue
        if not all(math.isfinite(v) for v in values):
            return None
        return values
    return None


def _quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    return Quaternion(
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    )


def _quat_conj(q: Quaternion) -> Quaternion:
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)


def _quat_rotvec(q: Quaternion):
    w = max(-1.0, min(1.0, q.w))
    x, y, z = q.x, q.y, q.z
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    half = math.acos(w)
    sine = math.sqrt(max(0.0, 1.0 - w * w))
    if sine < 1e-8:
        return (2.0 * x, 2.0 * y, 2.0 * z)
    scale = 2.0 * half / sine
    return (scale * x, scale * y, scale * z)


def _rotate_vector_by_quat(q, x: float, y: float, z: float):
    """Rotate a free vector by a geometry_msgs quaternion (x,y,z,w)."""
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def _list_keyboard_candidates():
    """Return evdev devices that look like QWERTY keyboards (path, name, score)."""
    required = {ecodes.KEY_R, ecodes.KEY_W, ecodes.KEY_A, ecodes.KEY_ESC}
    candidates = []
    for path in evdev.list_devices():
        try:
            device = evdev.InputDevice(path)
        except (FileNotFoundError, PermissionError, OSError):
            continue
        keys = set(device.capabilities().get(ecodes.EV_KEY, []))
        if not required.issubset(keys):
            continue
        name = device.name or ''
        phys = device.phys or ''
        name_l = name.lower()
        # Skip obvious non-keyboards that still expose a few KEY_* codes.
        if any(bad in name_l for bad in ('sleep', 'lid', 'power', 'video bus', 'hdmi', 'headphone')):
            continue
        score = 0
        if 'usb' in phys or phys.startswith('usb-'):
            score += 100
        if '/input0' in phys:
            score += 20  # main HID collection on multi-interface boards
        if 'keychron' in name_l or 'keyboard' in name_l:
            score += 10
        if 'at translated' in name_l or phys.startswith('isa'):
            score -= 50  # laptop PS/2 — usually wrong when an external KB is plugged in
        score += min(len(keys), 200) / 200.0
        candidates.append((score, path, name, phys))
    candidates.sort(reverse=True)
    return candidates


def _resolve_keyboard_device_path(requested: str) -> str | None:
    """Resolve ``auto`` / empty to the best keyboard path, else return ``requested``."""
    requested = (requested or '').strip()
    if requested and requested.lower() != 'auto':
        return requested
    candidates = _list_keyboard_candidates()
    if not candidates:
        return None
    return candidates[0][1]

# Must mirror moveit_servo::StatusCode (status_codes.h) exactly.
SERVO_STATUS_INVALID                              = -1
SERVO_STATUS_OK                                    = 0
SERVO_STATUS_DECELERATE_FOR_APPROACHING_SINGULARITY = 1
SERVO_STATUS_HALT_FOR_SINGULARITY                  = 2
SERVO_STATUS_DECELERATE_FOR_COLLISION              = 3
SERVO_STATUS_HALT_FOR_COLLISION                    = 4
SERVO_STATUS_JOINT_BOUND                           = 5
SERVO_STATUS_DECELERATE_FOR_LEAVING_SINGULARITY    = 6

SERVO_STATUS_NAMES = {
    SERVO_STATUS_INVALID: 'INVALID',
    SERVO_STATUS_OK: 'NO_WARNING',
    SERVO_STATUS_DECELERATE_FOR_APPROACHING_SINGULARITY: 'DECELERATE_FOR_APPROACHING_SINGULARITY',
    SERVO_STATUS_HALT_FOR_SINGULARITY: 'HALT_FOR_SINGULARITY',
    SERVO_STATUS_DECELERATE_FOR_COLLISION: 'DECELERATE_FOR_COLLISION',
    SERVO_STATUS_HALT_FOR_COLLISION: 'HALT_FOR_COLLISION',
    SERVO_STATUS_JOINT_BOUND: 'JOINT_BOUND',
    SERVO_STATUS_DECELERATE_FOR_LEAVING_SINGULARITY: 'DECELERATE_FOR_LEAVING_SINGULARITY',
}


class ServoController(Node):
    """ROS2 node that turns Cartesian velocity commands into MoveIt Servo
    ``TwistStamped`` messages, with helpers to start/stop Servo and drive
    the arm to a safe pose via a ``FollowJointTrajectory`` action.
    """

    def __init__(self):
        """Declare parameters and set up publishers/subscriptions/clients."""

        super().__init__('keyboard_servo_node')

        self.declare_parameter('linear_speed',  DEFAULT_LINEAR_SPEED)
        self.declare_parameter('angular_speed', DEFAULT_ANGULAR_SPEED)
        self.declare_parameter('publish_rate',  DEFAULT_PUBLISH_RATE)
        self.declare_parameter('linear_frame',  DEFAULT_LINEAR_FRAME)
        # Deprecated alias for linear_frame (older launch/params files).
        self.declare_parameter('command_frame', DEFAULT_LINEAR_FRAME)
        self.declare_parameter('ee_frame',      DEFAULT_EE_FRAME)
        self.declare_parameter('view_frame',    DEFAULT_VIEW_FRAME)
        # A / R move to this joint vector (defaults to poses.json "home").
        self.declare_parameter('safe_pose',     DEFAULT_HOME_POSE)
        # Empty (default): auto-pick '{end_effector}_home' if poses.json has
        # it (e.g. 'jaw_home'), else fall back to 'home'. Set explicitly to
        # pin a name regardless of end_effector.
        self.declare_parameter('home_pose_name', '')
        self.declare_parameter('keyboard_device_path', DEFAULT_KEYBOARD_DEVICE_PATH)
        self.declare_parameter('gamepad_shift_button', DEFAULT_GAMEPAD_SHIFT_BUTTON)
        self.declare_parameter('safe_pose_timeout', DEFAULT_SAFE_POSE_TIMEOUT)
        self.declare_parameter('plan_execute_velocity_scaling', DEFAULT_PLAN_EXECUTE_VELOCITY_SCALING)
        self.declare_parameter('plan_execute_acceleration_scaling', DEFAULT_PLAN_EXECUTE_ACCELERATION_SCALING)
        self.declare_parameter('gripper_speed', DEFAULT_GRIPPER_SPEED)
        self.declare_parameter('gripper_stroke', DEFAULT_GRIPPER_STROKE)
        self.declare_parameter('end_effector', 'jaw')
        self.declare_parameter('activity_indicator_topic', DEFAULT_ACTIVITY_INDICATOR_TOPIC)
        # Defaults to 0.0 (no wait) for bench testing; set explicitly to
        # REQUIRED_ACTIVITY_INDICATOR_PRE_DELAY_SEC (5.0) for real runs.
        self.declare_parameter('activity_indicator_pre_delay_sec', DEFAULT_ACTIVITY_INDICATOR_PRE_DELAY_SEC)

        self._linear_speed  = self.get_parameter('linear_speed').value
        self._angular_speed = self.get_parameter('angular_speed').value
        self._publish_rate  = self.get_parameter('publish_rate').value
        linear_frame = self.get_parameter('linear_frame').value
        command_frame = self.get_parameter('command_frame').value
        if linear_frame != DEFAULT_LINEAR_FRAME:
            self._linear_frame = linear_frame
        elif command_frame != DEFAULT_LINEAR_FRAME:
            self._linear_frame = command_frame
        else:
            self._linear_frame = DEFAULT_LINEAR_FRAME
        self._ee_frame      = self.get_parameter('ee_frame').value
        self._view_frame    = self.get_parameter('view_frame').value
        # Read before home_pose_name resolution below, which derives its
        # auto-pick from it.
        self._end_effector = self.get_parameter('end_effector').value
        home_pose_name_param = self.get_parameter('home_pose_name').value
        if home_pose_name_param:
            self._home_pose_name = home_pose_name_param
        else:
            auto_home_pose_name = f'{self._end_effector}_home'
            self._home_pose_name = (
                auto_home_pose_name
                if _load_home_pose_from_json(auto_home_pose_name) is not None
                else 'home'
            )
        # Prefer poses.json home unless the caller overrode safe_pose explicitly.
        pose_from_param = list(self.get_parameter('safe_pose').value)
        pose_from_json = _load_home_pose_from_json(self._home_pose_name)
        if pose_from_param == list(DEFAULT_HOME_POSE) and pose_from_json is not None:
            self._safe_pose = pose_from_json
            pose_source = f'poses.json["{self._home_pose_name}"]'
        else:
            self._safe_pose = pose_from_param
            pose_source = 'safe_pose parameter'
        self._keyboard_device_path = self.get_parameter('keyboard_device_path').value
        self._gamepad_shift_button = int(self.get_parameter('gamepad_shift_button').value)
        self._safe_pose_timeout    = self.get_parameter('safe_pose_timeout').value
        self._plan_execute_velocity_scaling = self.get_parameter('plan_execute_velocity_scaling').value
        self._plan_execute_acceleration_scaling = self.get_parameter('plan_execute_acceleration_scaling').value
        self._gripper_speed        = self.get_parameter('gripper_speed').value
        self._gripper_stroke       = self.get_parameter('gripper_stroke').value
        self._activity_indicator_topic  = self.get_parameter('activity_indicator_topic').value
        self._activity_indicator_pre_delay_sec = self.get_parameter('activity_indicator_pre_delay_sec').value

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        # View-relative translation, kept separate from vx/vy/vz because it is
        # expressed in view_frame and only resolved to linear_frame at publish
        # time — the transform changes as the arm moves.
        self.view_vx = 0.0
        self.view_vy = 0.0
        self.view_vz = 0.0
        self._hold_quat = None
        # Scales HOLD_ANGULAR_GAIN/MAX in _orientation_hold for boosted push (set_velocity's hold_boost).
        self._hold_boost = 1.0
        self._activity_delay_active = False
        self._joint_positions = {}

        self.gripper_vel = 0.0
        # Guess (closed) until the first /joint_states reading syncs this —
        # see _on_joint_state. Avoids commanding a jump from a wrong assumed
        # position on startup if the gripper wasn't actually closed.
        self._gripper_position = 0.0
        self._gripper_state_received = False
        self._last_gripper_tick_time = None

        self._motion_lock = threading.Lock()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Servo subscribes BEST_EFFORT; default RELIABLE can drop twists.
        self._pub = self.create_publisher(
            TwistStamped, 'servo_node/delta_twist_cmds', qos_profile_sensor_data
        )
        self._gripper_right_pub = self.create_publisher(
            Float64MultiArray, 'gripper_right_controller/commands', 10
        )
        self._gripper_left_pub = self.create_publisher(
            Float64MultiArray, 'gripper_left_controller/commands', 10
        )
        self._start_client = self.create_client(Trigger, 'servo_node/start_servo')
        self._stop_client  = self.create_client(Trigger, 'servo_node/stop_servo')
        self._switch_client = self.create_client(
            SwitchController, 'controller_manager/switch_controller'
        )
        self._list_controllers_client = self.create_client(
            ListControllers, 'controller_manager/list_controllers'
        )
        # Home moves plan through this (OMPL, collision-checked against the
        # live planning scene) instead of a raw FollowJointTrajectory goal —
        # see _move_to_joint_positions_locked().
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._acquire_lock_client = self.create_client(
            AcquireArmMotionLock, 'arm_motion_lock/acquire')
        self._release_lock_client = self.create_client(
            ReleaseArmMotionLock, 'arm_motion_lock/release')
        # Identifies THIS process to arm_motion_lock_server — hostname
        # covers the actual cross-host case, pid separates two runs on
        # the same host (e.g. sim + a stray leftover process).
        self._motion_lock_holder_id = f'{socket.gethostname()}/keyboard_servo_node/{os.getpid()}'
        self._js_sub = self.create_subscription(
            JointState, 'joint_states', self._on_joint_state, 10
        )
        # See run_planned_activity's own docstring — publishes
        # activity-indicator INTENT; nothing in this repo drives a physical
        # lamp off it yet.
        self._activity_indicator_pub = self.create_publisher(
            ColorRGBA, self._activity_indicator_topic, 10
        )
        self._timer = self.create_timer(1.0 / self._publish_rate, self._publish)

        self._servo_status = SERVO_STATUS_OK
        self._status_sub = self.create_subscription(
            Int8,
            'servo_node/status',
            self._on_servo_status,
            10
        )

        self.get_logger().info(
            f'ServoController ready — '
            f'linear_speed={self._linear_speed}, '
            f'angular_speed={self._angular_speed}, '
            f'linear_frame={self._linear_frame} (XYZ + Servo twist frame), '
            f'ee_frame={self._ee_frame} (roll/pitch/yaw input), '
            f'view_frame={self._view_frame} (view-relative translation), '
            f'A/R home from {pose_source}: {[round(v, 4) for v in self._safe_pose]}'
        )
        self.get_logger().warn(
            'Servo runs with check_collisions=true (self/scene proximity '
            'thresholds 0.003/0.005 m, see servo.yaml) — teleop WILL decelerate '
            'near a modeled collision, watch for "Close to a collision, '
            'decelerating" in servo_node\'s own log. Singularity deceleration is '
            'still effectively disabled (lower_singularity_threshold=10000.0).'
        )

    @property
    def linear_speed(self) -> float:
        """Return the configured linear speed scale, in meters per second."""
        return self._linear_speed

    @property
    def angular_speed(self) -> float:
        """Return the configured angular speed scale, in radians per second."""
        return self._angular_speed

    @property
    def gripper_speed(self) -> float:
        """Return the configured gripper speed, in meters per second."""
        return self._gripper_speed

    @property
    def gripper_stroke(self) -> float:
        """Return the configured gripper stroke (fully-open finger position, in meters)."""
        return self._gripper_stroke

    @property
    def keyboard_device_path(self) -> str:
        """Return the filesystem path of the keyboard input device (evdev)."""
        return self._keyboard_device_path

    @property
    def gamepad_shift_button(self) -> int:
        """Return the Joy button index that shifts the right stick."""
        return self._gamepad_shift_button

    @property
    def end_effector(self) -> str:
        """Return the 'end_effector' parameter (which tool is mounted)."""
        return self._end_effector

    def set_velocity(self, vx=0.0, vy=0.0, vz=0.0,
                     wx=0.0, wy=0.0, wz=0.0,
                     view_vx=0.0, view_vy=0.0, view_vz=0.0,
                     hold_boost=1.0):
        """Set the current Cartesian velocity command.

        vx/vy/vz/wx/wy/wz are in ``linear_frame``/``ee_frame``; view_vx/vy/vz
        are an added, independent translation in ``view_frame``.
        """
        if self._activity_delay_active:
            vx = vy = vz = wx = wy = wz = view_vx = view_vy = view_vz = 0.0
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.wx = wx
        self.wy = wy
        self.wz = wz
        self.view_vx = view_vx
        self.view_vy = view_vy
        self.view_vz = view_vz
        self._hold_boost = hold_boost

    def set_gripper_velocity(self, vel: float):
        """Set the current gripper velocity command, in meters per second.

        Positive opens (toward gripper_stroke), negative closes (toward 0,
        the touching/closed position set by finger_x_closed in the URDF).
        """
        self.gripper_vel = 0.0 if self._activity_delay_active else vel

    def set_gripper_target(self, position: float):
        """Jump the visualized gripper straight to ``position`` (meters),
        for one-shot commands instead of the velocity-integrated path.
        """
        self._gripper_position = max(0.0, min(self._gripper_stroke, position))

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = float(pos)
        # Runs only until the first message that names GRIPPER_JOINT_NAME —
        # after that, _gripper_position is our own commanded state and the
        # real joint may legitimately lag behind it while moving.
        if not self._gripper_state_received and GRIPPER_JOINT_NAME in msg.name:
            index = msg.name.index(GRIPPER_JOINT_NAME)
            self._gripper_position = msg.position[index]
            self._gripper_state_received = True

    def stop(self):
        """Zero out all velocity components, halting Cartesian and gripper motion."""
        self.set_velocity()
        self.set_gripper_velocity(0.0)
        self._hold_quat = None

    def _controller_states(self) -> dict:
        """Return {controller_name: state} via list_controllers, or {} on failure."""
        if not self._list_controllers_client.wait_for_service(timeout_sec=2.0):
            return {}
        done_event = threading.Event()
        states = {}

        def _cb(future):
            try:
                for c in future.result().controller:
                    states[c.name] = c.state
            except Exception as exc:
                self.get_logger().error(f'list_controllers exception: {exc!r}')
            finally:
                done_event.set()

        future = self._list_controllers_client.call_async(ListControllers.Request())
        future.add_done_callback(_cb)
        done_event.wait(timeout=3.0)
        return states

    def _switch_controllers(self, activate, deactivate) -> bool:
        """Activate/deactivate ros2_control controllers (JTC <-> forward)."""
        if not self._switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('controller_manager/switch_controller unavailable')
            return False

        states = self._controller_states()
        if states:
            activate = [c for c in activate if states.get(c) != 'active']
            deactivate = [c for c in deactivate if states.get(c) == 'active']
            if not activate and not deactivate:
                return True
        # If list_controllers itself failed, fall through with the original,
        # unfiltered lists rather than silently dropping deactivate targets.

        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = Duration(sec=3, nanosec=0)

        done_event = threading.Event()
        outcome = {'ok': False}

        def _cb(future):
            try:
                res = future.result()
                outcome['ok'] = bool(res.ok)
                if not res.ok:
                    self.get_logger().error(
                        f'Controller switch failed (activate={activate}, '
                        f'deactivate={deactivate})'
                    )
            except Exception as exc:
                self.get_logger().error(f'Controller switch exception: {exc!r}')
            finally:
                done_event.set()

        future = self._switch_client.call_async(req)
        future.add_done_callback(_cb)
        if not done_event.wait(timeout=5.0):
            self.get_logger().error('Controller switch timed out')
            return False
        if outcome['ok']:
            self.get_logger().info(
                f'Controllers: activate={list(activate)} deactivate={list(deactivate)}'
            )
        return outcome['ok']

    def use_trajectory_controller(self) -> bool:
        """Claim joints with JTC for home / Plan&Execute / teach_poses."""
        return self._switch_controllers(
            activate=[JTC_CONTROLLER_NAME],
            deactivate=[FORWARD_CONTROLLER_NAME],
        )

    def use_streaming_controller(self) -> bool:
        """Claim joints with forward position controller for Servo teleop."""
        return self._switch_controllers(
            activate=[FORWARD_CONTROLLER_NAME],
            deactivate=[JTC_CONTROLLER_NAME],
        )

    def _signal_activity_indicator(self, active: bool) -> None:
        """Publish the activity-indicator colour (best-effort, never raises)."""
        r, g, b, a = ACTIVITY_INDICATOR_COLOR_ACTIVE if active else ACTIVITY_INDICATOR_COLOR_IDLE
        msg = ColorRGBA(r=r, g=g, b=b, a=a)
        self._activity_indicator_pub.publish(msg)

    def run_planned_activity(self, action, label: str):
        """Run ``action`` gated by an activity-indicator warm-up: light on,
        wait ``activity_indicator_pre_delay_sec``, run the action, light
        off. Also force-zeroes velocity/gripper commands during the wait.
        """
        delay = self._activity_indicator_pre_delay_sec
        self.get_logger().info(
            f'{label}: activity indicator on, holding {delay:.1f}s before moving...'
        )
        self.stop()
        self._activity_delay_active = True
        self._signal_activity_indicator(True)
        try:
            time.sleep(delay)  # no-op for delay <= 0.0 (bench-testing default)
            self._activity_delay_active = False
            return action()
        finally:
            self._activity_delay_active = False
            self._signal_activity_indicator(False)

    def stop_servo(self) -> bool:
        """Stop Servo, then re-activate the trajectory controller.

        Returns True if Servo stopped AND the controller switch succeeded.
        """
        if not self._stop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servo stop service not available')
            return False

        done_event = threading.Event()

        def _cb(future):
            done_event.set()

        future = self._stop_client.call_async(Trigger.Request())
        future.add_done_callback(_cb)

        if not done_event.wait(timeout=10.0):
            self.get_logger().warn('Servo stop timed out')
            return False

        # Prefer JTC when teleop is idle so Plan&Execute / home work.
        return self.use_trajectory_controller()

    def move_to_safe_pose(self, positions=None, name=None):
        """Stop motion, confirm Servo/JTC are ready, and move to the home pose."""
        target_positions = list(self._safe_pose) if positions is None else list(positions)
        target_name = self._home_pose_name if name is None else name

        self.stop()

        if not self.stop_servo():
            self.get_logger().error(
                'Could not confirm Servo stopped — aborting home move.'
            )
            return False

        if not self.use_trajectory_controller():
            self.get_logger().error(
                'Could not activate trajectory controller — aborting home move.'
            )
            return False

        return self._move_to_joint_positions(target_positions, f'home ({target_name})')

    def _move_to_joint_positions(self, target_positions, label: str) -> bool:
        """Plan (OMPL, collision-checked) and execute a move to
        ``target_positions``, guarded by both an in-process lock and the
        cross-process ``arm_motion_lock()`` so two callers can't race.
        """
        if not self._motion_lock.acquire(blocking=False):
            self.get_logger().error(
                f'Another arm motion is already in progress — aborting {label}.'
            )
            return False
        lease_sec = self._safe_pose_timeout + 15.0 if self._safe_pose_timeout > 0.0 else 600.0
        try:
            try:
                with arm_motion_lock(
                        self._acquire_lock_client, self._release_lock_client,
                        self._motion_lock_holder_id, lease_sec):
                    return self._move_to_joint_positions_locked(target_positions, label)
            except ArmMotionBusy as exc:
                self.get_logger().error(f'{exc} — aborting {label}.')
                return False
        finally:
            self._motion_lock.release()

    def _move_to_joint_positions_locked(self, target_positions, label: str) -> bool:
        joint_constraints = [
            JointConstraint(joint_name=n, position=p, tolerance_above=0.005, tolerance_below=0.005, weight=1.0)
            for n, p in zip(HOME_POSE_JOINTS, target_positions)
        ]
        self.get_logger().info(
            f'Moving to {label} (collision-checked plan): {[round(v, 4) for v in target_positions]}'
        )
        success, error = self._execute_move_group_constraints(
            Constraints(joint_constraints=joint_constraints)
        )
        if not success:
            self.get_logger().error(f'{label} move failed: {error}')
            return False
        self.get_logger().info(f'{label} reached!')
        return True

    def _execute_move_group_constraints(self, constraints: Constraints) -> tuple[bool, str]:
        """Plan and execute a single move_action goal for ``constraints``.

        Returns ``(success, error)``; ``error`` is empty on success.
        """
        goal = MoveGroup.Goal()
        goal.request.group_name = MOVEIT_GROUP_NAME
        goal.request.pipeline_id = 'ompl'
        goal.request.goal_constraints = [constraints]
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0

        goal.request.max_velocity_scaling_factor = self._plan_execute_velocity_scaling
        goal.request.max_acceleration_scaling_factor = self._plan_execute_acceleration_scaling
        goal.planning_options.plan_only = False  # plan then execute in one goal
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5

        done_event = threading.Event()
        outcome = {'success': False, 'error': ''}
        goal_handle_box = {}

        def result_cb(future):
            try:
                wrapped = future.result()
                result = wrapped.result
                if (wrapped.status == GoalStatus.STATUS_SUCCEEDED
                        and result.error_code.val == MoveItErrorCodes.SUCCESS):
                    outcome['success'] = True
                else:
                    outcome['error'] = (
                        f'goal status {wrapped.status}, '
                        f'MoveIt error code {result.error_code.val}'
                    )
            except Exception as e:
                outcome['error'] = f'failed to read result: {e!r}'
            finally:
                done_event.set()

        def goal_response_cb(future):
            try:
                goal_handle = future.result()
            except Exception as e:
                goal_handle = None
                outcome['error'] = f'goal request failed: {e!r}'
            if not goal_handle or not goal_handle.accepted:
                outcome['error'] = outcome['error'] or 'goal rejected'
                done_event.set()
                return
            goal_handle_box['gh'] = goal_handle
            goal_handle.get_result_async().add_done_callback(result_cb)

        future = self._move_group_client.send_goal_async(goal)
        future.add_done_callback(goal_response_cb)

        timeout = self._safe_pose_timeout if self._safe_pose_timeout > 0.0 else None
        if not done_event.wait(timeout=timeout):
            self.get_logger().warn(
                f'No move_group result within {self._safe_pose_timeout:.1f}s — '
                'controller may be unresponsive '
                '(raise the safe_pose_timeout parameter if the sim is just slow).'
            )
            gh = goal_handle_box.get('gh')
            if gh is not None:
                cancel_done = threading.Event()
                gh.cancel_goal_async().add_done_callback(lambda _f: cancel_done.set())
                cancel_done.wait(timeout=5.0)
            return False, 'timed out waiting for a result'
        return outcome['success'], outcome['error']

    def start_servo(self) -> bool:
        """Switch to streaming controller, then start MoveIt Servo.

        Falls back to the trajectory controller on any failure.
        """
        if not self.use_streaming_controller():
            self.get_logger().error(
                'Could not activate forward position controller — Servo not started.'
            )
            return False
        if not self._start_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Servo start service not available')
            self.use_trajectory_controller()
            return False

        done_event = threading.Event()
        outcome = {'ok': False}

        def _cb(future):
            try:
                result = future.result()
                outcome['ok'] = bool(result.success)
                if not result.success:
                    self.get_logger().warn(f'Servo start failed: {result.message}')
            except Exception as e:
                self.get_logger().error(f'Servo start error: {e}')
            finally:
                done_event.set()

        future = self._start_client.call_async(Trigger.Request())
        future.add_done_callback(_cb)

        if not done_event.wait(timeout=20.0):
            self.get_logger().error('Servo start timed out')
            self.use_trajectory_controller()
            return False

        if outcome['ok']:
            self.get_logger().info('Servo started successfully')
        else:
            self.use_trajectory_controller()
        return outcome['ok']

    def _on_servo_status(self, msg: Int8):
        """Handle incoming Servo status updates.

        Auto-restarts Servo only on SERVO_STATUS_HALT_FOR_SINGULARITY;
        other halts (e.g. collision) are left for the operator to resolve.
        """
        code = msg.data
        if code != self._servo_status:
            name = SERVO_STATUS_NAMES.get(code, f'UNKNOWN({code})')
            if code in (SERVO_STATUS_OK, SERVO_STATUS_DECELERATE_FOR_APPROACHING_SINGULARITY,
                        SERVO_STATUS_DECELERATE_FOR_LEAVING_SINGULARITY):
                self.get_logger().info(f'Servo status -> {name}')
            elif code == SERVO_STATUS_DECELERATE_FOR_COLLISION:
                # Scales velocity down, doesn't zero it — motion continues.
                self.get_logger().warn(f'Servo status -> {name} (decelerating, not stopped)')
            else:
                self.get_logger().warn(f'Servo status -> {name} (motion stopped by Servo)')
            if code == SERVO_STATUS_HALT_FOR_SINGULARITY:
                self.start_servo()
        self._servo_status = code

    def _linear_in_command_frame(self):
        """Sum mount-frame and view-frame translation, both in ``linear_frame``.

        Falls back to the mount-frame part alone if TF is unavailable.
        """
        if (self.view_vx == 0.0 and self.view_vy == 0.0
                and self.view_vz == 0.0):
            return self.vx, self.vy, self.vz
        if self._view_frame == self._linear_frame:
            return (self.vx + self.view_vx,
                    self.vy + self.view_vy,
                    self.vz + self.view_vz)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._linear_frame,
                self._view_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'TF {self._linear_frame} <- {self._view_frame} unavailable '
                f'({exc}); ignoring view-relative translation',
                throttle_duration_sec=2.0,
            )
            return self.vx, self.vy, self.vz
        rx, ry, rz = _rotate_vector_by_quat(
            transform.transform.rotation,
            self.view_vx, self.view_vy, self.view_vz,
        )
        return self.vx + rx, self.vy + ry, self.vz + rz

    def _angular_in_command_frame(self):
        """Map EEF-frame angular velocity into ``linear_frame`` via TF, or
        (0, 0, 0) if TF is unavailable.
        """
        if self.wx == 0.0 and self.wy == 0.0 and self.wz == 0.0:
            return 0.0, 0.0, 0.0
        if self._ee_frame == self._linear_frame:
            return self.wx, self.wy, self.wz
        try:
            transform = self._tf_buffer.lookup_transform(
                self._linear_frame,
                self._ee_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'TF {self._linear_frame} <- {self._ee_frame} unavailable '
                f'({exc}); publishing zero angular command',
                throttle_duration_sec=2.0,
            )
            return 0.0, 0.0, 0.0
        return _rotate_vector_by_quat(
            transform.transform.rotation, self.wx, self.wy, self.wz
        )

    def _orientation_hold(self, wx, wy, wz):
        """Keep TCP attitude while translating (Q/E pitch, also WASD).

        Does not scale linear speed. I/K/U/O/J/L still command rotation.
        """
        driving_lin = (
            abs(self.vx) > HOLD_CMD_EPS or abs(self.vy) > HOLD_CMD_EPS or
            abs(self.vz) > HOLD_CMD_EPS or
            abs(self.view_vx) > HOLD_CMD_EPS or
            abs(self.view_vy) > HOLD_CMD_EPS or
            abs(self.view_vz) > HOLD_CMD_EPS
        )
        driving_ang = (
            abs(self.wx) > HOLD_CMD_EPS or abs(self.wy) > HOLD_CMD_EPS or
            abs(self.wz) > HOLD_CMD_EPS
        )
        if not driving_lin:
            self._hold_quat = None
            return wx, wy, wz
        if driving_ang:
            self._hold_quat = None
            return wx, wy, wz
        try:
            transform = self._tf_buffer.lookup_transform(
                self._linear_frame, self._ee_frame, rclpy.time.Time()
            )
        except TransformException:
            return wx, wy, wz
        q = transform.transform.rotation
        quat = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
        if self._hold_quat is None:
            self._hold_quat = quat
            return wx, wy, wz
        q_err = _quat_multiply(_quat_conj(quat), self._hold_quat)
        rx, ry, rz = _quat_rotvec(q_err)
        hx, hy, hz = _rotate_vector_by_quat(quat, rx, ry, rz)
        # Scaled by hold_boost (see set_velocity) so a boosted push doesn't
        # out-muscle a fixed-strength hold — the wrist's resistance to being
        # bent grows with the push instead of staying constant.
        gain = HOLD_ANGULAR_GAIN * self._hold_boost
        cap = HOLD_ANGULAR_MAX * self._hold_boost
        wx = max(-cap, min(cap, wx + gain * hx))
        wy = max(-cap, min(cap, wy + gain * hy))
        wz = max(-cap, min(cap, wz + gain * hz))
        return wx, wy, wz

    def _publish(self):
        """Publish twist in ``linear_frame`` (mount): mount XYZ as-is,
        view-frame XYZ and EEF angular velocity rotated in via TF.
        """
        vx, vy, vz = self._linear_in_command_frame()
        wx, wy, wz = self._angular_in_command_frame()
        wx, wy, wz = self._orientation_hold(wx, wy, wz)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._linear_frame
        msg.twist.linear.x  = vx
        msg.twist.linear.y  = vy
        msg.twist.linear.z  = vz
        msg.twist.angular.x = wx
        msg.twist.angular.y = wy
        msg.twist.angular.z = wz
        self._pub.publish(msg)

        self._publish_gripper()

    def _publish_gripper(self):
        """Integrate gripper position from ``gripper_vel`` and publish.

        Withheld until the first /joint_states sync so a restart can't
        slam the gripper shut before real state arrives.
        """
        if not self._gripper_state_received:
            return

        now = time.monotonic()
        if self.gripper_vel != 0.0:
            if self._last_gripper_tick_time is not None:
                dt = now - self._last_gripper_tick_time
                self._gripper_position += self.gripper_vel * dt
            self._last_gripper_tick_time = now
        else:
            self._last_gripper_tick_time = None

        self._gripper_position = max(0.0, min(self._gripper_stroke, self._gripper_position))

        self._gripper_right_pub.publish(Float64MultiArray(data=[self._gripper_position]))
        self._gripper_left_pub.publish(Float64MultiArray(data=[-self._gripper_position]))


HELP = """
╔══════════════════════════════════════════════════╗
║  Keyboard Servo — EEF control                    ║
╠══════════════════════════════════════════════════╣
║  EEF translation (absolute, arm_mount_link):     ║
║    w / s  — +X / -X                              ║
║    a / d  — +Y / -Y                              ║
║    q / e  — +Z / -Z                              ║
║  EEF translation (view-relative, camera):        ║
║    t / g  — up / down                            ║
║  EEF rotation (about arm_tcp_link):              ║
║    i / k  — pitch (wx)                           ║
║    u / o  — yaw   (wy)                           ║
║    j / l  — roll  (wz)                           ║
║  Gripper:                                        ║
║    b / v  — open / close                         ║
║  Other:                                          ║
║    r      — move to home + start servo           ║
║    ESC/x  — exit                                 ║
╚══════════════════════════════════════════════════╝
"""


class KeyboardInputLoop:
    """Reads raw keyboard events via evdev and drives a ``ServoController``."""

    _DIRECTIONS = {
        ecodes.KEY_W: ( 1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_S: (-1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_A: ( 0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_D: ( 0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_Q: ( 0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_E: ( 0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_I: ( 0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0),  # pitch
        ecodes.KEY_K: ( 0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_U: ( 0.0,  0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0),  # yaw
        ecodes.KEY_O: ( 0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_J: ( 0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0),  # roll
        ecodes.KEY_L: ( 0.0,  0.0,  0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0),
        ecodes.KEY_T:     ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  1.0),
        ecodes.KEY_G:     ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0, -1.0),
    }

    _GRIPPER_KEYS = {
        ecodes.KEY_B: 1.0,   # open
        ecodes.KEY_V: -1.0,  # close
    }

    _KEYSTATE_UP = 0
    _KEYSTATE_DOWN = 1
    _KEYSTATE_REPEAT = 2

    def __init__(self, controller: 'ServoController'):
        """Store a reference to the controller and initialize input state."""
        self._controller = controller
        self._linear_speed = controller.linear_speed
        self._angular_speed = controller.angular_speed
        self._gripper_speed = controller.gripper_speed
        self._device_path = controller.keyboard_device_path
        self._lock = threading.Lock()
        self._pressed = set()
        self._gripper_pressed = set()
        self._exit_event = threading.Event()
        self._devices = []
        self._read_thread = None
        self._servo_started = False
        self._safe_pose_running = threading.Lock()

    def _open_device(self) -> bool:
        """Open evdev keyboard(s) for teleop; ``auto`` merges every QWERTY
        keyboard found. Returns True if at least one device opened.
        """
        requested = (self._device_path or '').strip()
        if requested and requested.lower() != 'auto':
            paths = [requested]
        else:
            paths = [p for _s, p, _n, _ph in _list_keyboard_candidates()]

        if not paths:
            self._controller.get_logger().error(
                'No suitable keyboard found via evdev. Set keyboard_device_path '
                'to an explicit /dev/input/eventN.'
            )
            return False

        opened = []
        for path in paths:
            try:
                device = evdev.InputDevice(path)
                flag = fcntl.fcntl(device.fd, fcntl.F_GETFL)
                fcntl.fcntl(device.fd, fcntl.F_SETFL, flag | os.O_NONBLOCK)
            except (FileNotFoundError, PermissionError, OSError) as e:
                self._controller.get_logger().warn(f'Skipping {path!r}: {e!r}')
                continue
            opened.append(device)
            self._controller.get_logger().info(f'Listening on {path} ({device.name})')

        if not opened:
            self._controller.get_logger().error(f'Could not open any of {paths!r}')
            return False

        self._devices = opened
        self._device_path = opened[0].path
        print(
            '\nKeyboard input:\n  '
            + '\n  '.join(f'{d.name} ({d.path})' for d in opened)
            + '\nPress r = home + start Servo, then WASD to move.\n'
        )
        return True

    def _recompute_velocity(self):
        """Recompute and apply the combined velocity from all pressed keys."""
        vx = vy = vz = wx = wy = wz = 0.0
        cvx = cvy = cvz = 0.0
        with self._lock:
            active = list(self._pressed)
        for code in active:
            d = self._DIRECTIONS.get(code)
            if d is None:
                continue
            vx += d[0]
            vy += d[1]
            vz += d[2]
            wx += d[3]
            wy += d[4]
            wz += d[5]
            cvx += d[6]
            cvy += d[7]
            cvz += d[8]
        self._controller.set_velocity(
            vx * self._linear_speed, vy * self._linear_speed, vz * self._linear_speed,
            wx * self._angular_speed, wy * self._angular_speed, wz * self._angular_speed,
            view_vx=cvx * self._linear_speed,
            view_vy=cvy * self._linear_speed,
            view_vz=cvz * self._linear_speed,
        )

    def _recompute_gripper_velocity(self):
        """Recompute and apply gripper velocity from currently pressed b/v."""
        with self._lock:
            active = list(self._gripper_pressed)
        vel = sum(self._GRIPPER_KEYS.get(c, 0.0) for c in active) * self._gripper_speed
        self._controller.set_gripper_velocity(vel)

    def _handle_safe_pose(self):
        """Clear pressed keys, stop motion, and move to the safe pose.

        Servo is only started if the safe-pose move actually succeeded.
        """
        if not self._safe_pose_running.acquire(blocking=False):
            return
        try:
            with self._lock:
                self._pressed.clear()
                self._gripper_pressed.clear()
            self._controller.stop()
            print('Moving to home...')
            if self._controller.run_planned_activity(self._controller.move_to_safe_pose, 'move_to_safe_pose'):
                if self._exit_event.is_set():
                    print('Exit requested during home move — Servo not started.')
                    return
                print('Starting servo...')
                if self._controller.start_servo():
                    self._servo_started = True
                else:
                    print('Servo failed to start — staying on trajectory controller.')
            else:
                print('Home move failed — Servo not started.')
        finally:
            self._safe_pose_running.release()

    def _read_loop(self):
        """Continuously read raw key events from all opened keyboards."""
        try:
            while not self._exit_event.is_set():
                if not self._devices:
                    break
                try:
                    ready, _, _ = select.select(
                        [dev.fd for dev in self._devices], [], [], 0.2
                    )
                except (ValueError, OSError) as e:
                    self._controller.get_logger().error(f'Keyboard select failed: {e!r}')
                    break
                if not ready:
                    continue
                fd_to_dev = {dev.fd: dev for dev in self._devices}
                for fd in ready:
                    device = fd_to_dev.get(fd)
                    if device is None:
                        continue
                    try:
                        for event in device.read():
                            if event.type != ecodes.EV_KEY:
                                continue
                            code, value = event.code, event.value

                            if code in (ecodes.KEY_ESC, ecodes.KEY_X) and value == self._KEYSTATE_DOWN:
                                self._exit_event.set()
                                return

                            if code == ecodes.KEY_R and value == self._KEYSTATE_DOWN:
                                threading.Thread(
                                    target=self._handle_safe_pose, daemon=True
                                ).start()
                                continue

                            if code in self._GRIPPER_KEYS:
                                if not self._servo_started:
                                    continue
                                if value == self._KEYSTATE_DOWN:
                                    with self._lock:
                                        already_pressed = code in self._gripper_pressed
                                        other = (ecodes.KEY_V if code == ecodes.KEY_B
                                                 else ecodes.KEY_B)
                                        self._gripper_pressed.discard(other)
                                        self._gripper_pressed.add(code)
                                    if not already_pressed:
                                        self._recompute_gripper_velocity()
                                        key_name = ecodes.KEY[code].removeprefix('KEY_').lower()
                                        print(f'{key_name} gripper_vel={self._controller.gripper_vel:.4f}')
                                elif value == self._KEYSTATE_UP:
                                    with self._lock:
                                        self._gripper_pressed.discard(code)
                                    self._recompute_gripper_velocity()
                                continue

                            if code not in self._DIRECTIONS:
                                continue

                            if not self._servo_started:
                                continue

                            if value == self._KEYSTATE_DOWN:
                                with self._lock:
                                    already_pressed = code in self._pressed
                                    self._pressed.add(code)
                                if not already_pressed:
                                    self._recompute_velocity()
                                    key_name = ecodes.KEY[code].removeprefix('KEY_').lower()
                                    print(
                                        f'{key_name} vx={self._controller.vx:.2f} '
                                        f'vy={self._controller.vy:.2f} '
                                        f'vz={self._controller.vz:.2f} '
                                        f'wx={self._controller.wx:.2f} '
                                        f'wy={self._controller.wy:.2f} '
                                        f'wz={self._controller.wz:.2f} '
                                        f'| fwd={self._controller.view_vx:.2f} '
                                        f'left={self._controller.view_vy:.2f} '
                                        f'up={self._controller.view_vz:.2f}'
                                    )
                            elif value == self._KEYSTATE_UP:
                                with self._lock:
                                    self._pressed.discard(code)
                                self._recompute_velocity()
                    except BlockingIOError:
                        continue
                    except OSError as e:
                        self._controller.get_logger().warn(
                            f'Lost keyboard {device.path}: {e!r}'
                        )
                        self._devices = [d for d in self._devices if d.fd != fd]
                        if not self._devices:
                            raise
        except OSError as e:
            self._controller.get_logger().error(f'Keyboard read loop failed: {e!r}')
        finally:
            self._exit_event.set()

    def run(self):
        """Open the keyboard device and run the input loop until exit."""
        if not self._open_device():
            return
        print(HELP, flush=True)
        self._controller.get_logger().info(
            'Keyboard teleop ready. Press r = home + Servo, then WASD. '
            'Do not start a second keyboard_servo_node.'
        )

        # ros2 launch often has no TTY; fileno()/tcgetattr would abort the node.
        old_term_settings = None
        stdin_fd = None
        try:
            stdin_fd = sys.stdin.fileno()
        except (AttributeError, ValueError, OSError):
            stdin_fd = None
        if stdin_fd is not None:
            try:
                old_term_settings = termios.tcgetattr(stdin_fd)
                new_term_settings = termios.tcgetattr(stdin_fd)
                new_term_settings[3] &= ~termios.ECHO
                termios.tcsetattr(stdin_fd, termios.TCSADRAIN, new_term_settings)
            except (termios.error, OSError):
                old_term_settings = None

        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        try:
            self._exit_event.wait()
        finally:
            print('\nExiting...')
            with self._safe_pose_running:
                pass
            self._controller.stop()
            if old_term_settings is not None:
                termios.tcflush(stdin_fd, termios.TCIFLUSH)
                termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term_settings)


GAMEPAD_HELP = """
╔══════════════════════════════════════════════╗
║  Gamepad — EEF control (view-relative)       ║
╠══════════════════════════════════════════════╣
║  Left stick   ←→  — left / right  (camera)   ║
║               ↑↓  — forward / back (camera)  ║
║  Right stick  ↑↓  — up / down     (camera)   ║
║               ←→  — yaw   (TCP)              ║
║  R1 + right   ↑↓  — pitch (TCP)              ║
║               ←→  — roll  (TCP)              ║
║  9 (button)       — push boost (hold)        ║
║  11 (button)      — gripper OPEN             ║
║  13 (button)      — gripper CLOSE            ║
║  A                — home + start servo       ║
║  X                — exit                     ║
╚══════════════════════════════════════════════╝
"""


class GamepadInputLoop:
    """Reads sensor_msgs/Joy messages and drives a ``ServoController``.

    Launch via ``arm_teleop/launch/gamepad.launch.py``, which starts
    ``game_controller_node`` for a stable, SDL-mapped button/axis layout.
    """

    # Gamepad translation is view-relative (camera frame); mount-frame
    # XYZ stays available on the keyboard only.
    AXIS_LEFT_X = 0     # view +Y / -Y  (left / right)
    AXIS_LEFT_Y = 1     # view +X / -X  (forward / back)
    AXIS_RIGHT_X = 2    # yaw (-wy)     — roll  (-wz) while R1 held
    AXIS_RIGHT_Y = 3    # view +Z / -Z  — pitch (+wx) while R1 held
    AXIS_L2 = 4         # unmapped; used only for trigger rest calibration
    AXIS_R2 = 5

    BUTTON_SAFE_POSE = 0    # 'A' — move to home + start servo
    BUTTON_EXIT = 2         # 'X' — exit
    BUTTON_PUSH_BOOST = 9   # LEFTSHOULDER/L1 — scales up commanded velocity
    PUSH_BOOST_MULTIPLIER = 3.0

    BUTTON_GRIPPER_OPEN = 11
    BUTTON_GRIPPER_CLOSE = 13

    _DEADZONE = 0.2
    _JOY_TIMEOUT_SEC = 0.2
    _WATCHDOG_PERIOD_SEC = 0.1

    def __init__(self, controller: 'ServoController'):
        """Store a reference to the controller and subscribe to ``/joy``."""
        self._controller = controller
        self._linear_speed = controller.linear_speed
        self._angular_speed = controller.angular_speed
        self._shift_button = controller.gamepad_shift_button
        self._exit_event = threading.Event()
        self._prev_buttons = None
        self._safe_pose_running = threading.Lock()
        self._safe_pose_active = False
        self._prev_cmd = (0.0,) * 6

        self._last_joy_time = None
        self._joy_silent = False
        self._teleop_locked = True

        self._joy_settling = False
        # Per-trigger rest samples (axis index -> float). None until the
        # first centered settle so we do not assume both are +1.0.
        self._trigger_rest = {}

        self._sub = controller.create_subscription(Joy, 'joy', self._on_joy, 10)
        self._watchdog_timer = controller.create_timer(
            self._WATCHDOG_PERIOD_SEC, self._check_joy_timeout
        )

    @classmethod
    def _deadzone(cls, value: float) -> float:
        """Zero out small stick values so resting drift doesn't creep the arm."""
        return 0.0 if abs(value) < cls._DEADZONE else value

    def _axis(self, axes, index: int) -> float:
        """Return ``axes[index]`` with deadzone applied, or 0.0 if out of range."""
        if index >= len(axes):
            return 0.0
        return self._deadzone(axes[index])

    def _calibrate_triggers(self, axes) -> None:
        """Record L2/R2 rest values from a centered Joy snapshot."""
        for index in (self.AXIS_L2, self.AXIS_R2):
            if index < len(axes):
                self._trigger_rest[index] = float(axes[index])
        self._controller.get_logger().info(
            f'Trigger rest L2={self._trigger_rest.get(self.AXIS_L2, float("nan")):.2f} '
            f'R2={self._trigger_rest.get(self.AXIS_R2, float("nan")):.2f}'
        )

    def _sticks_centered(self, axes) -> bool:
        """True when both sticks are inside the deadzone (triggers ignored)."""
        return all(
            self._axis(axes, i) == 0.0
            for i in (self.AXIS_LEFT_X, self.AXIS_LEFT_Y,
                      self.AXIS_RIGHT_X, self.AXIS_RIGHT_Y)
        )

    def _trigger_amount(self, axes, index: int) -> float:
        """Return how far a trigger (L2/R2) is pressed: 0.0 .. 1.0.

        Supports both rest conventions: rest near +1 (press toward -1)
        and rest near 0 (press toward +/-1).
        """
        if index >= len(axes):
            return 0.0
        raw = float(axes[index])
        rest = self._trigger_rest.get(index)

        if rest is None:
            if raw >= 0.5 or raw <= -0.5:
                amount = (1.0 - raw) / 2.0
            else:
                amount = 0.0
        elif rest > 0.5:
            amount = (rest - raw) / (rest - (-1.0))
        else:
            amount = abs(raw - rest)

        if amount < 0.0:
            amount = 0.0
        elif amount > 1.0:
            amount = 1.0
        return 0.0 if amount < self._DEADZONE else amount

    def _button_pressed(self, buttons, index: int) -> bool:
        """Return True if ``buttons[index]`` is currently held down."""
        return index < len(buttons) and buttons[index] == 1

    def _button_rising_edge(self, buttons, index: int) -> bool:
        """Return True if ``buttons[index]`` was just pressed this message."""
        if self._prev_buttons is None:
            return False
        was_pressed = index < len(self._prev_buttons) and self._prev_buttons[index] == 1
        return self._button_pressed(buttons, index) and not was_pressed

    @staticmethod
    def _active_label(view_vx, view_vy, view_vz, wx, wy, wz, shift: bool) -> str:
        """Describe which physical control(s) are driving a nonzero command."""
        parts = []
        if view_vx or view_vy:
            parts.append('left stick')
        if view_vz or wy:
            parts.append('right stick')
        if wx or wz:
            parts.append('R1+right stick')
        return '+'.join(parts) if parts else ('R1' if shift else 'idle')

    def _check_joy_timeout(self):
        """Stop the arm if no ``/joy`` message has arrived recently.

        Control resumes automatically once ``/joy`` messages return.
        """
        if self._last_joy_time is None:
            return
        elapsed = (self._controller.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
        if elapsed > self._JOY_TIMEOUT_SEC:
            if not self._joy_silent:
                self._joy_silent = True
                self._joy_settling = True
                self._trigger_rest.clear()
                self._prev_buttons = None
                self._controller.get_logger().warn(
                    f'/joy timed out after {elapsed:.2f}s — stopping arm.'
                )
            self._controller.stop()

    def _log_raw_joy(self, axes, buttons, note: str = ''):
        """Log the full Joy message — an out-of-range index fails silently otherwise."""
        axes_str = ', '.join(f'{i}:{v:+.2f}' for i, v in enumerate(axes))
        buttons_str = ', '.join(f'{i}:{b}' for i, b in enumerate(buttons))
        self._controller.get_logger().info(
            f'/joy raw{note} — axes[{len(axes)}]: {{{axes_str}}}  '
            f'buttons[{len(buttons)}]: {{{buttons_str}}}'
        )

    def _on_joy(self, msg: Joy):
        """Translate one Joy snapshot into a velocity command and edge-triggered actions."""
        axes = msg.axes
        buttons = msg.buttons

        if self._joy_silent:
            self._joy_silent = False
            self._controller.get_logger().info('/joy resumed.')
        self._last_joy_time = self._controller.get_clock().now()

        if self._prev_buttons is None:
            # First message (also re-fires after a /joy dropout). Warn early
            # if gamepad_shift_button is out of range for this controller.
            self._log_raw_joy(axes, buttons, ' (first message)')
            if self._shift_button >= len(buttons):
                self._controller.get_logger().warn(
                    f'gamepad_shift_button={self._shift_button} but this '
                    f'/joy only reports {len(buttons)} button(s) '
                    f'(0..{len(buttons) - 1}) — that index can never be '
                    f'pressed. Pick a real index from the buttons[] list above.'
                )

        safe_pose_pressed = self._button_rising_edge(buttons, self.BUTTON_SAFE_POSE)
        exit_pressed = self._button_rising_edge(buttons, self.BUTTON_EXIT)
        gripper_open_pressed = self._button_rising_edge(buttons, self.BUTTON_GRIPPER_OPEN)
        gripper_close_pressed = self._button_rising_edge(buttons, self.BUTTON_GRIPPER_CLOSE)

        # Log the raw state on any button change, not just the mapped ones,
        # so an unmapped shift button still shows up.
        if self._prev_buttons is not None:
            width = max(len(buttons), len(self._prev_buttons))
            changed = [
                i for i in range(width)
                if self._button_pressed(buttons, i)
                != (i < len(self._prev_buttons) and self._prev_buttons[i] == 1)
            ]
            if changed:
                self._log_raw_joy(
                    axes, buttons,
                    f' (button(s) {changed} changed; shift configured as '
                    f'{self._shift_button})',
                )

        self._prev_buttons = list(buttons)

        if exit_pressed:
            self._exit_event.set()

        if gripper_open_pressed:
            self._controller.set_gripper_target(self._controller.gripper_stroke)
            self._controller.get_logger().info('Gripper: OPEN sent.')

        if gripper_close_pressed:
            self._controller.set_gripper_target(0.0)
            self._controller.get_logger().info('Gripper: CLOSE sent.')

        if safe_pose_pressed:
            threading.Thread(target=self._handle_safe_pose, daemon=True).start()

        if self._teleop_locked or self._safe_pose_active:
            self._controller.stop()
            return

        if self._joy_settling:
            centered = (
                self._sticks_centered(axes)
                and self._trigger_amount(axes, self.AXIS_L2) == 0.0
                and self._trigger_amount(axes, self.AXIS_R2) == 0.0
                and not self._button_pressed(buttons, self.BUTTON_PUSH_BOOST)
                and not self._button_pressed(buttons, self._shift_button)
            )
            self._controller.stop()
            if centered:
                self._calibrate_triggers(axes)
                self._joy_settling = False
                self._controller.get_logger().info('Sticks centered — resuming control.')
            return

        if not self._trigger_rest and self._sticks_centered(axes):
            self._calibrate_triggers(axes)

        boost = (self.PUSH_BOOST_MULTIPLIER
                 if self._button_pressed(buttons, self.BUTTON_PUSH_BOOST) else 1.0)
        linear_speed = self._linear_speed * boost
        angular_speed = self._angular_speed * boost

        left_x = self._axis(axes, self.AXIS_LEFT_X)
        left_y = self._axis(axes, self.AXIS_LEFT_Y)
        view_vx = left_y * linear_speed

        right_x = self._axis(axes, self.AXIS_RIGHT_X)
        right_y = self._axis(axes, self.AXIS_RIGHT_Y)
        shift = self._button_pressed(buttons, self._shift_button)

        view_vz = 0.0
        wx = wy = wz = 0.0
        if shift:
            view_vy = left_x * linear_speed
            wx = right_y * angular_speed          # pitch
            wz = -right_x * angular_speed         # roll
        else:
            view_vy = left_x * linear_speed
            view_vz = -right_y * linear_speed      # stick up = view +Z
            wy = -right_x * angular_speed         # yaw

        self._controller.set_velocity(
            0.0, 0.0, 0.0, wx, wy, wz,
            view_vx=view_vx, view_vy=view_vy, view_vz=view_vz,
            hold_boost=boost,
        )

        cmd = (view_vx, view_vy, view_vz, wx, wy, wz)
        if cmd != self._prev_cmd and any(c != 0.0 for c in cmd):
            label = self._active_label(view_vx, view_vy, view_vz, wx, wy, wz, shift)
            print(f'{label} fwd={view_vx:.2f} left={view_vy:.2f} up={view_vz:.2f} '
                  f'wx={wx:.2f} wy={wy:.2f} wz={wz:.2f}')
        self._prev_cmd = cmd

    def _handle_safe_pose(self):
        """Stop motion and move to the safe pose (mirrors KeyboardInputLoop's 'r').

        Guarded by a non-blocking lock against a second concurrent press.
        """
        if not self._safe_pose_running.acquire(blocking=False):
            return
        self._safe_pose_active = True
        try:
            self._controller.stop()
            print('Moving to home...')
            home_ok = self._controller.run_planned_activity(
                self._controller.move_to_safe_pose, 'move_to_safe_pose')
            if home_ok:
                print('Starting servo...')
                if self._controller.start_servo():
                    self._teleop_locked = False
                    self._controller.get_logger().info('Teleop enabled.')
                else:
                    self._controller.get_logger().warn(
                        'Servo failed to start — staying on trajectory controller.'
                    )
            else:
                print('Home move failed — Servo not started.')
        finally:
            self._safe_pose_active = False
            self._safe_pose_running.release()

    def run(self):
        """Print the help banner and block until the exit button is pressed."""
        print(GAMEPAD_HELP)
        try:
            self._exit_event.wait()
        finally:
            print('\nExiting...')
            self._controller.stop()


def _run_teleop(controller: 'ServoController', input_loop) -> None:
    """Spin ``controller`` in a background thread and run ``input_loop`` until exit.

    Shared by ``main`` (keyboard) and ``main_gamepad`` (gamepad).
    """
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()

    try:
        input_loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        if not controller.stop_servo():
            controller.get_logger().warn(
                'Could not confirm Servo stopped before shutdown — '
                'it may still be active.'
            )
        controller.destroy_node()
        rclpy.shutdown()


def main():
    """Entry point: initialize ROS2, run the keyboard input loop, and clean up.

    See ``_run_teleop`` for the shared spin/cleanup lifecycle.
    """
    rclpy.init()
    controller = ServoController()
    _run_teleop(controller, KeyboardInputLoop(controller))


def main_gamepad():
    """Entry point: initialize ROS2, run the gamepad input loop, and clean up.

    Requires a running ``joy`` publisher — see ``arm_teleop/launch/gamepad.launch.py``.
    See ``_run_teleop`` for the shared spin/cleanup lifecycle.
    """
    rclpy.init()
    controller = ServoController()
    _run_teleop(controller, GamepadInputLoop(controller))


if __name__ == '__main__':
    main()