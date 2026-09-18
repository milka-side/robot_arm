# robot_arm

Gazebo physics simulation of a robotic arm with gamepad teleop control and
RViz visualization. Self-contained set of ROS2 packages, runnable standalone
via Docker.

The arm ships with a standard jaw gripper end effector.

## Structure

```
docker-compose.yaml
docker/            # Dockerfile, entrypoint, Docker details README
src/
├── arm_description/    # URDF/xacro (jaw gripper only), physics, ros2_control
├── arm_moveit_config/  # SRDF, MoveIt/RViz configs
├── arm_sim/             # Gazebo launch files, world, ros_gz_bridge (/clock)
├── arm_teleop/           # keyboard/gamepad servo control
└── arm_interfaces/       # custom srv (motion lock), depended on by arm_teleop
```

## Build and run

```bash
xhost +local:docker        # allow GUI from the container
docker compose build
docker compose up -d
docker compose exec robot_arm_dev bash
```

Inside the container (once):
```bash
cd /opt/ws && colcon build --symlink-install && source install/setup.bash
```

Docker details (troubleshooting, NVIDIA runtime) — `docker/README.md`.

## Control

**Terminal 1 — simulation + RViz in one command:**
```bash
ros2 launch arm_sim arm_gazebo_rviz.launch.py
```
Wait for `spawner_joint_state_broadcaster: Configured and activated` in the logs.

**Terminal 2 (same container, `docker compose exec robot_arm_dev bash`) — gamepad:**
```bash
ros2 run joy game_controller_node --ros-args -p dev:=/dev/input/js0 -p deadzone:=0.0
ros2 run arm_teleop gamepad_servo_node
```
Launched directly via `ros2 run` (not `arm_teleop`'s `gamepad.launch.py`) —
that file hardcodes `namespace='arm'` for the real hardware bringup, while
this sim stack runs without a namespace.

**Alternative — keyboard (same `ServoController`, no gamepad):**
```bash
ros2 run arm_teleop keyboard_servo_node
```

**RViz alone, no Gazebo:**
```bash
# While a sim is already running (arm_gazebo.launch.py in another terminal):
ros2 launch arm_moveit_config moveit_rviz.launch.py

# Or with no sim at all — robot_state_publisher + joint_state_publisher_gui
# (drag sliders to move joints) + RViz:
ros2 launch arm_moveit_config display.launch.py
```

### Key bindings

Keyboard (EEF translation is mount-frame; rotation is about the TCP):
| Key | Action |
|---|---|
| w/s, a/d, q/e | +/- X, Y, Z translation |
| t/g | view-relative up/down |
| i/k, u/o, j/l | pitch, yaw, roll |
| b/v | gripper open/close |
| r | move to home + start servo |
| ESC/x | exit |

Gamepad (all translation/rotation is view-relative):
| Control | Action |
|---|---|
| Left stick | forward/back, left/right |
| Right stick | up/down, yaw |
| R1 + right stick | pitch, roll |
| A | move to home + start servo |
| X | exit |

## Sanity check

- Gazebo opens, the arm is visible, controllers are active (`spawner` log).
- RViz shows the robot model in sync with Gazebo (same `/joint_states`).
- Moving the gamepad sticks actually moves the arm in Gazebo and in RViz at
  the same time.
