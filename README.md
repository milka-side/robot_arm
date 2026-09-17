# robot_arm

Фізична симуляція робота-маніпулятора (Gazebo) з телеопераційним
керуванням від геймпада і візуалізацією в RViz. Самодостатній набір
ROS2-пакетів для окремого запуску через Docker.

Свідомо звужено до мінімуму завдання: лише стандартний jaw-гріпер (бур/
семплінг/astrobio інструменти прибрані з URDF), без панелі й без камери.

## Вимоги завдання -> де реалізовано

| Вимога | Реалізація |
|---|---|
| Фізична симуляція маніпулятора в Gazebo | `src/arm_sim/launch/arm_gazebo.launch.py` — піднімає gz sim, спавнить робота, контролери |
| Віртуальна сцена | `src/arm_sim/worlds/empty.sdf` |
| Фізико-динамічні параметри маніпулятора | `src/arm_description/urdf/arm_macro.xacro` — інерції, `<ros2_control>` блок з `gz_ros2_control/GazeboSimSystem` плагіном |
| Синхронізація стану робота | `ros2_control`/`gz_ros2_control` + `joint_state_broadcaster`, стан транслюється в `/joint_states`, звідти в RViz/MoveIt |
| Візуалізація в RViz | `src/arm_moveit_config/config/moveit.rviz` — модель робота (RobotModel/MotionPlanning), синхронна з `/joint_states`, і TF-дерево |
| Керування від геймпада | `src/arm_teleop/arm_teleop/keyboard_servo_node.py` (`main_gamepad`, спільний `ServoController`) через MoveIt Servo |

## Структура

```
docker-compose.yaml
docker/            # Dockerfile, entrypoint, README з деталями по Docker
src/
├── arm_description/    # URDF/xacro (лише jaw-гріпер), фізика, ros2_control
├── arm_moveit_config/  # SRDF, MoveIt/RViz конфіги
├── arm_sim/             # Gazebo launch-файли, world, ros_gz_bridge (/clock)
├── arm_teleop/           # keyboard/gamepad servo-керування
└── arm_interfaces/       # кастомні srv (motion lock), від яких залежить arm_teleop
```

## Збірка і запуск

```bash
xhost +local:docker        # дозволити GUI з контейнера
docker compose build
docker compose up -d
docker compose exec robot_arm_dev bash
```

Всередині контейнера (один раз):
```bash
cd /opt/ws && colcon build --symlink-install && source install/setup.bash
```

Деталі про Docker (troubleshooting, NVIDIA runtime) — `docker/README.md`.

## Керування

**Термінал 1 — симуляція + RViz одною командою:**
```bash
ros2 launch arm_sim arm_gazebo_rviz.launch.py
```
Дочекайся в логах `spawner_joint_state_broadcaster: Configured and activated`.

**Термінал 2 (той самий контейнер, `docker compose exec robot_arm_dev bash`) — геймпад:**
```bash
ros2 run joy game_controller_node --ros-args -p dev:=/dev/input/js0 -p deadzone:=0.0
ros2 run arm_teleop gamepad_servo_node
```
Запускається напряму через `ros2 run` (без `arm_teleop`'ного
`gamepad.launch.py`) — той файл хардкодить `namespace='arm'`, розрахований
на реальне залізо (`arm_bringup/arm.launch.py`), а симуляційний стек тут
працює без namespace.

**Альтернатива — клавіатура (той самий `ServoController`, без геймпада):**
```bash
ros2 run arm_teleop keyboard_servo_node
```

## Перевірка

- Gazebo відкривається, рука (лише jaw-гріпер, без бура/семплінгу/панелі/
  камери) видима, контролери активовані (лог `spawner`).
- RViz показує модель робота, синхронну з Gazebo (той самий `/joint_states`).
- Рух стіків геймпада реально рухає руку в Gazebo й одночасно в RViz.
