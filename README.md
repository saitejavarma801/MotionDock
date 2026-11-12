# MotionDock

**ROS 2-Based Robotic Control, Supervision and Task Management Platform**

MotionDock is a modular, single-robot control and supervision platform built on ROS 2. Its PySide6 desktop application brings controller-based manual driving, autonomous launch control, camera management, configurable task execution and centralized velocity-command supervision into one interface.

**ROS 2 compatibility:** Humble | Jazzy

---

## Demo

[![MotionDock overview](docs/images/motiondock_controller_mapping.png)](https://github.com/saitejavarma801/MotionDock)

---

## Key Features

### 1. Controller-Based Operation
- PlayStation controller integration through the controller-monitoring layer.
- Live display of controller connection, button and axis activity.
- Configurable button indices, joystick axes, trigger thresholds, dead zones and input directions.
- Interactive **Learn controller inputs** workflow for discovering controller indices.
- Controller connection and read diagnostics, including fallback handling for R2 input.

### 2. Operating Modes
- **Manual:** Controller-driven velocity commands.
- **Autonomous:** Enables the configured autonomous command source and can launch the selected autonomy workspace/command.
- **STOP:** Revokes motion authorization and commands zero velocity through the supervisor.
- Mode selection is available from the application and the mapped controller buttons.

### 3. Safety and Command Supervision
- A dedicated ROS 2 supervisor is the single authorized output gate to the robot's `/cmd_vel` topic.
- Manual and autonomous velocity commands use separate input topics.
- Manual driving requires the R2 dead-man input while Manual mode is active.
- Controller loss revokes manual driving authorization.
- Configurable linear and angular velocity limits.
- Command timeout monitoring and GUI heartbeat monitoring.
- The supervisor switches to STOP when its GUI heartbeat becomes stale and publishes zero velocity when stopped or when valid input is absent/stale.

### 4. Camera and Autonomous Launch Control
- Configure a camera workspace/folder and launch command from the GUI.
- Configure an autonomous workspace/folder and launch command.
- Start and stop the configured camera process from the controller.
- Start autonomy from Autonomous mode; selecting Manual or STOP stops the configured autonomy process.

### 5. Task Management
- Assign tasks to controller controls from the GUI.
- Support for user-provided Python and C++ task sources.
- Configure task workspace/folder, ROS environment sourcing and launch commands.
- Press an assigned controller button to start its task; hold the same button for three seconds to stop it.
- C++ task sources placed in the user-task package are discovered and built as individual task executables during a workspace build.
- Task controls display their configured assignment and runtime state.

### 6. Auto-Dock
- Dedicated Auto-Dock control in the application.
- Configure its workspace and launch command through the application.
- Hold the Auto-Dock application button for three seconds to stop a running Auto-Dock process.

### 7. Application Interface
- Single-page desktop control dashboard.
- Live controller diagnostics and mode/status indicators.
- Resizable controls and improved task/launch control layout.

---

## PlayStation Controller – Default Mapping

The following mapping is the default configuration. Controller button and axis indices, trigger direction and thresholds can be adapted in `config/controller.yaml` and through the application.

![MotionDock PlayStation Controller Mapping](docs/images/motiondock_controller_mapping.png)

| Controller input | Default function |
|---|---|
| Cross (X) | Select Manual mode |
| Triangle (△) | Select Autonomous mode |
| Square (□) | Select STOP mode |
| Circle (○) | Camera control — press to start; hold for 3 seconds to stop |
| R2 | Manual driving authorization (dead-man input) |
| Left stick – Up/Down | Linear velocity control |
| Left stick – Left/Right | Angular velocity control |

**Manual driving behaviour**
- Manual mode must be selected before manual velocity commands are authorized.
- R2 must remain held above the configured trigger threshold while driving.
- Releasing R2 revokes manual driving authorization.
- Controller disconnection stops manual command forwarding.
- After reconnecting, the operator must re-authorize manual driving.
- Input thresholds, hysteresis, dead zone, axis directions and controller indices are configurable.

---

## Installation

MotionDock supports ROS 2 Humble and Jazzy on Ubuntu-based ROS 2 environments. Use one ROS distribution consistently for dependency installation, building and launching.

### 1. Clone the repository

```bash
git clone https://github.com/saitejavarma801/MotionDock.git
cd MotionDock
```

### 2. Install system dependencies and create a virtual environment

The virtual environment must have access to the system-installed ROS 2 Python packages.

```bash
sudo apt update
sudo apt install -y python3-full python3-venv python3-rosdep \
  python3-colcon-common-extensions build-essential

python3 -m venv --system-site-packages .venv
```

### 3. Source ROS 2

For Humble:

```bash
source /opt/ros/humble/setup.bash
```

For Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

### 4. Install Python and ROS package dependencies

```bash
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt

rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the workspace

```bash
.venv/bin/python -m colcon build --symlink-install
```

### 6. Source the workspace

```bash
source install/setup.bash
```

### 7. Launch MotionDock

Start the desktop application:

```bash
ros2 run motiondock motiondock
```

The application starts its ROS 2 GUI bridge and command-supervision functionality. The supervisor is also installed as a separate executable:

```bash
ros2 run motiondock motiondock_supervisor
```

Run only one supervisor instance for a robot command path.

---

## Operating Modes

### Manual Mode

Manual velocity commands are authorized only when:
1. A supported controller is connected.
2. Manual mode is selected.
3. The R2 dead-man input is active above its configured threshold.

The left stick controls linear and angular velocity. Releasing R2 or losing the controller removes manual driving authorization.

### Autonomous Mode

Autonomous nodes should publish velocity commands to:

```text
/motiondock/cmd_vel_autonomous
```

The supervisor forwards autonomous commands to the final robot output only while Autonomous mode is selected. Configure the autonomy launch command to remap its velocity output to this topic.

### STOP Mode

STOP mode blocks both manual and autonomous motion commands and publishes zero velocity through the supervisor.

---

## ROS 2 Command Supervision

The supervisor provides a single authorized velocity-command path from the selected source to the robot.

| Interface | Default ROS 2 topic | Purpose |
|---|---|---|
| Final velocity output | `/cmd_vel` | Velocity command for the robot base |
| Manual input | `/motiondock/cmd_vel_manual` | Controller-generated manual velocity |
| Autonomous input | `/motiondock/cmd_vel_autonomous` | Velocity from the autonomous stack |
| Operating mode | `/motiondock/mode` | Current mode: `manual`, `autonomous` or `stop` |

### Supervisor behaviour
- Selects the permitted velocity source based on the current mode.
- Enforces configured linear and angular velocity limits.
- Monitors input command age and stops forwarding stale commands.
- Monitors GUI heartbeat; a heartbeat timeout switches the gate to STOP.
- Publishes zero velocity when STOP is active or valid authorized input is missing/stale.

### Default supervisor parameters

| Parameter | Default | Description |
|---|---:|---|
| Maximum linear velocity | `0.3 m/s` | Absolute linear velocity limit |
| Maximum angular velocity | `0.8 rad/s` | Absolute angular velocity limit |
| Command timeout | `0.35 s` | Maximum allowed age for a valid command/heartbeat |

These values are defaults from `config/supervisor.yaml` and can be adjusted for the robot platform. Verify safe limits on the target robot before operation.

The robot base controller should subscribe to `/cmd_vel`. The autonomous stack must remap its output to `/motiondock/cmd_vel_autonomous`; do not publish around the supervisor if mode exclusivity and its limits are required.

---

## Camera and Autonomous Launch Configuration

Use **Setup camera** to choose the camera workspace/folder and enter the commands required to source the ROS environment and launch the camera node or launch file.

Use **Set autonomy folder & command** to configure the autonomy workspace/folder and its launch command.

- Circle starts the configured camera process.
- Hold Circle for three seconds to stop the camera process.
- Selecting Autonomous starts the configured autonomy command.
- Selecting Manual or STOP stops the configured autonomy process.

Commands are launched in a shell so that environment-sourcing and launch-command chains can be used.

---

## Task Management

Use **Assign task** beside a controller control to configure its task.

Depending on the task type, MotionDock supports:
- Python source files stored in the workspace user-task area.
- C++ source files stored in `src/motiondock_user_tasks/tasks`.
- Workspace-based tasks configured with a ROS workspace, package and launch file.
- Custom shell commands with the required ROS/workspace sourcing and launch sequence.

Pressing the assigned controller button starts its task. Holding the same button for three seconds stops it. The application displays task assignment and status.

### Building C++ user tasks

C++ task sources in `src/motiondock_user_tasks/tasks` are discovered by the user-task package's CMake configuration and built as separate executables during the workspace build.

After adding or changing a C++ task, rebuild:

```bash
.venv/bin/python -m colcon build --symlink-install
source install/setup.bash
```

### Auto-Dock

Auto-Dock has a dedicated application control and configurable workspace/launch command. While Auto-Dock is running, hold its application button for three seconds to stop it.

**Security note:** User-configured task and launch commands execute on the host system. Only configure commands and source files you trust.

---

## Configuration

MotionDock reads YAML files from the workspace `config` directory.

| Configuration file | Purpose |
|---|---|
| `controller.yaml` | Controller button/axis mapping, R2 trigger settings, threshold, hysteresis, stick dead zone and manual velocity limits |
| `supervisor.yaml` | ROS topics, velocity limits and command timeout |
| `autonomy.yaml` | Autonomous workspace and launch configuration |
| `camera.yaml` | Camera launch configuration |
| `tasks.yaml` | User task registry |

The GUI also uses an interface configuration file when present. Configuration values can be adjusted to match a controller and robot platform.

### Current default control parameters

The checked-in controller defaults include:
- R2 trigger axis: index `5`, active when positive.
- R2 trigger threshold: `0.3`, with `0.08` hysteresis.
- Left-stick linear axis: index `1`.
- Left-stick angular axis: index `0`.
- Stick dead zone: `0.08`.
- Manual linear velocity limit: `0.3 m/s`.
- Manual angular velocity limit: `0.8 rad/s`.

Controller-specific index layouts may differ. Use the controller-learning and configuration tools to verify the correct inputs for your device.

---

## ROS 2 Monitoring

Inspect the ROS 2 graph while MotionDock is running:

List active nodes:

```bash
ros2 node list
```

List active topics:

```bash
ros2 topic list
```

Monitor final robot velocity:

```bash
ros2 topic echo /cmd_vel
```

Monitor manual velocity input:

```bash
ros2 topic echo /motiondock/cmd_vel_manual
```

Monitor autonomous velocity input:

```bash
ros2 topic echo /motiondock/cmd_vel_autonomous
```

Monitor the current operating mode:

```bash
ros2 topic echo /motiondock/mode
```

---

## Rebuilding After Code Changes

After modifying the source code, source the same ROS distribution used for the workspace, rebuild and re-source the install space.

For Humble:

```bash
source /opt/ros/humble/setup.bash
cd ~/MotionDock
.venv/bin/python -m colcon build --symlink-install
source install/setup.bash
```

For Jazzy, replace `humble` with `jazzy`.

---

## Safety Considerations

- Manual driving requires continuous R2 dead-man authorization.
- Controller disconnection revokes manual command forwarding.
- Mode selection determines which velocity-command source is authorized.
- Velocity limits, command timeouts and heartbeat monitoring are software safeguards.
- STOP commands zero velocity through the software supervisor; it is **not** a safety-rated emergency stop.
- Software safeguards do not replace independent hardware safety systems.

**Always implement and verify an independent hardware emergency-stop circuit before operating a physical robot.**

---

## Compatibility

| Component | Supported environment |
|---|---|
| ROS 2 | Humble, Jazzy |
| Operating system | Ubuntu-based ROS 2 environments |
| GUI framework | PySide6 |
| Controller input | PlayStation-compatible game controllers via SDL/pygame |
| Programming languages | Python and C++ |
| Build system | Colcon, ament and CMake |

---

## License

See the repository license and package metadata for licensing information.

## Contributing

Issues and contributions are welcome. Please open an issue to report bugs, request features or discuss improvements.

---

**Repository:** https://github.com/saitejavarma801/MotionDock
