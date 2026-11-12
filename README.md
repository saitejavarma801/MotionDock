# MotionDock

**ROS 2-Based Robotic Control, Supervision and Task Management Platform**

MotionDock is a modular, single-robot control and supervision platform built on ROS 2. It provides a PySide6 desktop interface for manual operation, autonomous navigation, controller input management, task execution and centralized robot command supervision.

**ROS 2 Compatibility:** Humble | Jazzy

---

## Key Features

### 1. Controller-Based Operation
- PlayStation controller integration.
- Real-time controller input monitoring.
- Configurable button and joystick mappings.
- Interactive controller input learning.

### 2. Operating Modes
- **Manual Mode:** Direct robot control using the PlayStation controller.
- **Autonomous Mode:** Allows configured autonomous navigation nodes to control the robot.
- **STOP Mode:** Blocks motion commands and sends zero velocity.

### 3. Safety and Command Supervision
- Centralized velocity command management.
- R2 dead-man switch for manual driving authorization.
- Controller disconnection detection.
- Command timeout and GUI heartbeat monitoring.
- Configurable linear and angular velocity limits.

### 4. Camera and Autonomous Navigation
- Configurable camera launch integration.
- Autonomous launch integration.
- Mode-based command switching through the supervisor.

### 5. Task Management
- Python and C++ task assignment.
- Controller-triggered task execution.
- Dynamic C++ task building.
- Dedicated Auto-Dock task execution.

### PlayStation Controller – Default Button Mapping

The following image illustrates the default controller assignments.

![MotionDock PlayStation Controller Mapping](docs/images/motiondock_controller_mapping.png)

| Controller Input | Default Function |
|---|---|
| Cross (X) | Select Manual Mode |
| Triangle (△) | Select Autonomous Mode |
| Square (□) | Select STOP Mode |
| Circle (○) | Camera control — press to start; hold for 3 seconds to stop |
| R2 | Manual driving authorization (dead-man switch) |
| Left Stick – Up/Down | Linear velocity control |
| Left Stick – Left/Right | Angular velocity control |

**Important controller behaviour:**
- Manual driving requires Manual Mode and continuous R2 authorization.
- Releasing R2 revokes manual driving permission.
- Disconnecting the controller stops manual command forwarding.
- After reconnecting, the operator must re-authorize manual driving.
- Controller mappings and input thresholds can be configured through the application.

---
<img width="1774" height="887" alt="image" src="https://github.com/user-attachments/assets/ec1a1fd8-f6c0-4f14-a9f9-6093de0d6244" />

## Download, Install Dependencies, Build and Launch

MotionDock supports ROS 2 Humble and ROS 2 Jazzy. Select the appropriate ROS distribution for your environment and use it consistently throughout the build and launch process.

### 1. Clone the Repository

```bash
git clone https://github.com/saitejavarma801/MotionDock.git
cd MotionDock
```

### 2. Create the Python Virtual Environment

Create a virtual environment with access to the system-installed ROS 2 Python packages:

```bash
sudo apt install -y python3-full python3-venv

python3 -m venv --system-site-packages .venv
```

### 3. Install Python Dependencies

```bash
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
```

### 4. Source the ROS 2 Environment

For ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

For ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

Use only the ROS distribution installed and selected for your workspace.

### 5. Install ROS Package Dependencies

```bash
sudo apt install -y python3-rosdep python3-colcon-common-extensions build-essential

rosdep install --from-paths src --ignore-src -r -y
```

### 6. Build the Workspace

Build using the Python interpreter from the virtual environment:

```bash
.venv/bin/python -m colcon build --symlink-install
```

Using the virtual environment's Python interpreter ensures that the installed ROS 2 Python executables use the environment containing the MotionDock dependencies.

### 7. Source the Workspace

```bash
source install/setup.bash
```

### 8. Launch MotionDock

```bash
ros2 run motiondock motiondock
```

The MotionDock GUI will start, and the application will initialize its command supervision functionality.

---

## Operating Modes

### Manual Mode
Manual control is enabled when:
1. A supported controller is connected.
2. Manual Mode is selected.
3. The R2 trigger is held above its configured threshold.

The left joystick controls linear and angular velocity. Releasing R2 removes manual driving authorization.

### Autonomous Mode
Autonomous nodes should publish velocity commands to:

```text
/motiondock/cmd_vel_autonomous
```

The supervisor forwards autonomous commands to the robot only when Autonomous Mode is active.

### STOP Mode
STOP Mode blocks manual and autonomous motion commands and commands zero velocity through the supervisor.

---

## Command Supervision

MotionDock uses a centralized supervisor to manage robot velocity commands.

| Interface | Default ROS 2 Topic | Purpose |
|---|---|---|
| Final velocity output | `/cmd_vel` | Robot velocity command |
| Manual input | `/motiondock/cmd_vel_manual` | Manual velocity commands |
| Autonomous input | `/motiondock/cmd_vel_autonomous` | Autonomous velocity commands |
| Operating mode | `/motiondock/mode` | Current operating mode |

The supervisor provides:
- Mode-based command selection.
- Linear and angular velocity limiting.
- Command timeout monitoring.
- GUI heartbeat monitoring.
- Zero-velocity output when command authorization or valid input is lost.

Robot-specific topic names and velocity limits can be adjusted through the configuration files.

The robot motor controller should subscribe to `/cmd_vel`. Autonomy launch commands must remap their velocity output to `/motiondock/cmd_vel_autonomous` so the supervisor can enforce mode exclusivity.

---

## Camera and Autonomous Launch Configuration

Use **Set autonomy folder & command** and **Setup camera** to choose a workspace/folder and enter the commands needed to source ROS and launch each system. Selecting Autonomous starts the configured autonomy command; selecting Manual or STOP stops it. Circle starts the camera, and holding Circle for three seconds stops it.

---

## Task Management

Use **Assign task** beside a controller control to choose its folder and enter its ROS/workspace sourcing and launch commands. Pressing that controller button starts the command; holding the same button for three seconds stops it. Each controller card displays its assigned folder and command.

### Auto-Dock
Auto-Dock uses the same folder-and-command setup. While it is running, hold the Auto-Dock app button for three seconds to stop it. Commands run in a shell to allow sourcing and launch command chains; only enter commands you trust.

**Security note:** User-entered commands execute on the host system. Only enter commands you trust.

---

## Configuration

MotionDock uses YAML configuration files to manage application behaviour.

| Configuration File | Purpose |
|---|---|
| `controller.yaml` | Controller mappings, trigger thresholds and joystick settings |
| `supervisor.yaml` | ROS topics, velocity limits and command timeout parameters |
| `autonomy.yaml` | Autonomous workspace and launch configuration |
| `camera.yaml` | Camera launch configuration |
| `tasks.yaml` | Assigned Python and C++ tasks |

Configuration values can be modified to adapt MotionDock to different robot platforms and operating requirements.

---

## ROS 2 Monitoring

The following commands can be used to inspect MotionDock while it is running.

List active nodes:

```bash
ros2 node list
```

List active topics:

```bash
ros2 topic list
```

Monitor the final velocity command:

```bash
ros2 topic echo /cmd_vel
```

Monitor the current operating mode:

```bash
ros2 topic echo /motiondock/mode
```

---

## Rebuilding After Code Changes

After modifying the source code, rebuild the workspace using the selected ROS distribution.

```bash
source /opt/ros/<your_ros_distribution>/setup.bash

cd ~/MotionDock

.venv/bin/python -m colcon build --symlink-install

source install/setup.bash
```

Replace `<your_ros_distribution>` with `humble` or `jazzy`.

---

## Safety Considerations

- Manual driving requires continuous R2 dead-man authorization.
- Controller disconnection must revoke manual driving permission.
- Operating-mode selection controls which command source is authorized.
- Velocity limits and command timeouts are configurable.
- STOP functionality commands zero velocity through the software supervisor.
- Software safeguards do not replace an independent hardware emergency-stop circuit.

**Always implement and verify an independent hardware emergency stop before operating the robot on physical hardware.**

---

## Compatibility

| Component | Supported Environment |
|---|---|
| ROS 2 | Humble |
| ROS 2 | Jazzy |
| Operating System | Ubuntu-based ROS 2 environments |
| GUI Framework | PySide6 |
| Programming Languages | Python and C++ |
| Build System | Colcon and CMake |

---
