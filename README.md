# MotionDock ROS 2 Workspace

MotionDock is a single-robot ROS 2 Humble workspace with a PySide6 PlayStation controller dashboard. The main screen shows controller input indicators, Manual/Autonomous mode controls, an R2 dead-man indicator, camera launch controls, task assignment, and Auto-Dock.

## Install and Launch

From the workspace root, with ROS 2 Humble installed:

```bash
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
colcon build --symlink-install
source install/setup.bash
ros2 run motiondock motiondock
```

For direct Python launch from the workspace root:

```bash
export PYTHONPATH="$PWD/src/motiondock:$PYTHONPATH"
python3 -m motiondock
```

The `PYTHONPATH` is needed because the Python module is inside the ROS package's `src/motiondock` directory. From inside that directory, `python3 -m motiondock` also works after installing the requirements.

## Controller Inputs and Tasks

MotionDock displays PlayStation labels and symbols, including Cross (×), Circle (○), Triangle (△), and Square (□).

Controller indices vary. Fill in the discovered button indices, D-pad hat index, R2 trigger axis, and left-stick X/Y axes on the main screen, then select **Save input mapping**. A green indicator appears beside a pressed control.

Use **Learn controller inputs** to map Cross (Manual), Triangle (Autonomous), Square (Stop), and Circle (Camera), followed by R2 and the left-stick forward/left axes. Follow each on-screen prompt; the wizard saves the discovered button and axis indices.

### Operating Modes

Manual and Autonomous are mutually exclusive modes, with an explicit Stop control.

- **Cross:** Select Manual.
- **Triangle:** Select Autonomous.
- **Square:** Select Stop.
- **Circle:** Control the camera.

Manual velocity requires Manual mode, a connected controller, R2 held above its configured threshold, and mapped left-stick axes. Stick speed scales from zero at the configured center deadzone to the editable maximum linear/angular speeds.

Releasing R2 or losing the controller sends zero velocity. Selecting Manual after another mode requires the operator to release and press R2 again.

The controller Stop button switches the supervisor to STOP and sends zero velocity. The GUI also shows STOP/MANUAL/AUTONOMOUS mode and the current manual Twist values.

A task assigned to the selected Manual or Autonomous mode control starts when entering that mode and stops when leaving it.

### Command Supervisor

The app starts a separate `motiondock_supervisor` ROS 2 process. It forwards only the active mode's configured source to the output topic, which defaults to `/cmd_vel`. It clamps configured linear/angular speed limits and sends zero when input or the GUI heartbeat times out.

Autonomous nodes must publish to the configured gated input topic, which defaults to `/motiondock/cmd_vel_autonomous`. Configure their launch files or remappings to use it.

Existing nodes must not publish directly to final `/cmd_vel`, or they bypass the gate.

### Task Assignment

Select **Assign task** beside a control to pick a Python or C++ source file. MotionDock copies it into this workspace and records the assignment in `config/tasks.yaml`.

A rising edge on that mapped controller input launches the assigned task. Python files run with the Python environment that launched MotionDock. C++ files are built as targets in the `motiondock_user_tasks` ROS 2 package, then launched with `ros2 run`.

On startup or controller reconnection, put all controls in their released position once before button assignments are armed.

### Camera and Autonomy

Configure a ROS 2 camera launch file using **Configure camera launch**.

- Circle starts the camera launch on a press.
- While the camera is running, hold Circle for three seconds to stop it.

Configure autonomy with its workspace and launch file. Selecting Autonomous starts that launch; selecting Manual stops it and blocks its velocity input.

### Auto-Dock

The **Auto-Dock** button asks for a `.py`, `.cpp`, `.cc`, or `.cxx` file, copies it into this workspace as the AutoDock task, and runs it.

Python scripts run directly. C++ source is built through the ROS 2 workspace. C++ tasks should be complete executable sources with their own `main`; use `rclcpp` when a task needs ROS 2.

Task files are executable code. Only assign files you trust. MotionDock starts them without a shell; user code must publish motion only to the configured supervisor input topic and must never publish directly to the final `/cmd_vel` output.

## Workspace Packages and Configuration

- `src/motiondock`: ROS 2 Python package containing the PySide6 dashboard, input monitor, task assignment, and launcher under `src/motiondock/motiondock`.
- `src/motiondock_user_tasks`: Dynamically built C++ task targets.
- `config/controller.yaml`: Controller mapping and R2 dead-man settings.
- `config/tasks.yaml`: Task file assignments.

## Safety Notes

The file-based Auto-Dock button asks for a Python or C++ file and runs the selected task. User-selected scripts are executable code; only assign files you trust. The task launcher does not sandbox user code.
