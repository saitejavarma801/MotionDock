# MotionDock development notes

- MotionDock manages one selected robot. Multiple ROS 2 workspaces and nodes for that robot are allowed.
- Keep robot-specific action names, message types, topics, limits, and mappings in configuration; never guess hardware interfaces.
- User-selected task files may run only after the operator explicitly assigns them and triggers the matching control (or selects a file through Auto-Dock). Copy task sources into this workspace and launch them without a shell.
- Never interpret a text field as executable code or launch an unassigned source file.
- Keep base commands behind the central supervisor and one authorized output gate. Manual driving requires manual mode and an active R2 dead-man input.
- Controller disconnect must revoke drive permission and stop motion; reconnect must not resume motion automatically.
- E-stop must latch, block every motion source, and require an authorized reset. GUI E-stop is not a substitute for independent hardware protection.
- Docking uses explicit task states and configured ROS interfaces. Report docked only when the robot's configured confirmation behavior verifies docking.
- Keep changes incremental. The first control milestone is controller monitoring, R2 trigger detection, and a validated configuration model.
- Do not claim hardware behavior has been verified without testing on the configured robot.
