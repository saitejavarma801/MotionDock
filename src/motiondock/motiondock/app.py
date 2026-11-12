"""Single-page PlayStation controller dashboard and user task launcher."""

from __future__ import annotations

import sys
import os
import signal
import time
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from PySide6.QtCore import QProcess, QTimer, Qt
from PySide6.QtGui import QIntValidator
from PySide6.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QInputDialog,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QLineEdit,
    QScrollArea,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

from motiondock.controller_monitor import ControllerMonitor
from motiondock.ros_bridge import MotionDockRosBridge
from motiondock.tasks import TaskManager


def find_workspace() -> Path:
    def is_workspace(path: Path) -> bool:
        return (
            (path / "src" / "motiondock" / "package.xml").is_file()
            and (path / "config" / "controller.yaml").is_file()
        )

    roots: list[Path] = []
    explicit = os.environ.get("MOTIONDOCK_WORKSPACE", "").strip()
    if explicit:
        roots.append(Path(explicit).expanduser())
    roots.append(Path.cwd())
    roots.extend(Path(prefix) for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if prefix)
    roots.extend((Path(__file__).absolute(), Path(__file__).resolve()))

    checked: set[Path] = set()
    for root in roots:
        try:
            root = root.resolve()
        except OSError:
            continue
        for candidate in (root, *root.parents):
            if candidate in checked:
                continue
            checked.add(candidate)
            if is_workspace(candidate):
                return candidate
    raise FileNotFoundError(
        "Could not locate the MotionDock workspace. Source its install/setup.bash, "
        "launch from the workspace root, or set MOTIONDOCK_WORKSPACE to the workspace path."
    )


WORKSPACE = find_workspace()
CONFIG_PATH = WORKSPACE / "config" / "controller.yaml"
TASKS = TaskManager(WORKSPACE)

CONTROLS = [
    ("l2", "L2", "L₂", "button"),
    ("l1", "L1", "L₁", "button"),
    ("r1", "R1", "R₁", "button"),
    ("r2", "R2", "R₂", "button"),
    ("dpad_up", "D-pad Up", "↑", "hat"),
    ("dpad_down", "D-pad Down", "↓", "hat"),
    ("dpad_left", "D-pad Left", "←", "hat"),
    ("dpad_right", "D-pad Right", "→", "hat"),
    ("triangle", "Triangle", "△", "button"),
    ("square", "Square", "□", "button"),
    ("circle", "Circle", "○", "button"),
    ("cross", "Cross", "×", "button"),
    ("create", "Create / Share", "⇧", "button"),
    ("options", "Options", "☰", "button"),
    ("ps", "PS button", "✧", "button"),
]
CONTROL_POSITIONS = {
    "l2": (0, 0), "l1": (0, 1), "r1": (0, 3), "r2": (0, 4),
    "dpad_up": (1, 0), "dpad_left": (2, 0), "dpad_right": (2, 1), "dpad_down": (3, 0),
    "triangle": (1, 4), "square": (2, 3), "circle": (2, 4), "cross": (3, 4),
    "create": (1, 2), "options": (2, 2), "ps": (3, 2),
}
SUPERVISOR_CONFIG = WORKSPACE / "config" / "supervisor.yaml"
CAMERA_CONFIG = WORKSPACE / "config" / "camera.yaml"
AUTONOMY_CONFIG = WORKSPACE / "config" / "autonomy.yaml"
UI_CONFIG = WORKSPACE / "config" / "interface.yaml"


class IndexInput(QLineEdit):
    """Plain, legible numeric field for a controller index."""

    def __init__(self, max_value: int = 16, parent=None) -> None:
        super().__init__(parent)
        self.max_value = max_value
        self.setValidator(QIntValidator(0, max_value, self))
        self.setMaxLength(2)
        self.setPlaceholderText("index")

    def value(self) -> int:
        try:
            return int(self.text())
        except ValueError:
            return -1

    def setValue(self, value: int) -> None:
        if isinstance(value, int) and 0 <= value <= self.max_value:
            self.setText(str(value))
        else:
            self.clear()


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("MotionDock")
        self.setMinimumSize(760, 520)
        self._previous_inputs: dict[str, bool] = {}
        self._input_armed = False
        self._processes: dict[str, QProcess] = {}
        self._stopping_process_ids: set[int] = set()
        self._control_cards: list[QFrame] = []
        self._input_config: dict = {}
        self._mapping_spins: dict[str, IndexInput] = {}
        self._trigger_spins: dict[str, IndexInput] = {}
        self._drive_axis_spins: dict[str, IndexInput] = {}
        self._max_speed_spins: dict[str, QDoubleSpinBox] = {}
        self._trigger_leds: dict[str, QLabel] = {}
        self._status_labels: dict[str, QLabel] = {}
        self._shown_inputs: dict[str, bool] = {}
        self._task_labels: dict[str, QLabel] = {}
        self._task_hold_started: dict[str, float] = {}
        self._task_stop_triggered: set[str] = set()
        self._axis_active: dict[str, bool] = {}
        self._led_labels: dict[str, QLabel] = {}
        self.mode = "stop"
        self._manual_release_seen = False
        self._camera_hold_started: float | None = None
        self._camera_stop_triggered = False
        self._controller_connected = False
        self._last_snapshot = None
        self._last_telemetry_text_update = 0.0
        self._learning_active = False
        self._learning_step = ""
        self._learning_button_steps: list[tuple[str, str]] = []
        self._learning_button_position = 0
        self._learning_previous_buttons: tuple[bool, ...] = ()
        self._learning_previous_hats: tuple[tuple[int, int], ...] = ()
        self._learning_axis_baseline: tuple[float, ...] = ()
        self._learning_release_index: int | None = None
        self._learning_release_value = 0.0
        self._learning_release_count = 0
        self.camera_process: QProcess | None = None
        self.autonomy_process: QProcess | None = None
        self.supervisor_process: QProcess | None = None
        self._set_theme()
        self._ui_settings = self._load_ui_settings()

        base = QWidget()
        base_layout = QVBoxLayout(base)
        base_layout.setContentsMargins(12, 8, 12, 8)
        base_layout.setSpacing(5)

        header = QHBoxLayout()
        brand = QVBoxLayout()
        title = QLabel("MotionDock")
        title.setObjectName("title")
        subtitle = QLabel("Single robot control · PlayStation controller")
        subtitle.setObjectName("subtitle")
        brand.addWidget(title)
        brand.addWidget(subtitle)
        header.addLayout(brand)
        header.addStretch(1)
        self.connection_badge = QLabel("●  SEARCHING")
        self.connection_badge.setObjectName("badge")
        header.addWidget(self.connection_badge)
        self.mode_badge = QLabel("MODE · STOPPED")
        self.mode_badge.setObjectName("modeBadge")
        header.addWidget(self.mode_badge)
        self.manual_mode_button = QPushButton("Manual")
        self.manual_mode_button.clicked.connect(lambda: self.set_mode("manual"))
        header.addWidget(self.manual_mode_button)
        self.autonomous_mode_button = QPushButton("Autonomous")
        self.autonomous_mode_button.clicked.connect(lambda: self.set_mode("autonomous"))
        header.addWidget(self.autonomous_mode_button)
        self.stop_mode_button = QPushButton("Stop")
        self.stop_mode_button.setObjectName("stop")
        self.stop_mode_button.clicked.connect(lambda: self.set_mode("stop"))
        header.addWidget(self.stop_mode_button)
        self.autodock_button = QPushButton("⚡  Auto-Dock")
        self.autodock_button.setObjectName("primary")
        self._autodock_was_running_on_press = False
        self._autodock_hold_started: float | None = None
        self._autodock_hold_timer = QTimer(self)
        self._autodock_hold_timer.setInterval(100)
        self._autodock_hold_timer.timeout.connect(self._update_autodock_hold)
        self.autodock_button.pressed.connect(self._autodock_pressed)
        self.autodock_button.released.connect(self._autodock_released)
        header.addWidget(self.autodock_button)
        base_layout.addLayout(header)

        self.activity_banner = QLabel("●  Ready · choose Manual or Autonomous")
        self.activity_banner.setObjectName("activityBanner")
        base_layout.addWidget(self.activity_banner)

        config_row = QVBoxLayout()
        autonomy_launch_row = QHBoxLayout()
        self.configure_autonomy_button = QPushButton("Set autonomy folder & command")
        self.configure_autonomy_button.clicked.connect(self.configure_autonomy_launch)
        autonomy_launch_row.addWidget(self.configure_autonomy_button)
        self.autonomy_status = QLabel("Autonomy launch not configured")
        self.autonomy_status.setObjectName("taskState")
        autonomy_launch_row.addWidget(self.autonomy_status)
        autonomy_launch_row.addStretch(1)
        config_row.addLayout(autonomy_launch_row)
        camera_config_row = QHBoxLayout()
        self.camera_status = QLabel("Camera not configured")
        self.camera_status.setObjectName("taskState")
        camera_config_row.addWidget(self.camera_status)
        camera_config_row.addWidget(QLabel("Press Circle to start · hold 3 seconds to stop"))
        config_row.addLayout(camera_config_row)
        base_layout.addLayout(config_row)

        mode_map_row = QHBoxLayout()
        mode_map_row.addWidget(QLabel("× Cross → Manual"))
        mode_map_row.addWidget(QLabel("△ Triangle → Autonomous"))
        mode_map_row.addWidget(QLabel("□ Square → Stop"))
        mode_map_row.addWidget(QLabel("○ Circle → Camera"))
        mode_map_row.addStretch(1)
        self.learn_controller_button = QPushButton("Learn controller inputs")
        self.learn_controller_button.clicked.connect(self.start_controller_learning)
        mode_map_row.addWidget(self.learn_controller_button)
        mode_map_row.addStretch(1)
        base_layout.addLayout(mode_map_row)

        info = QLabel(
            "Manual drive needs Manual mode + held R2 + left-stick movement. "
            "Use Learn controller inputs or set indices below; green lights show active inputs. "
            "The supervisor sends velocity to the configured motor topic."
        )
        info.setObjectName("hint")
        base_layout.addWidget(info)

        self.axis_card = QFrame()
        self.axis_card.setObjectName("panel")
        axis_layout = QVBoxLayout(self.axis_card)
        self.axis_heading = QLabel("JOYSTICKS & TRIGGERS")
        self.axis_heading.setObjectName("section")
        axis_layout.addWidget(self.axis_heading)
        self.axis_values = QLabel("Waiting for controller input…")
        self.axis_values.setObjectName("axisValues")
        axis_layout.addWidget(self.axis_values)
        self._raw_button_labels: list[QLabel] = []
        for start in (0, 7):
            button_row = QHBoxLayout()
            button_row.setSpacing(9)
            for index in range(start, start + 7):
                button_label = QLabel(f"BUTTON {index + 1} [index {index}]  ○")
                button_label.setObjectName("rawButton")
                button_label.setMinimumWidth(120)
                button_label.setProperty("pressed", False)
                button_label.setStyleSheet("color: #4a5568;")
                self._raw_button_labels.append(button_label)
                button_row.addWidget(button_label)
            button_row.addStretch(1)
            axis_layout.addLayout(button_row)
        self.dpad_input_hint = QLabel("D-pad: press a direction to see its active button index above; assign that index on the matching D-pad card.")
        self.dpad_input_hint.setObjectName("hint")
        axis_layout.addWidget(self.dpad_input_hint)
        self.deadman_label = QLabel("R2 dead-man: not configured")
        self.deadman_label.setObjectName("deadman")
        axis_layout.addWidget(self.deadman_label)
        trigger_row = QHBoxLayout()
        for key, title in (("l2", "L2 axis"), ("r2", "R2 axis")):
            trigger_row.addWidget(QLabel(title))
            spin = IndexInput()
            self._configure_index_spin(spin)
            self._trigger_spins[key] = spin
            trigger_row.addWidget(spin)
            led = QLabel("●")
            led.setObjectName("led")
            self._trigger_leds[key] = led
            trigger_row.addWidget(led)
        for key, title in (("linear_axis", "Left stick Y"), ("angular_axis", "Left stick X")):
            trigger_row.addWidget(QLabel(title))
            spin = IndexInput()
            self._configure_index_spin(spin)
            self._drive_axis_spins[key] = spin
            trigger_row.addWidget(spin)
        trigger_row.addStretch(1)
        axis_layout.addLayout(trigger_row)
        max_speed_row = QHBoxLayout()
        for key, title, maximum in (
            ("max_linear_velocity", "Max linear m/s", 5.0),
            ("max_angular_velocity", "Max angular rad/s", 10.0),
        ):
            max_speed_row.addWidget(QLabel(title))
            spin = QDoubleSpinBox()
            spin.setRange(0.01, maximum)
            spin.setDecimals(2)
            spin.setSingleStep(0.05)
            self._max_speed_spins[key] = spin
            max_speed_row.addWidget(spin)
        self.manual_velocity_label = QLabel("Manual cmd_vel: linear 0.00 · angular 0.00")
        self.manual_velocity_label.setObjectName("axisValues")
        max_speed_row.addWidget(self.manual_velocity_label)
        max_speed_row.addStretch(1)
        axis_layout.addLayout(max_speed_row)
        base_layout.addWidget(self.axis_card)

        controls_heading = QHBoxLayout()
        controls_title = QLabel("CONTROLLER BUTTONS")
        controls_title.setObjectName("section")
        controls_heading.addWidget(controls_title)
        controls_heading.addStretch(1)
        self.save_mapping_button = QPushButton("Save input mapping")
        self.save_mapping_button.clicked.connect(self.save_input_mapping)
        controls_heading.addWidget(self.save_mapping_button)
        base_layout.addLayout(controls_heading)

        display_row = QHBoxLayout()
        display_row.addWidget(QLabel("Card size"))
        self.card_scale_input = QSpinBox()
        self.card_scale_input.setRange(80, 150)
        self.card_scale_input.setSuffix(" %")
        self.card_scale_input.setValue(int(self._ui_settings.get("card_scale", 100)))
        display_row.addWidget(self.card_scale_input)
        display_row.addWidget(QLabel("Button text"))
        self.card_font_input = QSpinBox()
        self.card_font_input.setRange(9, 18)
        self.card_font_input.setSuffix(" pt")
        self.card_font_input.setValue(int(self._ui_settings.get("card_font", 11)))
        display_row.addWidget(self.card_font_input)
        display_row.addWidget(QLabel("Terminal height"))
        self.terminal_height_input = QSpinBox()
        self.terminal_height_input.setRange(70, 360)
        self.terminal_height_input.setSuffix(" px")
        self.terminal_height_input.setValue(int(self._ui_settings.get("terminal_height", 100)))
        display_row.addWidget(self.terminal_height_input)
        display_row.addWidget(QLabel("Terminal text"))
        self.terminal_font_input = QSpinBox()
        self.terminal_font_input.setRange(8, 18)
        self.terminal_font_input.setSuffix(" pt")
        self.terminal_font_input.setValue(int(self._ui_settings.get("terminal_font", 10)))
        display_row.addWidget(self.terminal_font_input)
        display_row.addStretch(1)
        base_layout.addLayout(display_row)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setMinimumHeight(120)
        controls_root = QWidget()
        grid = QGridLayout(controls_root)
        grid.setContentsMargins(2, 2, 2, 2)
        grid.setHorizontalSpacing(7)
        grid.setVerticalSpacing(7)
        for index, (key, name, glyph, kind) in enumerate(CONTROLS):
            card = self._control_card(key, name, glyph, kind)
            self._control_cards.append(card)
            grid.addWidget(card, index // 5, index % 5)
        for column in range(5):
            grid.setColumnStretch(column, 1)
        scroll.setWidget(controls_root)
        base_layout.addWidget(scroll, 1)

        self.activity_log = QTextEdit()
        self.activity_log.setReadOnly(True)
        self.activity_log.setMinimumHeight(70)
        self.activity_log.setMaximumHeight(360)
        self.activity_log.setPlaceholderText("Task output and status")
        base_layout.addWidget(self.activity_log)
        self.setCentralWidget(base)
        for setting in (self.card_scale_input, self.card_font_input, self.terminal_height_input, self.terminal_font_input):
            setting.valueChanged.connect(self._apply_display_settings)
        self._apply_display_settings()

        self.monitor = ControllerMonitor(CONFIG_PATH)
        self._load_input_mapping()
        self.bridge = MotionDockRosBridge()
        self._load_launch_settings()
        self._start_supervisor()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_controller)
        self.timer.start(50)
        self._log(f"Workspace: {WORKSPACE}")

    def _set_theme(self) -> None:
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #0a101c; color: #e8eef8; font-family: 'Inter', 'Ubuntu', sans-serif; font-size: 13px; }
            QLabel#title { color: #f7f9fd; font-size: 28px; font-weight: 750; }
            QLabel#subtitle, QLabel#hint { color: #91a1b9; }
            QLabel#hint { padding: 5px 0 8px; }
            QLabel#badge { color: #f3c977; background: #302817; border: 1px solid #554326; border-radius: 12px; padding: 8px 12px; font-weight: 700; }
            QLabel#modeBadge { color: #a9c8f0; background: #17263a; border: 1px solid #2d4666; border-radius: 10px; padding: 8px 10px; font-weight: 700; }
            QLabel#activityBanner { color: #67e5bc; background: #102c28; border: 1px solid #28644f; border-radius: 8px; padding: 8px 11px; font-weight: 650; }
            QLabel#section { color: #8fa8ca; font-size: 11px; font-weight: 750; letter-spacing: 1px; }
            QFrame#panel, QFrame#control { background: #111b2b; border: 1px solid #23334b; border-radius: 12px; }
            QFrame#control:hover { border: 1px solid #3d6799; background: #142239; }
            QLabel#glyph { color: #8bc7ff; font-size: 23px; font-weight: 700; min-width: 35px; }
            QLabel#led { font-size: 16px; min-width: 20px; }
            QLabel#rawButton { font-family: monospace; padding: 3px 2px; }
            QLabel#controlName { color: #ecf2fb; font-size: 14px; font-weight: 650; }
            QLabel#inputState, QLabel#taskState { color: #8fa3bf; font-size: 11px; }
            QLabel#pressed { color: #53e2b2; font-size: 11px; font-weight: 700; }
            QLabel#axisValues { color: #b8cae3; font-family: monospace; padding: 4px; }
            QLabel#deadman { color: #f0bc63; font-weight: 650; padding-top: 4px; }
            QLineEdit#indexInput { color: #f4f7fc; background: #0b1422; border: 1px solid #496586; border-radius: 6px; padding: 5px 7px; min-width: 64px; }
            QLineEdit#indexInput:focus { border: 1px solid #54a5f5; background: #111e30; }
            QDoubleSpinBox { background: #0b1422; border: 1px solid #2a3d58; border-radius: 6px; padding: 5px; min-width: 60px; }
            QLineEdit { background: #0b1422; border: 1px solid #2a3d58; border-radius: 6px; padding: 7px; }
            QScrollArea { border: 0; background: transparent; }
            QTextEdit { background: #0b1422; border: 1px solid #22334a; border-radius: 9px; padding: 7px; color: #bfd0e7; font-family: monospace; }
            QPushButton { color: #dce8f8; background: #1a2a40; border: 1px solid #324966; border-radius: 8px; padding: 8px 12px; font-weight: 600; }
            QPushButton:hover { background: #263b57; border-color: #4d6e96; }
            QPushButton#primary { color: white; background: #1678e7; border: 1px solid #4094fa; padding: 11px 20px; font-size: 14px; }
            QPushButton#primary:hover { background: #278bfa; }
            QPushButton#stop { color: #ffe4e4; background: #672d38; border-color: #a84b59; }
            QPushButton#stop:hover { background: #813b48; }
            QDialog, QMessageBox, QInputDialog, QFileDialog { background: #0d1624; color: #e6edf7; }
            QDialog QLabel, QMessageBox QLabel, QInputDialog QLabel, QFileDialog QLabel { color: #dce8f8; }
            QDialog QLineEdit, QDialog QTextEdit, QDialog QPlainTextEdit, QDialog QComboBox,
            QFileDialog QLineEdit, QFileDialog QComboBox { color: #f4f7fc; background: #101d2d; border: 1px solid #344b69; border-radius: 7px; padding: 7px; selection-background-color: #1678e7; }
            QDialog QTextEdit, QDialog QPlainTextEdit { min-width: 420px; min-height: 110px; }
            QDialog QPushButton, QMessageBox QPushButton, QFileDialog QPushButton { color: #e5efff; background: #1a2a40; border: 1px solid #3b5879; border-radius: 8px; padding: 8px 14px; min-width: 72px; font-weight: 650; }
            QDialog QPushButton:hover, QMessageBox QPushButton:hover, QFileDialog QPushButton:hover { background: #274363; border-color: #65aaf4; }
            QDialog QDialogButtonBox, QMessageBox QDialogButtonBox { button-layout: 0; }
            QFileDialog QListView, QFileDialog QTreeView { color: #dce8f8; background: #0b1422; alternate-background-color: #121f30; border: 1px solid #2a3d58; selection-background-color: #1e5da0; }
            QFileDialog QHeaderView::section { color: #dce8f8; background: #162439; border: 0; padding: 6px; }
            QComboBox QAbstractItemView { color: #e6edf7; background: #101d2d; selection-background-color: #1e5da0; border: 1px solid #344b69; }
            QScrollBar:vertical { background: #0b1422; width: 12px; margin: 2px; }
            QScrollBar::handle:vertical { background: #344b69; border-radius: 5px; min-height: 26px; }
        """)
        app = QApplication.instance()
        if app:
            app.setStyleSheet(self.styleSheet())

    @staticmethod
    def _configure_index_spin(spin: IndexInput) -> None:
        """Make index fields clearly editable by typing a controller index."""
        spin.setObjectName("indexInput")
        spin.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        spin.setMinimumWidth(84)
        spin.setToolTip("Click here and type the controller index number. Use Learn controller inputs to identify it automatically.")

    def _control_card(self, key: str, name: str, glyph: str, kind: str) -> QFrame:
        card = QFrame()
        card.setObjectName("control")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(12, 10, 12, 10)
        top = QHBoxLayout()
        icon = QLabel(glyph)
        icon.setObjectName("glyph")
        top.addWidget(icon)
        name_label = QLabel(name)
        name_label.setObjectName("controlName")
        top.addWidget(name_label, 1)
        led = QLabel("●")
        led.setObjectName("led")
        self._led_labels[key] = led
        top.addWidget(led)
        layout.addLayout(top)

        state = QLabel("waiting")
        state.setObjectName("inputState")
        self._status_labels[key] = state
        layout.addWidget(state)

        row = QHBoxLayout()
        if kind == "button":
            row.addWidget(QLabel("Index"))
            spin = IndexInput(max_value=13)
            self._configure_index_spin(spin)
            self._mapping_spins[key] = spin
            row.addWidget(spin)
        else:
            row.addWidget(QLabel("Button index"))
            spin = IndexInput(max_value=13)
            self._configure_index_spin(spin)
            self._mapping_spins[key] = spin
            row.addWidget(spin)
        assign = QPushButton("Setup camera" if key == "circle" else "Assign task")
        if key == "circle":
            assign.clicked.connect(self.configure_camera_launch)
        else:
            assign.clicked.connect(lambda _checked=False, control=key: self.assign_task(control))
        row.addWidget(assign)
        layout.addLayout(row)
        task_label = QLabel("No task assigned")
        task_label.setObjectName("taskState")
        task_label.setWordWrap(True)
        self._task_labels[key] = task_label
        layout.addWidget(task_label)
        return card

    @staticmethod
    def _task_assignment_text(folder: str, command: str) -> str:
        return f"Folder: {folder}\n$ {command}"

    def _show_command_assignment(self, key: str, folder: str, command: str) -> None:
        label = self._task_labels[key]
        label.setText(self._task_assignment_text(folder, command))
        label.setToolTip(f"Folder: {folder}\nCommand:\n{command}\n\nPress to launch · hold the assigned controller button for 3 seconds to stop.")

    @staticmethod
    def _load_ui_settings() -> dict:
        try:
            data = yaml.safe_load(UI_CONFIG.read_text(encoding="utf-8")) or {}
            return data if isinstance(data, dict) else {}
        except (OSError, yaml.YAMLError):
            return {}

    def _apply_display_settings(self) -> None:
        scale = self.card_scale_input.value() / 100.0
        font_pt = self.card_font_input.value()
        for card in self._control_cards:
            card.setMinimumHeight(int(118 * scale))
            card.setStyleSheet(
                f"QLabel#glyph {{ font-size: {23 * scale:.1f}px; }} "
                f"QLabel#controlName {{ font-size: {font_pt + 2}px; }} "
                f"QLabel#taskState, QLabel#inputState {{ font-size: {font_pt}px; }} "
                f"QPushButton {{ font-size: {font_pt}px; padding: {max(5, int(8 * scale))}px; }}"
            )
        height = self.terminal_height_input.value()
        self.activity_log.setMaximumHeight(height)
        self.activity_log.setMinimumHeight(min(70, height))
        terminal_font = self.activity_log.font()
        terminal_font.setPointSize(self.terminal_font_input.value())
        self.activity_log.setFont(terminal_font)
        self._ui_settings.update({
            "card_scale": self.card_scale_input.value(),
            "card_font": font_pt,
            "terminal_height": height,
            "terminal_font": self.terminal_font_input.value(),
        })
        try:
            UI_CONFIG.parent.mkdir(parents=True, exist_ok=True)
            UI_CONFIG.write_text(yaml.safe_dump(self._ui_settings, sort_keys=True), encoding="utf-8")
        except OSError as exc:
            self._log(f"Could not save display settings: {exc}")

    def _log(self, message: str) -> None:
        self.activity_log.append(message)

    def _set_banner(self, message: str) -> None:
        self.activity_banner.setText(message)

    def show_button_feedback(self, key: str, name: str) -> None:
        if key in self._led_labels:
            self._set_led(self._led_labels[key], True)
            QTimer.singleShot(350, lambda control=key: self._reset_led(control))
        self._set_banner(f"🟢  {name} pressed")
        QTimer.singleShot(1400, lambda: self._set_banner("●  Ready · controller active")
                          if self.activity_banner.text().endswith("pressed") else None)

    def _reset_led(self, key: str) -> None:
        if key in self._led_labels and not self._shown_inputs.get(key, False):
            self._set_led(self._led_labels[key], False)

    @staticmethod
    def _set_led(label: QLabel, active: bool) -> None:
        active = bool(active)
        if label.property("active") == active:
            return
        label.setProperty("active", active)
        color = "#35e58c" if active else "#4a5568"
        label.setText("●")
        label.setStyleSheet(f"color: {color}; font-size: 17px;")

    @staticmethod
    def _set_raw_button(label: QLabel, index: int, active: bool) -> None:
        active = bool(active)
        if label.property("pressed") == active:
            return
        label.setProperty("pressed", active)
        marker = "●" if active else "○"
        color = "#35e58c" if active else "#4a5568"
        label.setText(f"BUTTON {index + 1} [index {index}]  {marker}")
        label.setStyleSheet(f"color: {color}; font-family: monospace; padding: 3px 2px;")

    def set_mode(self, mode: str) -> None:
        if mode not in ("manual", "autonomous", "stop"):
            return
        if self.mode == mode:
            if mode == "stop":
                self.bridge.publish("stop", 0.0, 0.0)
                self._set_banner("Stop command sent · velocity output is zero")
            return
        previous = self.mode
        mode_buttons = {"manual": "cross", "autonomous": "triangle", "stop": "square"}
        if previous == "manual":
            self._stop_assigned_task(mode_buttons.get("manual", "cross"))
        elif previous == "autonomous":
            self._stop_assigned_task(mode_buttons.get("autonomous", "triangle"))
            self._stop_launch_process("autonomy")
        self.mode = mode
        self._manual_release_seen = False
        self.mode_badge.setText(f"MODE · {mode.upper()}")
        self.manual_mode_button.setText("● Manual active" if mode == "manual" else "Manual")
        self.autonomous_mode_button.setText("● Autonomous active" if mode == "autonomous" else "Autonomous")
        self.bridge.publish(mode, 0.0, 0.0)
        self._log(f"Mode changed to {mode}; previous velocity source stopped.")
        self._set_banner("Stop command sent · velocity output is zero" if mode == "stop" else f"Mode: {mode.title()} · inactive source blocked")
        if mode == "manual":
            self._run_mode_task(mode_buttons.get("manual", "cross"))
        if mode == "autonomous":
            config = self._load_launch_file(AUTONOMY_CONFIG)
            if config and config.get("workspace") and config.get("command"):
                self._start_launch_process("autonomy", config)
            else:
                self._log("Autonomous mode selected, but no folder and launch command are configured.")
            self._run_mode_task(mode_buttons.get("autonomous", "triangle"))

    def _run_mode_task(self, key: str) -> None:
        if key and (TASKS.assigned_path(key) or TASKS.assigned_workspace(key) or TASKS.assigned_command(key)):
            self.run_task(key)

    def _stop_assigned_task(self, key: str) -> None:
        process = self._processes.get(key)
        if process and process.state() != QProcess.ProcessState.NotRunning:
            self._stopping_process_ids.add(id(process))
            self._stop_process_group(process)

    def _load_launch_file(self, path: Path) -> dict:
        try:
            value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            return value if isinstance(value, dict) else {}
        except (OSError, yaml.YAMLError):
            return {}

    def _load_launch_settings(self) -> None:
        supervisor = self._load_launch_file(SUPERVISOR_CONFIG)
        params = supervisor.get("motiondock_supervisor", {}).get("ros__parameters", {})
        autonomy = self._load_launch_file(AUTONOMY_CONFIG)
        if autonomy.get("command"):
            self.autonomy_status.setText(f"{autonomy.get('workspace')} · {autonomy.get('command')}")
        camera = self._load_launch_file(CAMERA_CONFIG)
        if camera.get("command"):
            self.camera_status.setText(f"{camera.get('workspace')} · {camera.get('command')}")

    def _configure_launch(self, destination: Path, title: str) -> dict | None:
        selection = self._choose_folder_command(
            title,
            "Commands to run in the selected folder (include ROS/workspace sourcing, then launch):",
            self._load_launch_file(destination).get("command", ""),
        )
        if not selection:
            return None
        workspace, command = selection
        config = {"workspace": str(workspace), "command": command}
        try:
            old = self._load_launch_file(destination)
            old.update(config)
            destination.write_text(yaml.safe_dump(old, sort_keys=False), encoding="utf-8")
        except OSError as exc:
            QMessageBox.critical(self, "Could not save setup", str(exc))
            return None
        return config

    def _choose_folder_command(self, title: str, prompt: str, previous: str = "") -> tuple[Path, str] | None:
        folder_text = QFileDialog.getExistingDirectory(
            self,
            f"Choose {title} folder or workspace",
            str(Path.home()),
            QFileDialog.Option.DontUseNativeDialog,
        )
        if not folder_text:
            return None
        command, accepted = QInputDialog.getMultiLineText(
            self,
            f"Launch {title}",
            prompt,
            previous,
        )
        command = command.strip()
        if not accepted or not command:
            return None
        return Path(folder_text).resolve(), command

    def configure_autonomy_launch(self) -> None:
        config = self._configure_launch(AUTONOMY_CONFIG, "autonomy")
        if config:
            self.autonomy_status.setText(f"{config['workspace']} · {config['command']}")
            self._log(f"Autonomy command configured in {config['workspace']}.")

    def configure_camera_launch(self) -> None:
        config = self._configure_launch(CAMERA_CONFIG, "camera")
        if config:
            self.camera_status.setText(f"{config['workspace']} · {config['command']}")
            self._log(f"Camera command configured in {config['workspace']}.")

    def _start_supervisor(self) -> None:
        process = QProcess(self)
        process.setProgram("ros2")
        process.setArguments([
            "run", "motiondock", "motiondock_supervisor", "--ros-args", "--params-file", str(SUPERVISOR_CONFIG)
        ])
        process.setWorkingDirectory(str(WORKSPACE))
        process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        process.readyReadStandardOutput.connect(lambda p=process: self._read_task_output("supervisor", p))
        process.errorOccurred.connect(lambda error: self._log(f"ROS supervisor could not start: {error}"))
        self.supervisor_process = process
        process.start()

    def _restart_supervisor(self) -> None:
        process = self.supervisor_process
        if process and process.state() != QProcess.ProcessState.NotRunning:
            process.terminate()
            if not process.waitForFinished(1200):
                process.kill()
        self._start_supervisor()

    def _start_launch_process(self, key: str, config: dict) -> None:
        process_attr = "camera_process" if key == "camera" else "autonomy_process"
        current = getattr(self, process_attr)
        if current and current.state() != QProcess.ProcessState.NotRunning:
            return
        workspace = str(config.get("workspace", ""))
        command = str(config.get("command", "")).strip()
        if not workspace or not Path(workspace).is_dir() or not command:
            self._log(f"{key.title()} needs a valid folder and launch command.")
            return
        arguments = ["bash", "-lc", f"exec bash -lc {self._shell_quote(command)}", f"motiondock-{key}"]
        working_directory = workspace
        process = QProcess(self)
        process.setProgram("setsid")
        process.setArguments(arguments)
        process.setWorkingDirectory(working_directory)
        process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        process.readyReadStandardOutput.connect(lambda p=process, k=key: self._read_task_output(k, p))
        process.finished.connect(lambda _code, _status, k=key: self._launch_finished(k))
        process.errorOccurred.connect(lambda error, k=key: self._log(f"{k.title()} launch failed: {error}"))
        setattr(self, process_attr, process)
        process.start()
        if key == "camera":
            self.camera_status.setText("Starting camera… · hold Circle 3s to stop")
        self._log(f"Starting {key} command in {workspace}: {command}")

    @staticmethod
    def _shell_quote(value: str) -> str:
        return "'" + value.replace("'", "'\\''") + "'"

    def _stop_launch_process(self, key: str) -> None:
        process = self.camera_process if key == "camera" else self.autonomy_process
        if not process or process.state() == QProcess.ProcessState.NotRunning:
            return
        self._stop_process_group(process, signal.SIGINT)
        if key == "camera":
            self.camera_status.setText("Stopping camera…")
        self._log(f"Stopping {key} launch.")

    @staticmethod
    def _stop_process_group(process: QProcess, sig: int = signal.SIGTERM) -> None:
        """Stop the launch command and its ROS child processes as one session."""
        try:
            pid = int(process.processId())
            if pid > 0:
                os.killpg(pid, sig)
                if not process.waitForFinished(1200):
                    os.killpg(pid, signal.SIGKILL)
                    process.waitForFinished(500)
                return
        except (OSError, TypeError, ValueError):
            pass
        process.terminate()
        if not process.waitForFinished(800):
            process.kill()

    def _launch_finished(self, key: str) -> None:
        if key == "camera":
            self.camera_status.setText("Camera stopped")
        self._log(f"{key.title()} launch stopped.")

    def _start_camera(self) -> None:
        config = self._load_launch_file(CAMERA_CONFIG)
        if not config.get("workspace") or not config.get("command"):
            self._set_banner("Set the camera folder and launch command first")
            return
        self._start_launch_process("camera", config)

    def _load_input_mapping(self) -> None:
        try:
            raw = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8")) or {}
            if not isinstance(raw, dict):
                raise ValueError("Controller config must be a YAML mapping")
            self._input_config = raw
            self._input_config.pop("dpad_hat_index", None)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            self._input_config = {}
            self._log(f"Could not load controller mapping: {exc}")
        button_indices = self._input_config.get("button_indices", {})
        button_indices.pop("l3", None)
        button_indices.pop("r3", None)
        for key, spin in self._mapping_spins.items():
            index = button_indices.get(key)
            spin.setValue(index if isinstance(index, int) and 0 <= index <= 16 else -1)
        self._input_config["mode_buttons"] = {"manual": "cross", "autonomous": "triangle", "stop": "square"}
        axis_indices = self._input_config.get("axis_indices", {})
        manual = self._input_config.get("manual_drive", {})
        self._trigger_spins["l2"].setValue(
            axis_indices.get("l2") if isinstance(axis_indices.get("l2"), int) and 0 <= axis_indices["l2"] <= 16 else -1
        )
        r2_axis = manual.get("trigger_axis")
        self._trigger_spins["r2"].setValue(r2_axis if isinstance(r2_axis, int) and 0 <= r2_axis <= 16 else -1)
        stick = manual.get("left_stick", {})
        for key in ("linear_axis", "angular_axis"):
            value = stick.get(key)
            self._drive_axis_spins[key].setValue(value if isinstance(value, int) and 0 <= value <= 16 else -1)
        for key, spin in self._max_speed_spins.items():
            spin.setValue(float(manual.get(key, 0.3 if key == "max_linear_velocity" else 0.8)))
        for key in self._task_labels:
            path = TASKS.assigned_path(key)
            workspace = TASKS.assigned_workspace(key)
            command = TASKS.assigned_command(key)
            if command:
                self._show_command_assignment(key, command["path"], command["command"])
            elif workspace:
                self._task_labels[key].setText(f"ROS: {workspace['package']} · {workspace['launch_file']}")
            else:
                self._task_labels[key].setText(f"Assigned: {path.name}" if path else "No task assigned")

    def save_input_mapping(self) -> None:
        config = self._input_config.copy()
        config.pop("dpad_hat_index", None)
        indices = dict(config.get("button_indices", {}))
        indices.pop("l3", None)
        indices.pop("r3", None)
        for key, spin in self._mapping_spins.items():
            indices[key] = spin.value() if spin.value() >= 0 else None
        config["button_indices"] = indices
        config["mode_buttons"] = {"manual": "cross", "autonomous": "triangle", "stop": "square"}
        config["camera_button"] = "circle"
        axis_indices = dict(config.get("axis_indices", {}))
        axis_indices["l2"] = self._trigger_spins["l2"].value() if self._trigger_spins["l2"].value() >= 0 else None
        config["axis_indices"] = axis_indices
        manual = dict(config.get("manual_drive", {}))
        manual["trigger_axis"] = self._trigger_spins["r2"].value() if self._trigger_spins["r2"].value() >= 0 else None
        stick = dict(manual.get("left_stick", {}))
        for key, spin in self._drive_axis_spins.items():
            stick[key] = spin.value() if spin.value() >= 0 else None
        manual["left_stick"] = stick
        for key, spin in self._max_speed_spins.items():
            manual[key] = spin.value()
        config["manual_drive"] = manual
        previous_supervisor = self._load_launch_file(SUPERVISOR_CONFIG)
        previous_params = previous_supervisor.get("motiondock_supervisor", {}).get("ros__parameters", {})
        params = previous_supervisor.setdefault("motiondock_supervisor", {}).setdefault("ros__parameters", {})
        for key in self._max_speed_spins:
            params[key] = manual[key]
        try:
            CONFIG_PATH.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
            SUPERVISOR_CONFIG.write_text(yaml.safe_dump(previous_supervisor, sort_keys=False), encoding="utf-8")
        except OSError as exc:
            QMessageBox.critical(self, "Cannot save mapping", str(exc))
            return
        self._input_config = config
        speed_changed = any(float(previous_params.get(key, manual[key])) != float(manual[key]) for key in self._max_speed_spins)
        if speed_changed:
            self.set_mode("stop")
            self._restart_supervisor()
        self._log("Controller input mapping saved.")

    def start_controller_learning(self) -> None:
        if not self._controller_connected or self._last_snapshot is None:
            self._set_banner("Connect the controller before learning its inputs")
            return
        self.set_mode("stop")
        sequence = [
            ("cross", "Cross (×) — Manual mode"),
            ("triangle", "Triangle (△) — Autonomous mode"),
            ("square", "Square (□) — Stop"),
            ("circle", "Circle (○) — Camera"),
            ("dpad_up", "D-pad Up (or press the D-pad to detect its hat)"),
            ("dpad_down", "D-pad Down"),
            ("dpad_left", "D-pad Left"),
            ("dpad_right", "D-pad Right"),
        ]
        self._learning_button_steps = []
        seen: set[str] = set()
        for key, role in sequence:
            if key and key not in seen and key in self._mapping_spins:
                self._learning_button_steps.append((key, role))
                seen.add(key)
        self._learning_button_position = 0
        self._learning_previous_buttons = self._last_snapshot.buttons
        self._learning_previous_hats = self._last_snapshot.hats
        self._learning_active = True
        self._learning_step = "buttons"
        self._set_learning_prompt()

    def _set_learning_prompt(self) -> None:
        if self._learning_step == "buttons":
            if self._learning_button_position < len(self._learning_button_steps):
                _key, label = self._learning_button_steps[self._learning_button_position]
                self._set_banner(f"Controller setup · press {label}")
                return
            self._learning_step = "r2"
            self._learning_axis_baseline = self._last_snapshot.axes
        if self._learning_step == "r2":
            self._set_banner("Controller setup · leave sticks centered, then squeeze R2 fully")
        elif self._learning_step == "r2_release":
            self._set_banner("Controller setup · release R2")
        elif self._learning_step == "linear":
            self._set_banner("Controller setup · push the left stick fully forward")
        elif self._learning_step == "linear_release":
            self._set_banner("Controller setup · release the left stick")
        elif self._learning_step == "angular":
            self._set_banner("Controller setup · move the left stick fully left")
        elif self._learning_step == "complete":
            self._learning_active = False
            self._learning_step = ""
            self._input_armed = False
            self._previous_inputs.clear()
            self._manual_release_seen = False
            self._set_banner("Controller inputs learned and saved · choose Manual, then press R2 to drive")

    def _learn_button(self, snapshot) -> None:
        rising = [
            index for index, state in enumerate(snapshot.buttons)
            if state and (index >= len(self._learning_previous_buttons) or not self._learning_previous_buttons[index])
        ]
        key, label = self._learning_button_steps[self._learning_button_position] if self._learning_button_position < len(self._learning_button_steps) else ("", "")
        hat_directions = {"dpad_up": (0, 1), "dpad_down": (0, -1), "dpad_left": (-1, 0), "dpad_right": (1, 0)}
        newly_pressed_hat = next((i for i, hat in enumerate(snapshot.hats)
                                  if hat != (0, 0) and (i >= len(self._learning_previous_hats) or self._learning_previous_hats[i] == (0, 0))), None)
        self._learning_previous_buttons = snapshot.buttons
        self._learning_previous_hats = snapshot.hats
        if self._learning_button_position >= len(self._learning_button_steps):
            return
        if key in hat_directions and newly_pressed_hat is not None:
            direction = snapshot.hats[newly_pressed_hat]
            self.save_input_mapping()
            self._log(f"Detected D-pad hat input {direction}; using the controller's first hat.")
            self._learning_button_position = len(self._learning_button_steps)
            self._set_learning_prompt()
            return
        if not rising:
            return
        button_index = rising[0]
        existing = self._input_config.get("button_indices", {})
        conflict = next((name for name, value in existing.items() if name != key and value == button_index), None)
        if conflict:
            self._set_banner(f"Button {button_index} already mapped to {conflict}; press {label} again")
            return
        self._mapping_spins[key].setValue(button_index)
        self.save_input_mapping()
        self._log(f"Learned {label}: controller button {button_index}.")
        self._learning_button_position += 1
        self._set_learning_prompt()

    @staticmethod
    def _largest_axis_change(baseline: tuple[float, ...], axes: tuple[float, ...]) -> tuple[int, float, float] | None:
        changes = [
            (index, value - baseline[index], abs(value - baseline[index]))
            for index, value in enumerate(axes[:len(baseline)])
        ]
        if not changes:
            return None
        index, delta, magnitude = max(changes, key=lambda item: item[2])
        return index, delta, magnitude

    def _learn_axis(self, snapshot) -> None:
        if self._learning_step == "r2":
            change = self._largest_axis_change(self._learning_axis_baseline, snapshot.axes)
            if not change or change[2] < 0.55:
                rising = [
                    index for index, state in enumerate(snapshot.buttons)
                    if state and (index >= len(self._learning_previous_buttons) or not self._learning_previous_buttons[index])
                ]
                self._learning_previous_buttons = snapshot.buttons
                if rising:
                    button_index = rising[0]
                    self._input_config.setdefault("button_indices", {})["r2"] = button_index
                    manual = self._input_config.setdefault("manual_drive", {})
                    manual["trigger_axis"] = None
                    self._trigger_spins["r2"].setValue(-1)
                    self._mapping_spins["r2"].setValue(button_index)
                    self._save_learned_values()
                    self._learning_axis_baseline = snapshot.axes
                    self._learning_step = "linear"
                    self._set_banner("Digital R2 detected · release it, then push the left stick fully forward")
                return
            axis, delta, _magnitude = change
            direction = "positive" if delta > 0 else "negative"
            active_sign = 1.0 if direction == "positive" else -1.0
            rest_value = self._learning_axis_baseline[axis] * active_sign
            pressed_value = snapshot.axes[axis] * active_sign
            threshold = max(-0.8, min(0.8, (rest_value + pressed_value) / 2.0))
            manual = self._input_config.setdefault("manual_drive", {})
            manual["trigger_active_when"] = direction
            manual["trigger_threshold"] = threshold
            manual["trigger_hysteresis"] = 0.08
            self._trigger_spins["r2"].setValue(axis)
            self._save_learned_values()
            self._learning_release_index = axis
            self._learning_release_value = self._learning_axis_baseline[axis]
            self._learning_release_count = 0
            self._learning_step = "r2_release"
            self._set_learning_prompt()
        elif self._learning_step == "r2_release":
            self._wait_for_axis_release(snapshot)
        elif self._learning_step == "linear":
            change = self._largest_axis_change(self._learning_axis_baseline, snapshot.axes)
            if not change or change[2] < 0.5:
                return
            axis, delta, _magnitude = change
            manual = self._input_config.setdefault("manual_drive", {})
            manual.setdefault("left_stick", {})["linear_axis"] = axis
            manual["linear_axis_direction"] = 1.0 if delta > 0 else -1.0
            self._drive_axis_spins["linear_axis"].setValue(axis)
            self._save_learned_values()
            self._learning_release_index = axis
            self._learning_release_value = self._learning_axis_baseline[axis]
            self._learning_release_count = 0
            self._learning_step = "linear_release"
            self._set_learning_prompt()
        elif self._learning_step == "linear_release":
            self._wait_for_axis_release(snapshot)
        elif self._learning_step == "angular":
            change = self._largest_axis_change(self._learning_axis_baseline, snapshot.axes)
            if not change or change[2] < 0.5:
                return
            axis, delta, _magnitude = change
            manual = self._input_config.setdefault("manual_drive", {})
            manual.setdefault("left_stick", {})["angular_axis"] = axis
            manual["angular_axis_direction"] = -1.0 if delta < 0 else 1.0
            self._drive_axis_spins["angular_axis"].setValue(axis)
            self._save_learned_values()
            self._learning_step = "complete"
            self._set_learning_prompt()

    def _wait_for_axis_release(self, snapshot) -> None:
        axis = self._learning_release_index
        if axis is None or axis >= len(snapshot.axes):
            return
        if abs(snapshot.axes[axis] - self._learning_release_value) < 0.18:
            self._learning_release_count += 1
        else:
            self._learning_release_count = 0
        if self._learning_release_count >= 3:
            self._learning_axis_baseline = snapshot.axes
            self._learning_step = "linear" if self._learning_step == "r2_release" else "angular"
            self._set_learning_prompt()

    def _save_learned_values(self) -> None:
        self.save_input_mapping()

    def _handle_controller_learning(self, snapshot) -> None:
        if self._learning_step == "buttons":
            self._learn_button(snapshot)
        else:
            self._learn_axis(snapshot)

    def assign_task(self, key: str) -> None:
        control_name = dict((item[0], item[1]) for item in CONTROLS)[key]
        try:
            selection = self._choose_folder_command(
                control_name,
                "Commands to start this task from the selected folder. Include ROS and workspace sourcing, then the launch command:",
            )
            if not selection:
                return
            folder, command = selection
            stored = TASKS.assign_command(key, folder, command)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, "Could not assign task", str(exc))
            return
        self._show_command_assignment(key, str(stored), command)
        self._log(f"Assigned launch commands from {stored} to {key}.")

    @staticmethod
    def _workspace_launch_targets(workspace: Path) -> list[tuple[str, str]]:
        """Find launch files and their ROS package names inside a chosen workspace."""
        launch_suffixes = (".launch.py", ".launch.xml", ".launch.yaml", ".launch.yml")
        targets: list[tuple[str, str]] = []
        for package_xml in workspace.rglob("package.xml"):
            if any(part in {"build", "install", "log"} for part in package_xml.parts):
                continue
            package_root = package_xml.parent
            launch_dir = package_root / "launch"
            if not launch_dir.is_dir():
                continue
            try:
                root = ET.parse(package_xml).getroot()
                package_name = root.findtext("name")
            except (OSError, ET.ParseError):
                continue
            if not package_name:
                continue
            for launch_file in launch_dir.rglob("*"):
                if launch_file.is_file() and launch_file.name.endswith(launch_suffixes):
                    targets.append((package_name, launch_file.relative_to(launch_dir).as_posix()))
        return sorted(set(targets))

    def choose_autodock_file(self) -> None:
        try:
            assigned = TASKS.assigned_command("autodock")
            selection = self._choose_folder_command(
                "Auto-Dock",
                "Commands to start Auto-Dock from this folder. Include ROS/workspace sourcing, then the launch command:",
                assigned["command"] if assigned else "",
            )
            if not selection:
                return
            folder, command = selection
            stored = TASKS.assign_command("autodock", folder, command)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, "Could not configure Auto-Dock", str(exc))
            return
        self._log(f"Auto-Dock command saved for {stored}.")
        self.run_task("autodock")

    def _autodock_pressed(self) -> None:
        process = self._processes.get("autodock")
        self._autodock_was_running_on_press = bool(
            process and process.state() != QProcess.ProcessState.NotRunning
        )
        self._autodock_hold_started = time.monotonic()
        if self._autodock_was_running_on_press:
            self._autodock_hold_timer.start()

    def _update_autodock_hold(self) -> None:
        if self._autodock_hold_started is None or not self._autodock_was_running_on_press:
            return
        if time.monotonic() - self._autodock_hold_started >= 3.0:
            self._autodock_hold_timer.stop()
            self._stop_assigned_task("autodock")
            self._set_banner("Stopping Auto-Dock…")

    def _autodock_released(self) -> None:
        was_running = self._autodock_was_running_on_press
        held_to_stop = (
            was_running
            and self._autodock_hold_started is not None
            and time.monotonic() - self._autodock_hold_started >= 3.0
        )
        self._autodock_hold_timer.stop()
        self._autodock_hold_started = None
        self._autodock_was_running_on_press = False
        if was_running and not held_to_stop:
            self._set_banner("Auto-Dock is running · hold the button for 3 seconds to stop")
        elif not held_to_stop:
            self.choose_autodock_file()

    def run_task(self, key: str) -> None:
        command_assignment = TASKS.assigned_command(key)
        if command_assignment:
            active = self._processes.get(key)
            if active and active.state() != QProcess.ProcessState.NotRunning:
                return
            self._launch_shell_command(key, command_assignment["path"], command_assignment["command"])
            return
        workspace = TASKS.assigned_workspace(key)
        if workspace:
            active = self._processes.get(key)
            if active and active.state() != QProcess.ProcessState.NotRunning:
                return
            workspace_path = Path(workspace["path"])
            self._launch_process(
                key,
                "colcon",
                ["build"],
                phase="workspace_build",
                working_directory=workspace_path,
            )
            self._log(f"Building assigned ROS workspace for {key}: {workspace_path}")
            return
        path = TASKS.assigned_path(key)
        if path is None:
            if key != "autodock":
                self._log(f"No task assigned to {key}.")
            return
        active = self._processes.get(key)
        if active and active.state() != QProcess.ProcessState.NotRunning:
            return
        if TASKS.is_cpp(path):
            self._launch_process(
                key,
                "colcon",
                ["build", "--packages-select", "motiondock_user_tasks"],
                phase="build",
            )
        else:
            self._launch_process(key, sys.executable, [str(path)], phase="run")
        self._log(f"Starting task: {path.name}")

    def _launch_shell_command(self, key: str, folder: str, command: str) -> None:
        process = QProcess(self)
        process.setProgram("setsid")
        process.setArguments(["bash", "-lc", f"exec bash -lc {self._shell_quote(command)}", f"motiondock-{key}"])
        process.setWorkingDirectory(folder)
        process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        process.readyReadStandardOutput.connect(lambda p=process, k=key: self._read_task_output(k, p))
        process.finished.connect(
            lambda code, _status, k=key, p=process: self._task_finished(k, p, "run", code)
        )
        process.errorOccurred.connect(lambda error, k=key, p=process: self._task_process_error(k, p, error))
        self._processes[key] = process
        process.start()
        self._log(f"Starting {key} command in {folder}: {command}")
        if key == "autodock":
            self.autodock_button.setText("⚡  Auto-Dock · hold 3s to stop")

    def _launch_process(
        self,
        key: str,
        program: str,
        arguments: list[str],
        phase: str,
        working_directory: Path | None = None,
    ) -> None:
        process = QProcess(self)
        process.setProgram(program)
        process.setArguments(arguments)
        process.setWorkingDirectory(str(working_directory or WORKSPACE))
        process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        process.readyReadStandardOutput.connect(lambda p=process, k=key: self._read_task_output(k, p))
        process.finished.connect(
            lambda code, _status, k=key, p=process, current_phase=phase: self._task_finished(k, p, current_phase, code)
        )
        process.errorOccurred.connect(lambda error, k=key, p=process: self._task_process_error(k, p, error))
        self._processes[key] = process
        process.start()

    def _read_task_output(self, key: str, process: QProcess) -> None:
        output = bytes(process.readAllStandardOutput()).decode(errors="replace").strip()
        if output:
            for line in output.splitlines():
                self._log(f"[{key}] {line}")

    def _task_process_error(self, key: str, process: QProcess, error) -> None:
        if id(process) not in self._stopping_process_ids:
            self._log(f"Could not launch {key}: {error}")

    def _task_finished(self, key: str, process: QProcess, phase: str, exit_code: int) -> None:
        self._read_task_output(key, process)
        stopped_by_user = id(process) in self._stopping_process_ids
        self._stopping_process_ids.discard(id(process))
        if self._processes.get(key) is process:
            self._processes.pop(key, None)
        if key == "autodock":
            self.autodock_button.setText("⚡  Auto-Dock")
        if stopped_by_user:
            self._log(f"Task {key} stopped.")
            return
        if phase == "build" and exit_code == 0:
            path = TASKS.assigned_path(key)
            if path is not None:
                self._launch_process(
                    key,
                    "ros2",
                    ["run", "motiondock_user_tasks", TASKS.cpp_executable(path)],
                    phase="run",
                )
                self._log(f"Build complete; running ROS 2 task {path.stem}.")
            return
        if phase == "workspace_build":
            workspace = TASKS.assigned_workspace(key)
            if exit_code == 0 and workspace:
                workspace_path = Path(workspace["path"])
                setup_file = workspace_path / "install" / "setup.bash"
                script = 'source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" && source "$1" && exec ros2 launch "$2" "$3"'
                self._launch_process(
                    key,
                    "/bin/bash",
                    ["-lc", script, "motiondock", str(setup_file), workspace["package"], workspace["launch_file"]],
                    phase="run",
                    working_directory=workspace_path,
                )
                self._log(f"Workspace built; launching {workspace['package']} {workspace['launch_file']}.")
            else:
                self._log(f"ROS workspace build for {key} failed with exit code {exit_code}.")
            return
        self._log(f"Task {key} {'finished' if exit_code == 0 else f'exited with code {exit_code}'}.")

    def _pressed_controls(self, snapshot) -> dict[str, bool]:
        pressed: dict[str, bool] = {}
        indices = self._input_config.get("button_indices", {})
        for key, index in indices.items():
            pressed[key] = (
                isinstance(index, int)
                and 0 <= index < len(snapshot.buttons)
                and snapshot.buttons[index]
            )
        hat_index = 0 if snapshot.hats else None
        hat_directions = {
            "dpad_up": (0, 1),
            "dpad_down": (0, -1),
            "dpad_left": (-1, 0),
            "dpad_right": (1, 0),
        }
        for key, direction in hat_directions.items():
            button_index = indices.get(key)
            if isinstance(button_index, int) and 0 <= button_index < len(snapshot.buttons):
                pressed[key] = snapshot.buttons[button_index]
            elif isinstance(hat_index, int) and 0 <= hat_index < len(snapshot.hats):
                hat = snapshot.hats[hat_index]
                pressed[key] = hat[0] == direction[0] if direction[0] else hat[1] == direction[1]
            else:
                pressed[key] = False
        # R2 uses the same threshold detector as the displayed dead-man state.
        if isinstance(self._input_config.get("manual_drive", {}).get("trigger_axis"), int):
            pressed["r2"] = self._last_deadman
        axis_indices = self._input_config.get("axis_indices", {})
        directions = self._input_config.get("axis_active_when", {})
        thresholds = self._input_config.get("axis_thresholds", {})
        hysteresis = self._input_config.get("axis_hysteresis", {})
        for key, axis_index in axis_indices.items():
            if not isinstance(axis_index, int) or not 0 <= axis_index < len(snapshot.axes):
                pressed[key] = False
                self._axis_active[key] = False
                continue
            sign = -1.0 if directions.get(key, "positive") == "negative" else 1.0
            value = snapshot.axes[axis_index] * sign
            threshold = float(thresholds.get(key, 0.65))
            leave_threshold = threshold - max(0.0, float(hysteresis.get(key, 0.08)))
            cutoff = leave_threshold if self._axis_active.get(key, False) else threshold
            self._axis_active[key] = value >= cutoff
            pressed[key] = bool(pressed.get(key, False) or self._axis_active[key]) if key == "l2" else self._axis_active[key]
        return pressed

    @staticmethod
    def _apply_stick_deadzone(value: float, deadzone: float) -> float:
        deadzone = max(0.0, min(0.95, deadzone))
        magnitude = abs(value)
        if magnitude <= deadzone:
            return 0.0
        return math.copysign((magnitude - deadzone) / (1.0 - deadzone), value)

    def refresh_controller(self) -> None:
        try:
            snapshot = self.monitor.poll()
        except Exception as exc:
            self.bridge.publish(self.mode, 0.0, 0.0)
            self.connection_badge.setText("●  CONTROLLER READ ERROR")
            self.deadman_label.setText(f"R2 dead-man: disabled · {exc}")
            return
        self._last_snapshot = snapshot
        self._controller_connected = snapshot.connected
        if not snapshot.connected:
            if self._learning_active:
                self._learning_active = False
                self._learning_step = ""
                self._set_banner("Controller disconnected · setup canceled")
            self._set_label_text(self.connection_badge, "●  CONTROLLER DISCONNECTED")
            diagnostic = snapshot.name or "SDL sees no joystick devices"
            self._set_label_text(self.axis_values, f"No controller detected · {diagnostic}")
            for index, label in enumerate(self._raw_button_labels):
                self._set_raw_button(label, index, False)
            self._set_label_text(self.deadman_label, "R2 dead-man: inactive · no controller")
            self._set_label_text(self.manual_velocity_label, "Manual cmd_vel: linear 0.00 m/s · angular 0.00 rad/s")
            for key, label in self._status_labels.items():
                self._set_label_text(label, "waiting")
                self._set_led(self._led_labels[key], False)
            for label in self._trigger_leds.values():
                self._set_led(label, False)
            self._shown_inputs.clear()
            self._previous_inputs.clear()
            self._input_armed = False
            self._manual_release_seen = False
            self._camera_hold_started = None
            self.bridge.publish(self.mode, 0.0, 0.0)
            return

        self._set_label_text(self.connection_badge, f"●  {snapshot.name.upper()}")
        now = time.monotonic()
        update_telemetry_text = now - self._last_telemetry_text_update >= 0.15
        if update_telemetry_text:
            if snapshot.axes:
                self._set_label_text(self.axis_values, "     ".join(f"AXIS {i}  {value:+.2f}" for i, value in enumerate(snapshot.axes)))
            self._last_telemetry_text_update = now
        for index, label in enumerate(self._raw_button_labels):
            active = index < len(snapshot.buttons) and snapshot.buttons[index]
            self._set_raw_button(label, index, active)
        self._last_deadman = snapshot.deadman
        current = self._pressed_controls(snapshot)
        for trigger_key, label in self._trigger_leds.items():
            self._set_led(label, bool(current.get(trigger_key, False)))
        for key, label in self._status_labels.items():
            is_pressed = bool(current.get(key, False))
            self._set_led(self._led_labels[key], is_pressed)
            if self._shown_inputs.get(key) != is_pressed:
                self._set_label_text(label, "● PRESSED" if is_pressed else "released")
                self._shown_inputs[key] = is_pressed
        l2_state = "ACTIVE" if current.get("l2", False) else "inactive"
        self._set_label_text(self.deadman_label, f"L2 trigger: {l2_state} · R2 dead-man: {'ACTIVE' if snapshot.deadman else 'inactive'}")
        if self._learning_active:
            self._handle_controller_learning(snapshot)
            self.bridge.publish("stop", 0.0, 0.0)
            return
        if not self._input_armed:
            self._previous_inputs = current
            if not any(current.values()):
                self._input_armed = True
            self.bridge.publish(self.mode, 0.0, 0.0)
            return
        manual_button = "cross"
        autonomous_button = "triangle"
        stop_button = "square"
        camera_button = "circle"
        for key, label in self._status_labels.items():
            is_pressed = bool(current.get(key, False))
            if is_pressed and not self._previous_inputs.get(key, False):
                name = next((control[1] for control in CONTROLS if control[0] == key), key)
                self.show_button_feedback(key, name)
                if key == manual_button:
                    self.set_mode("manual")
                elif key == autonomous_button:
                    self.set_mode("autonomous")
                elif key == stop_button:
                    self.set_mode("stop")
                    self._set_banner("Stop command sent · velocity output is zero")
                elif key == camera_button:
                    if self.camera_process and self.camera_process.state() != QProcess.ProcessState.NotRunning:
                        self._camera_hold_started = time.monotonic()
                        self._camera_stop_triggered = False
                        self._set_banner("Camera running · keep holding Circle for 3 seconds to stop")
                    else:
                        self._start_camera()
                elif TASKS.assigned_path(key) or TASKS.assigned_workspace(key) or TASKS.assigned_command(key):
                    self.run_task(key)
        self._update_assigned_task_holds(current)
        camera_button = self._input_config.get("camera_button", "circle")
        camera_pressed = current.get(camera_button, False)
        if camera_pressed and self._camera_hold_started and not self._camera_stop_triggered:
            remaining = 3.0 - (time.monotonic() - self._camera_hold_started)
            if remaining <= 0:
                self._camera_stop_triggered = True
                self._stop_launch_process("camera")
                self._set_banner("Stopping camera launch…")
            else:
                self._set_banner(f"Hold Circle for {remaining:.1f}s to stop camera")
        if not camera_pressed and self._camera_hold_started is not None:
            if not self._camera_stop_triggered and self.camera_process and self.camera_process.state() != QProcess.ProcessState.NotRunning:
                self._set_banner("Camera running · hold Circle for 3 seconds to stop")
            self._camera_hold_started = None
            self._camera_stop_triggered = False
        self._previous_inputs = current

        manual = self._input_config.get("manual_drive", {})
        linear = 0.0
        angular = 0.0
        if self.mode == "manual":
            if not snapshot.deadman:
                self._manual_release_seen = True
            if snapshot.deadman and self._manual_release_seen:
                stick = manual.get("left_stick", {})
                linear_index = stick.get("linear_axis")
                angular_index = stick.get("angular_axis")
                max_linear = abs(float(manual.get("max_linear_velocity", 0.3)))
                max_angular = abs(float(manual.get("max_angular_velocity", 0.8)))
                deadzone = float(manual.get("stick_deadzone", 0.08))
                if isinstance(linear_index, int) and 0 <= linear_index < len(snapshot.axes):
                    value = snapshot.axes[linear_index] * float(manual.get("linear_axis_direction", -1.0))
                    linear = self._apply_stick_deadzone(value, deadzone) * max_linear
                if isinstance(angular_index, int) and 0 <= angular_index < len(snapshot.axes):
                    value = snapshot.axes[angular_index] * float(manual.get("angular_axis_direction", -1.0))
                    angular = self._apply_stick_deadzone(value, deadzone) * max_angular
        if update_telemetry_text:
            self._set_label_text(
                self.manual_velocity_label,
                f"Manual cmd_vel: linear {linear:+.2f} m/s · angular {angular:+.2f} rad/s",
            )
        self.bridge.publish(self.mode, linear, angular)

    def _update_assigned_task_holds(self, current: dict[str, bool]) -> None:
        now = time.monotonic()
        for key in self._status_labels:
            if key == "circle":
                continue  # Circle has its own camera hold behavior.
            task = self._processes.get(key)
            autonomy = self.autonomy_process if key == "triangle" else None
            process_running = any(
                process and process.state() != QProcess.ProcessState.NotRunning
                for process in (task, autonomy)
            )
            pressed = bool(current.get(key, False))
            if pressed and process_running and key not in self._task_stop_triggered:
                started = self._task_hold_started.setdefault(key, now)
                remaining = 3.0 - (now - started)
                if remaining <= 0:
                    self._task_stop_triggered.add(key)
                    self._stop_assigned_task(key)
                    if autonomy and autonomy.state() != QProcess.ProcessState.NotRunning:
                        self._stop_launch_process("autonomy")
                    self._set_banner(f"Stopping {key.replace('_', ' ').title()} task…")
                else:
                    self._set_banner(f"Hold {key.replace('_', ' ').title()} for {remaining:.1f}s to stop")
            elif not pressed:
                self._task_hold_started.pop(key, None)
                self._task_stop_triggered.discard(key)

    @staticmethod
    def _set_label_text(label: QLabel, text: str) -> None:
        if label.text() != text:
            label.setText(text)

    def closeEvent(self, event) -> None:
        self.timer.stop()
        self.bridge.publish("stop", 0.0, 0.0)
        time.sleep(0.12)
        self._stop_launch_process("camera")
        self._stop_launch_process("autonomy")
        for process in (self.camera_process, self.autonomy_process):
            if process and process.state() != QProcess.ProcessState.NotRunning:
                if not process.waitForFinished(1200):
                    process.kill()
        for process in self._processes.values():
            if process.state() != QProcess.ProcessState.NotRunning:
                self._stop_process_group(process)
        if self.supervisor_process and self.supervisor_process.state() != QProcess.ProcessState.NotRunning:
            self.supervisor_process.terminate()
            if not self.supervisor_process.waitForFinished(1200):
                self.supervisor_process.kill()
        self.bridge.close()
        self.monitor.close()
        super().closeEvent(event)


def main() -> None:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec())
