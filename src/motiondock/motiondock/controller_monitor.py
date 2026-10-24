"""Read-only SDL controller monitor and configurable R2 threshold detector."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pygame
import yaml


@dataclass(frozen=True)
class ControllerSnapshot:
    connected: bool
    name: str = ""
    buttons: tuple[bool, ...] = ()
    axes: tuple[float, ...] = ()
    hats: tuple[tuple[int, int], ...] = ()
    deadman: bool = False
    deadman_message: str = "R2 axis not configured"


class ControllerMonitor:
    """Poll the first attached SDL joystick. This class never publishes robot commands."""

    def __init__(self, config_path: Path) -> None:
        self.config_path = config_path
        self._joystick: pygame.joystick.Joystick | None = None
        self._deadman_active = False
        self._last_config: dict[str, Any] = {}
        pygame.display.init()
        pygame.joystick.init()

    def close(self) -> None:
        if self._joystick:
            self._joystick.quit()
            self._joystick = None
        pygame.joystick.quit()
        pygame.display.quit()

    def _read_config(self) -> dict[str, Any]:
        raw = yaml.safe_load(self.config_path.read_text(encoding="utf-8")) or {}
        if not isinstance(raw, dict):
            raise ValueError("Controller configuration must be a YAML mapping")
        return raw

    def poll(self) -> ControllerSnapshot:
        try:
            config = self._read_config()
            self._last_config = config
        except (OSError, ValueError, yaml.YAMLError):
            config = self._last_config
        pygame.event.pump()
        count = pygame.joystick.get_count()
        if count == 0:
            self._drop_joystick()
            self._deadman_active = False
            return ControllerSnapshot(False)

        try:
            if self._joystick is None or not self._joystick.get_init():
                self._joystick = pygame.joystick.Joystick(0)
                self._joystick.init()
            joy = self._joystick
            buttons = tuple(bool(joy.get_button(i)) for i in range(joy.get_numbuttons()))
            axes = tuple(round(float(joy.get_axis(i)), 4) for i in range(joy.get_numaxes()))
            hats = tuple(joy.get_hat(i) for i in range(joy.get_numhats()))
        except pygame.error:
            self._drop_joystick()
            self._deadman_active = False
            return ControllerSnapshot(False)

        drive = config.get("manual_drive", {})
        axis_index = drive.get("trigger_axis")
        stick_axes = drive.get("left_stick", {})
        configured_stick_axes = {
            value for value in stick_axes.values()
            if isinstance(value, int) and value >= 0
        }
        if isinstance(axis_index, int) and axis_index in configured_stick_axes:
            self._deadman_active = False
            message = f"Configured R2 axis {axis_index} overlaps a left-stick axis; run Learn controller inputs"
        elif axis_index is None:
            self._deadman_active = False
            message = "R2 axis not configured; inspect axes, then set trigger_axis in config/controller.yaml"
        elif not isinstance(axis_index, int) or axis_index < 0 or axis_index >= len(axes):
            self._deadman_active = False
            message = f"Configured R2 axis {axis_index!r} is unavailable"
        else:
            direction = drive.get("trigger_active_when", "positive")
            if direction not in ("positive", "negative"):
                self._deadman_active = False
                message = "trigger_active_when must be positive or negative"
            else:
                value = axes[axis_index] * (1.0 if direction == "positive" else -1.0)
                threshold = float(drive.get("trigger_threshold", 0.65))
                hysteresis = max(0.0, float(drive.get("trigger_hysteresis", 0.08)))
                cutoff = threshold - hysteresis if self._deadman_active else threshold
                self._deadman_active = value >= cutoff
                message = f"R2 axis {axis_index}: {value:+.3f} / threshold {threshold:.3f}"

        return ControllerSnapshot(
            True,
            joy.get_name(),
            buttons,
            axes,
            hats,
            self._deadman_active,
            message,
        )

    def _drop_joystick(self) -> None:
        if self._joystick:
            try:
                self._joystick.quit()
            except pygame.error:
                pass
            self._joystick = None
