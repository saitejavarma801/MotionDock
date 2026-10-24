"""Read-only SDL controller monitor and configurable R2 threshold detector."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pygame
import yaml
import time


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
        self._joystick_index: int | None = None
        self._deadman_active = False
        self._last_config: dict[str, Any] = {}
        self._last_rescan = 0.0
        self._last_error = ""
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
            now = time.monotonic()
            if now - self._last_rescan >= 1.5:
                self._last_rescan = now
                try:
                    pygame.joystick.quit()
                    pygame.joystick.init()
                    pygame.event.pump()
                    count = pygame.joystick.get_count()
                except pygame.error as exc:
                    self._last_error = str(exc)
            if count == 0:
                detail = f"SDL sees 0 joystick devices; {self._last_error}" if self._last_error else "SDL sees 0 joystick devices"
                return ControllerSnapshot(False, name=detail)

        try:
            if self._joystick is None or not self._joystick.get_init():
                if self._joystick is not None:
                    self._drop_joystick()
                self._joystick = self._open_best_joystick(config)
                if self._joystick is None:
                    detail = self._last_error or f"SDL sees {pygame.joystick.get_count()} device(s), but none could be opened"
                    return ControllerSnapshot(False, name=detail)
            joy = self._joystick
            buttons = tuple(bool(joy.get_button(i)) for i in range(joy.get_numbuttons()))
            axes = tuple(round(float(joy.get_axis(i)), 4) for i in range(joy.get_numaxes()))
            hats = tuple(joy.get_hat(i) for i in range(joy.get_numhats()))
        except pygame.error as exc:
            self._last_error = str(exc)
            self._drop_joystick()
            self._deadman_active = False
            return ControllerSnapshot(False, name=f"SDL controller read failed: {exc}")

        drive = config.get("manual_drive", {})
        axis_index = drive.get("trigger_axis")
        stick_axes = drive.get("left_stick", {})
        configured_stick_axes = {
            value for value in stick_axes.values()
            if isinstance(value, int) and value >= 0
        }
        button_index = config.get("button_indices", {}).get("r2")
        button_active = (
            isinstance(button_index, int)
            and 0 <= button_index < len(buttons)
            and buttons[button_index]
        )
        axis_available = (
            isinstance(axis_index, int)
            and 0 <= axis_index < len(axes)
            and axis_index not in configured_stick_axes
        )
        if axis_available:
            direction = drive.get("trigger_active_when", "positive")
            if direction not in ("positive", "negative"):
                self._deadman_active = bool(button_active)
                message = "R2 trigger direction is invalid; using its button input"
            else:
                value = axes[axis_index] * (1.0 if direction == "positive" else -1.0)
                threshold = float(drive.get("trigger_threshold", 0.65))
                hysteresis = max(0.0, float(drive.get("trigger_hysteresis", 0.08)))
                cutoff = threshold - hysteresis if self._deadman_active else threshold
                axis_active = value >= cutoff
                self._deadman_active = bool(axis_active or button_active)
                message = f"R2 axis {axis_index}: {value:+.3f} / threshold {threshold:.3f}"
        else:
            self._deadman_active = bool(button_active)
            if isinstance(axis_index, int) and axis_index in configured_stick_axes:
                reason = f"R2 axis {axis_index} overlaps a stick axis"
            elif axis_index is not None:
                reason = f"R2 axis {axis_index} is unavailable"
            else:
                reason = "no R2 axis configured"
            message = f"{reason}; using R2 button {button_index}" if isinstance(button_index, int) else reason

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
        self._joystick_index = None

    def _open_best_joystick(self, config: dict[str, Any]) -> pygame.joystick.Joystick | None:
        """Select the most gamepad-like attached device instead of assuming device 0."""
        preferred = str(config.get("device_name", "")).strip().casefold()
        gamepad_names = ("wireless controller", "dualsense", "dualshock", "playstation", "gamepad")
        best: pygame.joystick.Joystick | None = None
        best_index: int | None = None
        best_score = -1
        for index in range(pygame.joystick.get_count()):
            candidate = pygame.joystick.Joystick(index)
            try:
                candidate.init()
                name = candidate.get_name().casefold()
                axes = candidate.get_numaxes()
                buttons = candidate.get_numbuttons()
                hats = candidate.get_numhats()
                score = axes * 4 + buttons + hats * 2
                if any(token in name for token in gamepad_names):
                    score += 12
                if preferred and preferred in name:
                    score += 1000
                if score > best_score:
                    if best is not None:
                        best.quit()
                    best, best_index, best_score = candidate, index, score
                else:
                    candidate.quit()
            except pygame.error as exc:
                self._last_error = str(exc)
                try:
                    candidate.quit()
                except pygame.error:
                    pass
        self._joystick_index = best_index
        self._last_error = ""
        return best
