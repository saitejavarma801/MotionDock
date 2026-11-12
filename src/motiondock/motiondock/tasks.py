"""Copy, record, and launch user supplied task sources from this workspace."""

from __future__ import annotations

import re
import shutil
from pathlib import Path

import yaml


TASK_KEY = re.compile(r"^[a-z][a-z0-9_]*$")
ALLOWED_SUFFIXES = {".py", ".cpp", ".cc", ".cxx"}


class TaskManager:
    def __init__(self, workspace: Path) -> None:
        self.workspace = workspace.resolve()
        self.registry_path = self.workspace / "config" / "tasks.yaml"
        self.python_dir = self.workspace / "user_tasks"
        self.cpp_dir = self.workspace / "src" / "motiondock_user_tasks" / "tasks"

    def registry(self) -> dict[str, object]:
        try:
            values = yaml.safe_load(self.registry_path.read_text(encoding="utf-8")) or {}
        except FileNotFoundError:
            return {}
        except yaml.YAMLError as exc:
            raise ValueError(f"Invalid task registry YAML: {exc}") from exc
        if not isinstance(values, dict):
            raise ValueError("Task registry must be a YAML mapping")
        return {str(key): value for key, value in values.items()}

    def assign(self, key: str, source: Path) -> Path:
        if not TASK_KEY.fullmatch(key):
            raise ValueError("Invalid task identifier")
        source = source.expanduser().resolve(strict=True)
        if source.suffix.lower() not in ALLOWED_SUFFIXES or not source.is_file():
            raise ValueError("Choose a Python (.py) or C++ (.cpp, .cc, .cxx) source file")
        registry = self.registry()
        previous = registry.get(key)
        if isinstance(previous, str) and previous:
            previous_path = (self.workspace / previous).resolve()
            if previous_path.is_relative_to(self.workspace) and previous_path.is_file():
                previous_path.unlink()
        destination_dir = self.python_dir if source.suffix.lower() == ".py" else self.cpp_dir
        destination_dir.mkdir(parents=True, exist_ok=True)
        # Replacing a task also removes an older source with a different extension.
        for folder in (self.python_dir, self.cpp_dir):
            for suffix in ALLOWED_SUFFIXES:
                old_path = folder / f"{key}{suffix}"
                if old_path.is_file():
                    old_path.unlink()
        destination = destination_dir / f"{key}{source.suffix.lower()}"
        shutil.copy2(source, destination)
        relative = destination.relative_to(self.workspace).as_posix()
        registry[key] = relative
        self.registry_path.parent.mkdir(parents=True, exist_ok=True)
        self.registry_path.write_text(yaml.safe_dump(registry, sort_keys=True), encoding="utf-8")
        return destination

    def assign_workspace(self, key: str, workspace: Path, package: str, launch_file: str) -> Path:
        """Record a ROS workspace and launch target for a controller task."""
        if not TASK_KEY.fullmatch(key):
            raise ValueError("Invalid task identifier")
        workspace = workspace.expanduser().resolve(strict=True)
        if not workspace.is_dir():
            raise ValueError("Choose a ROS 2 workspace folder")
        if not package or Path(package).name != package:
            raise ValueError("Invalid ROS package name")
        if not launch_file or Path(launch_file).is_absolute() or ".." in Path(launch_file).parts:
            raise ValueError("Invalid ROS launch file")
        registry = self.registry()
        registry[key] = {
            "kind": "workspace",
            "path": str(workspace),
            "package": package,
            "launch_file": launch_file,
        }
        self.registry_path.parent.mkdir(parents=True, exist_ok=True)
        self.registry_path.write_text(yaml.safe_dump(registry, sort_keys=True), encoding="utf-8")
        return workspace

    def assign_command(self, key: str, folder: Path, command: str) -> Path:
        """Store an operator-entered command for a selected folder."""
        if not TASK_KEY.fullmatch(key):
            raise ValueError("Invalid task identifier")
        folder = folder.expanduser().resolve(strict=True)
        if not folder.is_dir():
            raise ValueError("Choose an existing folder")
        command = command.strip()
        if not command:
            raise ValueError("Enter the commands needed to launch this task")
        registry = self.registry()
        registry[key] = {"kind": "command", "path": str(folder), "command": command}
        self.registry_path.parent.mkdir(parents=True, exist_ok=True)
        self.registry_path.write_text(yaml.safe_dump(registry, sort_keys=True), encoding="utf-8")
        return folder

    def assigned_command(self, key: str) -> dict[str, str] | None:
        value = self.registry().get(key)
        if not isinstance(value, dict) or value.get("kind") != "command":
            return None
        folder = Path(str(value.get("path", ""))).expanduser()
        command = str(value.get("command", "")).strip()
        if not folder.is_dir() or not command:
            return None
        return {"path": str(folder.resolve()), "command": command}

    def assigned_workspace(self, key: str) -> dict[str, str] | None:
        value = self.registry().get(key)
        if not isinstance(value, dict) or value.get("kind") != "workspace":
            return None
        path = Path(str(value.get("path", ""))).expanduser()
        if not path.is_dir():
            return None
        return {
            "path": str(path.resolve()),
            "package": str(value.get("package", "")),
            "launch_file": str(value.get("launch_file", "")),
        }

    def assigned_path(self, key: str) -> Path | None:
        value = self.registry().get(key)
        if not isinstance(value, str) or not value:
            return None
        path = (self.workspace / value).resolve()
        if not path.is_relative_to(self.workspace) or not path.is_file():
            return None
        return path

    def is_cpp(self, path: Path) -> bool:
        return path.suffix.lower() in {".cpp", ".cc", ".cxx"}

    def cpp_executable(self, path: Path) -> str:
        return f"task_{path.stem}"
