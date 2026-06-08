"""Load and render the explicit atomic action library contract."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .errors import ErrorCode


ACTION_LIBRARY_FILENAME = "atomic_actions.json"

REQUIRED_TOP_LEVEL_FIELDS = {
    "schema_version",
    "library_name",
    "description",
    "actions",
}

REQUIRED_ACTION_FIELDS = {
    "name",
    "kind",
    "status",
    "description",
    "entrypoints",
    "inputs",
    "outputs",
    "preconditions",
    "side_effects",
    "errors",
    "safety",
    "implementation_refs",
}

VALID_KINDS = {"mcp_tool", "ros_action", "ros_service", "code"}


def default_action_library_path() -> Path:
    """Return the installed config path, falling back to the source tree."""

    candidates: list[Path] = []
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("panda_pick_place"))
            / "config"
            / ACTION_LIBRARY_FILENAME
        )
    except Exception:  # noqa: BLE001 - source-tree scripts may run without an ament index.
        pass

    candidates.append(Path(__file__).resolve().parents[1] / "config" / ACTION_LIBRARY_FILENAME)

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[-1]


def load_action_library(path: str | Path | None = None) -> dict[str, Any]:
    library_path = Path(path) if path is not None else default_action_library_path()
    with library_path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)

    errors = validate_action_library(data)
    if errors:
        formatted = "\n".join(f"- {err}" for err in errors)
        raise ValueError(f"invalid action library {library_path}:\n{formatted}")
    return data


def validate_action_library(data: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(data, dict):
        return ["top-level value must be an object"]

    for field in sorted(REQUIRED_TOP_LEVEL_FIELDS):
        if field not in data:
            errors.append(f"missing top-level field: {field}")

    actions = data.get("actions")
    if not isinstance(actions, list) or not actions:
        errors.append("top-level field actions must be a non-empty list")
        return errors

    seen_names: set[str] = set()
    valid_error_codes = {code.value for code in ErrorCode}

    for index, action in enumerate(actions):
        prefix = f"actions[{index}]"
        if not isinstance(action, dict):
            errors.append(f"{prefix} must be an object")
            continue

        for field in sorted(REQUIRED_ACTION_FIELDS):
            if field not in action:
                errors.append(f"{prefix} missing field: {field}")

        name = action.get("name")
        if not isinstance(name, str) or not name.strip():
            errors.append(f"{prefix}.name must be a non-empty string")
        elif name in seen_names:
            errors.append(f"{prefix}.name duplicates action name: {name}")
        else:
            seen_names.add(name)

        kind = action.get("kind")
        if kind not in VALID_KINDS:
            errors.append(f"{prefix}.kind must be one of {sorted(VALID_KINDS)}")

        for object_field in ("entrypoints", "inputs", "outputs"):
            if object_field in action and not isinstance(action[object_field], dict):
                errors.append(f"{prefix}.{object_field} must be an object")

        for list_field in ("preconditions", "side_effects", "errors", "safety", "implementation_refs"):
            value = action.get(list_field)
            if list_field in action and not _is_string_list(value):
                errors.append(f"{prefix}.{list_field} must be a list of strings")

        for code in action.get("errors", []):
            if isinstance(code, str) and code not in valid_error_codes:
                errors.append(f"{prefix}.errors contains unknown error code: {code}")

    return errors


def render_action_library(data: dict[str, Any] | None = None) -> str:
    library = data if data is not None else load_action_library()
    lines = [
        f"{library['library_name']} (schema v{library['schema_version']})",
        library["description"],
        "",
        "Actions:",
    ]

    for action in library["actions"]:
        inputs = _format_inputs(action["inputs"])
        errors = ", ".join(action["errors"]) if action["errors"] else "none"
        lines.append(
            f"- {action['name']} [{action['status']}]: {action['description']} "
            f"Inputs: {inputs}. Errors: {errors}."
        )

    return "\n".join(lines)


def _format_inputs(inputs: dict[str, Any]) -> str:
    if not inputs:
        return "none"

    chunks: list[str] = []
    for name, spec in inputs.items():
        if isinstance(spec, dict):
            type_name = spec.get("type", "unknown")
            enum = spec.get("enum")
            if isinstance(enum, list):
                chunks.append(f"{name}:{type_name}={enum}")
            else:
                chunks.append(f"{name}:{type_name}")
        else:
            chunks.append(name)
    return ", ".join(chunks)


def _is_string_list(value: Any) -> bool:
    return isinstance(value, list) and all(isinstance(item, str) and item.strip() for item in value)
