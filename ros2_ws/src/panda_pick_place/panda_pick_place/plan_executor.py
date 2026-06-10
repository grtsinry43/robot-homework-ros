"""Validate short LLM-authored tool plans before MCP execution."""

from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


MAX_PLAN_STEPS = 8

ALLOWED_PLAN_TOOLS = {
    "get_robot_context",
    "scan_scene",
    "pick_object",
    "place_at",
    "abort_current_task",
}

DISALLOWED_PLAN_TOOLS = {
    "execute_plan",
    "execute_arm_move",
    "set_gripper",
}

STEP_FIELDS = {"tool", "args"}


class PlanValidationError(ValueError):
    """Raised when an LLM plan does not match the action library contract."""


@dataclass(frozen=True)
class PlanStep:
    tool: str
    args: dict[str, Any]


def parse_plan(plan_json: str, action_library: Mapping[str, Any]) -> list[PlanStep]:
    if not isinstance(plan_json, str) or not plan_json.strip():
        raise PlanValidationError("plan_json must be a non-empty JSON string")

    try:
        raw_plan = json.loads(plan_json)
    except json.JSONDecodeError as exc:
        raise PlanValidationError(f"plan_json must be valid JSON: {exc.msg}") from exc

    return validate_plan(raw_plan, action_library)


def validate_plan(raw_plan: Any, action_library: Mapping[str, Any]) -> list[PlanStep]:
    action_specs = _action_specs_by_name(action_library)

    if not isinstance(raw_plan, list):
        raise PlanValidationError("plan must be a JSON array of steps")
    if not raw_plan:
        raise PlanValidationError("plan must contain at least one step")
    if len(raw_plan) > MAX_PLAN_STEPS:
        raise PlanValidationError(f"plan has {len(raw_plan)} steps; maximum is {MAX_PLAN_STEPS}")

    steps: list[PlanStep] = []
    for index, raw_step in enumerate(raw_plan):
        prefix = f"steps[{index}]"
        if not isinstance(raw_step, dict):
            raise PlanValidationError(f"{prefix} must be an object")

        unknown_step_fields = sorted(set(raw_step) - STEP_FIELDS)
        if unknown_step_fields:
            raise PlanValidationError(f"{prefix} contains unknown fields: {unknown_step_fields}")

        tool = raw_step.get("tool")
        if not isinstance(tool, str) or not tool.strip():
            raise PlanValidationError(f"{prefix}.tool must be a non-empty string")
        tool = tool.strip()

        if tool in DISALLOWED_PLAN_TOOLS:
            raise PlanValidationError(f"{prefix}.tool {tool!r} is not allowed inside execute_plan")
        if tool not in action_specs:
            raise PlanValidationError(f"{prefix}.tool {tool!r} is not registered in the action library")
        if tool not in ALLOWED_PLAN_TOOLS:
            raise PlanValidationError(f"{prefix}.tool {tool!r} is not safe for execute_plan")

        if "args" not in raw_step:
            raise PlanValidationError(f"{prefix}.args is required; use {{}} for actions without inputs")
        args = raw_step["args"]
        if not isinstance(args, dict):
            raise PlanValidationError(f"{prefix}.args must be an object")

        _validate_args(index, tool, args, action_specs[tool].get("inputs", {}))
        steps.append(PlanStep(tool=tool, args=dict(args)))

    return steps


def _action_specs_by_name(action_library: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    actions = action_library.get("actions")
    if not isinstance(actions, list):
        raise PlanValidationError("action library must contain an actions array")

    specs: dict[str, Mapping[str, Any]] = {}
    for action in actions:
        if not isinstance(action, Mapping):
            continue
        name = action.get("name")
        if isinstance(name, str):
            specs[name] = action
    return specs


def _validate_args(index: int, tool: str, args: Mapping[str, Any], inputs: Any) -> None:
    prefix = f"steps[{index}]"
    if not isinstance(inputs, Mapping):
        raise PlanValidationError(f"{prefix}.tool {tool!r} has invalid action-library inputs")

    unknown_args = sorted(set(args) - set(inputs))
    if unknown_args:
        raise PlanValidationError(f"{prefix}.args contains unknown arguments for {tool}: {unknown_args}")

    for name, spec in inputs.items():
        if not isinstance(spec, Mapping):
            continue
        if spec.get("required") is True and name not in args:
            raise PlanValidationError(f"{prefix}.args missing required argument {name!r} for {tool}")
        if name not in args:
            continue

        value = args[name]
        _validate_type(index, tool, name, value, spec.get("type"))
        enum = spec.get("enum")
        if isinstance(enum, list) and value not in enum:
            raise PlanValidationError(
                f"{prefix}.args.{name} must be one of {enum}; got {value!r}"
            )


def _validate_type(index: int, tool: str, name: str, value: Any, type_name: Any) -> None:
    if not isinstance(type_name, str):
        return

    ok = True
    if type_name == "string":
        ok = isinstance(value, str)
    elif type_name == "number":
        ok = (isinstance(value, int | float) and not isinstance(value, bool))
    elif type_name == "integer":
        ok = (isinstance(value, int) and not isinstance(value, bool))
    elif type_name == "boolean":
        ok = isinstance(value, bool)
    elif type_name == "object":
        ok = isinstance(value, dict)
    elif type_name == "array":
        ok = isinstance(value, list)

    if not ok:
        raise PlanValidationError(
            f"steps[{index}].args.{name} for {tool} must be {type_name}; got {type(value).__name__}"
        )
