"""Workspace safety envelope (DESIGN.md §5.5, inherited from mcp_panda_brain)."""

from __future__ import annotations

from dataclasses import dataclass

from .errors import ErrorCode, failed


@dataclass(frozen=True)
class WorkspaceEnvelope:
    """Reachable EE / goal region in panda_link0 (desk sim: table ~z=0.05–0.08 m)."""

    x_min: float = 0.2
    x_max: float = 0.6
    y_min: float = -0.4
    y_max: float = 0.4
    z_min: float = 0.04
    z_max: float = 0.75

    def contains(self, x: float, y: float, z: float) -> bool:
        return (
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
            and self.z_min <= z <= self.z_max
        )

    def check_or_error(self, x: float, y: float, z: float) -> str | None:
        if self.contains(x, y, z):
            return None
        return failed(
            ErrorCode.OUT_OF_REACH,
            f"目标 ({x:.3f}, {y:.3f}, {z:.3f}) 超出安全工作空间",
        )


DEFAULT_ENVELOPE = WorkspaceEnvelope()
