"""A joint target checks the execution path before object manipulation."""

import math

from pydantic import Field

from robo_arch.core.config.parameters import Parameters


class TrackingTask(Parameters):
    robot: str
    target: tuple[float, ...] = Field(min_length=1)
    tolerance: float = Field(gt=0)


def evaluate(task: TrackingTask, positions: tuple[float, ...]) -> dict:
    if len(positions) != len(task.target) or not all(map(math.isfinite, positions)):
        raise ValueError("Task evaluation requires a finite position for every joint")
    error = float(
        max(
            abs(actual - desired)
            for actual, desired in zip(positions, task.target, strict=True)
        )
    )
    return {"success": error <= task.tolerance, "max_joint_error_rad": error}
