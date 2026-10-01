"""A joint target checks the execution path before object manipulation."""

import math

from pydantic import Field

from robo_arch.core.config.parameters import Parameters


class WrenchCheck(Parameters):
    min_peak_force_N: float = Field(gt=0)


class TrackingTask(Parameters):
    robot: str
    target: tuple[float, ...] = Field(min_length=1)
    tolerance: float = Field(gt=0)
    wrenches: dict[str, WrenchCheck] = Field(default_factory=dict)


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


class RobotTarget(Parameters):
    target: tuple[float, ...] = Field(min_length=1)
    tolerance: float = Field(gt=0)


class MultiTrackingTask(Parameters):
    robots: dict[str, RobotTarget]
    wrenches: dict[str, WrenchCheck] = Field(default_factory=dict)


def tracking_tasks(parameters: dict) -> dict[str, TrackingTask]:
    """Accept the established one-arm task or explicitly named multiple targets."""
    if "robots" not in parameters:
        task = TrackingTask.model_validate(parameters)
        return {task.robot: task}
    task = MultiTrackingTask.model_validate(parameters)
    if not task.robots:
        raise ValueError("Tracking requires at least one robot target")
    return {
        name: TrackingTask(robot=name, target=value.target, tolerance=value.tolerance)
        for name, value in task.robots.items()
    }
