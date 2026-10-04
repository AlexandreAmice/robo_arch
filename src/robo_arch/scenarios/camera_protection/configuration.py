"""Scenario-owned selection and tuning; importing this module needs no SDK."""

from typing import Literal

from pydantic import Field

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.config.parameters import Parameters
from robo_arch.core.controllers.cbf.config import ProtectionParameters
from robo_arch.core.controllers.joint_tracking.definition import JointTrackingParameters


class CameraProtectionParameters(ProtectionParameters):
    """Scenario selection of controlled arm and nominal controller."""

    robot: str = "arm"
    nominal_controller: Literal["joint_tracking", "joint_pd"] = "joint_tracking"
    nominal: JointTrackingParameters


class TaskParameters(Parameters):
    unsafe_target: tuple[float, ...]
    retreat_target: tuple[float, ...]
    retreat_time: float = Field(gt=0)
    transition_seconds: float = Field(default=0.8, gt=0)
    tolerance: float = Field(default=0.03, gt=0)


def parameters_for(
    run: RunConfiguration,
) -> tuple[CameraProtectionParameters, TaskParameters]:
    if run.world not in {"drake", "isaac"}:
        raise ValueError("camera_protection supports only Drake and Isaac")
    if run.autonomy.controller != "cbf" or run.task.type != "camera_protection":
        raise ValueError(
            "camera_protection requires cbf autonomy and its task evaluator"
        )
    control = CameraProtectionParameters.model_validate(run.autonomy.parameters)
    if run.world == "isaac":
        if (
            run.world_config.control_backend != "torch"
            or control.backend != "torch_moreau"
        ):
            raise ValueError(
                "Isaac camera protection requires torch control and torch_moreau CBF"
            )
        if run.sensors_enabled:
            raise ValueError("GPU camera protection requires sensors_enabled: false")
        if run.world_config.physics.solver != "pgs":
            raise ValueError(
                "GPU camera protection requires PGS: imported TGS substeps lose "
                "small joint-position increments while reporting nonzero velocity"
            )
        if control.nominal_controller != "joint_tracking":
            raise ValueError(
                "GPU camera protection currently requires joint_tracking nominal control"
            )
    elif control.backend != "drake":
        raise ValueError("Drake camera protection requires backend: drake")
    task = TaskParameters.model_validate(run.task.parameters)
    if task.retreat_time >= run.duration:
        raise ValueError("Retreat must begin before the end of the run")
    if (
        task.transition_seconds >= task.retreat_time
        or task.retreat_time + task.transition_seconds >= run.duration
    ):
        raise ValueError(
            "Approach and retreat transitions must finish within their phases"
        )
    return control, task
