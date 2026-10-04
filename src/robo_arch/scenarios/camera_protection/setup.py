"""Resolve the camera task once, independently of its execution world."""

from dataclasses import dataclass
from typing import Any

import numpy as np

from robo_arch.core.config.declarations import RobotDefinition, RunConfiguration
from robo_arch.core.controllers.cbf.assembly import ProtectionGeometry, resolve_geometry
from robo_arch.core.worlds.assembly import Devices, PlacedRobot
from robo_arch.core.worlds.devices import DeviceDefinitions
from robo_arch.scenarios.camera_protection.configuration import (
    CameraProtectionParameters,
    TaskParameters,
    parameters_for,
)


@dataclass(frozen=True)
class ScenarioSetup:
    """Selected arm, motion task and protection geometry shared by both adapters."""

    control: CameraProtectionParameters
    task: TaskParameters
    robot: PlacedRobot
    definition: RobotDefinition
    geometry: ProtectionGeometry

    def validate_targets(self, lower: np.ndarray, upper: np.ndarray) -> None:
        """Validate task positions against limits in the declared joint order."""
        for target in (self.task.unsafe_target, self.task.retreat_target):
            if len(target) != len(lower) or not np.all(np.isfinite(target)):
                raise ValueError("Targets must match the selected robot's joint order")
            if np.any(target < lower) or np.any(target > upper):
                raise ValueError("Target exceeds robot joint limits")

    def describe(
        self,
        *,
        pair_names: tuple[str, ...],
        velocity_bound_names: tuple[str, ...],
        qp_constraint_count: int,
    ) -> dict[str, Any]:
        """Record the same ordered constraints and geometry for either runtime."""
        return {
            "pair_names": pair_names,
            "velocity_bound_names": velocity_bound_names,
            "qp_constraint_count": qp_constraint_count,
            "spheres": [vars(sphere) for sphere in self.geometry.spheres],
            "pairs": [vars(pair) for pair in self.geometry.pairs],
            "planes": [vars(plane) for plane in self.geometry.planes],
            "plane_pairs": [vars(pair) for pair in self.geometry.plane_pairs],
        }


def prepare(
    run: RunConfiguration, devices: Devices, definitions: DeviceDefinitions
) -> ScenarioSetup:
    """Validate scenario selections and resolve the common control inputs."""
    control, task = parameters_for(run)
    if {robot.name for robot in devices.robots} != {control.robot}:
        raise ValueError(
            "camera_protection requires the selected single controlled arm"
        )
    robot = devices.robots[0]
    return ScenarioSetup(
        control,
        task,
        robot,
        definitions.robots[robot.model],
        resolve_geometry(run.scene, control, ground=run.world_config.ground),
    )
