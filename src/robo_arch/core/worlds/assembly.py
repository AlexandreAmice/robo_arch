"""Resolve a physical composition into device instances for world construction."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

from robo_arch.core.config.declarations import (
    Pose,
    RobotSystem,
    SceneConfiguration,
    SensorInstance,
)

if TYPE_CHECKING:
    import numpy as np


@dataclass(frozen=True, kw_only=True)
class PlacedRobot:
    """Namespaced robot with poses composed left to right from world to base."""

    name: str
    model: str
    poses: tuple[Pose, ...]
    initial_positions: tuple[float, ...] | None


@dataclass(frozen=True, kw_only=True)
class Devices:
    """Resolved instances; model definitions and live runtime state stay separate."""

    robots: tuple[PlacedRobot, ...]
    sensors: tuple[SensorInstance, ...]


def resolve_devices(scene: SceneConfiguration) -> Devices:
    """Namespace devices and check attachments before constructing a world.

    Disabling observations preserves physical sensor bodies and their mounts.
    Their parent references are still checked.
    """
    robots: list[PlacedRobot] = []
    sensors: list[SensorInstance] = []

    def visit(system: RobotSystem, prefix: str, poses: tuple[Pose, ...]) -> None:
        poses = (*poses, system.pose)
        for robot in system.robots:
            robots.append(
                PlacedRobot(
                    name=prefix + robot.name,
                    model=robot.model,
                    poses=(*poses, robot.pose),
                    initial_positions=robot.initial_positions,
                )
            )
        for sensor in system.sensors:
            sensors.append(
                replace(
                    sensor, name=prefix + sensor.name, parent=prefix + sensor.parent
                )
            )
        for child in system.systems:
            visit(child, prefix + child.name + "/", poses)

    visit(scene.robot_system, "", ())
    robot_names = {robot.name for robot in robots}
    device_names = robot_names | {sensor.name for sensor in sensors}
    conflicts = device_names.intersection(obj.name for obj in scene.objects)
    if conflicts:
        raise ValueError(
            f"Object and device names must be distinct: {sorted(conflicts)}"
        )
    for sensor in sensors:
        robot, separator, body = sensor.parent.rpartition("/")
        if not separator or not body or robot not in robot_names:
            raise ValueError(
                f"Sensor {sensor.name!r} parent {sensor.parent!r} must name a robot/body"
            )
    return Devices(
        robots=tuple(robots),
        sensors=tuple(sensors),
    )


def pose_matrix(pose: Pose) -> np.ndarray:
    """Return the child-to-parent homogeneous transform, in metres."""
    import numpy as np

    result = np.eye(4)
    roll, pitch, yaw = pose.rpy
    sr, sp, sy = np.sin([roll, pitch, yaw])
    cr, cp, cy = np.cos([roll, pitch, yaw])
    result[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    result[:3, 3] = pose.translation
    return result
