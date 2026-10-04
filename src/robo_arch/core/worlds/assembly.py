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
    from pydrake.math import RigidTransform


@dataclass(frozen=True, kw_only=True)
class PlacedRobot:
    """Namespaced robot with poses composed left to right from world to base.

    :param name: Full instance name, including child-system namespaces.
    :param model: Robot/tool package identifier.
    :param poses: Root-system, child-system and robot-base placements, in order.
        Each pose maps its child frame into its parent; units are m and rad.
    :param initial_positions: Selected positions in declared joint order, or None
        for model defaults. These remain unchecked until model construction.

    This record stores the transform chain without evaluating SDK transforms.
    """

    name: str
    model: str
    poses: tuple[Pose, ...]
    initial_positions: tuple[float, ...] | None


@dataclass(frozen=True, kw_only=True)
class Devices:
    """Resolved physical instances, with no model definitions or live state.

    :param robots: PlacedRobot records in depth-first declaration order.
    :param sensors: SensorInstance records with qualified names and parents.
        Sensor parameter mappings are shared with the scene and remain read-only
        by convention; no deep copy is made during resolution.
    """

    robots: tuple[PlacedRobot, ...]
    sensors: tuple[SensorInstance, ...]


def resolve_devices(scene: SceneConfiguration) -> Devices:
    """Namespace devices and check attachments before constructing a world.

    Disabling observations preserves physical sensor bodies and their mounts.
    Their parent references are still checked.

    :param scene: Loaded physical composition with local names and relative poses.
    :returns: New Devices records with names like ``left/arm`` and sensor parents
        like ``left/arm/tool0``. Pose objects and nested settings are shared.
    :raises ValueError: Object/device name collisions or a sensor parent that
        does not name a resolved robot plus a nonempty body segment.

    Body existence, asset support and initial position dimensions are checked by
    the world implementation. This function imports no SDK and creates no devices.
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


def pose_transform(pose: Pose) -> RigidTransform:
    """Return a Drake child-to-parent RigidTransform from a Pose declaration.

    Translation is in metres and fixed-axis RPY is in radians. Imports pydrake
    on call; this transform helper requires the Drake environment. The input
    declaration is not modified.
    """
    from pydrake.math import RigidTransform, RollPitchYaw

    return RigidTransform(RollPitchYaw(pose.rpy), pose.translation)


def base_pose(robot: PlacedRobot) -> RigidTransform:
    """Return the robot base's pose in world, composing its declared pose chain.

    The result maps base-frame coordinates into world coordinates. Multiplication
    follows parent-to-child order. Imports pydrake on call and requires the Drake
    environment; it creates a new RigidTransform without modifying declarations.
    """
    from pydrake.math import RigidTransform

    result = RigidTransform()
    for pose in robot.poses:
        result = result @ pose_transform(pose)
    return result
