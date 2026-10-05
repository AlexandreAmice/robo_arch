"""Resolve a physical composition into device instances for world construction."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

from robo_arch.core.config.declarations import (
    CalibrationProfile,
    DeviceBinding,
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
    parent: str | None = None
    mount_frame: str | None = None
    binding: DeviceBinding | None = None
    calibration: CalibrationProfile | None = None
    mounting_revision: str | None = None


@dataclass(frozen=True)
class Mechanism:
    """One world-anchored connected mechanism; members retain device identity."""

    root: str
    robots: tuple[PlacedRobot, ...]


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

    @property
    def mechanisms(self) -> tuple[Mechanism, ...]:
        roots: dict[str, str] = {}
        groups: dict[str, list[PlacedRobot]] = {}
        for robot in self.robots:
            root = (
                robot.name
                if robot.parent is None
                else roots[robot.parent.rsplit("/", 1)[0]]
            )
            roots[robot.name] = root
            groups.setdefault(root, []).append(robot)
        return tuple(
            Mechanism(root, tuple(members)) for root, members in groups.items()
        )


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
                    poses=(robot.pose,) if robot.parent else (*poses, robot.pose),
                    initial_positions=robot.initial_positions,
                    parent=prefix + robot.parent if robot.parent else None,
                    mount_frame=robot.mount_frame,
                    binding=robot.binding,
                    calibration=robot.calibration,
                    mounting_revision=robot.mounting_revision,
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
    by_name = {robot.name: robot for robot in robots}
    if len(by_name) != len(robots):
        raise ValueError("Robot instance names must be unique")
    ordered: list[PlacedRobot] = []
    visiting: set[str] = set()
    visited: set[str] = set()

    def order(name: str) -> None:
        if name in visiting:
            raise ValueError(f"Cyclic robot attachment at {name}")
        if name in visited:
            return
        visiting.add(name)
        robot = by_name[name]
        if robot.parent:
            parent, separator, frame = robot.parent.rpartition("/")
            if not separator or not frame or parent not in by_name:
                raise ValueError(f"Robot {name} parent must name a robot/frame")
            order(parent)
        visiting.remove(name)
        visited.add(name)
        ordered.append(robot)

    for robot in robots:
        order(robot.name)
    robots = ordered
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
    identities = [item.binding.identity for item in (*robots, *sensors) if item.binding]
    if len(identities) != len(set(identities)):
        raise ValueError(
            "Each device binding must have a distinct installation identity"
        )
    for item in (*robots, *sensors):
        profile = item.calibration
        if profile is None:
            continue
        parent, _, frame = (item.parent or "world").rpartition("/")
        parent_binding = by_name[parent].binding if parent else None
        if (
            item.binding is None
            or profile.identity != item.binding.identity
            or profile.parent_identity
            != (parent_binding.identity if parent_binding else None)
            or profile.parent_frame != (frame or "world")
            or profile.mounting_revision != item.mounting_revision
            or (
                isinstance(item, PlacedRobot)
                and item.mount_frame
                and profile.child_frame != item.mount_frame
            )
        ):
            raise ValueError(
                f"Calibration identity/frame/revision mismatch for {item.name}"
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


def attachment_pose(robot: PlacedRobot) -> RigidTransform:
    """Effective parent-to-mount transform; calibrated pose replaces nominal pose."""
    return (
        pose_transform(robot.calibration.pose)
        if robot.calibration
        else base_pose(robot)
    )
