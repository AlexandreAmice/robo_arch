"""Loaded configuration records and reusable device metadata.

The loader turns YAML instance keys and system references into these records.
Device definitions describe reusable models, not live instances. Importing these
declarations does not load YAML or simulator SDKs.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Annotated

from pydantic import Field, JsonValue, StringConstraints

from robo_arch.core.config.parameters import Parameters
from robo_arch.core.config.schema import Schema
from robo_arch.core.config.worlds import WorldConfiguration

Name = Annotated[str, StringConstraints(pattern=r"^[A-Za-z][A-Za-z0-9_]*$")]


class Pose(Schema):
    """Child frame in parent frame; translation in metres, fixed-axis RPY radians."""

    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)


class TaskSelection(Schema):
    """The scenario evaluator validates task-specific parameters."""

    type: Name
    parameters: dict[str, JsonValue] = Field(default_factory=dict)


class AutonomySelection(Schema):
    """Select scenario-supported autonomy; its owner validates parameters."""

    controller: Name
    parameters: dict[str, JsonValue] = Field(default_factory=dict)


@dataclass(frozen=True, kw_only=True)
class RobotInstance:
    name: str
    model: str
    pose: Pose
    initial_positions: tuple[float, ...] | None


@dataclass(frozen=True, kw_only=True)
class SensorInstance:
    name: str
    model: str
    parent: str
    pose: Pose
    parameters: dict[str, JsonValue]


@dataclass(frozen=True, kw_only=True)
class RobotSystem:
    """Physical composition with local instance names and relative placements.

    The root has an empty name and a world-relative pose. Child-system poses are
    relative to their containing system; sensor poses are relative to their parent
    robot/body. Each inclusion loads independent parameter dictionaries.
    """

    name: str
    source: Path
    pose: Pose
    robots: tuple[RobotInstance, ...] = ()
    sensors: tuple[SensorInstance, ...] = ()
    systems: tuple["RobotSystem", ...] = ()


@dataclass(frozen=True, kw_only=True)
class ObjectInstance:
    name: str
    model: str
    pose: Pose


@dataclass(frozen=True, kw_only=True)
class SceneConfiguration:
    """Physical instances and observation selection, independent of a task."""

    robot_system: RobotSystem
    sensors_enabled: bool
    objects: tuple[ObjectInstance, ...]


@dataclass(frozen=True, kw_only=True)
class RunConfiguration:
    """Validated selections and names; device compatibility is checked at assembly.

    The robot system retains its hierarchy and local names. World assembly
    resolves device names, sensor parents and robot pose chains. Objects are fixed
    scene fixtures in this first runtime. Parameters remain caller-owned; do not
    mutate after loading.
    """

    source: Path
    world_config: WorldConfiguration
    duration: float
    robot_system: RobotSystem
    sensors_enabled: bool
    objects: tuple[ObjectInstance, ...]
    task: TaskSelection
    autonomy: AutonomySelection
    world_source: Path | None = None

    @property
    def scene(self) -> SceneConfiguration:
        return SceneConfiguration(
            robot_system=self.robot_system,
            sensors_enabled=self.sensors_enabled,
            objects=self.objects,
        )

    @property
    def world(self) -> str:
        return self.world_config.type

    @property
    def time_step(self) -> float:
        if self.world_config.type == "real":
            raise ValueError("The real world has no simulated physics time step")
        return self.world_config.physics.time_step

    @property
    def resources(self) -> tuple[Path, ...]:
        """Source documents used by this composition and its selected world."""
        paths = [self.source]
        if self.world_source is not None:
            paths.append(self.world_source)

        def visit(system: RobotSystem) -> None:
            paths.append(system.source)
            for child in system.systems:
                visit(child)

        visit(self.robot_system)
        return tuple(dict.fromkeys(paths))


@dataclass(frozen=True, kw_only=True)
class RobotDefinition:
    """Intrinsic model metadata and worlds with a device-owned adapter."""

    base_frame: str
    joints: tuple[str, ...]
    default_positions: tuple[float, ...]
    supported_worlds: tuple[str, ...]


@dataclass(frozen=True, kw_only=True)
class SensorDefinition:
    parameter_schema: type[Parameters]
    supported_worlds: tuple[str, ...]
    physical_worlds: tuple[str, ...] = ()
    kind: str = "camera"
    base_frame: str = "mount"
    measurement_frame: str = "depth_optical"
    resource: str = "assets/model.urdf"


@dataclass(frozen=True, kw_only=True)
class ObjectDefinition:
    package: str
    resource: str
    base_frame: str
    supported_worlds: tuple[str, ...]
