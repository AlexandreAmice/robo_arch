"""Configuration declarations, inspectable without YAML or simulator imports.

Private schemas describe YAML documents (where mapping keys name instances).
Loaded records retain those names and resolve referenced system documents into
compositions. Device definitions describe reusable models, not live instances.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Annotated

from pydantic import BaseModel, ConfigDict, Field, JsonValue, StringConstraints

from robo_arch.core.config.parameters import Parameters

Name = Annotated[str, StringConstraints(pattern=r"^[A-Za-z][A-Za-z0-9_]*$")]


class _Schema(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)


class Pose(_Schema):
    """Child frame in parent frame; translation in metres, fixed-axis RPY radians."""

    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)


class _Robot(_Schema):
    model: Name
    pose: Pose = Field(default_factory=Pose)
    initial_positions: tuple[float, ...] | None = None


class _Sensor(_Schema):
    model: Name
    parent: str
    pose: Pose = Field(default_factory=Pose)
    parameters: dict[str, JsonValue] = Field(default_factory=dict)


class _ChildSystem(_Schema):
    definition: str
    pose: Pose = Field(default_factory=Pose)


class _System(_Schema):
    robots: dict[Name, _Robot] = Field(default_factory=dict)
    sensors: dict[Name, _Sensor] = Field(default_factory=dict)
    systems: dict[Name, _ChildSystem] = Field(default_factory=dict)


class TaskSelection(_Schema):
    """The scenario evaluator validates task-specific parameters."""

    type: Name
    parameters: dict[str, JsonValue] = Field(default_factory=dict)


class AutonomySelection(_Schema):
    """Select scenario-supported autonomy; its owner validates parameters."""

    controller: Name
    parameters: dict[str, JsonValue] = Field(default_factory=dict)


class _RobotSystemSelection(_Schema):
    definition: str
    pose: Pose = Field(default_factory=Pose)
    autonomy: AutonomySelection


class _Object(_Schema):
    model: Name
    pose: Pose = Field(default_factory=Pose)


class _Scenario(_Schema):
    world: Name
    duration: float = Field(gt=0)
    time_step: float = Field(gt=0)
    sensors_enabled: bool = True
    robot_system: _RobotSystemSelection
    objects: dict[Name, _Object] = Field(default_factory=dict)
    task: TaskSelection


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
class RunConfiguration:
    """Validated selections and names; device compatibility is checked at assembly.

    The robot system retains its hierarchy and local names. World assembly
    resolves device names, sensor parents and robot pose chains. Objects are fixed
    scene fixtures in this first runtime. Parameters remain caller-owned; do not
    mutate after loading.
    """

    source: Path
    world: str
    duration: float
    time_step: float
    robot_system: RobotSystem
    sensors_enabled: bool
    objects: tuple[ObjectInstance, ...]
    task: TaskSelection
    autonomy: AutonomySelection


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


@dataclass(frozen=True, kw_only=True)
class ObjectDefinition:
    package: str
    resource: str
    base_frame: str
