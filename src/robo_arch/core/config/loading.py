"""Load physical assemblies and autonomy settings without simulator SDKs.

Mounts are nominal transforms, not measured calibration. Robot bases are fixed
relative to their containing system in this initial schema. Autonomy wiring
lives in Python; measured calibration profiles are not yet supported.
"""

from dataclasses import dataclass
from importlib.resources import files
from pathlib import Path
from typing import Annotated

import yaml
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    JsonValue,
    RootModel,
    StringConstraints,
)

from robo_arch.core.config.worlds import WorldConfiguration, validate_package_reference

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
    world: WorldConfiguration | str
    duration: float = Field(gt=0)
    sensors_enabled: bool = True
    robot_system: _RobotSystemSelection
    objects: dict[Name, _Object] = Field(default_factory=dict)
    task: TaskSelection


@dataclass(frozen=True, kw_only=True)
class RobotInstance:
    name: str
    model: str
    poses: tuple[Pose, ...]
    initial_positions: tuple[float, ...] | None


@dataclass(frozen=True, kw_only=True)
class SensorInstance:
    name: str
    model: str
    parent: str
    pose: Pose
    parameters: dict[str, JsonValue]


@dataclass(frozen=True, kw_only=True)
class ObjectInstance:
    name: str
    model: str
    pose: Pose


@dataclass(frozen=True, kw_only=True)
class RunConfiguration:
    """Validated selections and names; device compatibility is checked at assembly.

    Pose chains compose left to right from world to robot base. Sensors reference
    a fully namespaced robot/body pair. Objects are fixed scene fixtures in this
    first runtime. Parameters remain caller-owned; do not mutate after loading.
    """

    source: Path
    world_config: WorldConfiguration
    duration: float
    robots: tuple[RobotInstance, ...]
    sensors: tuple[SensorInstance, ...]
    objects: tuple[ObjectInstance, ...]
    task: TaskSelection
    autonomy: AutonomySelection
    resources: tuple[Path, ...]

    @property
    def world(self) -> str:
        return self.world_config.type

    @property
    def time_step(self) -> float:
        if self.world_config.type == "real":
            raise ValueError("The real world has no simulated physics time step")
        return self.world_config.physics.time_step


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _mapping(loader: _UniqueKeyLoader, node: yaml.MappingNode) -> dict:
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=True)
        if not isinstance(key, str):
            raise ValueError(
                f"YAML mapping keys must be strings at {key_node.start_mark}"
            )
        if key in result:
            raise ValueError(f"Duplicate YAML key {key!r} at {key_node.start_mark}")
        result[key] = loader.construct_object(value_node, deep=True)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping
)


def _read[T: BaseModel](path: Path, schema: type[T], resources: list[Path]) -> T:
    try:
        with path.open(encoding="utf-8") as stream:
            document = yaml.load(stream, Loader=_UniqueKeyLoader)
        value = schema.model_validate(document)
    except (OSError, ValueError, yaml.YAMLError) as error:
        raise ValueError(f"Invalid configuration {path}: {error}") from error
    if path not in resources:
        resources.append(path)
    return value


def _reference(owner: Path, reference: str) -> Path:
    try:
        validate_package_reference(reference)
    except ValueError as error:
        raise ValueError(f"{error} (in {owner})") from error
    parts = reference.removeprefix("package://robo_arch/").split("/")
    # Editable installs, unpacked wheels and Bazel runfiles expose resource files.
    # Resolve only the application package; YAML cannot import Python modules.
    return Path(str(files("robo_arch").joinpath(*parts))).resolve()


def resolve_resource(reference: str) -> Path:
    """Resolve a validated application-package URI independently of the cwd."""
    return _reference(Path("<resource>"), reference)


class _WorldProfile(RootModel[WorldConfiguration]):
    pass


def load_world(path: str | Path) -> WorldConfiguration:
    """Load a complete world profile from an explicit file or package URI.

    File paths are accepted at the Python/CLI entry point. References inside YAML
    must use package URIs; profiles contain no inheritance or implicit merges.
    """
    source = (
        _reference(Path("<world>"), path)
        if isinstance(path, str) and path.startswith("package:")
        else Path(path).resolve()
    )
    return _read(source, _WorldProfile, []).root


def load_run(path: str | Path) -> RunConfiguration:
    """Load a file or package URI; YAML references use installed package resources."""
    source = (
        _reference(Path("<run>"), path)
        if isinstance(path, str) and path.startswith("package:")
        else Path(path).resolve()
    )
    resources: list[Path] = []
    scenario = _read(source, _Scenario, resources)
    world_config = (
        _read(_reference(source, scenario.world), _WorldProfile, resources).root
        if isinstance(scenario.world, str)
        else scenario.world
    )
    robots: list[RobotInstance] = []
    sensors: list[SensorInstance] = []

    def expand(
        file: Path,
        prefix: str,
        poses: tuple[Pose, ...],
        ancestors: tuple[Path, ...],
    ) -> None:
        if file in ancestors:
            chain = " -> ".join(str(item) for item in (*ancestors, file))
            raise ValueError(f"Recursive robot system inclusion: {chain}")
        system = _read(file, _System, resources)
        names = [*system.robots, *system.sensors, *system.systems]
        if len(set(names)) != len(names):
            raise ValueError(f"Device and child-system names must be unique in {file}")
        for name, robot in system.robots.items():
            robots.append(
                RobotInstance(
                    name=prefix + name,
                    model=robot.model,
                    poses=(*poses, robot.pose),
                    initial_positions=robot.initial_positions,
                )
            )
        for name, sensor in system.sensors.items():
            sensors.append(
                SensorInstance(
                    name=prefix + name,
                    model=sensor.model,
                    parent=prefix + sensor.parent,
                    pose=sensor.pose,
                    parameters=sensor.parameters,
                )
            )
        for name, child in system.systems.items():
            expand(
                _reference(file, child.definition),
                prefix + name + "/",
                (*poses, child.pose),
                (*ancestors, file),
            )

    expand(
        _reference(source, scenario.robot_system.definition),
        "",
        (scenario.robot_system.pose,),
        (),
    )
    robot_names = {robot.name for robot in robots}
    device_names = robot_names | {sensor.name for sensor in sensors}
    conflicts = device_names.intersection(scenario.objects)
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

    return RunConfiguration(
        source=source,
        world_config=world_config,
        duration=scenario.duration,
        robots=tuple(robots),
        sensors=tuple(sensors) if scenario.sensors_enabled else (),
        objects=tuple(
            ObjectInstance(name=name, model=obj.model, pose=obj.pose)
            for name, obj in scenario.objects.items()
        ),
        task=scenario.task,
        autonomy=scenario.robot_system.autonomy,
        resources=tuple(resources),
    )
