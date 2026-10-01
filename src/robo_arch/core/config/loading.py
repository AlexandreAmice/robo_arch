"""Load declared physical assemblies and run settings from strict YAML.

Mounts are nominal transforms, not measured calibration. Robot bases are fixed
relative to their containing system in this initial schema. Autonomy wiring
lives in Python; measured calibration profiles are not yet supported.
"""

from importlib.resources import files
from pathlib import Path

import yaml
from pydantic import BaseModel, Field, JsonValue, RootModel

from robo_arch.core.config.declarations import (
    AutonomySelection,
    Name,
    ObjectInstance,
    Pose,
    RobotDefinition,
    RobotInstance,
    RobotSystem,
    RunConfiguration,
    SensorInstance,
    TaskSelection,
)
from robo_arch.core.config.resources import validate_package_reference
from robo_arch.core.config.schema import Schema
from robo_arch.core.config.worlds import WorldConfiguration


class _Robot(Schema):
    model: Name
    pose: Pose = Field(default_factory=Pose)
    initial_positions: tuple[float, ...] | None = None


class _Sensor(Schema):
    model: Name
    parent: str
    pose: Pose = Field(default_factory=Pose)
    parameters: dict[str, JsonValue] = Field(default_factory=dict)


class _ChildSystem(Schema):
    definition: str
    pose: Pose = Field(default_factory=Pose)


class _System(Schema):
    robots: dict[Name, _Robot] = Field(default_factory=dict)
    sensors: dict[Name, _Sensor] = Field(default_factory=dict)
    systems: dict[Name, _ChildSystem] = Field(default_factory=dict)


class _RobotSystemSelection(Schema):
    definition: str
    pose: Pose = Field(default_factory=Pose)
    autonomy: AutonomySelection


class _Object(Schema):
    model: Name
    pose: Pose = Field(default_factory=Pose)


class _Scenario(Schema):
    world: WorldConfiguration | str
    duration: float = Field(gt=0)
    sensors_enabled: bool = True
    robot_system: _RobotSystemSelection
    objects: dict[Name, _Object] = Field(default_factory=dict)
    task: TaskSelection


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML parsing with string keys and duplicate-key rejection."""


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


def _read_validated_yaml[T: BaseModel](path: Path, schema: type[T]) -> T:
    """Parse strict YAML and validate its fields; preserve original exceptions."""
    with path.open(encoding="utf-8") as stream:
        document = yaml.load(stream, Loader=_UniqueKeyLoader)
    return schema.model_validate(document)


def _resolve_package_reference(owner: Path, reference: str) -> Path:
    """Resolve an installed resource; owner identifies the referring file in errors."""
    validate_package_reference(reference, owner=owner)
    parts = reference.removeprefix("package://robo_arch/").split("/")
    # Editable installs, unpacked wheels and Bazel runfiles expose resource files.
    # Resolve only the application package; YAML cannot import Python modules.
    return Path(str(files("robo_arch").joinpath(*parts))).resolve()


def resolve_resource(reference: str) -> Path:
    """Resolve a validated application-package URI independently of the cwd."""
    return _resolve_package_reference(Path("<resource>"), reference)


def load_robot(path: str | Path) -> RobotDefinition:
    """Load a robot's asset declaration without importing robot code or SDKs."""
    source = (
        resolve_resource(path)
        if isinstance(path, str) and path.startswith("package:")
        else Path(path).resolve()
    )
    return _read_validated_yaml(source, RobotDefinition)


class _WorldProfile(RootModel[WorldConfiguration]):
    """Validate a complete world profile without inheritance or implicit merges."""


def load_world(path: str | Path) -> WorldConfiguration:
    """Load a profile from an explicit file or package URI at the Python/CLI boundary."""
    source = (
        _resolve_package_reference(Path("<world>"), path)
        if isinstance(path, str) and path.startswith("package:")
        else Path(path).resolve()
    )
    return _read_validated_yaml(source, _WorldProfile).root


def load_run(path: str | Path) -> RunConfiguration:
    """Load a file or package URI; YAML references use installed package resources."""
    source = (
        _resolve_package_reference(Path("<run>"), path)
        if isinstance(path, str) and path.startswith("package:")
        else Path(path).resolve()
    )
    scenario = _read_validated_yaml(source, _Scenario)
    world_source = (
        _resolve_package_reference(source, scenario.world)
        if isinstance(scenario.world, str)
        else None
    )
    world_config = (
        load_world(world_source) if world_source is not None else scenario.world
    )

    def load_system(
        file: Path,
        name: str,
        pose: Pose,
        ancestors: tuple[Path, ...],
    ) -> RobotSystem:
        if file in ancestors:
            chain = " -> ".join(str(item) for item in (*ancestors, file))
            raise ValueError(f"Recursive robot system inclusion: {chain}")
        system = _read_validated_yaml(file, _System)
        names = [*system.robots, *system.sensors, *system.systems]
        if len(set(names)) != len(names):
            raise ValueError(f"Device and child-system names must be unique in {file}")
        return RobotSystem(
            name=name,
            source=file,
            pose=pose,
            robots=tuple(
                RobotInstance(
                    name=name,
                    model=robot.model,
                    pose=robot.pose,
                    initial_positions=robot.initial_positions,
                )
                for name, robot in system.robots.items()
            ),
            sensors=tuple(
                SensorInstance(
                    name=name,
                    model=sensor.model,
                    parent=sensor.parent,
                    pose=sensor.pose,
                    parameters=sensor.parameters,
                )
                for name, sensor in system.sensors.items()
            ),
            systems=tuple(
                load_system(
                    _resolve_package_reference(file, child.definition),
                    name,
                    child.pose,
                    (*ancestors, file),
                )
                for name, child in system.systems.items()
            ),
        )

    robot_system = load_system(
        _resolve_package_reference(source, scenario.robot_system.definition),
        "",
        scenario.robot_system.pose,
        (),
    )

    return RunConfiguration(
        source=source,
        world_config=world_config,
        world_source=world_source,
        duration=scenario.duration,
        robot_system=robot_system,
        sensors_enabled=scenario.sensors_enabled,
        objects=tuple(
            ObjectInstance(name=name, model=obj.model, pose=obj.pose)
            for name, obj in scenario.objects.items()
        ),
        task=scenario.task,
        autonomy=scenario.robot_system.autonomy,
    )
