"""Load declared physical assemblies and run settings from strict YAML.

Instance selections retain explicit installation and calibration identities.
World loaders validate named frames and native assets before building devices.
Autonomy remains Python code selected independently of physical composition.
"""

from dataclasses import replace
from importlib.resources import files
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, Field, JsonValue, RootModel

from robo_arch.core.config.declarations import (
    AutonomySelection,
    CalibrationProfile,
    DeviceBinding,
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
    parent: str | None = Field(default=None, min_length=1)
    mount_frame: str | None = Field(default=None, min_length=1)
    binding: DeviceBinding | None = None
    calibration: str | None = None
    mounting_revision: str | None = None


class _Sensor(Schema):
    model: Name
    parent: str
    pose: Pose = Field(default_factory=Pose)
    parameters: dict[str, JsonValue] = Field(default_factory=dict)
    binding: DeviceBinding | None = None
    calibration: str | None = None
    mounting_revision: str | None = None


class _InstanceSelection(Schema):
    binding: DeviceBinding
    calibration: str | None = None
    mounting_revision: str | None = None


class _ChildSystem(Schema):
    definition: str
    pose: Pose = Field(default_factory=Pose)
    instances: dict[str, _InstanceSelection] = Field(default_factory=dict)


class _System(Schema):
    robots: dict[Name, _Robot] = Field(default_factory=dict)
    sensors: dict[Name, _Sensor] = Field(default_factory=dict)
    systems: dict[Name, _ChildSystem] = Field(default_factory=dict)


class _RobotSystemSelection(Schema):
    definition: str
    pose: Pose = Field(default_factory=Pose)
    autonomy: AutonomySelection
    instances: dict[str, _InstanceSelection] = Field(default_factory=dict)


class _Object(Schema):
    model: Name
    pose: Pose = Field(default_factory=Pose)
    motion: Literal["fixed", "free"] = "fixed"
    angular_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    linear_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)


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


def read_validated_yaml[T: BaseModel](path: Path, schema: type[T]) -> T:
    """Parse strict YAML and validate it with an SDK-independent Pydantic schema.

    :param path: File to read as UTF-8; relative paths follow the caller's cwd.
    :param schema: Model class used to validate the loaded mapping or root value.
    :returns: A new validated model. YAML does not construct arbitrary objects.
    :raises ValueError: Duplicate or non-string YAML mapping keys.

    File, YAML parser and Pydantic validation exceptions propagate unchanged.
    """
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
    """Resolve a validated application-package URI independently of the cwd.

    :param reference: A ``package://robo_arch/...`` resource reference.
    :returns: Absolute path in the installed package, editable tree or Bazel
        runfiles. Existence is not checked and no file is opened.
    :raises ValueError: Invalid URI syntax, traversal or a foreign package.

    Package resources must be available as unpacked files, not zip-only imports.
    """
    return _resolve_package_reference(Path("<resource>"), reference)


def load_robot(path: str | Path) -> RobotDefinition:
    """Load a robot's asset declaration without importing robot code or SDKs.

    :param path: A declaration file path or ``package://robo_arch/...`` URI.
        Relative file paths are relative to the caller's cwd, never another YAML.
    :returns: Validated asset, joint order, default positions and base frame.

    Resource syntax, file, YAML and validation errors propagate. This does not
    parse the referenced model asset or establish world/hardware support.
    """
    source = (
        resolve_resource(path)
        if isinstance(path, str) and path.startswith("package:")
        else Path(path).resolve()
    )
    return read_validated_yaml(source, RobotDefinition)


class _WorldProfile(RootModel[WorldConfiguration]):
    """Validate a complete world profile without inheritance or implicit merges."""


def load_world(path: str | Path) -> WorldConfiguration:
    """Load one complete world profile without importing simulator SDKs.

    :param path: File path (cwd-relative if needed) or application-package URI.
    :returns: DrakeWorld, IsaacWorld or RealWorld selected by ``type``.

    Profiles have no inheritance or merge semantics. Unknown fields, an invalid
    discriminator and foreign settings raise Pydantic ValidationError; file and
    YAML errors propagate unchanged. Valid settings do not establish execution
    support or the presence of an SDK.
    """
    source = (
        _resolve_package_reference(Path("<world>"), path)
        if isinstance(path, str) and path.startswith("package:")
        else Path(path).resolve()
    )
    return read_validated_yaml(source, _WorldProfile).root


def load_run(path: str | Path) -> RunConfiguration:
    """Load a scenario and its recursive physical composition without SDKs.

    :param path: Scenario file (cwd-relative if needed) or application-package URI.
    :returns: New RunConfiguration preserving local instance names and hierarchy.
        Referenced world/system YAML uses ``package://robo_arch/...`` resources.
    :raises ValueError: Recursive inclusions, duplicate instance names in a
        system, duplicate YAML keys or invalid resource references.

    File/YAML/schema errors propagate. Device assets, attachment frames and
    controller/task parameter semantics are checked later by their owners.
    Reusing a child-system file creates distinct records and parameter mappings;
    no runtime devices are created. Treat returned nested dictionaries as read-only.

    Example::

        run = load_run("package://robo_arch/scenarios/arm_tracking/scenario.yaml")
        scene = run.scene
        print(run.world, run.duration)
    """
    source = (
        _resolve_package_reference(Path("<run>"), path)
        if isinstance(path, str) and path.startswith("package:")
        else Path(path).resolve()
    )
    scenario = read_validated_yaml(source, _Scenario)
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
        selections: dict[str, _InstanceSelection],
    ) -> RobotSystem:
        if file in ancestors:
            chain = " -> ".join(str(item) for item in (*ancestors, file))
            raise ValueError(f"Recursive robot system inclusion: {chain}")
        system = read_validated_yaml(file, _System)
        names = [*system.robots, *system.sensors, *system.systems]
        if len(set(names)) != len(names):
            raise ValueError(f"Device and child-system names must be unique in {file}")
        result = RobotSystem(
            name=name,
            source=file,
            pose=pose,
            robots=tuple(
                RobotInstance(
                    name=name,
                    model=robot.model,
                    pose=robot.pose,
                    initial_positions=robot.initial_positions,
                    parent=robot.parent,
                    mount_frame=robot.mount_frame,
                    binding=robot.binding,
                    calibration_source=robot.calibration,
                    mounting_revision=robot.mounting_revision,
                    calibration=(
                        read_validated_yaml(
                            resolve_resource(robot.calibration), CalibrationProfile
                        )
                        if robot.calibration
                        else None
                    ),
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
                    binding=sensor.binding,
                    calibration_source=sensor.calibration,
                    mounting_revision=sensor.mounting_revision,
                    calibration=(
                        read_validated_yaml(
                            resolve_resource(sensor.calibration), CalibrationProfile
                        )
                        if sensor.calibration
                        else None
                    ),
                )
                for name, sensor in system.sensors.items()
            ),
            systems=tuple(
                load_system(
                    _resolve_package_reference(file, child.definition),
                    name,
                    child.pose,
                    (*ancestors, file),
                    child.instances,
                )
                for name, child in system.systems.items()
            ),
        )
        remaining = set(selections)

        def select(system: RobotSystem, prefix: str) -> RobotSystem:
            def device(item):
                key = prefix + item.name
                if key not in selections:
                    return item
                remaining.remove(key)
                selection = selections[key]
                return replace(
                    item,
                    binding=selection.binding,
                    calibration_source=selection.calibration,
                    mounting_revision=selection.mounting_revision,
                    calibration=(
                        read_validated_yaml(
                            resolve_resource(selection.calibration), CalibrationProfile
                        )
                        if selection.calibration
                        else None
                    ),
                )

            return replace(
                system,
                robots=tuple(device(item) for item in system.robots),
                sensors=tuple(device(item) for item in system.sensors),
                systems=tuple(
                    select(child, prefix + child.name + "/") for child in system.systems
                ),
            )

        result = select(result, "")
        if remaining:
            raise ValueError(f"Unknown device instance selections: {sorted(remaining)}")
        return result

    robot_system = load_system(
        _resolve_package_reference(source, scenario.robot_system.definition),
        "",
        scenario.robot_system.pose,
        (),
        scenario.robot_system.instances,
    )

    return RunConfiguration(
        source=source,
        world_config=world_config,
        world_source=world_source,
        duration=scenario.duration,
        robot_system=robot_system,
        sensors_enabled=scenario.sensors_enabled,
        objects=tuple(
            ObjectInstance(
                name=name,
                model=obj.model,
                pose=obj.pose,
                motion=obj.motion,
                angular_velocity=obj.angular_velocity,
                linear_velocity=obj.linear_velocity,
            )
            for name, obj in scenario.objects.items()
        ),
        task=scenario.task,
        autonomy=scenario.robot_system.autonomy,
    )
