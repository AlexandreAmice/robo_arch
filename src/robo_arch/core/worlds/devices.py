"""Look up YAML-selected device packages without importing simulator SDKs."""

import re
from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
from types import ModuleType
from typing import Literal

from robo_arch.core.config.declarations import (
    ObjectDefinition,
    RobotDefinition,
    SceneConfiguration,
    SensorDefinition,
)
from robo_arch.core.config.loading import load_robot, resolve_resource
from robo_arch.core.worlds.assembly import resolve_devices


@dataclass(frozen=True, kw_only=True)
class DeviceDefinitions:
    """Metadata keyed by model identifier, shared by instances of that model.

    :param robots: RobotDefinition mapping loaded from robot YAML.
    :param sensors: SensorDefinition mapping from declaration modules.
    :param objects: ObjectDefinition mapping from declaration modules.

    No runtime state is stored. Frozen records contain mutable dictionaries;
    treat these mappings as read-only after loading.
    """

    robots: dict[str, RobotDefinition]
    sensors: dict[str, SensorDefinition]
    objects: dict[str, ObjectDefinition]


def _identifier(value: str, label: str) -> str:
    if re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", value) is None:
        raise ValueError(f"Invalid {label} identifier: {value!r}")
    return value


def load_device_module(
    category: Literal["robots", "sensors", "objects"], model: str, module: str
) -> ModuleType:
    """Import a conventional device module; YAML supplies only the model name.

    Sensor/object declaration modules expose describe(); sensor observation
    adapters expose native construction functions. Robot assets are declared
    in YAML and loaded by the world, without importing a robot module.

    :param category: Owning package category: robots, sensors or objects.
    :param model: Model identifier, beginning with a letter and otherwise using
        letters, digits and underscores.
    :param module: Single module identifier with the same restrictions.
    :returns: Imported module, reused from Python's module cache on repeated calls.
    :raises ValueError: Invalid model or module identifier.

    Import errors propagate. Selecting ``definition`` is SDK-independent by
    convention; selecting a runtime adapter can import its SDK.
    """
    for label, value in ((f"{category} model", model), ("device module", module)):
        _identifier(value, label)
    return import_module(f"robo_arch.{category}.{model}.{module}")


def _describe[T](
    category: Literal["robots", "sensors", "objects"], model: str, expected: type[T]
) -> T:
    definition = load_device_module(category, model, "definition").describe()
    if not isinstance(definition, expected):
        raise TypeError(
            f"{category}/{model}.describe() must return {expected.__name__}"
        )
    return definition


def load_definitions(scene: SceneConfiguration, world: str) -> DeviceDefinitions:
    """Load selected metadata and check declared world support before assembly.

    :param scene: Physical instances and observation selection.
    :param world: Requested world name, normally drake, isaac or real.
    :returns: Model-keyed DeviceDefinitions, with each selected model loaded once.
    :raises ValueError: Invalid identifiers/attachments or unsupported world/asset.
    :raises TypeError: A declaration module returns an unexpected metadata type.
    :raises FileNotFoundError: Missing robot declaration/asset or resource file.

    Sensor settings undergo their declared schema validation; observation support
    is required only when enabled, but physical support is always required.
    Robot support currently requires URDF plus Drake/Isaac. Metadata imports do
    not start SDKs; file, import and schema errors propagate. Asset contents and
    actual SDK availability are not validated here.
    """
    devices = resolve_devices(scene)
    definitions = DeviceDefinitions(
        robots={
            model: load_robot(
                "package://robo_arch/robots/"
                + _identifier(model, "robots model")
                + "/robot.yaml"
            )
            for model in dict.fromkeys(robot.model for robot in devices.robots)
        },
        sensors={
            model: _describe("sensors", model, SensorDefinition)
            for model in dict.fromkeys(sensor.model for sensor in devices.sensors)
        },
        objects={
            model: _describe("objects", model, ObjectDefinition)
            for model in dict.fromkeys(obj.model for obj in scene.objects)
        },
    )
    for robot in devices.robots:
        asset = definitions.robots[robot.model].asset
        # Both implemented robot import paths consume fixed-base effort URDFs.
        # Hardware requires a driver; an asset alone does not supply one.
        if world not in ("drake", "isaac") or Path(asset).suffix != ".urdf":
            raise ValueError(
                f"Robot {robot.name} has no {world} implementation for asset {asset}"
            )
        if not resolve_resource(asset).is_file():
            raise FileNotFoundError(f"Robot {robot.name} asset does not exist: {asset}")
    for sensor in devices.sensors:
        definition = definitions.sensors[sensor.model]
        if world not in definition.physical_worlds:
            raise ValueError(
                f"Sensor {sensor.name} has no {world} physical implementation"
            )
        if scene.sensors_enabled and world not in definition.supported_worlds:
            raise ValueError(f"Sensor {sensor.name} has no {world} implementation")
        definition.parameter_schema.model_validate(sensor.parameters)
    for obj in scene.objects:
        if world not in definitions.objects[obj.model].supported_worlds:
            raise ValueError(f"Object {obj.name} has no {world} implementation")
    return definitions
