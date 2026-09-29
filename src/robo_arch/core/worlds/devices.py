"""Look up YAML-selected device packages without importing simulator SDKs."""

import re
from dataclasses import dataclass
from importlib import import_module
from types import ModuleType
from typing import Literal

from robo_arch.core.config.declarations import (
    ObjectDefinition,
    RobotDefinition,
    RunConfiguration,
    SensorDefinition,
)
from robo_arch.core.worlds.assembly import resolve_devices


@dataclass(frozen=True, kw_only=True)
class DeviceDefinitions:
    """Metadata shared by instances of each selected model; no runtime state."""

    robots: dict[str, RobotDefinition]
    sensors: dict[str, SensorDefinition]
    objects: dict[str, ObjectDefinition]


def load_device_module(
    category: Literal["robots", "sensors", "objects"], model: str, module: str
) -> ModuleType:
    """Import a conventional device module; YAML supplies only the model name.

    Declaration modules expose describe(). Drake robot adapters expose
    add_to_plant(), Drake sensor adapters add_to_builder(), and Isaac robot
    adapters add_to_stage(). World assembly calls these explicit entry points;
    their signatures use that world's native types.
    """
    for label, value in ((f"{category} model", model), ("device module", module)):
        if re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", value) is None:
            raise ValueError(f"Invalid {label} identifier: {value!r}")
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


def load_definitions(run: RunConfiguration) -> DeviceDefinitions:
    """Load selected metadata and check world support before constructing devices."""
    devices = resolve_devices(run)
    definitions = DeviceDefinitions(
        robots={
            model: _describe("robots", model, RobotDefinition)
            for model in dict.fromkeys(robot.model for robot in devices.robots)
        },
        sensors={
            model: _describe("sensors", model, SensorDefinition)
            for model in dict.fromkeys(sensor.model for sensor in devices.sensors)
        },
        objects={
            model: _describe("objects", model, ObjectDefinition)
            for model in dict.fromkeys(obj.model for obj in run.objects)
        },
    )
    for robot in devices.robots:
        if run.world not in definitions.robots[robot.model].supported_worlds:
            raise ValueError(f"Robot {robot.name} has no {run.world} implementation")
    for sensor in devices.sensors:
        definition = definitions.sensors[sensor.model]
        if run.world not in definition.supported_worlds:
            raise ValueError(f"Sensor {sensor.name} has no {run.world} implementation")
        definition.parameter_schema.model_validate(sensor.parameters)
    return definitions
