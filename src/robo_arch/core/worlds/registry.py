"""Trusted device factories and model metadata, inspectable without SDK imports."""

import re
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from importlib import import_module

from robo_arch.core.config.loading import RunConfiguration
from robo_arch.core.config.parameters import Parameters


@dataclass(frozen=True, kw_only=True)
class FactoryReference:
    """A trusted Python factory imported only when its world is constructed.

    Each owner documents its factory signature. YAML selects registered names;
    it cannot supply executable import paths.
    """

    module: str
    attribute: str

    def load(self) -> Callable[..., object]:
        factory = getattr(import_module(self.module), self.attribute)
        if not callable(factory):
            raise TypeError(f"Factory {self.module}:{self.attribute} is not callable")
        return factory


@dataclass(frozen=True, kw_only=True)
class RobotDefinition:
    base_frame: str
    joints: tuple[str, ...]
    default_positions: tuple[float, ...]
    # Factories add a named model; world assembly owns placement.
    implementations: Mapping[str, FactoryReference]


@dataclass(frozen=True, kw_only=True)
class SensorDefinition:
    parameter_schema: type[Parameters]
    implementations: Mapping[str, FactoryReference]


@dataclass(frozen=True, kw_only=True)
class ObjectDefinition:
    package: str
    resource: str
    base_frame: str


@dataclass(frozen=True, kw_only=True)
class Registry:
    """Definitions selected from the installed device packages for one run."""

    robots: Mapping[str, RobotDefinition]
    sensors: Mapping[str, SensorDefinition]
    objects: Mapping[str, ObjectDefinition]


def _definition[T](category: str, name: str, expected: type[T]) -> T:
    if re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name) is None:
        raise ValueError(f"Invalid {category} model identifier: {name!r}")
    module_name = f"robo_arch.{category}.{name}.definition"
    try:
        module = import_module(module_name)
    except ModuleNotFoundError as error:
        # Preserve errors for a broken installed definition's own dependencies.
        if error.name == module_name or module_name.startswith(f"{error.name}."):
            raise ValueError(f"Unknown {category} model: {name}") from error
        raise
    definition = getattr(module, "DEFINITION", None)
    if not isinstance(definition, expected):
        raise ValueError(f"{module_name}.DEFINITION must be a {expected.__name__}")
    return definition


def discover(run: RunConfiguration) -> Registry:
    """Load only selected declarations, then validate support without SDK imports.

    Model identifiers select ``robo_arch.<category>.<model>.definition:DEFINITION``.
    They cannot provide import paths. Factories remain lazy until world assembly.
    """
    from robo_arch.core.worlds.selection import validate_devices

    registry = Registry(
        robots={
            name: _definition("robots", name, RobotDefinition)
            for name in dict.fromkeys(robot.model for robot in run.robots)
        },
        sensors={
            name: _definition("sensors", name, SensorDefinition)
            for name in dict.fromkeys(sensor.model for sensor in run.sensors)
        },
        objects={
            name: _definition("objects", name, ObjectDefinition)
            for name in dict.fromkeys(obj.model for obj in run.objects)
        },
    )
    validate_devices(run, registry)
    return registry
