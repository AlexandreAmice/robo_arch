"""Minimal construction boundaries, with no world execution or scheduler.

Opaque handles/services are owned by a selected world and borrowed by factories.
They may contain SDK objects only after world construction. Controller models
are supplied separately from scene state to avoid accidental ground-truth access.
Factories must request any privileged observations through explicit bindings.
"""

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Protocol

from robo_arch.config.resolved import (
    ResolvedComponent,
    ResolvedComposition,
    ResolvedConfiguration,
    ResolvedPort,
)
from robo_arch.contracts.ports import PortDescription


@dataclass(frozen=True, kw_only=True)
class WorldPort:
    name: str
    description: PortDescription
    handle: object


@dataclass(frozen=True, kw_only=True)
class MeasurementBinding:
    source: WorldPort
    destinations: tuple[ResolvedPort, ...]


@dataclass(frozen=True, kw_only=True)
class CommandBinding:
    source: ResolvedPort
    destination: WorldPort


@dataclass(frozen=True, kw_only=True)
class ScenarioRealization:
    """World I/O and explicit shared services, without exposing raw scene state.

    Service keys and handle types are specified by the selected world. The I0
    interface deliberately does not promise cross-SDK compatibility for them.
    """

    measurements: tuple[WorldPort, ...]
    commands: tuple[WorldPort, ...]
    controller_models: Mapping[str, object]
    services: Mapping[str, object]


@dataclass(frozen=True, kw_only=True)
class ConstructionContext:
    configuration: ResolvedConfiguration
    scenario: ScenarioRealization
    measurements: tuple[MeasurementBinding, ...]
    commands: tuple[CommandBinding, ...]


class ComponentFactory(Protocol):
    def __call__(
        self,
        *,
        instance: ResolvedComponent | ResolvedComposition,
        context: ConstructionContext,
    ) -> object:
        """Build a selected instance; returned objects belong to this world."""
        ...
