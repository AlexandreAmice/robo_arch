"""S0 output records. Creating these records does not perform resolution.

Parameters and mappings retain caller ownership; consumers must not mutate a
resolved configuration. Instance paths are tuples of names, avoiding ambiguous
separator escaping. Connections and exports carry full paths within the tree.
"""

from __future__ import annotations

from dataclasses import dataclass

from robo_arch.config.descriptions import CompositionDescription, ScenarioDescription
from robo_arch.contracts.components import (
    ComponentDeclaration,
    ImplementationRegistration,
    Parameters,
)
from robo_arch.contracts.ports import PortDescription


@dataclass(frozen=True, kw_only=True)
class ResolvedPort:
    instance_path: tuple[str, ...]
    name: str
    description: PortDescription


@dataclass(frozen=True, kw_only=True)
class ResolvedConnection:
    source: ResolvedPort
    destination: ResolvedPort


@dataclass(frozen=True, kw_only=True)
class ResolvedInput:
    name: str
    description: PortDescription
    destinations: tuple[ResolvedPort, ...]


@dataclass(frozen=True, kw_only=True)
class ResolvedOutput:
    name: str
    description: PortDescription
    source: ResolvedPort


@dataclass(frozen=True, kw_only=True)
class ResolvedComponent:
    instance_path: tuple[str, ...]
    declaration: ComponentDeclaration
    parameters: Parameters
    implementation: ImplementationRegistration
    inputs: tuple[ResolvedPort, ...]
    outputs: tuple[ResolvedPort, ...]


@dataclass(frozen=True, kw_only=True)
class ResolvedComposition:
    instance_path: tuple[str, ...]
    declaration: CompositionDescription
    parameters: Parameters
    children: tuple[ResolvedComponent | ResolvedComposition, ...]
    connections: tuple[ResolvedConnection, ...]
    inputs: tuple[ResolvedInput, ...]
    outputs: tuple[ResolvedOutput, ...]
    # None means assemble the children; otherwise construct the selected variant.
    implementation: ImplementationRegistration | None = None


@dataclass(frozen=True, kw_only=True)
class ResourceVersion:
    """Effective model/calibration identity captured by resolution."""

    resource: str
    version: str


@dataclass(frozen=True, kw_only=True)
class ResolvedConfiguration:
    scenario: ScenarioDescription
    world: str
    autonomy: ResolvedComponent | ResolvedComposition
    resources: tuple[ResourceVersion, ...]
