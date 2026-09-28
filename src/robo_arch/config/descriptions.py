"""Unresolved, inspectable configuration descriptions.

These records preserve declared information; they do not validate references,
infer ports, choose implementations, or load YAML. Mapping values are owned by
the caller and must not be mutated while a description is being consumed.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

from pydantic import JsonValue

from robo_arch.contracts.components import (
    ComponentDeclaration,
    ImplementationRegistration,
    Parameters,
)


@dataclass(frozen=True, kw_only=True)
class FileReference:
    """Keep the declaring file so S0 never resolves against the process cwd."""

    path: str
    declared_in: Path
    version: str | None = None


@dataclass(frozen=True, kw_only=True)
class CalibrationReference:
    """Measured calibration tied to particular devices and mounting setup."""

    data: FileReference
    installation: str
    device_ids: tuple[str, ...]
    mounting_arrangement: str


@dataclass(frozen=True, kw_only=True)
class ScenarioDescription:
    robot: FileReference
    sensors: FileReference
    objects: FileReference
    task: FileReference
    layout: FileReference
    calibration: tuple[CalibrationReference, ...] = ()


@dataclass(frozen=True, kw_only=True)
class ChildPort:
    """A direct child's port; that child may itself expose a composition port."""

    child: str
    port: str


@dataclass(frozen=True, kw_only=True)
class Connection:
    source: ChildPort
    destination: ChildPort


@dataclass(frozen=True, kw_only=True)
class ExposedInput:
    name: str
    destinations: tuple[ChildPort, ...]


@dataclass(frozen=True, kw_only=True)
class ExposedOutput:
    name: str
    source: ChildPort


@dataclass(frozen=True, kw_only=True)
class InstanceDescription:
    """A leaf (including a monolithic policy), composition, or file reference.

    Parameters are explicit values; defaults belong to the definition's schema.
    Variant selection is explicit and defaults to the registration named default.
    """

    name: str
    definition: ComponentDeclaration | CompositionDescription | FileReference
    parameters: Mapping[str, JsonValue] = field(default_factory=dict)
    variant: str = "default"


@dataclass(frozen=True, kw_only=True)
class CompositionDescription:
    """Export types derive from children; there is no duplicate port schema.

    Optional registrations replace the entire subsystem with a named variant.
    They must preserve the recursively derived public interface. S0 owns checks.
    """

    identifier: str
    parameter_schema: type[Parameters]
    children: tuple[InstanceDescription, ...]
    connections: tuple[Connection, ...]
    inputs: tuple[ExposedInput, ...]
    outputs: tuple[ExposedOutput, ...]
    implementations: tuple[ImplementationRegistration, ...] = ()
