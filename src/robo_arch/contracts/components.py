"""Declarations can be inspected without loading any implementation module."""

from collections.abc import Callable
from dataclasses import dataclass
from enum import StrEnum
from importlib import import_module

from pydantic import BaseModel, ConfigDict

from robo_arch.contracts.ports import PortDeclaration


class Parameters(BaseModel):
    """Base for component-specific schemas with explicit fields and defaults.

    S0 validates effective values using ``model_validate``. Frozen fields do not
    make nested containers immutable; callers own them and must not mutate a
    configuration after resolution. Prefer tuples for sequence parameters.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)


class ExecutionKind(StrEnum):
    SCALAR = "scalar"
    CPU_BATCH = "cpu_batch"
    TENSOR = "tensor"


@dataclass(frozen=True, kw_only=True)
class Transfer:
    source_device: str
    destination_device: str
    quantity: str
    synchronizes: bool


@dataclass(frozen=True, kw_only=True)
class TimingRequirements:
    """Implementation requirements, not instructions for a universal scheduler.

    A sampled implementation states a positive period in seconds. Other trigger
    names are implementation/world-specific. ``immediate_inputs`` identifies
    inputs with same-step output dependencies for later cycle checks.
    """

    trigger: str
    period_seconds: float | None
    clock: str
    immediate_inputs: tuple[str, ...] = ()


@dataclass(frozen=True, kw_only=True)
class ResetRequirements:
    initialization_required: bool
    reset_supported: bool
    selective_reset_supported: bool


@dataclass(frozen=True, kw_only=True)
class ImplementationCapabilities:
    """Declared support, not measured performance or automatic device fallback.

    Devices name supported execution devices (e.g. ``cpu`` or ``cuda``).
    Transfers describe known movement at the wrapper boundary. S0 will diagnose
    scalar/CPU work in a batched GPU world; declaring it here emits no warning.
    """

    execution: ExecutionKind
    devices: tuple[str, ...]
    transfers: tuple[Transfer, ...]
    timing: TimingRequirements
    reset: ResetRequirements


@dataclass(frozen=True, kw_only=True)
class FactoryReference:
    """Explicit import path; merely constructing this reference imports nothing.

    References belong to trusted Python declarations, not executable YAML.
    The selected world calls ``load`` at construction time. Factories follow
    ``config.world.ComponentFactory``; SDK objects remain world-owned.
    """

    module: str
    attribute: str

    def load(self) -> Callable[..., object]:
        factory = getattr(import_module(self.module), self.attribute)
        if not callable(factory):
            raise TypeError(f"Factory {self.module}:{self.attribute} is not callable")
        return factory


@dataclass(frozen=True, kw_only=True)
class ImplementationRegistration:
    world: str
    variant: str
    factory: FactoryReference
    capabilities: ImplementationCapabilities
    additional_requirements: tuple[str, ...] = ()
    approximation: str | None = None


@dataclass(frozen=True, kw_only=True)
class ComponentDeclaration:
    identifier: str
    parameter_schema: type[Parameters]
    inputs: tuple[PortDeclaration, ...]
    outputs: tuple[PortDeclaration, ...]
    implementations: tuple[ImplementationRegistration, ...]
