"""Logical port semantics, independent of transport and simulator storage.

Symbolic dimensions (for example ``robot.num_joints``) are resolved by S0 after
model selection. These records describe values; they do not allocate arrays or
establish compatibility. A world adapter must honor the declared ownership.
"""

from dataclasses import dataclass
from enum import StrEnum


class CommandMode(StrEnum):
    POSITION = "position"
    VELOCITY = "velocity"
    EFFORT = "effort"


class ArrayOwnership(StrEnum):
    OWNED = "owned"
    BORROWED_READ_ONLY = "borrowed_read_only"


@dataclass(frozen=True, kw_only=True)
class TimestampConvention:
    """Clock identity and the event represented by a timestamp, in seconds."""

    clock: str
    event: str  # E.g. "measurement_capture" or "command_application".


@dataclass(frozen=True, kw_only=True)
class PortDescription:
    """A quantity's meaning, including joint order and optional command mode.

    ``None`` for robot/frame and an empty joint tuple mean not applicable, not
    wildcard compatibility. Joint names are ordered. A scalar has dimensions
    ``()``; a vector may use ``("robot.num_joints",)`` until model resolution.
    """

    quantity: str
    units: str
    dimensions: tuple[int | str, ...]
    timestamp: TimestampConvention
    ownership: ArrayOwnership
    robot: str | None = None
    joints: tuple[str, ...] = ()
    frame: str | None = None
    command_mode: CommandMode | None = None


@dataclass(frozen=True, kw_only=True)
class PortDeclaration:
    name: str
    description: PortDescription
