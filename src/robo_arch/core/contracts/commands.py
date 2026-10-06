"""Command semantics, checked before constructing a world transport."""

from dataclasses import dataclass
from enum import StrEnum


class CommandKind(StrEnum):
    """Distinct physical commands; adapters must not convert between these."""

    # Complete generalized effort at each joint, including nominal gravity.
    # This is not a gravity-compensated residual command or motor current.
    JOINT_EFFORT = "joint_effort"
    JOINT_POSITION_TRAJECTORY = "joint_position_trajectory"


@dataclass(frozen=True)
class CommandCapabilities:
    """Commands whose units and semantics a selected deployment supports."""

    kinds: frozenset[CommandKind]

    def require(self, kind: CommandKind) -> None:
        if kind not in self.kinds:
            raise ValueError(
                f"Unsupported command {kind}; available: {sorted(self.kinds)}"
            )
