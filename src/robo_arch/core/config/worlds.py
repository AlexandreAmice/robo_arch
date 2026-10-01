"""Select and validate a world configuration at the run composition boundary."""

from typing import Annotated

from pydantic import Field, TypeAdapter

from robo_arch.core.worlds.drake.config import DrakeWorld
from robo_arch.core.worlds.isaac.config import IsaacWorld
from robo_arch.core.worlds.real.config import RealWorld

WorldConfiguration = Annotated[
    DrakeWorld | IsaacWorld | RealWorld, Field(discriminator="type")
]
_WORLD_VALIDATOR = TypeAdapter(WorldConfiguration)


def parse_world(value: object) -> WorldConfiguration:
    """Validate one complete native world configuration, without importing SDKs."""
    return _WORLD_VALIDATOR.validate_python(value)
