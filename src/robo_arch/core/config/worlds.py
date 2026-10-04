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
    """Validate one complete world configuration without importing SDKs.

    :param value: Mapping with ``type`` equal to ``drake``, ``isaac`` or ``real``,
        or an instance of the corresponding configuration class.
    :returns: The selected world model, with its default fields populated.
    :raises pydantic.ValidationError: Unknown/missing type or invalid fields.

    No files are read, profiles merged or runtime support inferred.
    """
    return _WORLD_VALIDATOR.validate_python(value)
