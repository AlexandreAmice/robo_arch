"""Base type for parameter schemas owned by controllers, sensors and tasks."""

from robo_arch.core.config.schema import Schema


class Parameters(Schema):
    """Base for controller, sensor and task parameter schemas.

    Subclasses declare their own fields and validation; this base adds no fields
    to :class:`robo_arch.core.config.schema.Schema`. Construct them directly or
    use ``model_validate(mapping)`` at a configuration boundary. Invalid settings
    raise Pydantic ValidationError. Importing a parameter schema must not require
    its owner's runtime SDK.
    """
