"""Base type for parameter schemas owned by controllers, sensors and tasks."""

from robo_arch.core.config.schema import Schema


class Parameters(Schema):
    """Owner-specific parameters using the common configuration validation policy."""
