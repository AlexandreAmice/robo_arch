"""Packaged geometry for a fixed box fixture."""

from robo_arch.core.config.declarations import ObjectDefinition


def describe() -> ObjectDefinition:
    return ObjectDefinition(
        package="robo_arch.objects.box",
        resource="model.sdf",
        base_frame="box",
        supported_worlds=("drake", "isaac"),
    )
