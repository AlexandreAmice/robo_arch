"""Packaged box model, inspectable without a simulator SDK."""

from robo_arch.core.worlds.registry import ObjectDefinition

DEFINITION = ObjectDefinition(
    package="robo_arch.objects.box", resource="model.sdf", base_frame="box"
)
