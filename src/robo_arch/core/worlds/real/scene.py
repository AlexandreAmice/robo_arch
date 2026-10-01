"""Real installation assembly boundary; hardware adapters are not implemented."""

from typing import NoReturn

from robo_arch.core.config.declarations import SceneConfiguration
from robo_arch.core.worlds.real.config import RealWorld


def build_scene(scene: SceneConfiguration, config: RealWorld) -> NoReturn:
    """Reject execution rather than treating RViz as a hardware installation."""
    raise NotImplementedError("Real scene construction requires hardware adapters")
