"""Mini45 joint reaction using Lab's child-side incoming-joint convention."""

from collections.abc import Callable
from typing import Any

import numpy as np

from robo_arch.core.worlds.urdf import sensor_link


def create(articulation_path: str) -> Any:
    """Construct before Lab initializes physics; no SDK import for declarations."""
    from isaaclab.sensors import JointWrenchSensor, JointWrenchSensorCfg

    return JointWrenchSensor(JointWrenchSensorCfg(prim_path=articulation_path))


def bind(sensor: Any, *, name: str) -> Callable[[], np.ndarray]:
    """Return owned [environment, force/torque] arrays at the child joint anchor."""
    link = sensor_link(name, "tool")

    def sample() -> np.ndarray:
        names = tuple(sensor.body_names)
        if link not in names:
            raise ValueError(f"Mini45 sensing link {link} was lost during conversion")
        index = names.index(link)
        result = np.concatenate(
            (
                sensor.data.force.torch[:, index].cpu().numpy(),
                sensor.data.torque.torch[:, index].cpu().numpy(),
            ),
            axis=-1,
        ).astype(float)
        if result.ndim != 2 or result.shape[1] != 6 or not np.isfinite(result).all():
            raise RuntimeError(f"Invalid wrench for {name}")
        return result

    return sample
