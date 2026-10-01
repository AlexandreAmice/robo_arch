"""Mini45 fixed-joint reaction in PhysX's child joint frame."""

import numpy as np

from robo_arch.core.worlds.urdf import sensor_link


def bind(articulation, *, name: str):
    """Bind by link identity, never by actuated-DOF index."""
    link = sensor_link(name, "tool")
    names = tuple(articulation.shared_metatype.link_names)
    if link not in names:
        raise ValueError(f"Mini45 sensing link {link} was lost during conversion")
    index = names.index(link)

    def sample() -> np.ndarray:
        # PhysX orders force before torque, expressed at the child joint frame.
        result = articulation.get_link_incoming_joint_force()[0, index].astype(float)
        if result.shape != (6,) or not np.isfinite(result).all():
            raise RuntimeError(f"Invalid wrench for {name}")
        return result

    return sample
