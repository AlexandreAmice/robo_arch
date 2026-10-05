"""Scalar independent-model control through the shared native Isaac execution."""

import warnings

from robo_arch.core.worlds.drake.scene import build_mechanism_model
from robo_arch.robot_system.ur7e_wsg50.control import Controller


def configure(scene, reference):
    warnings.warn(
        "Mounted manipulation uses shared scalar CPU control with per-step transfers.",
        RuntimeWarning,
        stacklevel=2,
    )
    model = build_mechanism_model(scene.devices.mechanisms[0], scene.definitions)
    return {"arm": Controller(model, reference)}


def sample_objects(scene):
    """Export privileged ground truth in Drake q(wxyz,xyz),v(angular,linear) order."""
    state = (
        scene.native.rigid_objects["block"]
        .data.root_link_state_w.torch.detach()
        .cpu()
        .numpy()
    )
    return state[:, [6, 3, 4, 5, 0, 1, 2, 10, 11, 12, 7, 8, 9]].copy()
