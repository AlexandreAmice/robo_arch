"""World and command capabilities determine implementations without SDK imports."""

from types import SimpleNamespace

import pytest

from robo_arch.core.contracts.commands import CommandCapabilities, CommandKind
from robo_arch.core.controllers.selection import select_controller
from robo_arch.core.worlds.isaac.config import IsaacWorld


def test_trajectory_transport_rejects_effort_algorithm():
    capabilities = CommandCapabilities(
        frozenset({CommandKind.JOINT_POSITION_TRAJECTORY})
    )
    with pytest.raises(ValueError, match="Unsupported command joint_effort"):
        select_controller(
            SimpleNamespace(type="real"), "joint_tracking", capabilities=capabilities
        )


def test_automatic_backend_and_metadata():
    world = IsaacWorld(physics={"solver": "pgs"}, num_envs=4)
    selected = select_controller(world, "cbf", batched=True)
    assert selected.describe() == {
        "algorithm": "cbf",
        "implementation": "tensor_moreau",
        "numerical_model": "jaxsim",
        "device": "cuda:0",
        "batched": True,
    }
    with pytest.warns(RuntimeWarning, match="Scalar CPU"):
        selected = select_controller(world, "joint_tracking")
    assert selected.device == "cpu" and not selected.batched


def test_missing_support_is_not_substituted():
    with pytest.raises(ValueError, match="only PhysX"):
        select_controller(
            IsaacWorld(physics={"backend": "newton"}), "cbf", batched=True
        )
    with pytest.raises(ValueError, match="Unsupported controller"):
        select_controller(SimpleNamespace(type="drake"), "unknown")
