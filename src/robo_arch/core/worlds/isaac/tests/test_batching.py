"""Command validation and selective reset at the shared Lab tensor boundary."""

from types import SimpleNamespace

import numpy as np
import pytest

from robo_arch.core.worlds.isaac.config import IsaacWorld


@pytest.mark.parametrize(
    "settings",
    [
        {"batch_size": 2},
        {"environment_spacing": 2},
        {"num_envs": 0},
        {"control_backend": "torch"},
        {"log_every_n_steps": 0},
    ],
)
def test_invalid_or_retired_batch_settings(settings):
    with pytest.raises(ValueError):
        IsaacWorld.model_validate(settings)


@pytest.fixture
def execution(monkeypatch):
    torch = pytest.importorskip("torch")
    wp = pytest.importorskip("warp")
    wp.init()
    from robo_arch.core.worlds.isaac import batched

    q = torch.zeros((2, 1))
    v, u = q.clone(), q.clone()

    def state(*, position, velocity, env_mask):
        mask = torch.from_numpy(env_mask.numpy())
        q[mask], v[mask] = position[mask], velocity[mask]

    def effort(*, value, env_mask=None):
        mask = slice(None) if env_mask is None else torch.from_numpy(env_mask.numpy())
        u[mask] = value[mask]

    arm = SimpleNamespace(
        write_joint_state_to_sim_mask=state,
        actuators=SimpleNamespace(
            target_command=SimpleNamespace(
                set_effort_mask=effort, set_effort_index=effort
            )
        ),
        data=SimpleNamespace(
            joint_pos=SimpleNamespace(torch=q), joint_vel=SimpleNamespace(torch=v)
        ),
    )
    scene = SimpleNamespace(
        observations={},
        world=IsaacWorld(num_envs=2, physics={"device": "cpu"}),
        native=SimpleNamespace(articulations={"arm": arm}, reset=lambda: None),
    )
    monkeypatch.setattr(
        batched,
        "initialize_scene",
        lambda _: {"arm": np.array([0.2], dtype=np.float32)},
    )
    return batched.BatchedExecution(scene), q, v, u


def test_selected_reset_preserves_other_state_commands_and_clocks(execution):
    import torch

    runtime, q, v, u = execution
    q[:] = torch.tensor([[0.6], [0.9]])
    v.fill_(0.4)
    runtime.times[:] = torch.tensor([1.0, 2.0])
    runtime.set_effort("arm", torch.tensor([[1.6], [2.9]], dtype=torch.float64))
    runtime.reset(torch.tensor([False, True]))
    torch.testing.assert_close(q, torch.tensor([[0.6], [0.2]]))
    torch.testing.assert_close(v, torch.tensor([[0.4], [0.0]]))
    torch.testing.assert_close(u, torch.tensor([[1.6], [0.0]]))
    torch.testing.assert_close(
        runtime.times, torch.tensor([1.0, 0.0], dtype=torch.float64)
    )
    with pytest.raises(ValueError, match="boolean"):
        runtime.reset(torch.tensor([1]))


def test_nonfinite_and_overflowing_commands_rejected(execution):
    import torch

    runtime, *_ = execution
    for value in (
        torch.full((2, 1), float("nan")),
        torch.zeros((1, 1)),
        torch.full((2, 1), 1e300, dtype=torch.float64),
    ):
        with pytest.raises(ValueError):
            runtime.set_effort("arm", value)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
