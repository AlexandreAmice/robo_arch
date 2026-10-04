"""Seeded goals, task completion and reset isolation without simulator imports."""

import pytest

from robo_arch.scenarios.batched_reaching.config import Reaching


@pytest.fixture
def torch():
    return pytest.importorskip("torch")


def make_task(torch):
    from robo_arch.scenarios.batched_reaching.task import Episodes

    initial = torch.zeros(4, 6)
    limits = torch.empty(4, 6, 2)
    limits[..., 0], limits[..., 1] = -0.1, 0.1
    return Episodes(initial, limits, Reaching(settle_seconds=0.1, episode_seconds=0.2))


def test_seed_limits_and_masked_reset(torch):
    task, other = make_task(torch), make_task(torch)
    assert torch.equal(task.targets, other.targets)
    assert not torch.equal(task.targets[0], task.targets[1])
    assert (task.targets.abs() <= 0.1).all()
    before = task.targets.clone()
    task.age.fill_(0.05)
    task.reset(torch.tensor([False, True, False, False]))
    assert torch.equal(task.targets[[0, 2, 3]], before[[0, 2, 3]])
    assert not torch.equal(task.targets[1], before[1])
    assert task.age.tolist() == pytest.approx([0.05, 0, 0.05, 0.05])


def test_success_timeout_counted_once_and_reset_independently(torch):
    task = make_task(torch)
    q = task.targets.clone()
    q[1:] += 0.3
    velocity = torch.zeros_like(q)
    for _ in range(5):
        done = task.observe(q, velocity, 0.05)
    assert done.all()
    assert task.successes.tolist() == [1, 0, 0, 0]
    assert task.timeouts.tolist() == [0, 1, 1, 1]
    task.reset(torch.tensor([True, False, False, False]))
    assert task.pending_timeout.tolist() == [False, True, True, True]
    assert task.successes.tolist() == [1, 0, 0, 0]


def test_nonfinite_state_remains_visible_after_reset(torch):
    task = make_task(torch)
    q = task.targets.clone()
    q[2, 1] = float("nan")
    task.observe(q, torch.zeros_like(q), 0.01)
    task.reset(torch.ones(4, dtype=torch.bool))
    assert task.invalid.tolist() == [False, False, True, False]


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
