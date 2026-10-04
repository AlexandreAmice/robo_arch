"""Reset requests fail before touching physics or constructing autonomy."""

from types import SimpleNamespace

import pytest

from robo_arch.core.worlds.isaac.scenario import Execution


@pytest.mark.parametrize("ids", [[-1], [2], [0, 0], [True], [0.5], ["0"]])
def test_invalid_reset_does_not_construct_or_change_controllers(ids):
    execution = Execution.__new__(Execution)
    execution.commands = [object(), object()]
    before = execution.commands.copy()
    with pytest.raises(ValueError, match="Reset IDs"):
        execution.reset(ids)
    assert execution.commands == before


def test_empty_reset_needs_no_sdk_or_controller():
    execution = Execution.__new__(Execution)
    execution.reset([])


def test_bad_replacement_commands_fail_before_physics_changes():
    execution = Execution.__new__(Execution)
    execution.commands = [{"arm": object()}, {"arm": object()}]
    before = execution.commands.copy()
    execution.initial = {"arm": [0.0]}
    execution.scene = SimpleNamespace()
    execution.configure = lambda scene: {"wrong_arm": object()}
    with pytest.raises(ValueError, match="Commands must name exactly"):
        execution.reset([0])
    assert execution.commands == before


def test_newton_observation_support_is_checked_without_sdk():
    from robo_arch.core.worlds.isaac.backend import validate_observations
    from robo_arch.core.worlds.isaac.config import IsaacPhysics, NewtonPhysics

    newton = NewtonPhysics(backend="newton")
    validate_observations(newton, ())
    validate_observations(IsaacPhysics(), ("load_cell",))
    with pytest.raises(ValueError, match="Newton sensor observations.*load_cell"):
        validate_observations(newton, ("load_cell",))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
