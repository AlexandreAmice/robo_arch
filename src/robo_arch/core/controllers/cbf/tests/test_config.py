"""Protection declarations stay independent of nominal controllers and tasks."""

import pytest

from robo_arch.core.controllers.cbf.config import ProtectionParameters


def parameters(**changes):
    return ProtectionParameters.model_validate(
        {
            "profiles": {"device": "package://robo_arch/synthetic/protection.yaml"},
            "protected": ["device"],
            **changes,
        }
    )


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"protected": []}, "nonempty"),
        ({"protected": ["device", "device"]}, "unique"),
        ({"protected": ["missing"]}, "needs a sphere profile"),
        ({"profiles": {"device": "../protection.yaml"}}, "package://"),
        ({"margin": -0.1}, "greater than or equal"),
        ({"alpha1": 0}, "greater than"),
        ({"alpha2": float("inf")}, "finite"),
        ({"velocity_limit_gain": 0}, "greater than"),
        ({"velocity_limit_gain": float("nan")}, "finite"),
        ({"residual_tolerance": float("nan")}, "finite"),
        ({"robot": "arm"}, "Extra inputs"),
        ({"backend": "unknown"}, "Extra inputs"),
        ({"nominal_controller": "joint_tracking"}, "Extra inputs"),
        ({"nominal": {"kp": [1], "kd": [1]}}, "Extra inputs"),
        ({"unsafe_target": [1]}, "Extra inputs"),
    ],
)
def test_protection_only_configuration_validation(changes, message):
    with pytest.raises(ValueError, match=message):
        parameters(**changes)


def test_defaults_preserve_existing_barrier_tuning():
    selected = parameters()
    assert selected.margin == 0.01
    assert selected.alpha1 == selected.alpha2 == 5
    assert selected.velocity_limit_gain == 20
    assert selected.residual_tolerance == 1e-6
    assert selected.exclude_frames == ()
    assert selected.protected == ("device",)
    assert not selected.compile_model
    assert parameters(compile_model=True).compile_model


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
