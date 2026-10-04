"""Task validation shared by native ports and batched callbacks."""

from dataclasses import replace

import numpy as np
import pytest

from robo_arch.core.config.loading import load_run
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.devices import load_definitions
from robo_arch.scenarios.camera_protection.setup import prepare


@pytest.mark.parametrize(
    ("target", "values", "message"),
    [
        ("unsafe_target", [0.0] * 5, "joint order"),
        ("retreat_target", [0.0] * 7, "joint order"),
        ("unsafe_target", [float("nan")] * 6, "finite number"),
        ("retreat_target", [4.0] * 6, "joint limits"),
    ],
)
def test_shared_setup_rejects_invalid_motion_targets(target, values, message):
    run = load_run("package://robo_arch/scenarios/camera_protection/scenario.yaml")
    run = replace(
        run,
        task=run.task.model_copy(
            update={"parameters": {**run.task.parameters, target: values}}
        ),
    )
    with pytest.raises(ValueError, match=message):
        setup = prepare(
            run, resolve_devices(run.scene), load_definitions(run.scene, "drake")
        )
        setup.validate_targets(np.full(6, -3.0), np.full(6, 3.0))


def test_shared_setup_rejects_wrong_controlled_arm():
    run = load_run("package://robo_arch/scenarios/camera_protection/scenario.yaml")
    devices = resolve_devices(run.scene)
    devices = replace(devices, robots=(replace(devices.robots[0], name="other_arm"),))
    with pytest.raises(ValueError, match="selected single controlled arm"):
        prepare(run, devices, load_definitions(run.scene, "drake"))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
