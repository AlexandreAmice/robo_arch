"""Physical topology, compound nominal state and calibrated instance isolation."""

from dataclasses import replace

import numpy as np
import pytest

from robo_arch.core.config.declarations import CalibrationProfile, DeviceBinding, Pose
from robo_arch.core.config.loading import load_run
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.devices import load_definitions
from robo_arch.core.worlds.drake.scene import build_mechanism_model
from robo_arch.core.worlds.urdf import compose_mechanism


def run():
    return load_run("package://robo_arch/scenarios/grasping/scenario.yaml")


def test_connected_model_preserves_tool_motion_inertia_and_vector_ownership(tmp_path):
    selected = run()
    devices = resolve_devices(selected.scene)
    (mechanism,) = devices.mechanisms
    definitions = load_definitions(selected.scene, "drake")
    model = build_mechanism_model(mechanism, definitions)
    assert model.plant.num_positions() == model.plant.num_actuated_dofs() == 8
    assert model.indices["arm"].q == tuple(range(6))
    assert model.indices["gripper"].q == (6, 7)
    context = model.plant.CreateDefaultContext()
    q = np.r_[definitions.robots["ur7e"].default_positions, [-0.045, 0.045]]
    model.plant.SetPositions(context, q)
    body = model.plant.GetFrameByName("body", model.robots["gripper"])
    flange = model.plant.GetFrameByName("flange", model.robots["arm"])
    mount = model.plant.CalcRelativeTransform(context, flange, body).GetAsMatrix4()
    first = body.CalcPoseInWorld(context).translation()
    q[0] += 0.4
    q[6:] = [-0.01, 0.01]
    model.plant.SetPositions(context, q)
    np.testing.assert_allclose(
        model.plant.CalcRelativeTransform(context, flange, body).GetAsMatrix4(),
        mount,
        atol=1e-12,
    )
    assert np.linalg.norm(body.CalcPoseInWorld(context).translation() - first) > 0.1
    mass = model.plant.CalcMassMatrix(context)
    assert np.linalg.eigvalsh(mass).min() > 0
    assert np.linalg.norm(mass[:6, 6:]) > 0  # Tool accelerations couple into the arm.
    names = compose_mechanism(
        mechanism, devices.sensors, definitions, tmp_path / "mechanism.urdf"
    )
    assert len(set((*names["arm"], *names["gripper"]))) == 8
    assert "robot_" in names["gripper"][0]


@pytest.mark.parametrize(
    "parent, message", [("missing/body", "parent"), ("gripper/body", "Cyclic")]
)
def test_attachment_failures(parent, message):
    selected = run()
    arm, grip = selected.robot_system.robots
    grip = replace(grip, parent=parent)
    scene = replace(
        selected.scene, robot_system=replace(selected.robot_system, robots=(arm, grip))
    )
    with pytest.raises(ValueError, match=message):
        resolve_devices(scene)


def test_calibration_identity_and_mount_revision_checked_without_sdk():
    selected = run()
    arm, grip = selected.robot_system.robots
    arm = replace(arm, binding=DeviceBinding(identity="test-arm"))
    profile = CalibrationProfile(
        identity="test-gripper",
        parent_identity="test-arm",
        parent_frame="flange",
        child_frame="body",
        mounting_revision="synthetic-v1",
        kind="synthetic",
        pose=Pose(translation=(0, 0, 0.06)),
    )
    grip = replace(
        grip,
        binding=DeviceBinding(identity="test-gripper"),
        calibration=profile,
        mounting_revision="synthetic-v1",
    )
    scene = replace(
        selected.scene, robot_system=replace(selected.robot_system, robots=(arm, grip))
    )
    assert resolve_devices(scene).robots[1].calibration == profile
    bad = replace(grip, binding=DeviceBinding(identity="other-gripper"))
    with pytest.raises(ValueError, match="Calibration identity"):
        resolve_devices(
            replace(scene, robot_system=replace(scene.robot_system, robots=(arm, bad)))
        )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
