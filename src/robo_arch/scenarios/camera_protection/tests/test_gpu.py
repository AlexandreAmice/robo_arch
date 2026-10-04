"""GPU task reference and complete camera-model/filter integration."""

from dataclasses import replace

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("moreau")
pytest.importorskip("pydrake")

from robo_arch.core.config.loading import load_run, load_world  # noqa: E402
from robo_arch.core.controllers.dynamics.drake import build_tensor_model  # noqa: E402
from robo_arch.core.controllers.joint_tracking.drake import make_policy  # noqa: E402
from robo_arch.core.controllers.joint_tracking.torch import (  # noqa: E402
    TensorJointTracking,
)
from robo_arch.core.worlds.assembly import resolve_devices  # noqa: E402
from robo_arch.core.worlds.devices import load_definitions  # noqa: E402
from robo_arch.core.worlds.drake.scene import build_controller_model  # noqa: E402
from robo_arch.core.worlds.isaac.scene import IsaacScene  # noqa: E402
from robo_arch.scenarios.camera_protection.configuration import (  # noqa: E402
    parameters_for,
)
from robo_arch.scenarios.camera_protection.isaac import configure  # noqa: E402
from robo_arch.scenarios.camera_protection.reference import desired_state  # noqa: E402

pytestmark = pytest.mark.skipif(not torch.cuda.is_available(), reason="CUDA required")


def gpu_run(batch_size=2):
    run = load_run("package://robo_arch/scenarios/camera_protection/scenario.yaml")
    world = load_world("package://robo_arch/scenarios/camera_protection/isaac_gpu.yaml")
    return replace(
        run,
        world_config=world.model_copy(update={"batch_size": batch_size}),
        autonomy=run.autonomy.model_copy(
            update={
                "parameters": {**run.autonomy.parameters, "backend": "torch_moreau"}
            }
        ),
    )


def test_reference_and_inverse_dynamics_match_native_controller():
    run = gpu_run()
    control, task = parameters_for(run)
    devices = resolve_devices(run.scene)
    definitions = load_definitions(run.scene, "drake")
    robot = devices.robots[0]
    definition = definitions.robots[robot.model]
    initial = np.asarray(robot.initial_positions or definition.default_positions)
    model = build_controller_model(
        robot, definition, sensors=devices.sensors, definitions=definitions
    )
    tensor_model = build_tensor_model(
        model, definition.joints, ("world",), ((0, 0, 0),)
    )
    tracking = TensorJointTracking(tensor_model, control.nominal)
    times = np.array([0, 0.4, 0.8, 2.9, 3.0, 3.4, 3.8, 6.0])
    args = dict(
        retreat_time=task.retreat_time, transition_seconds=task.transition_seconds
    )
    cpu_q, cpu_v = desired_state(
        times,
        initial,
        np.asarray(task.unsafe_target),
        np.asarray(task.retreat_target),
        **args,
    )
    gpu_q, gpu_v = desired_state(
        torch.tensor(times, device="cuda"),
        torch.tensor(initial, device="cuda"),
        torch.tensor(task.unsafe_target, dtype=torch.float64, device="cuda"),
        torch.tensor(task.retreat_target, dtype=torch.float64, device="cuda"),
        namespace=torch,
        **args,
    )
    np.testing.assert_allclose(gpu_q.cpu(), cpu_q, atol=1e-14)
    np.testing.assert_allclose(gpu_v.cpu(), cpu_v, atol=1e-14)
    states = np.broadcast_to(
        np.r_[initial, np.linspace(-0.1, 0.1, 6)], (len(times), 12)
    ).copy()
    actual = tracking.effort(torch.tensor(states, device="cuda"), gpu_q, gpu_v)
    policy = make_policy(
        model=model,
        parameters=control.nominal,
        joints=definition.joints,
        desired_state=lambda t: np.concatenate(
            desired_state(
                np.asarray(t),
                initial,
                np.asarray(task.unsafe_target),
                np.asarray(task.retreat_target),
                **args,
            )
        ),
    )
    expected = np.array(
        [policy(state, time) for state, time in zip(states, times, strict=True)]
    )
    np.testing.assert_allclose(actual.cpu(), expected, atol=1e-10)


def test_camera_callback_keeps_state_effort_and_diagnostics_on_cuda():
    run = gpu_run()
    devices = resolve_devices(run.scene)
    scene = IsaacScene(
        None, None, {}, devices, load_definitions(run.scene, "drake"), run.scene
    )
    description = {}
    command = configure(scene, run=run, description=description)["arm"]
    initial = torch.tensor(
        devices.robots[0].initial_positions
        or scene.definitions.robots[devices.robots[0].model].default_positions,
        dtype=torch.float64,
        device="cuda",
    )
    state = torch.cat((initial, torch.zeros_like(initial)))[None].repeat(2, 1)
    effort = command(state, torch.zeros(2, dtype=torch.float64, device="cuda"))
    assert effort.shape == (2, 6) and effort.device.type == "cuda"
    assert all(value.device.type == "cuda" for value in command.diagnostics.values())
    assert len(description["pair_names"]) == 128
    assert torch.isfinite(effort).all()


def test_mounted_camera_batch_constraints_match_drake_random_states():
    from robo_arch.core.controllers.cbf.assembly import resolve_geometry
    from robo_arch.core.controllers.cbf.drake import build_filter as drake_filter
    from robo_arch.core.controllers.cbf.isaac import build_filter as tensor_filter

    run = gpu_run(batch_size=13)
    control, _ = parameters_for(run)
    devices = resolve_devices(run.scene)
    definitions = load_definitions(run.scene, "drake")
    robot = devices.robots[0]
    definition = definitions.robots[robot.model]
    model = build_controller_model(
        robot, definition, sensors=devices.sensors, definitions=definitions
    )
    geometry = resolve_geometry(run.scene, control, ground=True)
    cpu = drake_filter(
        model=model,
        joints=definition.joints,
        geometry=geometry,
        parameters=control.model_copy(update={"backend": "drake"}),
    )
    gpu = tensor_filter(
        model=model,
        joints=definition.joints,
        geometry=geometry,
        parameters=control,
        batch_size=13,
        command_dtype=None,
    )
    states = np.random.default_rng(73).uniform(-0.5, 0.5, (13, 12))
    states[:, :6] += definition.default_positions
    actual = gpu.evaluate(torch.tensor(states, device="cuda"))
    for index, state in enumerate(states):
        expected = cpu.evaluate(state)
        for name in ("coefficient", "constant", "clearance", "h", "psi1"):
            values = np.asarray([getattr(row, name) for row in expected])
            np.testing.assert_allclose(
                getattr(actual, name)[index].cpu(), values, atol=1e-9
            )
