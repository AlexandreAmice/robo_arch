"""Batched CUDA filter parity, domain failures and arbitrary nominal wrapping."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("pydrake")

from pydrake.multibody.plant import MultibodyPlant  # noqa: E402
from pydrake.multibody.tree import (  # noqa: E402
    PrismaticJoint,
    SpatialInertia,
    UnitInertia,
)

from robo_arch.core.controllers.cbf.assembly import ProtectionGeometry  # noqa: E402
from robo_arch.core.controllers.cbf.definition import (  # noqa: E402
    CbfParameters,
    Plane,
    Sphere,
    SpherePair,
    SpherePlanePair,
)
from robo_arch.core.controllers.cbf.drake import SphereCbfFilter  # noqa: E402
from robo_arch.core.controllers.cbf.errors import CbfFailure  # noqa: E402
from robo_arch.core.controllers.cbf.tensor import TensorCbfFilter  # noqa: E402
from robo_arch.core.controllers.dynamics.drake import build_tensor_model  # noqa: E402

cuda_required = pytest.mark.skipif(
    not torch.cuda.is_available(), reason="CUDA required"
)


def filters(batch_size, velocity_limit=None):
    pytest.importorskip("moreau")
    from robo_arch.core.controllers.cbf.moreau import MoreauProjection

    model = MultibodyPlant(0.0)
    instance = model.AddModelInstance("robot")
    body = model.AddRigidBody(
        "body", instance, SpatialInertia(1.0, [0, 0, 0], UnitInertia(1, 1, 1))
    )
    joint = model.AddJoint(
        PrismaticJoint(
            "slide", model.world_frame(), body.body_frame(), [0, 0, 1], damping=0.5
        )
    )
    if velocity_limit is not None:
        joint.set_velocity_limits([-velocity_limit], [velocity_limit])
    model.AddJointActuator("motor", joint, 100)
    model.Finalize()
    geometry = ProtectionGeometry(
        spheres=(
            Sphere("camera", "robot/body", (0, 0, 0), 0.1),
            Sphere("obstacle", "world", (0, 0, 1), 0.1),
        ),
        pairs=(SpherePair("camera", "obstacle", margin=0),),
        planes=(Plane("floor", (0, 0, 1), 0),),
        plane_pairs=(SpherePlanePair("camera", "floor", 0.01),),
    )
    parameters = CbfParameters()
    cpu = SphereCbfFilter(
        model=model,
        joints=("slide",),
        spheres=geometry.spheres,
        pairs=geometry.pairs,
        planes=geometry.planes,
        plane_pairs=geometry.plane_pairs,
        parameters=parameters,
    )
    tensor_model = build_tensor_model(
        model,
        ("slide",),
        tuple(s.frame for s in geometry.spheres),
        tuple(s.center for s in geometry.spheres),
        device="cuda:0",
    )
    gpu = TensorCbfFilter(
        model=tensor_model,
        geometry=geometry,
        projection=MoreauProjection(
            count=1,
            constraint_count=cpu.qp_constraint_count,
            batch_size=batch_size,
            limits=tensor_model.limits,
            device="cuda:0",
        ),
        parameters=parameters,
    )
    return cpu, gpu


@cuda_required
def test_batch_filter_matches_drake_and_preserves_feasible_nominal():
    cpu, gpu = filters(3)
    states = np.array([[0.5, 0.1], [0.2, -0.1], [0.5, 0.0]])
    nominal = np.array([[80.0], [-80.0], [9.81]])
    actual = gpu.filter(
        torch.tensor(states, device="cuda"), torch.tensor(nominal, device="cuda")
    )
    for index in range(3):
        expected = cpu.filter(states[index], nominal[index])
        np.testing.assert_allclose(
            actual.effort[index].cpu(), expected.effort, atol=1e-7
        )
        # Timing fields differ by backend; all barrier values must agree.
        np.testing.assert_allclose(
            actual.diagnostics[index, :8].cpu(), expected.diagnostics[:8], atol=1e-7
        )
    assert torch.equal(actual.effort[2], torch.tensor(nominal[2], device="cuda"))
    assert actual.effort.device.type == "cuda"
    torch.testing.assert_close(
        gpu.wrap(lambda state, time: torch.tensor(nominal, device="cuda"))(
            torch.tensor(states, device="cuda"), 0.0
        ),
        actual.effort,
    )


@cuda_required
def test_bad_initial_environment_and_nonfinite_state_fail():
    _, gpu = filters(2)
    state = torch.tensor([[0.5, 0.0], [0.05, 0.0]], device="cuda", dtype=torch.float64)
    with pytest.raises(CbfFailure, match="initial state") as failure:
        gpu.validate_initial_state(state)
    assert failure.value.snapshot["environment_constraint_indices"] == [[1, 1]]
    state[1, 0] = float("nan")
    with pytest.raises(CbfFailure, match="nonfinite"):
        gpu.filter(state, torch.zeros((2, 1), device="cuda", dtype=torch.float64))


@cuda_required
def test_native_float32_commands_are_checked_after_quantization():
    _, gpu = filters(2)
    gpu.command_dtype = torch.float32
    state = torch.tensor(
        [[0.5003, 0.102], [0.2007, -0.101]], dtype=torch.float64, device="cuda"
    )
    nominal = torch.tensor([[80.0], [-80.0]], dtype=torch.float64, device="cuda")
    result = gpu.filter(state, nominal)
    assert torch.equal(result.effort, result.effort.float().double())
    rows = gpu.evaluate(state)
    residual = (
        torch.einsum("bpi,bi->bp", rows.coefficient, result.effort) + rows.constant
    )
    assert (residual >= -gpu.parameters.residual_tolerance).all()
    assert (result.effort.abs() <= gpu.model.limits).all()


def test_float32_guard_keeps_binding_floor_visible_in_active_diagnostics():
    model = MultibodyPlant(0.0)
    instance = model.AddModelInstance("robot")
    body = model.AddRigidBody(
        "body", instance, SpatialInertia(1.0, [0, 0, 0], UnitInertia(1, 1, 1))
    )
    joint = model.AddJoint(
        PrismaticJoint("slide", model.world_frame(), body.body_frame(), [0, 0, 1])
    )
    model.AddJointActuator("motor", joint, 1000)
    model.Finalize()
    geometry = ProtectionGeometry(
        spheres=(Sphere("camera", "robot/body", (0, 0, 0), 0.1),),
        pairs=(),
        planes=(Plane("floor", (0, 0, 1), 0), Plane("distant", (0, 0, 1), -10)),
        plane_pairs=(
            SpherePlanePair("camera", "floor", 0.01),
            SpherePlanePair("camera", "distant", 0.01),
        ),
    )

    class AnalyticProjection:
        def solve(self, coefficient, constant, nominal):
            # Unit-mass vertical slider: both plane rows impose scalar lower
            # bounds. Projection onto their intersection is analytic.
            lower = (-constant / coefficient[..., 0]).amax(dim=1, keepdim=True)
            return torch.maximum(nominal, lower)

    tensor_model = build_tensor_model(
        model, ("slide",), ("robot/body",), ((0, 0, 0),), device="cpu"
    )
    controller = TensorCbfFilter(
        model=tensor_model,
        geometry=geometry,
        projection=AnalyticProjection(),
        command_dtype=torch.float32,
    )
    state = torch.tensor([[0.5, 0.0], [0.5003, 0.102]], dtype=torch.float64)
    result = controller.filter(
        state, torch.tensor([[-100.0], [9.81]], dtype=torch.float64)
    )
    rows = controller.evaluate(state)
    physical_residual = (
        torch.einsum("bpi,bi->bp", rows.coefficient, result.effort) + rows.constant
    )
    tolerance = controller.parameters.residual_tolerance
    assert physical_residual[0, 0] > 10 * tolerance
    torch.testing.assert_close(result.diagnostics[:, 6:8], physical_residual)
    torch.testing.assert_close(
        result.diagnostics[:, 8:10],
        torch.tensor([[1.0, 0.0], [0.0, 0.0]], dtype=torch.float64),
    )
    assert torch.equal(result.effort, result.effort.float().double())


@cuda_required
def test_joint_velocity_bounds_match_cpu_and_reject_bad_initial_velocity():
    cpu, gpu = filters(2, velocity_limit=0.5)
    states = np.array([[0.5, -0.49], [0.5, 0.49]])
    nominal = np.array([[-80.0], [80.0]])
    result = gpu.filter(
        torch.tensor(states, device="cuda"), torch.tensor(nominal, device="cuda")
    )
    assert gpu.qp_constraint_count == 4
    for i, state in enumerate(states):
        expected = cpu.filter(state, nominal[i])
        np.testing.assert_allclose(result.effort[i].cpu(), expected.effort, atol=1e-7)
        acceleration = float(result.effort[i, 0]) - 9.81 - 0.5 * state[1]
        np.testing.assert_allclose(acceleration, np.sign(state[1]) * 0.2, atol=1e-7)
    states[1, 1] = 0.501
    with pytest.raises(CbfFailure, match="initial velocity"):
        gpu.validate_initial_state(torch.tensor(states, device="cuda"))
