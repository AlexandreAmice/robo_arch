"""Independent Drake parity for batched nominal dynamics and point kinematics."""

from importlib.util import find_spec

import numpy as np
import pytest
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import (
    FixedOffsetFrame,
    JacobianWrtVariable,
    MultibodyForces,
    PrismaticJoint,
    RevoluteJoint,
    SpatialInertia,
    UnitInertia,
)

if find_spec("torch") is not None:
    import torch

    from robo_arch.core.controllers.dynamics.drake import build_tensor_model
else:
    torch = None

pytestmark = pytest.mark.skipif(
    torch is None, reason="Requires the Torch vendor profile"
)


def synthetic_model(base_rpy=(0.0, 0.0, 0.0)):
    model = MultibodyPlant(0.0)
    instance = model.AddModelInstance("robot")
    inertia = SpatialInertia(2.0, [0.1, 0.02, -0.03], UnitInertia(0.2, 0.3, 0.4))
    base = model.AddRigidBody("base", instance, inertia)
    first = model.AddRigidBody("first", instance, inertia)
    second = model.AddRigidBody("second", instance, inertia)
    tip = model.AddRigidBody("tip", instance, inertia)
    camera = model.AddRigidBody("camera", instance, inertia)
    model.WeldFrames(
        model.world_frame(),
        base.body_frame(),
        RigidTransform(RollPitchYaw(base_rpy), [0.3, -0.4, 0.7]),
    )

    def offset(name, body, xyz, rpy):
        return model.AddFrame(
            FixedOffsetFrame(
                name, body.body_frame(), RigidTransform(RollPitchYaw(rpy), xyz)
            )
        )

    joint1 = model.AddJoint(
        RevoluteJoint(
            "shoulder",
            offset("parent", base, [0.2, 0.1, 0.4], [0.3, -0.2, 0.1]),
            offset("child", first, [-0.1, 0.03, 0.04], [0.1, 0.2, -0.3]),
            [0, 1, 0],
            damping=0.2,
        )
    )
    joint2 = model.AddJoint(
        PrismaticJoint(
            "slide",
            offset("slide_mount", first, [0.7, -0.1, 0.2], [0.2, 0.1, 0.4]),
            second.body_frame(),
            [0, 0, 1],
            damping=0.3,
        )
    )
    joint3 = model.AddJoint(
        RevoluteJoint(
            "wrist",
            second.body_frame(),
            offset("wrist_child", tip, [0.1, 0.2, 0.3], [-0.2, 0.3, 0.1]),
            [1, 0, 0],
            damping=0.4,
        )
    )
    model.WeldFrames(
        tip.body_frame(),
        camera.body_frame(),
        RigidTransform(RollPitchYaw([0.3, -0.4, 0.2]), [0.2, 0.07, 0.1]),
    )
    for joint in (joint1, joint2, joint3):
        actuator = model.AddJointActuator(joint.name() + "_motor", joint, 100)
        actuator.set_default_rotor_inertia(0.01)
        actuator.set_default_gear_ratio(2.0)
    model.Finalize()
    return model


def assert_parity(model, backend, states, frames, points):
    result = backend.evaluate(
        torch.tensor(states, device=backend.device, dtype=backend.dtype)
    )
    arrays = {
        name: getattr(result, name).cpu().numpy()
        for name in result.__dataclass_fields__
    }
    assert arrays["valid"].all()
    context = model.CreateDefaultContext()
    for batch, state in enumerate(states):
        model.SetPositionsAndVelocities(context, state)
        forces = MultibodyForces(model)
        model.CalcForceElementsContribution(context, forces)
        mass = model.CalcMassMatrix(context)
        bias = model.CalcInverseDynamics(context, np.zeros(backend.count), forces)
        np.testing.assert_allclose(arrays["mass"][batch], mass, atol=2e-12, rtol=2e-12)
        np.testing.assert_allclose(
            arrays["bias_force"][batch], bias, atol=2e-11, rtol=2e-12
        )
        np.testing.assert_allclose(
            arrays["acceleration_drift"][batch],
            np.linalg.solve(mass, -bias),
            atol=2e-11,
            rtol=2e-12,
        )
        np.testing.assert_allclose(
            arrays["acceleration_control"][batch],
            np.linalg.inv(mass),
            atol=2e-11,
            rtol=2e-12,
        )
        for i, (name, point) in enumerate(zip(frames, points, strict=True)):
            frame = (
                model.world_frame()
                if name == "world"
                else model.GetFrameByName(
                    name.rsplit("/", 1)[1],
                    model.GetModelInstanceByName(name.rsplit("/", 1)[0]),
                )
            )
            expected = model.CalcPointsPositions(
                context, frame, np.asarray(point), model.world_frame()
            ).ravel()
            jacobian = model.CalcJacobianTranslationalVelocity(
                context,
                JacobianWrtVariable.kV,
                frame,
                np.asarray(point),
                model.world_frame(),
                model.world_frame(),
            )
            bias_acceleration = model.CalcBiasTranslationalAcceleration(
                context,
                JacobianWrtVariable.kV,
                frame,
                np.asarray(point)[:, None],
                model.world_frame(),
                model.world_frame(),
            ).ravel()
            np.testing.assert_allclose(
                arrays["positions"][batch, i], expected, atol=2e-12
            )
            np.testing.assert_allclose(
                arrays["jacobians"][batch, i], jacobian, atol=2e-12
            )
            np.testing.assert_allclose(
                arrays["bias_accelerations"][batch, i], bias_acceleration, atol=2e-11
            )


@pytest.mark.parametrize("device", ["cpu", "cuda:0"])
@pytest.mark.parametrize("rpy", [(0.0, 0.0, 0.0), (0.3, -0.4, 0.7)])
def test_independent_dynamics_matches_drake_with_welded_payload(device, rpy):
    if device.startswith("cuda") and not torch.cuda.is_available():
        pytest.skip("CUDA is unavailable")
    model = synthetic_model(rpy)
    frames = ("robot/first", "robot/camera", "robot/slide_mount", "world")
    points = ((0.2, -0.3, 0.4), (0.07, 0.01, 0.02), (0.1, 0.2, 0.3), (1.0, 2.0, 3.0))
    backend = build_tensor_model(
        model, ("shoulder", "slide", "wrist"), frames, points, device=device
    )
    states = np.random.default_rng(431).uniform(-1.0, 1.0, (17, 6))
    assert_parity(model, backend, states, frames, points)
    # One environment's values and ordering are independent of batch membership.
    assert_parity(model, backend, states[[-1, 0, 5]], frames, points)


def test_model_rejects_joint_order_and_tensor_contract_errors():
    model = synthetic_model()
    with pytest.raises(ValueError, match="order"):
        build_tensor_model(
            model,
            ("slide", "shoulder", "wrist"),
            ("world",),
            ((0, 0, 0),),
            device="cpu",
        )
    backend = build_tensor_model(
        model, ("shoulder", "slide", "wrist"), ("world",), ((0, 0, 0),), device="cpu"
    )
    with pytest.raises(ValueError, match="shape"):
        backend.evaluate(torch.zeros(6, dtype=torch.float64))
    with pytest.raises(ValueError, match="dtype/device"):
        backend.evaluate(torch.zeros(1, 6, dtype=torch.float32))
    states = torch.zeros(2, 6, dtype=torch.float64)
    states[1, 0] = torch.nan
    result = backend.evaluate(states)
    assert result.valid.tolist() == [True, False]


def test_compiled_cuda_evaluations_match_drake_and_retain_owned_outputs():
    if not torch.cuda.is_available():
        pytest.skip("CUDA is unavailable")
    model = synthetic_model((0.3, -0.4, 0.7))
    frames = ("robot/first", "robot/camera", "world")
    points = ((0.2, -0.1, 0.3), (0.05, 0.02, 0.07), (1, 2, 3))
    backend = build_tensor_model(
        model, ("shoulder", "slide", "wrist"), frames, points, device="cuda:0"
    )
    states = np.random.default_rng(824).uniform(-1, 1, (3, 6))
    with torch.no_grad():
        first = backend.evaluate(torch.tensor(states, device="cuda:0"))
        saved = {
            name: getattr(first, name).clone() for name in first.__dataclass_fields__
        }
        # Exercise multiple graph replays and changed values without a new shape.
        for shift in (0.1, -0.2, 0.3, 0.0):
            assert_parity(model, backend, states + shift, frames, points)
        for name, expected in saved.items():
            torch.testing.assert_close(getattr(first, name), expected, rtol=0, atol=0)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
