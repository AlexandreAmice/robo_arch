"""Controller construction and gravity compensation without scene simulation."""

import pytest

pytest.importorskip("pydrake")

import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant

from robo_arch.core.controllers.joint_tracking.definition import JointTrackingParameters
from robo_arch.core.controllers.joint_tracking.drake import build, make_policy


@pytest.fixture
def model():
    plant = MultibodyPlant(0.0)
    Parser(plant).AddModelsFromString(
        """<robot name="pendulum">
        <link name="base"/>
        <link name="arm"><inertial><origin xyz="0 0 -0.5"/>
          <mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1"
            ixy="0" ixz="0" iyz="0"/></inertial></link>
        <joint name="pivot" type="revolute"><parent link="base"/>
          <child link="arm"/><axis xyz="0 1 0"/>
          <limit lower="-3" upper="3" effort="100" velocity="10"/>
        </joint></robot>""",
        "urdf",
    )
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    plant.AddJointActuator("pivot_motor", plant.GetJointByName("pivot"))
    plant.Finalize()
    return plant


def test_controller_compensates_gravity_at_desired_state(model):
    controller = build(
        model=model,
        parameters=JointTrackingParameters(kp=(100.0,), kd=(20.0,)),
        joints=("pivot",),
    )
    context = controller.CreateDefaultContext()
    state = [0.5, 0.0]
    controller.get_input_port_estimated_state().FixValue(context, state)
    controller.get_input_port_desired_state().FixValue(context, state)
    model_context = model.CreateDefaultContext()
    model.SetPositionsAndVelocities(model_context, state)
    torque = controller.get_output_port_control().Eval(context)
    np.testing.assert_allclose(
        torque, -model.CalcGravityGeneralizedForces(model_context)
    )


@pytest.mark.parametrize(
    ("joints", "gains", "message"),
    [
        (("wrong_joint",), (100.0,), "joint order"),
        (("pivot",), (100.0, 100.0), "gain count"),
    ],
)
def test_incompatible_model_is_rejected(model, joints, gains, message):
    with pytest.raises(ValueError, match=message):
        build(
            model=model,
            parameters=JointTrackingParameters(kp=gains, kd=gains),
            joints=joints,
        )


def test_unfinalized_model_is_rejected():
    with pytest.raises(ValueError, match="finalized"):
        build(
            model=MultibodyPlant(0.0),
            parameters=JointTrackingParameters(kp=(1.0,), kd=(1.0,)),
            joints=("pivot",),
        )


def test_external_state_adapter_matches_native_controller(model):
    parameters = JointTrackingParameters(kp=(100.0,), kd=(20.0,))
    reference = np.array([0.2, 0.1])
    command = make_policy(
        model=model,
        parameters=parameters,
        joints=("pivot",),
        desired_state=lambda time: reference,
    )
    native = build(model=model, parameters=parameters, joints=("pivot",))
    context = native.CreateDefaultContext()
    native.get_input_port_desired_state().FixValue(context, reference)
    for state in ([0.5, 0.0], [-0.2, 0.4], [0.5, 0.0]):
        native.get_input_port_estimated_state().FixValue(context, state)
        np.testing.assert_allclose(
            command(np.array(state), 0.1),
            native.get_output_port_control().Eval(context),
            rtol=0,
            atol=1e-12,
        )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
