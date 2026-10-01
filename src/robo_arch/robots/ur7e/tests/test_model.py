"""Check the packaged model's dynamics interface and independent instances."""

import pytest

pytest.importorskip("pydrake")

import numpy as np
from pydrake.multibody.plant import MultibodyPlant

from robo_arch.core.config.loading import load_robot
from robo_arch.core.worlds.drake.models import add_robot


def test_fixed_base_models_have_six_ordered_actuators_and_valid_dynamics():
    definition = load_robot("package://robo_arch/robots/ur7e/robot.yaml")
    plant = MultibodyPlant(time_step=0.001)
    instances = [add_robot(plant, definition, name=name) for name in ("left", "right")]
    for instance in instances:
        plant.WeldFrames(
            plant.world_frame(), plant.GetFrameByName(definition.base_frame, instance)
        )
    plant.Finalize()
    context = plant.CreateDefaultContext()
    for instance in instances:
        assert plant.num_positions(instance) == 6
        assert plant.num_velocities(instance) == 6
        assert plant.num_actuated_dofs(instance) == 6
        actuators = [
            plant.get_joint_actuator(index)
            for index in plant.GetJointActuatorIndices(instance)
        ]
        assert (
            tuple(actuator.joint().name() for actuator in actuators)
            == definition.joints
        )
        plant.SetPositions(context, instance, definition.default_positions)
    assert np.linalg.eigvalsh(plant.CalcMassMatrix(context)).min() > 0
    assert np.isfinite(plant.CalcGravityGeneralizedForces(context)).all()
    plant.SetPositions(context, instances[0], np.zeros(6))
    np.testing.assert_allclose(
        plant.GetPositions(context, instances[1]), definition.default_positions
    )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
