"""Check the packaged model's dynamics interface and independent instances."""

import pytest

pytest.importorskip("pydrake")

import numpy as np
from pydrake.multibody.plant import MultibodyPlant

from robo_arch.robots.ur7e.definition import BASE_FRAME, DEFAULT_POSITIONS, JOINT_NAMES
from robo_arch.robots.ur7e.drake import add_to_plant


def test_fixed_base_models_have_six_ordered_actuators_and_valid_dynamics():
    plant = MultibodyPlant(time_step=0.001)
    instances = [add_to_plant(plant, name=name) for name in ("left", "right")]
    for instance in instances:
        plant.WeldFrames(
            plant.world_frame(), plant.GetFrameByName(BASE_FRAME, instance)
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
        assert tuple(actuator.joint().name() for actuator in actuators) == JOINT_NAMES
        plant.SetPositions(context, instance, DEFAULT_POSITIONS)
    assert np.linalg.eigvalsh(plant.CalcMassMatrix(context)).min() > 0
    assert np.isfinite(plant.CalcGravityGeneralizedForces(context)).all()
    plant.SetPositions(context, instances[0], np.zeros(6))
    np.testing.assert_allclose(
        plant.GetPositions(context, instances[1]), DEFAULT_POSITIONS
    )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
