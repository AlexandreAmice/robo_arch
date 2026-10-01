"""Seven distinct iiwa actuators and positive nominal dynamics."""

import numpy as np
import pytest

pytest.importorskip("pydrake")
from pydrake.multibody.plant import MultibodyPlant

from robo_arch.robots.iiwa7.definition import BASE_FRAME, DEFAULT_POSITIONS, JOINT_NAMES
from robo_arch.robots.iiwa7.drake import add_to_plant


def test_seven_joint_model():
    model = MultibodyPlant(0.001)
    instance = add_to_plant(model, name="arm")
    model.WeldFrames(model.world_frame(), model.GetFrameByName(BASE_FRAME, instance))
    model.Finalize()
    assert (
        tuple(
            model.get_joint_actuator(i).joint().name()
            for i in model.GetJointActuatorIndices()
        )
        == JOINT_NAMES
    )
    assert model.num_positions() == model.num_actuators() == 7
    context = model.CreateDefaultContext()
    model.SetPositions(context, DEFAULT_POSITIONS)
    assert np.linalg.eigvalsh(model.CalcMassMatrix(context)).min() > 0


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
