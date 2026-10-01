"""Analytic load-cell checks of force sign, rotated frames and moment shifts."""

import os
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pydrake")

from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.math import SpatialForce
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    ExternallyAppliedSpatialForce,
)
from pydrake.multibody.tree import SpatialInertia, UnitInertia
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

from robo_arch.core.config.declarations import Pose, SensorInstance
from robo_arch.core.worlds.drake.sensors import add_sensor_body
from robo_arch.core.worlds.traces import Trace, plot_trace
from robo_arch.sensors.ati_mini45.definition import describe
from robo_arch.sensors.ati_mini45.drake import add_to_builder


@pytest.mark.parametrize("angles", [(0, 0, 0), (0.4, -0.6, 0.8)])
def test_known_offset_load_in_sensor_coordinates(angles):
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, 0.001)
    sensor = SensorInstance(
        name="ft",
        model="ati_mini45",
        parent="fixture/mount",
        pose=Pose(),
        parameters={},
    )
    rotation = RollPitchYaw(angles).ToRotationMatrix()
    instance = add_sensor_body(
        plant,
        sensor,
        describe(),
        plant.world_frame(),
        RigidTransform(rotation, [0, 0, 1]),
    )
    payload_mass = 0.25
    payload = plant.AddRigidBody(
        "payload",
        instance,
        SpatialInertia(payload_mass, [0, 0, 0], UnitInertia.SolidSphere(0.01)),
    )
    offset = np.array([0.03, -0.02, 0.05])
    plant.WeldFrames(
        plant.GetFrameByName("tool_flange", instance),
        payload.body_frame(),
        RigidTransform(offset),
    )
    plant.Finalize()
    observer = add_to_builder(builder, plant, instance=instance)
    simulator = Simulator(builder.Build())
    context = plant.GetMyMutableContextFromRoot(simulator.get_mutable_context())
    force_world = np.array([1.0, -2.0, 0.5])
    torque_world = np.array([0.1, 0.03, -0.02])
    external = ExternallyAppliedSpatialForce()
    external.body_index = payload.index()
    external.p_BoBq_B = np.zeros(3)
    external.F_Bq_W = SpatialForce(torque_world, force_world)
    plant.get_applied_spatial_force_input_port().FixValue(context, [external])
    trace = Trace()
    for step in range(1, 11):
        simulator.AdvanceTo(step * 0.001)
        actual = observer.get_output_port().Eval(
            observer.GetMyContextFromRoot(simulator.get_context())
        )
        trace.append(step * 0.001, {"ft/wrench": actual})
    tool = plant.GetBodyByName("tool", instance)
    gravity = rotation.matrix().T @ np.array([0, 0, -9.81])
    payload_force = payload_mass * gravity + rotation.matrix().T @ force_world
    expected_force = -payload_force - tool.default_mass() * gravity
    expected_torque = -np.cross(offset + [0, 0, 0.002], payload_force)
    expected_torque -= np.cross(tool.default_com(), tool.default_mass() * gravity)
    expected_torque -= rotation.matrix().T @ torque_world
    try:
        np.testing.assert_allclose(
            actual, np.r_[expected_force, expected_torque], atol=1e-10
        )
    finally:
        print(
            "Inspect: ROBO_ARCH_VISUALIZE=1 uv run pytest src/robo_arch/sensors/ati_mini45/tests/test_wrench.py -s"
        )
        if os.environ.get("ROBO_ARCH_VISUALIZE") == "1":
            path = (
                Path(os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR", "recordings"))
                / f"wrench_{angles[0]}.npz"
            )
            trace.save(path)
            print(plot_trace(path))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
