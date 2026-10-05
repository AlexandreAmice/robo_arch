"""The nominal demonstration must start and travel clear of self-collision."""

import numpy as np
import pytest
from pydrake.systems.framework import DiagramBuilder

from robo_arch.core.worlds.drake.scene import build_scene
from robo_arch.scenarios.grasping.run import prepare


@pytest.mark.parametrize("mode", ["grasp", "push"])
def test_reference_has_self_collision_clearance(mode):
    run, reference = prepare(mode)
    builder = DiagramBuilder()
    scene = build_scene(
        run.scene,
        run.world_config.model_copy(update={"ground": False}),
        builder=builder,
    )
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = scene.plant.GetMyMutableContextFromRoot(context)
    query_context = scene.scene_graph.GetMyContextFromRoot(context)
    for time in np.linspace(0, run.duration, 101):
        target, _, aperture = reference(time)
        scene.plant.SetPositions(plant_context, scene.robots["arm"], target)
        scene.plant.SetPositions(
            plant_context, scene.robots["gripper"], [-aperture / 2, aperture / 2]
        )
        query = scene.scene_graph.get_query_output_port().Eval(query_context)
        inspector = query.inspector()
        for pair in query.ComputeSignedDistancePairwiseClosestPoints(0.01):
            bodies = [
                scene.plant.GetBodyFromFrameId(inspector.GetFrameId(geometry))
                for geometry in (pair.id_A, pair.id_B)
            ]
            if any(body.model_instance() == scene.objects["block"] for body in bodies):
                continue
            # Closed fingers can touch one another; neither may strike the arm.
            if all(body.model_instance() == scene.robots["gripper"] for body in bodies):
                continue
            pytest.fail(f"Self-clearance {pair.distance} m at {time}: {bodies}")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
