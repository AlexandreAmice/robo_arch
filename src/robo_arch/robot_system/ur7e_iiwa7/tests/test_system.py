"""Physical composition, collision topology and mixed-joint-count execution."""

from dataclasses import replace

import numpy as np
import pytest

pytest.importorskip("pydrake")

from pydrake.geometry import Role
from pydrake.systems.framework import DiagramBuilder

from robo_arch.core.config.declarations import Pose
from robo_arch.core.config.loading import load_run
from robo_arch.core.worlds.drake.scene import build_scene
from robo_arch.scenarios.arm_tracking.run import run_scenario


def scene(run):
    builder = DiagramBuilder()
    result = build_scene(run.scene, run.world_config, builder=builder)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    pc = result.plant.GetMyMutableContextFromRoot(context)
    for name, initial in result.initial_positions.items():
        result.plant.SetPositions(pc, result.robots[name], initial)
    query = result.scene_graph.get_query_output_port().Eval(
        result.scene_graph.GetMyContextFromRoot(context)
    )
    return result, diagram, context, query


@pytest.mark.parametrize("filename", ["scenario", "iiwa7", "bimanual"])
def test_initial_geometry_and_disabled_observation_mass(filename):
    run = load_run(f"package://robo_arch/scenarios/arm_tracking/{filename}.yaml")
    enabled, diagram, context, query = scene(run)
    assert not query.ComputePointPairPenetration()
    inspector = query.inspector()
    for instance in (*enabled.robots.values(), *enabled.sensor_instances.values()):
        # Every modeled physical device has proximity geometry, not just visuals.
        ids = [
            enabled.plant.GetBodyFrameIdOrThrow(index)
            for index in enabled.plant.GetBodyIndices(instance)
        ]
        assert (
            sum(len(inspector.GetGeometries(frame, Role.kProximity)) for frame in ids)
            > 0
        )
    disabled, _, dc, _ = scene(replace(run, sensors_enabled=False))
    assert not disabled.cameras and not disabled.wrenches
    assert set(enabled.sensor_instances) == set(disabled.sensor_instances)
    a = enabled.plant.GetMyContextFromRoot(context)
    b = disabled.plant.GetMyContextFromRoot(dc)
    np.testing.assert_allclose(
        enabled.plant.CalcMassMatrix(a), disabled.plant.CalcMassMatrix(b), atol=0
    )


def test_inter_arm_collisions_are_not_filtered():
    run = load_run("package://robo_arch/scenarios/arm_tracking/bimanual.yaml")
    system = run.robot_system
    run = replace(
        run,
        robot_system=replace(
            system,
            robots=(replace(system.robots[0], pose=Pose()),),
            systems=(replace(system.systems[0], pose=Pose()),),
        ),
    )
    result, diagram, context, query = scene(run)
    inspector = query.inspector()
    pairs = query.ComputePointPairPenetration()

    def robot(geometry):
        return result.plant.GetBodyFromFrameId(
            inspector.GetFrameId(geometry)
        ).model_instance()

    assert any(
        {robot(p.id_A), robot(p.id_B)} == set(result.robots.values()) for p in pairs
    )


def test_each_robot_tracks_its_own_target(tmp_path):
    run = load_run("package://robo_arch/scenarios/arm_tracking/bimanual.yaml")
    world = run.world_config.model_copy(
        update={
            "visualization": run.world_config.visualization.model_copy(
                update={"mode": "off"}
            )
        }
    )
    run = replace(run, world_config=world)
    metadata = tmp_path / "bimanual.json"
    try:
        result = run_scenario(run, metadata=metadata)
        assert result["success"]
        assert set(result["robots"]) == {"left_arm", "right/arm"}
        assert len(result["robots"]["left_arm"]["final_positions_rad"]) == 6
        assert len(result["robots"]["right/arm"]["final_positions_rad"]) == 7
        assert set(result["final_wrenches_N_Nm"]) == {"left_ft", "right/wrist_ft"}
    finally:
        print(
            f"Inspect: uv run python -m robo_arch.scenarios.arm_tracking.run --inspect {metadata} --visualization live_and_record"
        )


def test_ur7e_nonadjacent_self_collision_is_detected():
    run = load_run("package://robo_arch/scenarios/arm_tracking/scenario.yaml")
    result, diagram, context, query = scene(run)
    pc = result.plant.GetMyMutableContextFromRoot(context)
    result.plant.SetPositions(
        pc,
        result.robots["arm"],
        [
            0.625477333,
            1.986069005,
            1.378428451,
            -1.373964050,
            -0.999168575,
            1.867767227,
        ],
    )
    inspector = query.inspector()

    def arm_geometry(geometry):
        body = result.plant.GetBodyFromFrameId(inspector.GetFrameId(geometry))
        return body.model_instance() == result.robots["arm"]

    assert any(
        arm_geometry(pair.id_A) and arm_geometry(pair.id_B)
        for pair in query.ComputePointPairPenetration()
    )


def test_contact_load_reaches_the_sensor_tool_plate():
    from functools import partial

    from robo_arch.core.worlds.drake.scenario import build_simulation
    from robo_arch.scenarios.arm_tracking.control import parameters_for
    from robo_arch.scenarios.arm_tracking.drake import configure
    from robo_arch.scenarios.arm_tracking.evaluation import tracking_tasks

    run = load_run("package://robo_arch/scenarios/arm_tracking/iiwa7_contact.yaml")
    tasks = tracking_tasks(run.task.parameters)
    simulator, result = build_simulation(
        run,
        configure=partial(
            configure,
            run=run,
            parameters=parameters_for(run, ["arm"]),
            desired_positions={name: task.target for name, task in tasks.items()},
        ),
    )
    tool = result.plant.GetBodyByName(
        "tool", result.sensor_instances["wrist_ft"]
    ).index()
    touched = False
    for step in range(1, 201):
        simulator.AdvanceTo(step * run.time_step)
        context = result.plant.GetMyContextFromRoot(simulator.get_context())
        contacts = result.plant.get_contact_results_output_port().Eval(context)
        touched |= any(
            tool
            in (
                contacts.point_pair_contact_info(i).bodyA_index(),
                contacts.point_pair_contact_info(i).bodyB_index(),
            )
            for i in range(contacts.num_point_pair_contacts())
        )
    assert touched, (
        "Inspect: python -m robo_arch.scenarios.arm_tracking.run --run package://robo_arch/scenarios/arm_tracking/iiwa7_contact.yaml --visualization live_and_record"
    )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
