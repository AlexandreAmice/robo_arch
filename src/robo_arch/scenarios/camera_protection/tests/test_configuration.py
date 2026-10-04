"""Scene identity, explicit exclusions, and unsupported selection failures."""

from dataclasses import replace

import numpy as np
import pytest

from robo_arch.core.config.loading import load_run
from robo_arch.core.config.worlds import parse_world
from robo_arch.scenarios.camera_protection.configuration import (
    CameraProtectionParameters,
    parameters_for,
)


@pytest.fixture
def run():
    return load_run("package://robo_arch/scenarios/camera_protection/scenario.yaml")


@pytest.mark.parametrize("world", ["isaac", "real"])
def test_unsupported_world_fails_before_execution(run, world):
    with pytest.raises(ValueError, match="only Drake"):
        parameters_for(replace(run, world_config=parse_world({"type": world})))


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"protected": []}, "nonempty"),
        ({"protected": ["missing"]}, "needs a sphere profile"),
        ({"margin": -0.1}, "greater than or equal"),
        ({"alpha1": 0}, "greater than"),
        ({"profiles": {"arm": "../robot.yaml"}}, "package://"),
        ({"nominal_controller": "unknown"}, "joint_tracking"),
    ],
)
def test_invalid_protection_configuration(run, changes, message):
    with pytest.raises(ValueError, match=message):
        CameraProtectionParameters.model_validate(
            {**run.autonomy.parameters, **changes}
        )


def test_retreat_must_occur_within_run(run):
    task = run.task.model_copy(
        update={"parameters": {**run.task.parameters, "retreat_time": run.duration}}
    )
    with pytest.raises(ValueError, match="before the end"):
        parameters_for(replace(run, task=task))


def test_exclusions_preserve_camera_neighbors_and_other_cameras(run):
    from robo_arch.core.controllers.cbf.assembly import resolve_geometry

    control, _ = parameters_for(run)
    geometry = resolve_geometry(run.scene, control)
    spheres, pairs = geometry.spheres, geometry.pairs
    by_name = {sphere.name: sphere for sphere in spheres}
    frame_pairs = {
        frozenset((by_name[pair.first].frame, by_name[pair.second].frame))
        for pair in pairs
    }
    for excluded in control.exclude_frames:
        assert frozenset(excluded) not in frame_pairs
    assert frozenset(("wrist_camera/body", "arm/wrist_2_link")) in frame_pairs
    assert frozenset(("wrist_camera/body", "tool_camera/body")) in frame_pairs
    assert frozenset(("tool_camera/body", "world")) in frame_pairs
    assert len(pairs) == len({frozenset((p.first, p.second)) for p in pairs})


def test_fixed_objects_can_be_protected_without_camera_specific_logic(run):
    from robo_arch.core.controllers.cbf.assembly import resolve_geometry

    control, _ = parameters_for(run)
    selected = control.model_copy(update={"protected": ("obstacle",)})
    geometry = resolve_geometry(run.scene, selected)
    spheres, pairs = geometry.spheres, geometry.pairs
    by_name = {sphere.name: sphere for sphere in spheres}
    obstacle = next(s for s in spheres if s.name.startswith("obstacle/"))
    assert obstacle.frame == "world"
    obj = next(obj for obj in run.objects if obj.name == "obstacle")
    np.testing.assert_allclose(obstacle.center, obj.pose.translation)
    assert pairs and all(
        by_name[pair.first].name.startswith("obstacle/")
        or by_name[pair.second].name.startswith("obstacle/")
        for pair in pairs
    )


def test_typo_in_mount_exclusion_is_an_error(run):
    from robo_arch.core.controllers.cbf.assembly import resolve_geometry

    control, _ = parameters_for(run)
    selected = control.model_copy(
        update={"exclude_frames": (("tool_camera/body", "arm/missing_link"),)}
    )
    with pytest.raises(ValueError, match="covered frames"):
        resolve_geometry(run.scene, selected)


@pytest.mark.parametrize("enabled", [True, False])
def test_ground_barriers_follow_world_floor_and_protected_instances(run, enabled):
    from robo_arch.core.controllers.cbf.assembly import resolve_geometry

    run = replace(
        run, world_config=run.world_config.model_copy(update={"ground": enabled})
    )
    control, _ = parameters_for(run)
    geometry = resolve_geometry(run.scene, control, ground=run.world_config.ground)
    spheres, planes, pairs = geometry.spheres, geometry.planes, geometry.plane_pairs
    if not enabled:
        assert not planes and not pairs
        return
    assert len(planes) == 1
    assert planes[0].normal == (0.0, 0.0, 1.0) and planes[0].offset == 0.0
    assert {pair.sphere for pair in pairs} == {
        sphere.name
        for sphere in spheres
        if sphere.name.split("/", 1)[0] in control.protected
    }
    assert all(
        pair.plane == planes[0].name and pair.margin == control.margin for pair in pairs
    )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
