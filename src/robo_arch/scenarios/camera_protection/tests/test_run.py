"""Actual-run safety, comparison, controller composition and failure evidence."""

import json
import os
from dataclasses import replace
from functools import partial
from pathlib import Path

import numpy as np
import pytest

from robo_arch.core.config.loading import load_run
from robo_arch.scenarios.camera_protection.run import (
    default_run,
    load_inspection,
    run_scenario,
)

pytest.importorskip("pydrake")


def run_input():
    run = load_run(default_run())
    return replace(
        run,
        world_config=run.world_config.model_copy(
            update={
                "visualization": run.world_config.visualization.model_copy(
                    update={"mode": "off", "open_browser": False}
                )
            }
        ),
    )


def test_default_filtered_and_unfiltered_comparison(tmp_path):
    run = run_input()
    visual = os.environ.get("ROBO_ARCH_VISUALIZE") == "1"
    destination = Path(os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR", str(tmp_path)))
    for filtered in (True, False):
        stem = "filtered" if filtered else "baseline"
        metadata = destination / f"camera_{stem}.json"
        try:
            result = run_scenario(
                run,
                filtered=filtered,
                metadata=metadata,
                recording=metadata.with_suffix(".html") if visual else None,
            )
            assert result["success"], result
            assert result["plane_pair_count"] == 3
            assert result["pair_count"] == result["sphere_pair_count"] + 3
            assert result["minimum_ground_clearance_m"] > 0
            if filtered:
                assert result["minimum_clearance_m"] >= -1e-5
                assert result["maximum_torque_correction_Nm"] > 1e-3
                assert result["retreat_error_rad"] < 0.03
            else:
                assert result["minimum_clearance_m"] < -0.001
            assert metadata.with_suffix(".png").is_file()
            restored, selected = load_inspection(metadata)
            assert restored == run if not visual else restored.scene == run.scene
            assert selected == filtered
        finally:
            print(
                f"Inspect actual inputs: uv run python -m robo_arch.scenarios.camera_protection.run --inspect {metadata} --visualization live_and_record"
            )


def test_filter_wraps_native_joint_pd(tmp_path):
    run = run_input()
    values = dict(
        run.autonomy.parameters,
        nominal_controller="joint_pd",
        nominal={"kp": [40, 80, 50, 8, 3, 1], "kd": [13, 20, 13, 1, 0.5, 0.08]},
    )
    run = replace(run, autonomy=run.autonomy.model_copy(update={"parameters": values}))
    metadata = tmp_path / "native_pd.json"
    try:
        assert run_scenario(run, metadata=metadata)["success"]
    finally:
        print(
            f"Inspect native PD case: uv run python -m robo_arch.scenarios.camera_protection.run --inspect {metadata} --visualization live_and_record"
        )


def test_failure_keeps_partial_trace_playback_and_snapshot(tmp_path, monkeypatch):
    from robo_arch.core.controllers.cbf.drake import CbfFailure, SphereCbfFilter

    original = SphereCbfFilter.filter

    def fail(self, state, nominal_effort, time=0):
        if time >= 0.03:
            raise CbfFailure("injected infeasibility", time=time, state=state.tolist())
        return original(self, state, nominal_effort, time)

    monkeypatch.setattr(SphereCbfFilter, "filter", fail)
    recording = tmp_path / "failure.html"
    with pytest.raises(CbfFailure, match="injected infeasibility"):
        run_scenario(run_input(), recording=recording)
    report = json.loads(recording.with_suffix(".json").read_text())
    assert report["status"] == "error"
    assert report["failure_snapshot"]["time"] == pytest.approx(0.03)
    assert recording.is_file() and recording.with_suffix(".png").is_file()
    with np.load(recording.with_suffix(".npz")) as trace:
        assert trace["times"][-1] <= 0.03
        assert len(trace["cbf/diagnostics"]) > 1
    print(f"Inspect actual partial run: python -m webbrowser {recording.as_uri()}")


def test_early_failure_does_not_present_stale_trace(tmp_path):
    run = run_input()
    run = replace(
        run,
        task=run.task.model_copy(
            update={"parameters": dict(run.task.parameters, retreat_time=10)}
        ),
    )
    metadata = tmp_path / "failure.json"
    trace = metadata.with_suffix(".npz")
    trace.write_bytes(b"old evidence")
    with pytest.raises(ValueError, match="Retreat must begin"):
        run_scenario(run, metadata=metadata)
    report = json.loads(metadata.read_text())
    assert report["status"] == "error" and report["trace"] is None
    assert trace.read_bytes() == b"old evidence"
    assert not metadata.with_suffix(".png").exists()


def test_visual_overlay_preserves_dynamics(tmp_path):
    from robo_arch.core.worlds.drake.scenario import build_simulation
    from robo_arch.scenarios.camera_protection.drake import configure

    run = run_input()
    run = replace(
        run,
        duration=0.04,
        task=run.task.model_copy(
            update={
                "parameters": dict(
                    run.task.parameters, retreat_time=0.02, transition_seconds=0.01
                )
            }
        ),
    )
    sim, scene = build_simulation(run, configure=partial(configure, run=run))
    inspector = scene.scene_graph.model_inspector()
    names = []
    for geometry in inspector.GetAllGeometryIds():
        if inspector.GetName(geometry).startswith("protection/"):
            names.append(inspector.GetName(geometry))
            assert inspector.GetIllustrationProperties(geometry) is not None
            assert inspector.GetProximityProperties(geometry) is None
            assert inspector.GetPerceptionProperties(geometry) is None
    assert names
    # A near-static reference keeps this short viewer consistency check feasible.
    initial = load_run(default_run()).task.parameters["retreat_target"]
    run = replace(
        run,
        task=run.task.model_copy(
            update={"parameters": dict(run.task.parameters, unsafe_target=initial)}
        ),
    )
    headless, visual = tmp_path / "headless.json", tmp_path / "visual.json"
    run_scenario(run, metadata=headless)
    run_scenario(run, metadata=visual, recording=visual.with_suffix(".html"))
    with (
        np.load(headless.with_suffix(".npz")) as a,
        np.load(visual.with_suffix(".npz")) as b,
    ):
        np.testing.assert_allclose(a["arm/q"][-1], b["arm/q"][-1], atol=1e-12)
    print(
        f"Inspect actual run: python -m webbrowser {visual.with_suffix('.html').as_uri()}"
    )


@pytest.mark.parametrize(
    ("enabled", "margin"), [(True, 0.01), (True, 0.0), (True, 0.035), (False, 0.01)]
)
def test_protection_geometry_layers_and_ground_limit(enabled, margin):
    from pydrake.geometry import Box, Role, Sphere
    from pydrake.systems.framework import DiagramBuilder

    from robo_arch.core.worlds.drake.scene import build_scene
    from robo_arch.scenarios.camera_protection.drake import configure

    run = run_input()
    run = replace(
        run,
        world_config=run.world_config.model_copy(update={"ground": enabled}),
        autonomy=run.autonomy.model_copy(
            update={"parameters": dict(run.autonomy.parameters, margin=margin)}
        ),
    )
    builder = DiagramBuilder()
    scene = build_scene(run.scene, run.world_config, builder=builder)
    inspector = scene.scene_graph.model_inspector()
    roles = {
        role: inspector.NumGeometriesWithRole(role)
        for role in (Role.kProximity, Role.kPerception)
    }
    configure(builder, scene, run=run)
    assert all(
        inspector.NumGeometriesWithRole(role) == count for role, count in roles.items()
    )
    coverings, ground = [], []
    physical_boxes = 0
    for geometry in inspector.GetAllGeometryIds():
        name = inspector.GetName(geometry)
        properties = inspector.GetIllustrationProperties(geometry)
        if not name.startswith("protection/"):
            if properties is not None:
                assert not properties.HasProperty("meshcat", "accepting")
                if any(name.startswith(obj.name + "::") for obj in run.objects):
                    physical_boxes += int(isinstance(inspector.GetShape(geometry), Box))
            continue
        assert properties.GetProperty("meshcat", "accepting") == "protections"
        assert inspector.GetProximityProperties(geometry) is None
        assert inspector.GetPerceptionProperties(geometry) is None
        if name.startswith("protection/ground/"):
            ground.append(name)
            shape = inspector.GetShape(geometry)
            assert isinstance(shape, Box)
            top = (
                inspector.GetPoseInFrame(geometry).translation()[2] + shape.height() / 2
            )
            assert top == pytest.approx(
                margin - 0.0002 if name.endswith("/fill") else margin
            )
            assert inspector.GetFrameId(geometry) == scene.scene_graph.world_frame_id()
        else:
            coverings.append(name)
            assert isinstance(inspector.GetShape(geometry), Sphere)
    assert physical_boxes == len(run.objects)
    assert any(name.startswith("protection/obstacle/") for name in coverings)
    assert any(name.startswith("protection/arm/") for name in coverings)
    assert bool(ground) == enabled
    if enabled:
        assert "protection/ground/fill" in ground
        assert any("/grid/" in name for name in ground)
        assert any("/border/" in name for name in ground)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
