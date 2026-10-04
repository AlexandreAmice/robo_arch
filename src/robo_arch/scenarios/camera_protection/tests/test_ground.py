"""Exercise a camera approaching the physical floor with an active barrier."""

import json
import os
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from robo_arch.core.config.loading import load_run
from robo_arch.scenarios.camera_protection.run import default_run, run_scenario

pytest.importorskip("pydrake")


def ground_approach_run():
    """Lower the tool camera toward the floor, then return to its initial pose."""
    run = load_run(default_run())
    initial = (0.0, -1.18, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0)
    target = (0.0, -0.92, *initial[2:])
    objects = {obj.name for obj in run.objects}
    control = dict(run.autonomy.parameters)
    control["profiles"] = {
        name: profile
        for name, profile in control["profiles"].items()
        if name not in objects
    }
    return replace(
        run,
        duration=4.0,
        objects=(),
        robot_system=replace(
            run.robot_system,
            robots=(replace(run.robot_system.robots[0], initial_positions=initial),),
        ),
        autonomy=run.autonomy.model_copy(update={"parameters": control}),
        task=run.task.model_copy(
            update={
                "parameters": dict(
                    run.task.parameters,
                    unsafe_target=list(target),
                    retreat_target=list(initial),
                    retreat_time=2.0,
                )
            }
        ),
        world_config=run.world_config.model_copy(
            update={
                "ground": True,
                "visualization": run.world_config.visualization.model_copy(
                    update={"mode": "off", "open_browser": False}
                ),
            }
        ),
    )


def test_ground_barrier_intervenes_and_retreats(tmp_path):
    run = ground_approach_run()
    visualize = os.environ.get("ROBO_ARCH_VISUALIZE") == "1"
    destination = Path(
        os.environ.get(
            "TEST_UNDECLARED_OUTPUTS_DIR", "recordings" if visualize else str(tmp_path)
        )
    )
    for filtered in (True, False):
        stem = "filtered" if filtered else "baseline"
        metadata = destination / f"camera_ground_approach_{stem}.json"
        try:
            result = run_scenario(
                run,
                filtered=filtered,
                metadata=metadata,
                recording=metadata.with_suffix(".html") if visualize else None,
            )
            assert result["success"], result
            assert result["plane_pair_count"] == 3
            if filtered:
                assert -1e-5 <= result["minimum_ground_clearance_m"] < 0.001
                assert result["maximum_torque_correction_Nm"] > 0.01
                assert result["retreat_error_rad"] < 0.03
            else:
                assert result["minimum_ground_clearance_m"] < -0.001
            report = json.loads(metadata.read_text())
            count = result["sphere_pair_count"]
            with np.load(metadata.with_suffix(".npz")) as trace:
                channel = "cbf/diagnostics" if filtered else "baseline/clearance"
                clearance = trace[channel][:, : result["pair_count"]]
                assert np.min(clearance[:, :count]) > 0.01
                assert np.min(clearance[:, count:]) == pytest.approx(
                    result["minimum_ground_clearance_m"]
                )
            assert len(report["geometry"]["plane_pairs"]) == 3
        finally:
            print(
                "Inspect the same ground approach: uv run python -m "
                "robo_arch.scenarios.camera_protection.run --inspect "
                f"{metadata} --visualization live_and_record"
            )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
