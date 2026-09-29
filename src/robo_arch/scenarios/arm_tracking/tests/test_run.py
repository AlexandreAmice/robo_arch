"""Exercise the packaged scene, control loop, sensor and task evaluator together."""

import json
import os
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import pytest

from robo_arch.core.config.loading import load_run
from robo_arch.core.worlds.devices import load_definitions
from robo_arch.scenarios.arm_tracking.run import (
    default_run,
    main,
    run_scenario,
)


@pytest.fixture
def tracking_result(monkeypatch, capsys):
    pytest.importorskip("pydrake")
    args = ["arm_tracking", "--headless"]
    if os.environ.get("ROBO_ARCH_VISUALIZE") == "1":
        destination = Path(os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR", "recordings"))
        args = ["arm_tracking", "--record", str(destination / "test_tracking.html")]
        if os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR"):
            args.append("--no-browser")
    monkeypatch.setattr(sys, "argv", args)
    try:
        main()
        yield json.loads(capsys.readouterr().out)
    finally:
        print(
            "Visualize this case: ROBO_ARCH_VISUALIZE=1 uv run --locked pytest "
            "src/robo_arch/scenarios/arm_tracking/tests/test_run.py "
            "-k test_headless_tracking\n"
            "Bazel: bazel test //src/robo_arch/scenarios/arm_tracking:run_test "
            "--test_env=ROBO_ARCH_VISUALIZE=1 --nocache_test_results"
        )


def test_headless_tracking(tracking_result):
    result = tracking_result
    assert result["success"]
    assert result["max_joint_error_rad"] < 0.002
    assert (
        max(
            abs(a - b)
            for a, b in zip(
                result["initial_positions_rad"],
                result["final_positions_rad"],
                strict=True,
            )
        )
        > 0.04
    )
    assert result["initial_finite_depth_pixels"]["camera"] > 0
    assert result["final_finite_depth_pixels"]["camera"] > 0


def test_recording_preserves_tracking_results(tmp_path):
    pytest.importorskip("pydrake")
    run = replace(load_run(default_run()), duration=0.1)
    recording = tmp_path / "playback.html"
    headless = run_scenario(run)
    visualized = run_scenario(run, recording=recording)
    assert visualized["final_positions_rad"] == pytest.approx(
        headless["final_positions_rad"], abs=1e-12
    )
    assert visualized["success"] == headless["success"]
    assert (
        visualized["final_finite_depth_pixels"] == headless["final_finite_depth_pixels"]
    )
    assert "<html" in recording.read_text()


def test_recording_survives_simulation_failure(tmp_path, monkeypatch):
    pytest.importorskip("pydrake")
    from pydrake.systems.analysis import Simulator

    advance = Simulator.AdvanceTo

    def fail_after_advancing(self, time_seconds):
        advance(self, 0.03)
        raise RuntimeError("injected simulation failure")

    monkeypatch.setattr(Simulator, "AdvanceTo", fail_after_advancing)
    recording = tmp_path / "partial.html"
    with pytest.raises(RuntimeError, match="injected simulation failure"):
        run_scenario(load_run(default_run()), recording=recording)
    assert "<html" in recording.read_text()


def test_cli_records_failed_evaluation_before_exiting(tmp_path, monkeypatch, capsys):
    pytest.importorskip("pydrake")
    import robo_arch.scenarios.arm_tracking.run as runner

    run = replace(load_run(default_run()), duration=0.001)
    monkeypatch.setattr(runner, "load_run", lambda path: run)
    recording = tmp_path / "failed.html"
    monkeypatch.setattr(
        sys, "argv", ["arm_tracking", "--no-browser", "--record", str(recording)]
    )
    with pytest.raises(SystemExit, match="1"):
        main()
    assert not json.loads(capsys.readouterr().out)["success"]
    assert recording.is_file()


def test_world_switch_rejects_missing_support():
    run = replace(load_run(default_run()), world="real")
    with pytest.raises(ValueError, match="no real implementation"):
        load_definitions(run)


def test_configuration_checks_need_no_simulator_sdk():
    script = f"""
import sys
sys.path[:] = {sys.path!r}
class RejectSDK:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {{'pydrake', 'isaacsim', 'omni', 'pxr', 'rclpy'}}:
            raise AssertionError('Unexpected SDK import: ' + fullname)
sys.meta_path.insert(0, RejectSDK())
from dataclasses import replace
from robo_arch.scenarios.arm_tracking.run import default_run, run_scenario
from robo_arch.core.config.loading import load_run
from robo_arch.core.worlds.devices import load_definitions
run = load_run(default_run())
load_definitions(run)
import pytest
with pytest.raises(ValueError, match='no real implementation'):
    run_scenario(replace(run, world='real'))
"""
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize(
    ("changes", "error_type", "message"),
    [
        (
            {"controller": "unknown"},
            ValueError,
            "Unsupported arm-tracking controller: unknown",
        ),
        (
            {"parameters": {"kp": [1.0], "kd": [-1.0]}},
            ValueError,
            "finite positive gains",
        ),
    ],
)
def test_invalid_autonomy_selection(changes, error_type, message):
    run = load_run(default_run())
    run = replace(run, autonomy=run.autonomy.model_copy(update=changes))
    with pytest.raises(error_type, match=message):
        run_scenario(run)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
