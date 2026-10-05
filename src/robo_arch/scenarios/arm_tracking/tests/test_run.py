"""Exercise the packaged scene, control loop, sensor and task evaluator together."""

import json
import os
import shlex
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import pytest

from robo_arch.core.config.loading import load_run
from robo_arch.core.worlds.devices import load_definitions
from robo_arch.core.worlds.drake.config import DrakeWorld
from robo_arch.core.worlds.isaac.config import IsaacWorld
from robo_arch.core.worlds.real.config import RealWorld
from robo_arch.scenarios.arm_tracking.run import (
    default_run,
    load_inspection,
    main,
    run_scenario,
)


@pytest.fixture(autouse=True)
def prepared_runtime(monkeypatch):
    # These tests exercise main in-process; launch/re-exec has separate tests.
    calls = []
    monkeypatch.setattr(
        "robo_arch.scenarios.arm_tracking.run.prepare",
        lambda profile, **kwargs: calls.append((profile, kwargs)),
    )
    return calls


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


@pytest.fixture(autouse=True)
def disable_browser(monkeypatch, request):
    # Test explicit visual reruns through the CLI; other recordings stay unattended.
    if (
        os.environ.get("ROBO_ARCH_VISUALIZE") != "1"
        or request.node.name != "test_headless_tracking"
    ):
        monkeypatch.setattr("webbrowser.open", lambda url: True)


def test_recording_preserves_tracking_results(tmp_path):
    pytest.importorskip("pydrake")
    run = replace(load_run(default_run()), duration=0.1, world_config=DrakeWorld())
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
    import numpy as np

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
    with np.load(recording.with_suffix(".npz")) as data:
        assert data["times"][-1] == pytest.approx(0.03)
        assert len(data["times"]) > 1
        assert data["arm/q"].shape == (len(data["times"]), 6)


def test_failed_run_retains_effective_configuration(tmp_path, monkeypatch):
    pytest.importorskip("pydrake")
    from pydrake.systems.analysis import Simulator

    def fail(self, time_seconds):
        raise RuntimeError("injected failure")

    monkeypatch.setattr(Simulator, "AdvanceTo", fail)
    run = replace(load_run(default_run()), world_config=DrakeWorld())
    metadata = tmp_path / "failure.json"
    with pytest.raises(RuntimeError, match="injected failure"):
        run_scenario(run, metadata=metadata)
    report = json.loads(metadata.read_text())
    assert report["status"] == "error"
    assert report["configuration"]["world_config"]["physics"]["time_step"] == 0.001
    assert report["application_sha256"]["robots/ur7e/assets/model.urdf"]
    assert report["configuration_sha256"]
    assert report["inspection_command"].endswith("--visualization live_and_record")
    saved = json.loads(metadata.with_suffix(".world.json").read_text())
    assert saved == run.world_config.model_dump(mode="json")
    assert load_inspection(metadata) == run


def test_isaac_inspection_uses_vendor_dependency_profile(tmp_path, monkeypatch):
    import robo_arch.scenarios.arm_tracking.run as runner

    monkeypatch.setattr(runner, "_run_isaac", lambda *args: {"success": True})
    run = replace(
        load_run(default_run()), world_config=IsaacWorld(), sensors_enabled=False
    )
    metadata = tmp_path / "result.json"
    run_scenario(run, metadata=metadata)
    command = shlex.split(json.loads(metadata.read_text())["inspection_command"])
    script = next(
        item for item in command if item.endswith("/scenarios/arm_tracking/run.py")
    )
    assert Path(script).is_absolute()
    assert "tools/dev.py" not in command
    assert command[command.index("--inspect") + 1] == str(metadata.resolve())
    assert command[-2:] == ["--visualization", "live"]


def test_isaac_batch_report_evaluates_each_environment(monkeypatch):
    import numpy as np

    from robo_arch.scenarios.arm_tracking.evaluation import tracking_tasks

    run = replace(
        load_run(default_run()),
        world_config=IsaacWorld(num_envs=2),
        sensors_enabled=False,
    )
    target = np.asarray(tracking_tasks(run.task.parameters)["arm"].target)
    trace = {"times": np.array([0.0, run.duration])}
    for env, error in enumerate((0.0, 0.1)):
        trace[f"env_{env}/arm/q"] = np.stack([target, target + error])
        trace[f"env_{env}/arm/v"] = np.zeros((2, len(target)))
        trace[f"env_{env}/arm/effort"] = np.zeros((2, len(target)))
        trace[f"env_{env}/episode_time"] = trace["times"].copy()
    monkeypatch.setattr(
        "robo_arch.core.worlds.isaac.scenario.run_scenario",
        lambda *args, **kwargs: {"trace": trace, "num_envs": 2},
    )
    result = run_scenario(run)
    assert result["environments"]["env_0"]["success"]
    assert not result["environments"]["env_1"]["success"]
    assert not result["success"]
    assert result["num_envs"] == 2


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


def test_cli_world_switch_replaces_configuration_without_dropping_sensors(
    tmp_path, monkeypatch, capsys, prepared_runtime
):
    import robo_arch.scenarios.arm_tracking.run as runner

    captured = []

    def capture(run, **kwargs):
        captured.append(run)
        return {"success": True}

    monkeypatch.setattr(runner, "run_scenario", capture)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "arm_tracking",
            "--world",
            "isaac",
            "--headless",
            "--metadata",
            str(tmp_path / "result.json"),
        ],
    )
    main()
    assert prepared_runtime == [("isaac", {"live": False})]
    assert isinstance(captured[0].world_config, IsaacWorld)
    assert captured[0].world_config.visualization.mode == "off"
    assert captured[0].world_config.physics.solver == "tgs"
    assert len(captured[0].robot_system.sensors) == 1
    assert json.loads(capsys.readouterr().out)["success"]


def test_cli_rejects_unsupported_native_recording_before_execution(
    tmp_path, monkeypatch
):
    from pydantic import ValidationError

    import robo_arch.scenarios.arm_tracking.run as runner

    def unexpected(*args, **kwargs):
        pytest.fail("Unsupported visualization must fail before execution")

    monkeypatch.setattr(runner, "run_scenario", unexpected)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "arm_tracking",
            "--world",
            "isaac",
            "--record",
            str(tmp_path / "unsupported.html"),
        ],
    )
    with pytest.raises(ValidationError, match="mode"):
        main()


def test_world_switch_rejects_missing_support():
    run = replace(load_run(default_run()), world_config=RealWorld())
    with pytest.raises(ValueError, match="no real implementation"):
        load_definitions(run.scene, run.world)


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
from robo_arch.core.worlds.drake.config import DrakeWorld
from robo_arch.core.worlds.isaac.config import IsaacWorld
from robo_arch.core.worlds.real.config import RealWorld
from robo_arch.core.worlds.devices import load_definitions
run = load_run(default_run())
load_definitions(run.scene, run.world)
try:
    run_scenario(replace(run, world_config=RealWorld()))
except ValueError as error:
    assert 'no real implementation' in str(error), str(error)
else:
    raise AssertionError('Unsupported world was accepted')
"""
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"controller": "unknown"}, "Unsupported arm-tracking controller: unknown"),
        ({"parameters": {"kp": [1.0], "kd": [-1.0]}}, "finite positive gains"),
    ],
)
def test_invalid_autonomy_selection(changes, message):
    run = load_run(default_run())
    run = replace(run, autonomy=run.autonomy.model_copy(update=changes))
    with pytest.raises(ValueError, match=message):
        run_scenario(run)


def test_inspection_restores_test_overrides_after_source_changes(tmp_path, monkeypatch):
    import robo_arch.scenarios.arm_tracking.run as runner

    source = tmp_path / "scenario.yaml"
    source.write_text("old source")
    child_source = tmp_path / "arm.yaml"
    child_source.write_text("old child source")
    world_source = tmp_path / "world.yaml"
    world_source.write_text("type: drake\n")
    original = load_run(default_run())
    run = replace(
        original,
        source=source,
        duration=0.001,
        sensors_enabled=False,
        world_config=DrakeWorld(),
        world_source=world_source,
        robot_system=replace(
            original.robot_system,
            robots=(),
            sensors=(),
            systems=(
                replace(
                    original.robot_system,
                    name="left",
                    source=child_source,
                    robots=(
                        replace(
                            original.robot_system.robots[0],
                            initial_positions=(0.1,) * 6,
                        ),
                    ),
                ),
            ),
        ),
        task=original.task.model_copy(
            update={
                "parameters": {
                    "robot": "left/arm",
                    "target": [0.2] * 6,
                    "tolerance": 0.001,
                }
            }
        ),
    )
    monkeypatch.setattr(runner, "_run_drake", lambda *args: {"success": False})
    metadata = tmp_path / "failure.json"
    run_scenario(run, metadata=metadata)
    hashes = json.loads(metadata.read_text())["configuration_sha256"]
    assert all(str(path) in hashes for path in (source, child_source, world_source))
    source.unlink()
    child_source.unlink()
    world_source.unlink()
    assert load_inspection(metadata) == run
    captured = []
    monkeypatch.setattr(
        runner,
        "run_scenario",
        lambda actual, **kwargs: captured.append(actual) or {"success": True},
    )
    monkeypatch.setattr(
        sys,
        "argv",
        ["arm_tracking", "--inspect", str(metadata), "--visualization", "live"],
    )
    main()
    assert replace(captured[0], world_config=run.world_config) == run


def test_output_failure_keeps_simulation_exception_as_context(tmp_path, monkeypatch):
    import robo_arch.scenarios.arm_tracking.run as runner

    def fail(*args):
        raise RuntimeError("original simulation error")

    write_text = Path.write_text
    metadata = tmp_path / "failure.json"

    def cannot_write(self, *args, **kwargs):
        if self == metadata:
            raise OSError("disk full")
        return write_text(self, *args, **kwargs)

    monkeypatch.setattr(runner, "_run_drake", fail)
    monkeypatch.setattr(Path, "write_text", cannot_write)
    with pytest.raises(OSError, match="disk full") as caught:
        run_scenario(
            replace(load_run(default_run()), world_config=DrakeWorld()),
            metadata=metadata,
        )
    assert isinstance(caught.value.__context__, RuntimeError)
    assert str(caught.value.__context__) == "original simulation error"


def test_inspection_preserves_original_failure_recording(tmp_path, monkeypatch):
    original = tmp_path / "failure.html"
    run = replace(load_run(default_run()), duration=0.001)
    assert not run_scenario(run, recording=original)["success"]
    saved = original.read_bytes()
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "arm_tracking",
            "--inspect",
            str(original.with_suffix(".json")),
            "--visualization",
            "record",
            "--no-browser",
        ],
    )
    with pytest.raises(SystemExit, match="1"):
        main()
    assert original.read_bytes() == saved
    assert (tmp_path / "failure_inspection.html").is_file()
    assert (tmp_path / "failure_inspection.json").is_file()


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
