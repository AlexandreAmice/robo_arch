"""Environment selection and launch behavior without starting a simulator."""

import importlib.util
import json
import subprocess
from pathlib import Path

import pytest

from robo_arch.core.config import loading

_SPEC = importlib.util.spec_from_file_location(
    "dev", Path(__file__).resolve().parents[1] / "dev.py"
)
dev = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(dev)


@pytest.fixture
def launches(monkeypatch):
    events = []
    monkeypatch.setattr(dev.os, "chdir", lambda path: events.append(("cwd", path)))

    def install(profile):
        events.append(("install", profile))
        return Path("/environments") / profile / "bin/python"

    monkeypatch.setattr(dev, "install", install)
    monkeypatch.setattr(
        dev.os, "execv", lambda python, argv: events.append(("exec", python, argv))
    )
    return events


@pytest.mark.parametrize("world", ["drake", "isaac"])
def test_short_command_selects_environment_and_preserves_options(world, launches):
    options = [
        "--world",
        world,
        "--visualization",
        "live",
        "--no-sensors",
        "--metadata",
        "recordings/path with spaces.json",
    ]
    dev.main(["run", "arm_tracking", *options])
    python = f"/environments/{world}/bin/python"
    assert launches == [
        ("cwd", dev.ROOT),
        ("install", world),
        (
            "exec",
            python,
            [python, "-m", "robo_arch.scenarios.arm_tracking.run", *options],
        ),
    ]


def test_world_profile_is_forwarded_without_replacing_its_physics(tmp_path, launches):
    profile = tmp_path / "desktop.yaml"
    profile.write_text("type: isaac\nphysics: {device: cpu, solver: pgs}\n")
    options = ["--world-config", str(profile), "--visualization", "live"]
    dev.main(["run", "arm_tracking", *options])
    assert launches[1] == ("install", "isaac")
    assert launches[2][2][3:] == options


def test_inspection_selects_saved_world_without_overriding_configuration(
    tmp_path, launches
):
    report = tmp_path / "run.json"
    report.write_text(
        json.dumps({"configuration": {"world_config": {"type": "isaac"}}})
    )
    options = ["--inspect", str(report), "--visualization", "live"]
    dev.main(["run", "arm_tracking", *options])
    assert launches[1] == ("install", "isaac")
    assert launches[2][2][3:] == options


def test_scenario_world_is_used_when_no_override_is_given(
    tmp_path, monkeypatch, launches
):
    (tmp_path / "system.yaml").write_text("robots: {}\n")
    monkeypatch.setattr(loading, "files", lambda package: tmp_path)
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text(
        "world: {type: isaac}\n"
        "duration: 1\n"
        "robot_system:\n"
        "  definition: package://robo_arch/system.yaml\n"
        "  autonomy: {controller: example}\n"
        "task: {type: example}\n"
    )
    dev.main(["run", "arm_tracking", "--run", str(scenario)])
    assert launches[1] == ("install", "isaac")


def test_legacy_python_command_retains_its_arguments(launches):
    dev.main(["run", "--profile", "isaac", "--", "python", "-c", "print('hello')"])
    python = "/environments/isaac/bin/python"
    assert launches[-1] == ("exec", python, [python, "-c", "print('hello')"])


def test_native_only_does_not_launch(launches):
    dev.main(["native", "--profile", "drake"])
    assert launches == [("install", "drake")]


def test_failed_native_build_prevents_launch(monkeypatch, launches):
    def fail(profile):
        raise subprocess.CalledProcessError(1, ["bazel", "build", dev.TARGET])

    monkeypatch.setattr(dev, "install", fail)
    with pytest.raises(subprocess.CalledProcessError):
        dev.main(["run", "arm_tracking", "--world", "drake"])
    assert not any(event[0] == "exec" for event in launches)


@pytest.mark.parametrize(
    "args",
    [
        ["run"],
        ["run", "missing_scenario"],
        ["run", "arm_tracking", "--world", "real"],
        ["run", "--profile", "isaac", "arm_tracking"],
        ["native", "--profile", "drake", "unexpected"],
    ],
)
def test_invalid_commands_fail_before_install(args, launches):
    with pytest.raises(SystemExit, match="2"):
        dev.main(args)
    assert not any(event[0] == "install" for event in launches)


def test_help_does_not_build_or_launch(launches):
    with pytest.raises(SystemExit, match="0"):
        dev.main(["run", "arm_tracking", "--help"])
    assert not any(event[0] == "install" for event in launches)


def test_missing_isaac_environment_has_setup_command(tmp_path, monkeypatch):
    monkeypatch.setattr(dev, "ROOT", tmp_path)
    with pytest.raises(FileNotFoundError, match="uv sync --project third_party/isaac"):
        dev.install("isaac")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
