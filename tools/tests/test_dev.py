"""Runtime routing must work before importing either simulator SDK."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from robo_arch.core.config import loading
from robo_arch.core.config.worlds import parse_world
from tools import dev


@pytest.fixture
def declarations(monkeypatch):
    def load(resource):
        kind = "isaac" if "batched_reaching" in resource else "drake"
        return SimpleNamespace(world_config=parse_world({"type": kind}))

    monkeypatch.setattr(loading, "load_run", load)


@pytest.mark.parametrize(
    "scenario, options, expected",
    [
        ("arm_tracking", [], ("drake", False)),
        ("camera_protection", [], ("drake", False)),
        (
            "camera_protection",
            [
                "--world-config",
                "package://robo_arch/scenarios/camera_protection/isaac_gpu.yaml",
            ],
            ("isaac", False),
        ),
        ("arm_tracking", ["--world", "isaac"], ("isaac", False)),
        ("arm_tracking", ["--world=isaac", "--visualization=live"], ("isaac", True)),
        ("batched_reaching", ["--backend", "newton"], ("isaac", False)),
        ("batched_reaching", ["--live", "--hold"], ("isaac", True)),
    ],
)
def test_routing(declarations, scenario, options, expected):
    assert dev.scenario_environment(scenario, options) == expected


def test_world_profile_and_headless_override(declarations, tmp_path):
    profile = tmp_path / "world.yaml"
    profile.write_text("type: isaac\nvisualization: {mode: live}\n")
    options = ["--world-config", str(profile)]
    assert dev.scenario_environment("arm_tracking", options) == ("isaac", True)
    assert dev.scenario_environment("arm_tracking", [*options, "--headless"]) == (
        "isaac",
        False,
    )


def test_inspection_selects_saved_world(tmp_path):
    report = tmp_path / "run.json"
    report.write_text(
        json.dumps(
            {
                "configuration": {
                    "source": "scenario.yaml",
                    "world_config": {"type": "isaac"},
                    "duration": 1,
                    "robot_system": {"name": "", "source": "system.yaml", "pose": {}},
                    "sensors_enabled": False,
                    "objects": [],
                    "task": {"type": "joint_tracking"},
                    "autonomy": {"controller": "joint_pd"},
                }
            }
        )
    )
    assert dev.scenario_environment("arm_tracking", ["--inspect", str(report)]) == (
        "isaac",
        False,
    )


def test_child_display_and_eula_settings(monkeypatch):
    monkeypatch.setenv("DISPLAY", ":1")
    monkeypatch.setenv("WAYLAND_DISPLAY", "wayland-0")
    monkeypatch.delenv("OMNI_KIT_ACCEPT_EULA", raising=False)
    assert "OMNI_KIT_ACCEPT_EULA" not in dev.launch_environment("drake", live=False)
    assert dev.launch_environment("drake", live=False)["DISPLAY"] == ":1"
    assert dev.launch_environment("isaac", live=True)["DISPLAY"] == ":1"
    headless = dev.launch_environment("isaac", live=False)
    assert headless["OMNI_KIT_ACCEPT_EULA"] == "YES"
    assert "DISPLAY" not in headless and "WAYLAND_DISPLAY" not in headless
    monkeypatch.setenv("OMNI_KIT_ACCEPT_EULA", "NO")
    assert dev.launch_environment("isaac", live=True)["OMNI_KIT_ACCEPT_EULA"] == "NO"


@pytest.mark.parametrize("operation", ["run", "benchmark"])
def test_launch_routes_options_and_help(declarations, monkeypatch, operation):
    installed, launched = [], []

    def install(profile):
        installed.append(profile)
        return Path("/vendor/bin/python")

    monkeypatch.setattr(dev, "install", install)
    monkeypatch.setattr(dev.os, "execve", lambda *a: launched.append(a))
    dev.main([operation, "batched_reaching", "--help"])
    assert installed == ["isaac"]
    assert launched[0][1] == [
        "/vendor/bin/python",
        "-m",
        f"robo_arch.scenarios.batched_reaching.{operation}",
        "--help",
    ]
    assert launched[0][2]["OMNI_KIT_ACCEPT_EULA"] == "YES"


def test_conflicting_profile_never_launches(declarations, monkeypatch):
    monkeypatch.setattr(dev, "install", lambda _: pytest.fail("unexpected build"))
    with pytest.raises(SystemExit):
        dev.main(["run", "--profile", "drake", "batched_reaching"])


def test_legacy_python_launch(monkeypatch):
    monkeypatch.setattr(dev, "install", lambda profile: Path(f"/{profile}/python"))
    calls = []
    monkeypatch.setattr(dev.os, "execve", lambda *a: calls.append(a[1]))
    dev.main(["run", "--profile", "drake", "--", "python", "-c", "print(1)"])
    assert calls == [["/drake/python", "-c", "print(1)"]]


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
