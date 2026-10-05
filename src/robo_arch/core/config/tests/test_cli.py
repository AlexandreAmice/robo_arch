"""CLI overrides validate the same effective settings used for world selection."""

import argparse
from pathlib import Path

import pytest
from pydantic import TypeAdapter, ValidationError

from robo_arch.core.config.cli import add_overrides, apply_overrides
from robo_arch.core.config.declarations import RunConfiguration


@pytest.fixture
def run():
    return TypeAdapter(RunConfiguration).validate_python(
        {
            "source": "scenario.yaml",
            "world_config": {"type": "drake"},
            "duration": 2,
            "robot_system": {"name": "", "source": "system.yaml", "pose": {}},
            "sensors_enabled": False,
            "objects": [],
            "task": {"type": "joint_tracking"},
            "autonomy": {"controller": "joint_pd"},
        }
    )


def options(*args):
    parser = argparse.ArgumentParser()
    add_overrides(parser)
    return parser.parse_args(args)


def test_world_replacement_and_duration(run):
    changed = apply_overrides(
        run, options("--world", "isaac", "--duration", "5", "--visualization", "live")
    )
    assert changed.world == "isaac"
    assert changed.world_config.visualization.mode == "live"
    assert changed.duration == 5
    assert run.world == "drake" and run.duration == 2


def test_world_file_and_caller_relative_path(run, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    Path("world.yaml").write_text("type: isaac\nvisualization: {mode: live}\n")
    changed = apply_overrides(
        run, options("--world-config", "world.yaml", "--headless")
    )
    assert changed.world_source == tmp_path / "world.yaml"
    assert changed.world == "isaac"
    assert changed.world_config.visualization.mode == "off"


@pytest.mark.parametrize("duration", ["0", "-1", "nan", "inf"])
def test_invalid_duration(run, duration):
    with pytest.raises(ValueError, match="finite and positive"):
        apply_overrides(run, options("--duration", duration))


def test_conflicting_selections_and_unsupported_visualization(run):
    with pytest.raises(SystemExit):
        options("--world", "isaac", "--world-config", "world.yaml")
    with pytest.raises(ValidationError):
        apply_overrides(run, options("--world", "isaac", "--visualization", "record"))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
