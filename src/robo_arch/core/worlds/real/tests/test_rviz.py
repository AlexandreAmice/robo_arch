"""Check launch arguments and process ownership without ROS or hardware."""

from pathlib import Path
from unittest.mock import Mock, patch

import pytest

from robo_arch.core.config.worlds import RealWorld
from robo_arch.core.worlds.real.visualization import launch_rviz, rviz_command


def test_rviz_arguments_resolve_package_profile_outside_checkout(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    world = RealWorld.model_validate(
        {
            "transport": {"namespace": "/inspection", "clock": "ros"},
            "visualization": {"mode": "live", "fixed_frame": "map"},
        }
    )
    argv = rviz_command(world)
    assert Path(argv[2]).is_absolute() and Path(argv[2]).is_file()
    assert argv[3:] == (
        "--fixed-frame",
        "map",
        "--ros-args",
        "-r",
        "__ns:=/inspection",
        "-p",
        "use_sim_time:=true",
    )


def test_missing_rviz_profile_fails_before_process_launch():
    world = RealWorld.model_validate(
        {
            "visualization": {
                "mode": "live",
                "config": "package://robo_arch/missing.rviz",
            }
        }
    )
    with pytest.raises(ValueError, match="configuration does not exist"):
        rviz_command(world)


def test_viewer_off_does_not_launch():
    with patch("subprocess.Popen") as launch, launch_rviz(RealWorld()) as process:
        assert process is None
    launch.assert_not_called()


def test_owned_viewer_stops_on_error():
    process = Mock()
    process.poll.return_value = None
    world = RealWorld.model_validate({"visualization": {"mode": "live"}})
    with patch("subprocess.Popen", return_value=process):
        with pytest.raises(RuntimeError, match="caller failed"), launch_rviz(world):
            raise RuntimeError("caller failed")
    process.terminate.assert_called_once()
    process.wait.assert_called_once_with(timeout=5)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
