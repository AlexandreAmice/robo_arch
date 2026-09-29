"""Validate visualization prerequisites before importing the vendor runtime."""

import subprocess
import sys

import pytest

from robo_arch.core.config.worlds import IsaacWorld
from robo_arch.core.worlds.isaac.visualization import validate_visualization


def test_live_viewer_requires_display_before_native_startup(monkeypatch):
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    world = IsaacWorld.model_validate({"visualization": {"mode": "live"}})
    with pytest.raises(ValueError, match="desktop display"):
        validate_visualization(world)
    validate_visualization(IsaacWorld())


def test_collision_debug_requires_an_active_viewer():
    world = IsaacWorld.model_validate({"visualization": {"collision_geometry": True}})
    with pytest.raises(ValueError, match="mode: live"):
        validate_visualization(world)


def test_import_and_disabled_viewer_do_not_start_optional_runtime():
    script = """
import sys
from robo_arch.core.config.worlds import IsaacWorld
from robo_arch.core.worlds.isaac.visualization import validate_visualization
validate_visualization(IsaacWorld())
assert not any(name.split('.')[0] in {'isaacsim', 'omni', 'pxr'} for name in sys.modules)
"""
    subprocess.run([sys.executable, "-c", script], check=True)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
