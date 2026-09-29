"""World declarations reject foreign settings without loading runtime SDKs."""

import subprocess
import sys

import pytest
from pydantic import ValidationError

from robo_arch.core.config.worlds import DrakeWorld, IsaacWorld, RealWorld, parse_world


@pytest.mark.parametrize(
    "value",
    [
        {"type": "unknown"},
        {"type": "drake", "physics": {"solver": "tgs"}},
        {"type": "isaac", "physics": {"contact_model": "point"}},
        {"type": "real", "physics": {"time_step": 0.001}},
        {"type": "drake", "visualization": {"type": "isaac"}},
        {"type": "isaac", "visualization": {"mode": "record"}},
        {"type": "real", "visualization": {"mode": "record"}},
        {"type": "drake", "physics": {"time_step": 0}},
        {"type": "isaac", "physics": {"time_step": float("nan")}},
        {"type": "drake", "visualization": {"publish_period": float("inf")}},
        {"type": "real", "visualization": {"config": "./viewer.rviz"}},
    ],
)
def test_invalid_native_settings_fail(value):
    with pytest.raises(ValidationError):
        parse_world(value)


@pytest.mark.parametrize("world", [DrakeWorld(), IsaacWorld(), RealWorld()])
def test_effective_configuration_round_trip_is_complete_and_immutable(world):
    serialized = world.model_dump(mode="json")
    assert serialized["visualization"]["mode"] == "off"
    assert parse_world(serialized) == world
    with pytest.raises(ValidationError, match="frozen"):
        world.visualization.mode = "live"


def test_declarations_and_loading_do_not_import_optional_sdks():
    code = """
import sys
from robo_arch.core.config.loading import load_run, load_world
from robo_arch.core.config.worlds import parse_world
for name in ('drake', 'isaac', 'real'):
    parse_world({'type': name})
for name in sys.modules:
    assert name.split('.')[0] not in {'pydrake', 'isaacsim', 'omni', 'rclpy'}, name
"""
    subprocess.run([sys.executable, "-c", code], check=True)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
