"""World declarations reject foreign settings without loading runtime SDKs."""

import subprocess
import sys

import pytest
from pydantic import ValidationError

from robo_arch.core.config.worlds import parse_world
from robo_arch.core.worlds.drake.config import DrakeWorld
from robo_arch.core.worlds.isaac.config import IsaacWorld
from robo_arch.core.worlds.real.config import RealWorld


@pytest.mark.parametrize(
    "value",
    [
        {"type": "unknown"},
        {"type": "drake", "physics": {"discrete_contact_approximation": "tamsi"}},
        {"type": "drake", "physics": {"sap_near_rigid_threshold": -0.1}},
        {"type": "drake", "target_realtime_rate": -1},
        {"type": "drake", "target_realtime_rate": float("inf")},
        {"type": "drake", "physics": {"solver": "tgs"}},
        {"type": "isaac", "physics": {"contact_model": "point"}},
        {"type": "real", "physics": {"time_step": 0.001}},
        {"type": "drake", "visualization": {"type": "isaac"}},
        {"type": "drake", "visualization": {"role": "kIllustration"}},
        {"type": "drake", "visualization": {"default_illustration_color": [1, 0, 0]}},
        {
            "type": "drake",
            "visualization": {"default_proximity_color": [1, 0, 0, 1.1]},
        },
        {"type": "drake", "visualization": {"initial_proximity_alpha": -0.1}},
        {"type": "drake", "visualization": {"mouse_interaction_stiffness": 100}},
        {"type": "isaac", "visualization": {"mode": "record"}},
        {"type": "isaac", "visualization": {"collision_geometry": True}},
        {"type": "isaac", "visualization": {"publish_period": 0}},
        {"type": "isaac", "visualization": {"width": 0}},
        {"type": "isaac", "target_realtime_rate": -1},
        {"type": "isaac", "num_envs": 0},
        {"type": "isaac", "num_envs": 1.5},
        {"type": "isaac", "env_spacing": 0},
        {"type": "isaac", "env_spacing": float("nan")},
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


def test_isaac_live_settings_round_trip_without_sdk():
    world = parse_world(
        {
            "type": "isaac",
            "target_realtime_rate": 0.5,
            "visualization": {"mode": "live", "width": 800, "height": 600},
        }
    )
    assert world.visualization.mode == "live"
    assert parse_world(world.model_dump()) == world


@pytest.mark.parametrize("kind", ["drake", "isaac"])
def test_simulation_ground_defaults_on_and_can_be_disabled(kind):
    assert parse_world({"type": kind}).ground is True
    world = parse_world({"type": kind, "ground": False})
    assert world.ground is False
    assert parse_world(world.model_dump()) == world


def test_drake_meshcat_settings_round_trip_without_sdk():
    world = parse_world(
        {
            "type": "drake",
            "visualization": {
                "publish_illustration": True,
                "publish_proximity": True,
                "default_illustration_color": [0.2, 0.4, 0.6, 0.8],
                "default_proximity_color": [1.0, 0.0, 0.0, 1.0],
                "initial_proximity_alpha": 0.4,
                "enable_alpha_sliders": True,
            },
        }
    )
    assert world.visualization.default_illustration_color == (0.2, 0.4, 0.6, 0.8)
    assert world.visualization.default_proximity_color == (1.0, 0.0, 0.0, 1.0)
    assert parse_world(world.model_dump(mode="json")) == world


def test_declarations_and_loading_do_not_import_optional_sdks():
    code = """
import sys
from robo_arch.core.config.loading import load_run, load_world
from robo_arch.core.config.worlds import parse_world
for name in ('drake', 'isaac', 'real'):
    parse_world({'type': name})
parse_world({'type': 'isaac', 'physics': {'backend': 'newton'}})
for name in sys.modules:
    assert name.split('.')[0] not in {'pydrake', 'isaacsim', 'isaaclab', 'isaaclab_physx', 'isaaclab_newton', 'torch', 'omni', 'rclpy'}, name
"""
    subprocess.run([sys.executable, "-c", code], check=True)


@pytest.mark.parametrize(
    "physics",
    [
        {"backend": "newton", "solver": "mujoco_warp"},
        {"solver": "pgs", "device": "cpu"},
    ],
)
def test_isaac_backend_round_trip(physics):
    world = parse_world({"type": "isaac", "physics": physics})
    assert parse_world(world.model_dump(mode="json")) == world


@pytest.mark.parametrize(
    "physics",
    [
        {"backend": "physx", "solver": "mujoco_warp"},
        {"backend": "newton", "solver": "tgs"},
        {"backend": "newton", "device": "cpu"},
        {"backend": "newton", "gpu_found_lost_aggregate_pairs_capacity": 32768},
        {"backend": "physx", "iterations": 50},
        {"backend": "newton", "iterations": 0},
    ],
)
def test_isaac_rejects_mixed_backend_settings(physics):
    with pytest.raises(ValidationError):
        parse_world({"type": "isaac", "physics": physics})


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
