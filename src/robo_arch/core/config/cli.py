"""Named scenario overrides shared by direct Python entry points."""

import argparse
import math
from dataclasses import replace
from pathlib import Path

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.config.loading import load_world, resolve_resource
from robo_arch.core.config.worlds import parse_world


def add_overrides(parser: argparse.ArgumentParser) -> None:
    """Add common world, duration and visualization options."""
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument(
        "--world",
        choices=("drake", "isaac", "real"),
        help="Replace world configuration with the selected world's defaults",
    )
    selection.add_argument("--world-config", help="World YAML file or package URI")
    parser.add_argument("--duration", type=float, help="Run duration in seconds")
    viewing = parser.add_mutually_exclusive_group()
    viewing.add_argument(
        "--visualization", choices=("off", "live", "record", "live_and_record")
    )
    viewing.add_argument("--headless", action="store_true", help="Disable the viewer")


def apply_overrides(
    run: RunConfiguration, args: argparse.Namespace
) -> RunConfiguration:
    """Apply explicit flags after YAML/saved inputs, validating native settings."""
    world = run.world_config
    if args.world is not None:
        world = parse_world({"type": args.world})
        run = replace(run, world_source=None)
    elif args.world_config is not None:
        world = load_world(args.world_config)
        source = (
            resolve_resource(args.world_config)
            if args.world_config.startswith("package:")
            else Path(args.world_config).resolve()
        )
        run = replace(run, world_source=source)
    payload = world.model_dump()
    if args.visualization is not None:
        payload["visualization"]["mode"] = args.visualization
    if args.headless:
        payload["visualization"]["mode"] = (
            "record" if getattr(args, "record", None) else "off"
        )
    duration = run.duration if args.duration is None else args.duration
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("Duration must be finite and positive")
    return replace(run, world_config=parse_world(payload), duration=duration)
