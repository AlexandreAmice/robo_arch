"""Construct a declared physical scene in a caller-owned Drake diagram."""

from argparse import ArgumentParser
from collections.abc import Sequence

from pydrake.systems.framework import DiagramBuilder

from robo_arch.core.config.loading import load_run
from robo_arch.core.worlds.drake.scene import build_scene

DEFAULT_RUN = "package://robo_arch/scenarios/arm_tracking/scenario.yaml"


def describe(reference: str) -> str:
    """Build one scene and summarize the native systems it created."""
    run = load_run(reference)
    builder = DiagramBuilder()

    # The world owns physical construction. A scenario would add autonomy to this
    # same builder before building the diagram and advancing its native simulator.
    scene = build_scene(run.scene, run.world_config, builder=builder)
    builder.Build()

    lines = [
        f"world: {run.world}",
        f"plant_positions: {scene.plant.num_positions()}",
        f"plant_velocities: {scene.plant.num_velocities()}",
        "robots: " + ", ".join(scene.robots),
        "controller_models: " + ", ".join(scene.controller_models),
        "cameras: " + (", ".join(scene.cameras) or "none"),
        "wrenches: " + (", ".join(scene.wrenches) or "none"),
        "objects: " + (", ".join(obj.name for obj in run.objects) or "none"),
    ]
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run",
        default=DEFAULT_RUN,
        help="Drake scenario YAML path or package://robo_arch/... resource",
    )
    args = parser.parse_args(argv)
    print(describe(args.run))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
