"""Load a scenario and inspect its physical composition without a simulator SDK."""

from argparse import ArgumentParser
from collections.abc import Sequence

from robo_arch.core.config.loading import load_run
from robo_arch.core.worlds.assembly import resolve_devices

DEFAULT_RUN = "package://robo_arch/scenarios/arm_tracking/bimanual.yaml"


def describe(reference: str) -> str:
    """Return a compact, deterministic view of one resolved run declaration."""
    run = load_run(reference)
    devices = resolve_devices(run.scene)

    lines = [
        f"source: {run.source}",
        f"world: {run.world}",
        f"duration_s: {run.duration}",
        "robots:",
    ]
    lines.extend(f"  {robot.name}: {robot.model}" for robot in devices.robots)
    lines.append("sensors:")
    lines.extend(
        f"  {sensor.name}: {sensor.model} -> {sensor.parent}"
        for sensor in devices.sensors
    )
    lines.append("objects:")
    lines.extend(f"  {obj.name}: {obj.model}" for obj in run.objects)
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run",
        default=DEFAULT_RUN,
        help="Scenario YAML path or package://robo_arch/... resource",
    )
    args = parser.parse_args(argv)
    print(describe(args.run))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
