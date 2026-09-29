"""Launch RViz independently of device drivers or hardware command connections."""

import subprocess
import time
from collections.abc import Iterator
from contextlib import contextmanager

from robo_arch.core.config.loading import resolve_resource
from robo_arch.core.config.worlds import RealWorld


def rviz_command(world: RealWorld) -> tuple[str, ...]:
    """Prepare argv without importing ROS; the environment supplies rviz2."""
    if world.visualization.mode == "off":
        return ()
    config = world.visualization.config or (
        "package://robo_arch/core/worlds/real/inspection.rviz"
    )
    path = resolve_resource(config)
    if not path.is_file():
        raise ValueError(f"RViz configuration does not exist: {config}")
    return (
        "rviz2",
        "--display-config",
        str(path),
        "--fixed-frame",
        world.visualization.fixed_frame,
        "--ros-args",
        "-r",
        f"__ns:={world.transport.namespace}",
        "-p",
        f"use_sim_time:={'true' if world.transport.clock == 'ros' else 'false'}",
    )


@contextmanager
def launch_rviz(world: RealWorld) -> Iterator[subprocess.Popen | None]:
    """Own only the viewer process; callers supply existing observations and TF.

    The supplied RViz profile determines display subscriptions. The default
    profile contains Grid and TF displays and only observational tools. No
    state publishers, drivers, hardware connections or command outputs are made.
    """
    command = rviz_command(world)
    if not command:
        yield None
        return
    process = subprocess.Popen(command)
    try:
        yield process
    finally:
        if process.poll() is None:
            process.terminate()
            deadline = time.monotonic() + 5
            while process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            if process.poll() is None:
                process.kill()
            process.wait()


def main() -> None:
    """Inspect existing ROS observations; this entry point never starts drivers."""
    import argparse

    from robo_arch.core.config.loading import load_world

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--world-config", required=True, help="Real-world profile file or package URI"
    )
    args = parser.parse_args()
    world = load_world(args.world_config)
    if not isinstance(world, RealWorld):
        parser.error("RViz inspection requires a real-world configuration")
    with launch_rviz(world) as process:
        if process is not None and process.wait() != 0:
            raise RuntimeError(
                "RViz exited unsuccessfully; check its display/ROS diagnostics"
            )


if __name__ == "__main__":
    main()
