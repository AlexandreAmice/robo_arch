"""Construct selected device drivers without confusing viewers with execution."""

from dataclasses import dataclass

from robo_arch.core.config.declarations import SceneConfiguration
from robo_arch.core.contracts.commands import CommandKind
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.devices import load_device_module
from robo_arch.core.worlds.real.config import RealWorld
from robo_arch.core.worlds.real.trajectory import TrajectoryTransport


@dataclass
class RealScene:
    """Independent adapters keyed by scene instance; caller owns their lifecycle."""

    robots: dict[str, TrajectoryTransport]

    def close(self) -> None:
        """Close every adapter even if one cannot cancel its remote command."""

        def close_next(adapters: list[TrajectoryTransport]) -> None:
            if adapters:
                try:
                    adapters.pop().close()
                finally:
                    close_next(adapters)

        close_next(list(self.robots.values()))


def build_scene(
    scene: SceneConfiguration,
    config: RealWorld,
    *,
    command: CommandKind,
) -> RealScene:
    """Resolve device-owned drivers; missing support and incompatible commands fail."""
    devices = resolve_devices(scene)
    if devices.sensors and scene.sensors_enabled:
        raise NotImplementedError("This deployment boundary has no sensor drivers")
    adapters: dict[str, TrajectoryTransport] = {}
    completed = False
    try:
        for robot in devices.robots:
            module = load_device_module("robots", robot.model, "real")
            adapters[robot.name] = module.build_adapter(robot, config, command)
        completed = True
        return RealScene(adapters)
    finally:
        if not completed:
            RealScene(adapters).close()
