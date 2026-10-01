"""Measured multi-device traces shared by local execution and inspection."""

from pathlib import Path

import numpy as np


class Trace:
    """Append owned samples after each physics step; retain a time-zero sample."""

    def __init__(self):
        self.times: list[float] = []
        self.values: dict[str, list[np.ndarray]] = {}

    def append(self, time: float, values: dict[str, np.ndarray]) -> None:
        if self.times and set(values) != set(self.values):
            raise ValueError("Trace channels must stay constant during a run")
        self.times.append(time)
        for name, value in values.items():
            self.values.setdefault(name, []).append(
                np.array(value, dtype=float, copy=True)
            )

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            path,
            times=np.asarray(self.times),
            **{name: np.asarray(values) for name, values in self.values.items()},
        )


def plot_trace(path: Path) -> Path:
    """Plot measured positions/efforts/wrenches, without reconstructing contacts."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with np.load(path) as data:
        channels = [
            name for name in data.files if name != "times" and not name.endswith("/v")
        ]
        figure, axes = plt.subplots(
            len(channels), 1, figsize=(10, max(3, 2.5 * len(channels))), squeeze=False
        )
        for axis, name in zip(axes[:, 0], channels, strict=True):
            values = data[name]
            if name.endswith("/wrench"):
                axis.plot(data["times"], values[:, :3], label=["Fx", "Fy", "Fz"])
                torque_axis = axis.twinx()
                torque_axis.plot(
                    data["times"],
                    values[:, 3:],
                    linestyle="--",
                    label=["Tx", "Ty", "Tz"],
                )
                axis.set_ylabel("Force [N]")
                torque_axis.set_ylabel("Torque [N m]")
                torque_axis.legend(loc="upper right", ncol=3)
                axis.legend(loc="upper left", ncol=3)
            else:
                axis.plot(data["times"], values)
                axis.set_ylabel(
                    "Position [rad]" if name.endswith("/q") else "Effort [N m]"
                )
            axis.set_title(name)
            axis.grid(alpha=0.3)
        axes[-1, 0].set_xlabel("Simulation time [s]")
        figure.tight_layout()
        output = path.with_suffix(".png")
        figure.savefig(output, dpi=130)
        plt.close(figure)
    return output
