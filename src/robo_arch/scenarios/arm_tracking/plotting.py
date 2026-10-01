"""Plot arm-tracking joint and load-cell diagnostics from saved run arrays."""

from pathlib import Path

import numpy as np


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
