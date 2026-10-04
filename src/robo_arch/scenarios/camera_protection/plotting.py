"""Measured camera clearances and controller correction from a retained run."""

from collections.abc import Mapping
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np

matplotlib.use("Agg")
from matplotlib import pyplot as plt


def clearance_series(
    data: np.ndarray,
    *,
    filtered: bool,
    description: Mapping[str, Any] | None = None,
) -> dict[str, np.ndarray]:
    """Map recorded rows to distinct ground and sphere-pair measurements.

    Clearances/gaps remain in meters here. Ground gap is measured from each
    protective sphere to its plane, before subtracting that pair's margin.
    """
    if data.ndim == 3:
        # Aggregate only for reporting; the controller retains every environment.
        series = [
            clearance_series(data[:, index], filtered=filtered, description=description)
            for index in range(data.shape[1])
        ]
        return {
            name: (
                np.any([item[name] for item in series], axis=0)
                if name == "ground_active"
                else np.max([item[name] for item in series], axis=0)
                if name == "torque_correction"
                else np.min([item[name] for item in series], axis=0)
            )
            for name in series[0]
        }
    count = (data.shape[1] - 3) // 5 if filtered else data.shape[1]
    clearances = data[:, :count]
    result = {}
    if description is None or "pairs" not in description:
        result["all_clearance"] = clearances.min(axis=1)
    else:
        names = description["pair_names"]
        if len(names) != count or len(set(names)) != count:
            raise ValueError(
                "Plot metadata must identify every recorded constraint row"
            )
        indices = {name: i for i, name in enumerate(names)}
        sphere_indices = [
            indices[f"{pair['first']}|{pair['second']}"]
            for pair in description["pairs"]
        ]
        if sphere_indices:
            result["sphere_clearance"] = clearances[:, sphere_indices].min(axis=1)
        plane_pairs = description.get("plane_pairs", ())
        if plane_pairs:
            plane_indices = [
                indices[f"{pair['sphere']}|{pair['plane']}"] for pair in plane_pairs
            ]
            margins = np.asarray([pair["margin"] for pair in plane_pairs])
            ground = clearances[:, plane_indices]
            result["ground_clearance"] = ground.min(axis=1)
            result["ground_gap"] = (ground + margins).min(axis=1)
            result["ground_margins"] = margins
            if filtered:
                result["ground_active"] = (
                    data[:, 4 * count + np.asarray(plane_indices)] > 0
                ).any(axis=1)
    if filtered:
        result["torque_correction"] = data[:, count * 5]
    return result


def plot_trace(
    path: Path,
    *,
    filtered: bool,
    description: Mapping[str, Any] | None = None,
) -> Path:
    with np.load(path) as trace:
        channel = "cbf/diagnostics" if filtered else "baseline/clearance"
        if channel not in trace or not len(trace[channel]):
            return path.with_suffix(".png")
        data, times = trace[channel], trace[channel + "/times"]
        series = clearance_series(data, filtered=filtered, description=description)
        panels = [
            (name, label)
            for name, label in (
                ("ground_clearance", "Ground clearance beyond\nsafety margin [mm]"),
                (
                    "sphere_clearance",
                    "Sphere-pair clearance beyond\nsafety margin [mm]",
                ),
                ("all_clearance", "Minimum clearance beyond\nsafety margin [mm]"),
            )
            if name in series
        ]
        figure, axes = plt.subplots(
            len(panels) + int(filtered),
            1,
            squeeze=False,
            sharex=True,
            figsize=(10, 2.8 * (len(panels) + int(filtered))),
        )
        figure.suptitle(
            "Filtered camera protection"
            if filtered
            else "Unfiltered nominal controller"
        )
        for axis, (name, label) in zip(axes[:, 0], panels, strict=False):
            axis.plot(times, 1000 * series[name], label="Minimum recorded clearance")
            axis.axhline(0, color="red", linestyle="--", label="Safety-margin boundary")
            axis.set_ylabel(label)
            axis.grid(alpha=0.2)
            if name == "ground_clearance":
                margins = 1000 * series["ground_margins"]
                margin_label = (
                    f"{margins[0]:g} mm"
                    if np.all(margins == margins[0])
                    else f"{margins.min():g}–{margins.max():g} mm"
                )
                axis.set_title(
                    f"Minimum logged sphere–ground gap: {1000 * series['ground_gap'].min():.3f} mm"
                    f"; safety margin: {margin_label}",
                    fontsize=10,
                )
                if filtered:
                    axis.fill_between(
                        times,
                        0,
                        1,
                        where=series["ground_active"],
                        transform=axis.get_xaxis_transform(),
                        color="orange",
                        alpha=0.2,
                        label="At least one ground CBF constraint active",
                    )
                axis.legend(fontsize=8, loc="upper right")
        if filtered:
            axes[-1, 0].plot(times, series["torque_correction"])
            axes[-1, 0].set_ylabel("Torque correction norm [N m]")
            axes[-1, 0].grid(alpha=0.2)
        axes[-1, 0].set_xlabel("Simulation time [s]")
        figure.tight_layout()
        output = path.with_suffix(".png")
        figure.savefig(output)
        plt.close(figure)
    return output
