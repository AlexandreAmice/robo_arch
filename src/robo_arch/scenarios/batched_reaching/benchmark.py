"""Compare batch sizes and tensor/scalar control in fresh, sequential processes."""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

from robo_arch.core.worlds.launch import prepare as prepare_launch


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=Path("recordings/batched_reaching/benchmark")
    )
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument(
        "--backends",
        nargs="+",
        choices=("physx", "newton"),
        default=["physx", "newton"],
    )
    args = parser.parse_args()
    prepare_launch("isaac")
    args.output.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, OMNI_KIT_ACCEPT_EULA="YES")
    env.pop("DISPLAY", None)
    env.pop("WAYLAND_DISPLAY", None)
    rows = []
    for backend in args.backends:
        for count, mode in (
            (1, "tensor"),
            (16, "tensor"),
            (64, "tensor"),
            (16, "scalar"),
        ):
            output = args.output / f"{backend}_{count}_{mode}"
            command = [
                sys.executable,
                "-m",
                "robo_arch.scenarios.batched_reaching.run",
                "--backend",
                backend,
                "--num-envs",
                str(count),
                "--duration",
                str(args.duration),
                "--controller-mode",
                mode,
                "--output",
                str(output),
            ]
            print(
                f"Measuring {backend}, {count} environments, {mode} control", flush=True
            )
            with output.with_suffix(".log").open("w") as log:
                result = subprocess.run(
                    command, env=env, stdout=log, stderr=subprocess.STDOUT
                )
            row = {
                "backend": backend,
                "num_envs": count,
                "controller_mode": mode,
                "returncode": result.returncode,
                "report": str(output.with_suffix(".json")),
            }
            if result.returncode == 0:
                report = json.loads(output.with_suffix(".json").read_text())
                for field in (
                    "environment_steps_per_second",
                    "success_rate",
                    "mean_step_seconds",
                    "torch_peak_allocated_bytes",
                    "device_used_bytes_including_other_processes",
                ):
                    row[field] = report[field]
            rows.append(row)
            (args.output / "summary.json").write_text(json.dumps(rows, indent=2) + "\n")
    plot(rows, args.output / "throughput.png")
    if any(row["returncode"] for row in rows):
        raise RuntimeError(
            f"Some measurements failed; see {args.output}/summary.json and individual logs"
        )


def plot(rows: list[dict], destination: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    figure, axis = plt.subplots(figsize=(9, 5))
    for backend in ("physx", "newton"):
        selected = [
            row
            for row in rows
            if row["backend"] == backend
            and row["controller_mode"] == "tensor"
            and row["returncode"] == 0
        ]
        if not selected:
            continue
        axis.plot(
            [row["num_envs"] for row in selected],
            [row["environment_steps_per_second"] for row in selected],
            "o-",
            label=f"{backend}: tensor",
        )
        for row in rows:
            if (
                row["backend"] == backend
                and row["controller_mode"] == "scalar"
                and row["returncode"] == 0
            ):
                axis.scatter(
                    row["num_envs"],
                    row["environment_steps_per_second"],
                    marker="x",
                    s=90,
                    label=f"{backend}: scalar",
                )
    axis.set(
        xlabel="Environment count",
        ylabel="Environment steps / second",
        title="Batched UR7e reaching: local steady-state measurements",
    )
    axis.legend()
    axis.grid(alpha=0.3)
    figure.tight_layout()
    figure.savefig(destination)
    plt.close(figure)


if __name__ == "__main__":
    main()
