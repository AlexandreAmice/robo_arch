"""Measure the independent CUDA control loop on the camera scenario's model.

This measures model evaluation, nominal effort, barrier assembly and Moreau
projection. It excludes physics, rendering, state transfers and trace logging.
"""

import argparse
import json
from importlib.metadata import version
from pathlib import Path
from time import perf_counter

from robo_arch.core.worlds.launch import prepare as prepare_launch


def _step(safety, nominal, state, target, desired_velocity):
    data = safety.model.evaluate(state)
    command = nominal.effort(state, target, desired_velocity, evaluation=data)
    return safety.filter(state, command, evaluation=data)


def benchmark(
    *,
    batches: tuple[int, ...],
    iterations: int,
    warmup: int,
    device: str,
    compile_model: bool = False,
) -> dict:
    """Use deterministic perturbed safe states, mixing holding and active commands."""
    import torch

    from robo_arch.core.config.loading import load_run
    from robo_arch.core.controllers.cbf.isaac import build_filter
    from robo_arch.core.controllers.joint_tracking.torch import TensorJointTracking
    from robo_arch.core.worlds.assembly import resolve_devices
    from robo_arch.core.worlds.devices import load_definitions
    from robo_arch.core.worlds.drake.scene import build_controller_model
    from robo_arch.scenarios.camera_protection.setup import prepare

    with torch.no_grad():
        if not batches or min(batches) < 1 or iterations < 1 or warmup < 1:
            raise ValueError("Batches, iterations and warmup must be positive")
        started = perf_counter()
        run = load_run("package://robo_arch/scenarios/camera_protection/scenario.yaml")
        devices = resolve_devices(run.scene)
        definitions = load_definitions(run.scene, "drake")
        setup = prepare(run, devices, definitions)
        control = setup.control.model_copy(
            update={"backend": "torch_moreau", "compile_model": compile_model}
        )
        task, robot, definition = setup.task, setup.robot, setup.definition
        model = build_controller_model(
            robot, definition, sensors=devices.sensors, definitions=definitions
        )
        setup.validate_targets(
            model.GetPositionLowerLimits(), model.GetPositionUpperLimits()
        )
        geometry = setup.geometry
        torch.cuda.synchronize(device)
        source_setup_seconds = perf_counter() - started
        initial = torch.tensor(
            robot.initial_positions or definition.default_positions,
            dtype=torch.float64,
            device=device,
        )
        unsafe = torch.tensor(task.unsafe_target, dtype=torch.float64, device=device)
        report = {
            "device": torch.cuda.get_device_name(device),
            "dtype": "float64",
            "compile_model": compile_model,
            "versions": {
                name: version(name)
                for name in ("torch", "moreau", "moreau-cuda13", "drake")
            },
            "iterations": iterations,
            "warmup": warmup,
            "source_setup_seconds": source_setup_seconds,
            "geometry_constraint_count": len(geometry.pairs)
            + len(geometry.plane_pairs),
            "includes": [
                "independent dynamics",
                "nominal joint tracking",
                "barrier assembly",
                "Moreau projection",
                "host status and acceptance synchronization",
            ],
            "excludes": [
                "physics",
                "rendering",
                "state transfers",
                "trace logging",
                "construction and warmup",
            ],
            "rows": [],
        }
        near = None
        generator = torch.Generator(device=device).manual_seed(421)
        for batch in batches:
            started = perf_counter()
            safety = build_filter(
                model=model,
                joints=definition.joints,
                geometry=geometry,
                # Delay lazy compilation until after benchmark input preparation so
                # its actual first use is included in the measured warmup.
                parameters=control.model_copy(update={"compile_model": False}),
                batch_size=batch,
                device=device,
                command_dtype=None,
            )
            report["constraint_count"] = safety.qp_constraint_count
            nominal = TensorJointTracking(safety.model, control.nominal)
            torch.cuda.synchronize(device)
            construction_seconds = perf_counter() - started
            if near is None:
                # Pick a reproducible safe pose near the obstructed target using
                # one batched GPU evaluation. No per-environment Drake execution.
                fraction = torch.linspace(0, 1, 256, dtype=initial.dtype, device=device)
                candidates = initial + fraction[:, None] * (unsafe - initial)
                states = torch.cat((candidates, torch.zeros_like(candidates)), dim=1)
                rows = safety.evaluate(states)
                admissible = (rows.clearance >= 0.002).all(dim=1)
                indices = torch.where(admissible)[0]
                if not len(indices):
                    raise ValueError("No safe benchmark pose along the task approach")
                near = candidates[indices[-1]]
            active = torch.arange(batch, device=device) % 2 == 0
            q = torch.where(active[:, None], near, initial).clone()
            q += 1e-5 * torch.randn(
                q.shape, generator=generator, device=device, dtype=q.dtype
            )
            state = torch.cat((q, torch.zeros_like(q)), dim=1)
            safety.validate_initial_state(state)
            target = torch.where(active[:, None], unsafe, q)
            desired_velocity = torch.zeros_like(q)

            started = perf_counter()
            if control.compile_model:
                safety.model.enable_compilation()
            for _ in range(warmup):
                result = _step(safety, nominal, state, target, desired_velocity)
            torch.cuda.synchronize(device)
            warmup_seconds = perf_counter() - started
            begin, end = (
                torch.cuda.Event(enable_timing=True),
                torch.cuda.Event(enable_timing=True),
            )
            started = perf_counter()
            begin.record()
            for _ in range(iterations):
                result = _step(safety, nominal, state, target, desired_velocity)
            end.record()
            end.synchronize()
            seconds = perf_counter() - started
            count = safety.constraint_count
            correction = result.diagnostics[:, 5 * count]
            intervened = int((correction > 1e-3).sum())
            if not intervened:
                raise ValueError("Benchmark failed to exercise an active QP")
            row = {
                "batch_size": batch,
                "construction_seconds": construction_seconds,
                "warmup_seconds": warmup_seconds,
                "wall_milliseconds_per_batch": 1e3 * seconds / iterations,
                "cuda_stream_milliseconds_per_batch": begin.elapsed_time(end)
                / iterations,
                "environments_per_second": batch * iterations / seconds,
                "intervened_environments": intervened,
                "minimum_clearance_m": float(result.diagnostics[:, :count].min()),
                "minimum_cbf_residual": float(
                    result.diagnostics[:, 3 * count : 4 * count].min()
                ),
                "maximum_correction_Nm": float(correction.max()),
            }
            report["rows"].append(row)
            print(json.dumps(row), flush=True)
        return report


def plot_report(report: dict, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    rows = report["rows"]
    labels = [str(row["batch_size"]) for row in rows]
    figure, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    axes[0].bar(labels, [row["wall_milliseconds_per_batch"] for row in rows])
    axes[0].set(ylabel="Wall latency per batch (ms)", xlabel="Environments per batch")
    axes[1].bar(labels, [row["environments_per_second"] for row in rows])
    axes[1].set(
        ylabel="Environment commands per second", xlabel="Environments per batch"
    )
    mode = "compiled" if report.get("compile_model", False) else "eager"
    figure.suptitle(
        f"Camera CBF: {mode} CUDA model + Moreau\n{report['device']}; float64; {report['constraint_count']} constraints"
    )
    figure.savefig(path, dpi=150)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batches", nargs="+", type=int, default=[1, 32, 128, 512])
    parser.add_argument("--iterations", type=int, default=25)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument(
        "--compile-model",
        action="store_true",
        help="Compile the shared tensor dynamics; first use incurs compilation",
    )
    parser.add_argument(
        "--output", type=Path, default=Path("recordings/camera_cbf_gpu_benchmark.json")
    )
    args = parser.parse_args()
    prepare_launch("isaac")
    report = benchmark(
        batches=tuple(args.batches),
        iterations=args.iterations,
        warmup=args.warmup,
        device=args.device,
        compile_model=args.compile_model,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    plot_report(report, args.output.with_suffix(".png"))


if __name__ == "__main__":
    main()
