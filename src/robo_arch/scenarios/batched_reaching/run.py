"""Run, measure and inspect the same batched reaching workload on either engine."""

import argparse
import json
import math
import sys
import time
import warnings
from dataclasses import replace
from importlib.metadata import version
from pathlib import Path

import numpy as np
from pydantic import TypeAdapter

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.config.loading import load_run
from robo_arch.core.controllers.joint_pd.definition import JointPdParameters
from robo_arch.core.worlds.isaac.config import IsaacWorld
from robo_arch.core.worlds.isaac.scene import IsaacScene
from robo_arch.core.worlds.isaac.visualization import Viewer
from robo_arch.scenarios.batched_reaching.config import Measurement, Reaching


def rollout(
    scene: IsaacScene,
    viewer: Viewer | None,
    *,
    run: RunConfiguration,
    measurement: Measurement,
    destination: Path,
) -> dict:
    import torch

    from robo_arch.core.controllers.joint_pd.tensor import compute
    from robo_arch.core.worlds.isaac.batched import BatchedExecution
    from robo_arch.scenarios.batched_reaching.task import Episodes

    execution = BatchedExecution(scene)
    if len(execution.initial) != 1:
        raise ValueError("Batched reaching requires exactly one robot per environment")
    name = next(iter(execution.initial))
    arm = scene.native.articulations[name]
    parameters = JointPdParameters.model_validate(run.autonomy.parameters)
    if len(parameters.kp) != arm.num_joints:
        raise ValueError("PD gains must match the declared joint order")
    config = Reaching.model_validate(run.task.parameters)
    device = scene.world.physics.device
    kp = torch.tensor(parameters.kp, device=device)
    kd = torch.tensor(parameters.kd, device=device)
    limits = arm.data.joint_effort_limits.torch.clone()
    if not torch.isfinite(limits).all() or not (limits > 0).all():
        raise ValueError("Reaching requires finite positive effort limits")
    zero = torch.zeros_like(execution.initial[name])
    task = Episodes(execution.initial[name], arm.data.joint_pos_limits.torch, config)
    if measurement.controller_mode == "scalar":
        warnings.warn(
            "Scalar comparison copies state/feedforward to CPU each step",
            RuntimeWarning,
            stacklevel=2,
        )
        from robo_arch.core.controllers.joint_pd.native import compute as native_compute

        kp_cpu, kd_cpu = np.array(parameters.kp), np.array(parameters.kd)
        limits_cpu = limits.cpu().numpy().astype(float)

    def command(
        robot: str, q: torch.Tensor, v: torch.Tensor, gravity: torch.Tensor
    ) -> torch.Tensor:
        if measurement.controller_mode == "tensor":
            return compute(q, v, task.targets, zero, gravity, kp, kd, limits)
        state = [x.cpu().numpy().astype(float) for x in (q, v, task.targets, gravity)]
        effort = np.stack(
            [
                native_compute(qi, vi, goal, np.zeros_like(vi), g, kp_cpu, kd_cpu)
                for qi, vi, goal, g in zip(*state, strict=True)
            ]
        )
        return torch.as_tensor(
            np.clip(effort, -limits_cpu, limits_cpu), dtype=torch.float32, device=device
        )

    dt = run.time_step
    for _ in range(measurement.warmup_steps):
        execution.step(command, dt)
    execution.reset(torch.ones(scene.world.num_envs, device=device, dtype=torch.bool))
    execution.time = 0.0
    task = Episodes(execution.initial[name], arm.data.joint_pos_limits.torch, config)
    sampled = min(measurement.sampled_envs, scene.world.num_envs)
    sample_every = max(1, round(measurement.sample_period / dt))
    reset_every = max(1, round(measurement.reset_period / dt))
    times, samples = [], []
    reset_counts = torch.zeros(scene.world.num_envs, dtype=torch.int64, device=device)
    chunks = []
    steps = math.ceil(run.duration / dt)
    torch.cuda.synchronize()
    torch.cuda.reset_peak_memory_stats()
    started = chunk_started = time.perf_counter()
    if viewer is not None:
        viewer.wall_start = time.monotonic()
    try:
        for step in range(steps):
            actual_dt = min(dt, run.duration - execution.time)
            execution.step(command, actual_dt)
            q, v = arm.data.joint_pos.torch, arm.data.joint_vel.torch
            mask = task.observe(q, v, actual_dt)
            # Buffer the terminal state before any reset. CPU transfer happens below.
            if sampled and (step % sample_every == 0 or step == steps - 1):
                times.append(execution.time)
                samples.append(
                    torch.cat(
                        (
                            q[:sampled],
                            task.targets[:sampled],
                            v[:sampled],
                            execution.efforts[name][:sampled],
                            task.age[:sampled, None],
                            (task.successes + task.timeouts)[:sampled, None],
                        ),
                        dim=-1,
                    ).clone()
                )
            if (step + 1) % reset_every == 0:
                reset_counts += mask
                execution.reset(mask)
                task.reset(mask)
            if viewer is not None:
                viewer.update(execution.time, scene.world.target_realtime_rate)
            if (step + 1) % 100 == 0 or step == steps - 1:
                torch.cuda.synchronize()
                now = time.perf_counter()
                chunk_steps = 100 if (step + 1) % 100 == 0 else (step + 1) % 100
                chunks.append((now - chunk_started) / chunk_steps)
                chunk_started = now
        torch.cuda.synchronize()
        elapsed = time.perf_counter() - started
        if task.invalid.any():
            raise RuntimeError(
                "Nonfinite state in reaching rollout; inspect saved trace"
            )
        successes, timeouts = (
            task.successes.cpu().tolist(),
            task.timeouts.cpu().tolist(),
        )
        completed = sum(successes) + sum(timeouts)
        free, total = torch.cuda.mem_get_info()
        return {
            "device_name": torch.cuda.get_device_name(),
            "measured_steps": steps,
            "controller_mode": measurement.controller_mode,
            "feedforward": config.feedforward,
            "steady_state_seconds": elapsed,
            "environment_steps_per_second": steps * scene.world.num_envs / elapsed,
            "mean_step_seconds": elapsed / steps,
            "chunk_mean_step_seconds_p50_p95": np.percentile(chunks, [50, 95]).tolist(),
            "timing_includes_viewer": viewer is not None,
            "warmup_steps": measurement.warmup_steps,
            "torch_peak_allocated_bytes": torch.cuda.max_memory_allocated(),
            "device_used_bytes_including_other_processes": total - free,
            "successes": successes,
            "timeouts": timeouts,
            "reset_counts": reset_counts.cpu().tolist(),
            "success_rate": sum(successes) / completed if completed else None,
            "success": all(n > 0 for n in successes) and not any(timeouts),
            "mean_max_joint_error_rad": (task.error_sum / task.steps).cpu().tolist(),
            "maximum_joint_error_rad": task.max_error.cpu().tolist(),
            "mean_terminal_joint_error_rad": (
                task.completed_error_sum.sum().item() / completed if completed else None
            ),
        }
    finally:
        if samples:
            destination.parent.mkdir(parents=True, exist_ok=True)
            np.savez(
                destination.with_suffix(".npz"),
                times=times,
                samples=torch.stack(samples).cpu().numpy(),
                joints=arm.joint_names,
            )


def plot_trace(path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    data = np.load(path.with_suffix(".npz"))
    samples, times = data["samples"], data["times"]
    joints = len(data["joints"])
    figure, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for env in range(samples.shape[1]):
        error = np.abs(
            samples[:, env, :joints] - samples[:, env, joints : 2 * joints]
        ).max(axis=-1)
        axes[0].plot(times, error, label=f"env {env}")
        axes[1].plot(times, samples[:, env, -2], label=f"env {env}")
    axes[0].set_ylabel("Max joint error (rad)")
    axes[1].set_ylabel("Episode age (s)")
    axes[1].set_xlabel("Simulation time (s)")
    axes[0].legend()
    figure.tight_layout()
    figure.savefig(path.with_suffix(".png"))
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="package://robo_arch/scenarios/batched_reaching/scenario.yaml",
    )
    parser.add_argument("--inspect", type=Path)
    parser.add_argument("--backend", choices=("physx", "newton"))
    parser.add_argument("--num-envs", type=int)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--sample-period", type=float)
    parser.add_argument("--sampled-envs", type=int)
    parser.add_argument("--warmup-steps", type=int)
    parser.add_argument("--reset-period", type=float)
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--hold", action="store_true")
    parser.add_argument("--controller-mode", choices=("tensor", "scalar"))
    parser.add_argument(
        "--output", type=Path, default=Path("recordings/batched_reaching/run")
    )
    args = parser.parse_args()
    measurement = Measurement()
    if args.inspect:
        report = json.loads(args.inspect.read_text())
        run = TypeAdapter(RunConfiguration).validate_python(report["configuration"])
        measurement = Measurement.model_validate(report["measurement"])
    else:
        run = load_run(args.config)
    if (
        run.world != "isaac"
        or run.task.type != "batched_reaching"
        or run.autonomy.controller != "joint_pd"
    ):
        raise ValueError("Expected Isaac batched_reaching with joint_pd autonomy")
    world = run.world_config.model_dump(mode="json")
    if args.backend is not None and args.backend != world["physics"]["backend"]:
        world["physics"] = {
            "backend": args.backend,
            "time_step": run.time_step,
            "solver": "tgs" if args.backend == "physx" else "mujoco_warp",
        }
    if args.num_envs is not None:
        world["num_envs"] = args.num_envs
    if args.live:
        world["visualization"]["mode"] = "live"
    duration = run.duration if args.duration is None else args.duration
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("Duration must be finite and positive")
    run = replace(run, world_config=IsaacWorld.model_validate(world), duration=duration)
    if run.world_config.physics.device != "cuda:0":
        raise ValueError("This measurement runner requires cuda:0")
    Reaching.model_validate(run.task.parameters)
    JointPdParameters.model_validate(run.autonomy.parameters)
    settings = measurement.model_dump()
    for key in (
        "controller_mode",
        "sample_period",
        "sampled_envs",
        "warmup_steps",
        "reset_period",
    ):
        if (value := getattr(args, key)) is not None:
            settings[key] = value
    measurement = Measurement.model_validate(settings)
    started_ns = time.time_ns()
    output = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {
        "status": "starting",
        "configuration": TypeAdapter(RunConfiguration).dump_python(run, mode="json"),
        "measurement": measurement.model_dump(mode="json"),
        "versions": {
            name: version(name)
            for name in ("isaaclab", "isaacsim", "newton", "mujoco-warp", "torch")
        },
    }
    output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    from robo_arch.core.worlds.isaac.scenario import run_scenario

    try:
        result = run_scenario(
            run,
            rollout=lambda scene, viewer: rollout(
                scene, viewer, run=run, measurement=measurement, destination=output
            ),
            trace_path=output.with_suffix(".npz"),
            keep_viewer_open=args.hold,
        )
        report.update(result, status="complete")
        print(json.dumps(result, indent=2))
    finally:
        if report["status"] == "starting":
            report["status"] = "failed"
            if error := sys.exception():
                report["error"] = {"type": type(error).__name__, "message": str(error)}
        output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
        trace = output.with_suffix(".npz")
        if trace.exists() and trace.stat().st_mtime_ns >= started_ns:
            plot_trace(output)


if __name__ == "__main__":
    main()
