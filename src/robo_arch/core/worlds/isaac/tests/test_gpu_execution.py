"""Opt-in native CUDA stepping, independent resets and subprocess teardown."""

import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest


def _worker(mode: str, trace_path: Path, batch_size: int = 2) -> None:
    from robo_arch.core.config.declarations import (
        AutonomySelection,
        Pose,
        RobotInstance,
        RobotSystem,
        RunConfiguration,
        TaskSelection,
    )
    from robo_arch.core.worlds.isaac.config import IsaacWorld
    from robo_arch.core.worlds.isaac.scenario import run_scenario

    visual = os.environ.get("ROBO_ARCH_VISUALIZE") == "1"
    run = RunConfiguration(
        source=Path(__file__),
        world_config=IsaacWorld(
            num_envs=batch_size,
            log_every_n_steps=1 if mode == "velocity" else 2,
            physics={"solver": "pgs" if mode == "velocity" else "tgs"},
            visualization={"mode": "live" if visual else "off"},
        ),
        duration=0.02 if mode == "velocity" else 0.005,
        robot_system=RobotSystem(
            source=Path(__file__),
            name="",
            pose=Pose(),
            robots=(
                RobotInstance(
                    name="arm", model="ur7e", pose=Pose(), initial_positions=None
                ),
            ),
        ),
        sensors_enabled=False,
        objects=(),
        task=TaskSelection(type="external"),
        autonomy=AutonomySelection(controller="external"),
    )

    def rollout(scene, viewer):
        import torch

        from robo_arch.core.worlds.isaac.batched import BatchedExecution

        execution = BatchedExecution(scene)
        arm = scene.native.articulations["arm"]
        if mode == "velocity":
            arm.write_joint_state_to_sim_index(
                position=execution.initial["arm"],
                velocity=torch.full((batch_size, 6), 0.001, device="cuda:0"),
            )
        times, samples, clocks = [], [], []
        try:
            steps = round(run.duration / run.time_step)
            for step in range(steps + 1):
                if mode == "failure" and step == 3:
                    raise RuntimeError("injected controller failure")
                if mode != "velocity" and step == 2:
                    mask = torch.zeros(batch_size, device="cuda:0", dtype=torch.bool)
                    mask[1] = True
                    execution.reset(mask)
                if step % run.world_config.log_every_n_steps == 0 or step == steps:
                    times.append(execution.time)
                    samples.append(
                        torch.cat(
                            (arm.data.joint_pos.torch, arm.data.joint_vel.torch), -1
                        )
                        .cpu()
                        .numpy()
                        .copy()
                    )
                    clocks.append(execution.times.cpu().numpy().copy())
                if step == steps:
                    break

                def command(name, q, v, gravity):
                    if mode == "velocity":
                        return gravity
                    effort = torch.zeros_like(q)
                    effort[0, 0] = 1.0
                    return effort

                execution.step(command, run.time_step)
                if viewer is not None:
                    viewer.update(execution.time, 0)
            trace = {
                "times": np.asarray(times),
                "arm/q": np.asarray(samples)[..., :6],
                "arm/v": np.asarray(samples)[..., 6:],
                "environment_times": np.asarray(clocks),
            }
            return {"trace": trace}
        finally:
            np.savez(trace_path, times=times, state=samples)

    if mode == "failure":
        with pytest.raises(RuntimeError, match="injected controller failure"):
            run_scenario(run, rollout=rollout, trace_path=trace_path)
        with np.load(trace_path) as trace:
            np.testing.assert_allclose(trace["times"], [0, 0.002])
        return
    result = run_scenario(run, rollout=rollout, trace_path=trace_path)
    trace = result["trace"]
    if mode == "velocity":
        measured = trace["arm/q"][-1] - trace["arm/q"][0]
        integrated = trace["arm/v"][1:].sum(axis=0) * run.time_step
        np.testing.assert_allclose(measured, integrated, atol=1.5e-6, rtol=0)
        assert (measured > 0.8 * integrated).all()
        return
    assert trace["arm/q"].shape == (4, batch_size, 6)
    np.testing.assert_allclose(trace["times"], [0, 0.002, 0.004, 0.005])
    expected_times = np.full(batch_size, 0.005)
    expected_times[1] = 0.003
    np.testing.assert_allclose(trace["environment_times"][-1], expected_times)
    assert trace["arm/q"][-1, 0, 0] != trace["arm/q"][-1, 1, 0]


@pytest.mark.skipif(
    os.environ.get("ROBO_ARCH_ISAAC_GPU_TEST") != "1",
    reason="Requires opt-in native Isaac CUDA execution in the vendor environment",
)
@pytest.mark.parametrize(
    ("mode", "batch_size"),
    [("success", 2), ("failure", 2), ("velocity", 2), ("success", 32)],
)
def test_native_cuda_execution_and_process_cleanup(tmp_path, mode, batch_size):
    # Kit cannot restart reliably in one process. Checking the subprocess exit
    # also catches GPU native wrappers surviving plugin teardown.
    trace = tmp_path / f"isaac_gpu_{mode}_{batch_size}.npz"
    result = subprocess.run(
        [sys.executable, __file__, "--worker", mode, str(trace), str(batch_size)],
        capture_output=True,
        text=True,
        timeout=180,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert trace.is_file()


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--worker":
        _worker(
            sys.argv[2], Path(sys.argv[3]), int(sys.argv[4]) if len(sys.argv) > 4 else 2
        )
    else:
        raise SystemExit(pytest.main([__file__]))
