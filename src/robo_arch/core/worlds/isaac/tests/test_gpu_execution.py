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
            control_backend="torch",
            batch_size=batch_size,
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

    def configure(scene):
        class Policy:
            calls = 0

            def __call__(self, state, times):
                import torch

                if mode == "velocity":
                    arm = scene.runtime.arms["arm"]
                    if self.calls == 0:
                        arm.set_dof_velocities(
                            torch.full((batch_size, 6), 0.001, device="cuda:0"),
                            scene.runtime.indices,
                        )
                    self.calls += 1
                    return arm.get_gravity_compensation_forces()
                if self.calls == 2:
                    scene.runtime.reset(
                        torch.tensor([1], device="cuda:0", dtype=torch.int32)
                    )
                    # This policy initiates reset after receiving its input, so
                    # refresh that now-stale input before producing diagnostics.
                    arm = scene.runtime.arms["arm"]
                    state = torch.cat(
                        (arm.get_dof_positions(), arm.get_dof_velocities()), dim=-1
                    ).to(dtype=state.dtype)
                if mode == "failure" and self.calls == 3:
                    raise RuntimeError("injected controller failure")
                self.calls += 1
                self.diagnostics = {"probe/state": state[:, :6].clone()}
                self.statistics = {"last_time": times.clone()}
                effort = torch.zeros_like(state[:, :6])
                effort[0, 0] = 1.0
                return effort

        return {"arm": Policy()}

    if mode == "failure":
        with pytest.raises(RuntimeError, match="injected controller failure"):
            run_scenario(run, configure=configure, trace_path=trace_path)
        with np.load(trace_path) as trace:
            np.testing.assert_allclose(trace["times"], [0, 0.002])
        return
    result = run_scenario(run, configure=configure, trace_path=trace_path)
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
    np.testing.assert_allclose(
        result["controller_statistics"]["arm"]["last_time"], expected_times
    )
    assert trace["arm/q"][-1, 0, 0] != trace["arm/q"][-1, 1, 0]
    np.testing.assert_array_equal(trace["probe/state"], trace["arm/q"])
    assert result["simulation_wall_seconds"] > 0


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
