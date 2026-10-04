"""Opt-in physics/reset tests, each in a fresh vendor process.

ROBO_ARCH_VISUALIZE=1 opens and captures the exact same inputs for inspection.
"""

import os
import subprocess
from pathlib import Path

import pytest


@pytest.mark.skipif(
    os.environ.get("ROBO_ARCH_ISAAC_TESTS") != "1",
    reason="requires Isaac vendor environment and NVIDIA GPU",
)
@pytest.mark.parametrize("backend", ["physx", "newton"])
def test_native_backend(backend):
    root = Path(__file__).resolve().parents[5]
    destination = Path(
        os.environ.get(
            "TEST_UNDECLARED_OUTPUTS_DIR", root / "recordings/batched_reaching/tests"
        )
    )
    destination.mkdir(parents=True, exist_ok=True)
    python = os.environ.get(
        "ROBO_ARCH_ISAAC_PYTHON", str(root / "third_party/isaac/.venv/bin/python")
    )
    command = [
        python,
        str(Path(__file__).resolve()),
        backend,
        str(destination / backend),
    ]
    env = dict(os.environ, OMNI_KIT_ACCEPT_EULA="YES")
    if env.get("ROBO_ARCH_VISUALIZE") != "1":
        env.pop("DISPLAY", None)
        env.pop("WAYLAND_DISPLAY", None)
    with (destination / f"{backend}.log").open("w") as log:
        process = subprocess.run(
            command, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=420
        )
    assert process.returncode == 0, (
        f"Inspect {destination / backend}.log; replay: ROBO_ARCH_VISUALIZE=1 {' '.join(command)}"
    )


def run_case(backend: str, destination: Path) -> None:
    from dataclasses import replace

    import torch

    from robo_arch.core.config.loading import load_run
    from robo_arch.core.worlds.isaac.batched import BatchedExecution
    from robo_arch.core.worlds.isaac.config import IsaacWorld
    from robo_arch.core.worlds.isaac.scenario import run_scenario
    from robo_arch.scenarios.batched_reaching.config import Measurement
    from robo_arch.scenarios.batched_reaching.run import plot_trace, rollout

    run = load_run("package://robo_arch/scenarios/batched_reaching/scenario.yaml")
    world = run.world_config.model_dump()
    world.update(num_envs=2, env_spacing=3.0)
    world["physics"] = {"backend": backend}
    world["visualization"]["mode"] = (
        "live" if os.environ.get("ROBO_ARCH_VISUALIZE") == "1" else "off"
    )
    run = replace(run, world_config=IsaacWorld.model_validate(world), duration=2.5)

    def check(scene, viewer):
        execution = BatchedExecution(scene)
        arm = scene.native.articulations["arm"]
        q = execution.initial["arm"]
        # Keep both arms still before testing a selective reset from displaced state.
        for _ in range(20):
            execution.step(lambda name, q, v, g: g, run.time_step)
        base = arm.data.body_link_pose_w.torch[:, 0, :3].clone()
        torch.testing.assert_close(
            base[1] - base[0],
            torch.tensor([3.0, 0, 0], device=q.device),
            atol=1e-4,
            rtol=0,
        )
        arm.write_joint_state_to_sim_index(
            position=q + 0.05, velocity=torch.ones_like(q) * 0.1
        )
        effort = torch.ones_like(q)
        arm.actuators.target_command.set_effort_index(value=effort)
        execution.efforts["arm"] = effort.clone()
        before_q, before_v = (
            arm.data.joint_pos.torch.clone(),
            arm.data.joint_vel.torch.clone(),
        )
        execution.reset(torch.tensor([True, False], device=q.device))
        torch.testing.assert_close(arm.data.joint_pos.torch[0], q[0])
        torch.testing.assert_close(arm.data.joint_vel.torch[0], torch.zeros_like(q[0]))
        assert torch.equal(arm.data.joint_pos.torch[1], before_q[1])
        assert torch.equal(arm.data.joint_vel.torch[1], before_v[1])
        assert torch.equal(execution.efforts["arm"][1], effort[1])
        assert not execution.efforts["arm"][0].any()
        target = arm.actuators.target_command.effort.torch
        assert not target[0].any()
        assert torch.equal(target[1], effort[1])
        before = arm.data.joint_pos.torch.clone()
        execution.reset(torch.zeros(2, dtype=torch.bool, device=q.device))
        assert torch.equal(arm.data.joint_pos.torch, before)
        torch.testing.assert_close(
            arm.data.joint_effort_limits.torch,
            torch.tensor(
                [150.0, 150.0, 150.0, 28.0, 28.0, 28.0], device=q.device
            ).expand(2, -1),
        )
        if backend == "newton":
            model = arm.root_view.model
            assert set(model.body_world.numpy().tolist()) == {0, 1}
        else:
            physics = scene.stage.GetPrimAtPath(scene.simulation.cfg.physics_prim_path)
            assert physics.GetAttribute("physxScene:invertCollisionGroupFilter").Get()
            groups = scene.stage.GetPrimAtPath("/World/collisions").GetChildren()
            assert len(groups) == 2
            for group in groups:
                assert group.GetRelationship("physics:filteredGroups").GetTargets() == [
                    group.GetPath()
                ]
        result = rollout(
            scene, viewer, run=run, measurement=Measurement(), destination=destination
        )
        assert all(count > 0 for count in result["successes"]), result
        assert sum(result["timeouts"]) == 0, result
        assert max(result["maximum_joint_error_rad"]) < 0.3, result
        return result

    run_scenario(run, rollout=check, trace_path=destination.with_suffix(".npz"))
    plot_trace(destination)


if __name__ == "__main__":
    import sys

    if len(sys.argv) == 3:
        run_case(sys.argv[1], Path(sys.argv[2]))
    else:
        raise SystemExit(pytest.main([__file__]))
