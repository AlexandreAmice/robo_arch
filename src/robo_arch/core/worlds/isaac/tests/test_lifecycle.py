"""Native Lab ownership: terminal state, selective resets and clean process exit."""

import os
import subprocess
import sys
from pathlib import Path

import pytest


@pytest.mark.skipif(
    os.environ.get("ROBO_ARCH_NATIVE_ISAAC") != "1",
    reason="Requires native Isaac profile",
)
def test_native_terminal_observations_and_selective_reset(tmp_path):
    log = tmp_path / "native_lifecycle.log"
    with log.open("w") as output:
        process = subprocess.run(
            [sys.executable, __file__, "--child"],
            env=dict(os.environ, OMNI_KIT_ACCEPT_EULA="YES"),
            stdout=output,
            stderr=subprocess.STDOUT,
            timeout=180,
        )
    assert process.returncode == 0, log.read_text()[-12000:]
    assert "LIFECYCLE_OK" in log.read_text()


def exercise():
    from dataclasses import replace

    import torch

    from robo_arch.core.config.loading import load_run
    from robo_arch.core.worlds.isaac.config import IsaacWorld
    from robo_arch.core.worlds.isaac.scenario import run_scenario

    run = load_run("package://robo_arch/scenarios/grasping/scenario.yaml")
    run = replace(
        run,
        duration=0.004,
        world_config=IsaacWorld(
            num_envs=2,
            physics={"device": "cpu", "solver": "pgs"},
        ),
    )

    def rollout(scene, viewer):
        env = scene.environment
        memory = torch.zeros(2, device=env.device)
        reset_ids = []

        def command():
            memory.add_(1)
            for effort in env.efforts.values():
                effort.fill_(1)

        def status():
            step = env.common_step_counter
            return torch.tensor([step == 2, False], device=env.device), torch.tensor(
                [False, step == 3], device=env.device
            )

        def reset(ids):
            reset_ids.append(ids.tolist())
            memory[ids] = 0

        env.before_step = command
        env.episode_status = status
        env.after_reset = reset
        env.task_observations = lambda: {"task/memory": memory}
        saved = None
        for step in range(1, 5):
            obs, _, terminated, timeout, extras = env.advance(run.time_step)
            assert terminated.tolist() == [step == 2, False]
            assert timeout.tolist() == [False, step == 3]
            if step in (2, 3):
                terminal = extras["final_obs"]
                selected, other = (0, 1) if step == 2 else (1, 0)
                assert terminal["task/memory"][selected] > 0
                assert obs["task/memory"][selected] == 0
                assert obs["environment_times"][selected] == 0
                assert terminal["environment_times"][selected] > 0
                torch.testing.assert_close(
                    obs["arm/q"][selected], env.initial["arm"][selected], rtol=0, atol=0
                )
                for name in ("arm/q", "arm/v", "block/pose", "block/twist"):
                    torch.testing.assert_close(
                        obs[name][other], terminal[name][other], rtol=0, atol=0
                    )
                assert torch.count_nonzero(obs["arm/effort"][selected]) == 0
                saved = {name: value.clone() for name, value in terminal.items()}
            else:
                assert "final_obs" not in extras
        assert reset_ids == [[0], [1]]
        assert saved["task/memory"][1] == 3
        assert memory.tolist() == [2, 1]
        return {"lifecycle_verified": True}

    assert run_scenario(run, rollout=rollout)["lifecycle_verified"]
    print("LIFECYCLE_OK", flush=True)


if __name__ == "__main__":
    if "--child" in sys.argv:
        exercise()
    else:
        raise SystemExit(pytest.main([__file__]))
