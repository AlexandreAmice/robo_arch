"""Opt-in native Lab checks; each case owns a fresh Kit process.

Run with ROBO_ARCH_ISAAC_TESTS=1 in the core pytest environment. Set
ROBO_ARCH_VISUALIZE=1 to inspect the same cases in the native viewport.
"""

import os
import subprocess
from pathlib import Path

import pytest


@pytest.mark.skipif(
    os.environ.get("ROBO_ARCH_ISAAC_TESTS") != "1",
    reason="requires the independently locked Isaac Lab environment and NVIDIA GPU",
)
@pytest.mark.parametrize(
    "case", ["ur7e", "iiwa", "bimanual", "desktop", "contact", "reset", "failure"]
)
def test_native_lab(case):
    root = Path(__file__).resolve().parents[5]
    python = Path(
        os.environ.get(
            "ROBO_ARCH_ISAAC_PYTHON", root / "third_party/isaac/.venv/bin/python"
        )
    )
    destination = Path(
        os.environ.get(
            "TEST_UNDECLARED_OUTPUTS_DIR", root / "recordings" / "isaac_lab_tests"
        )
    )
    destination.mkdir(parents=True, exist_ok=True)
    command = [str(python), str(Path(__file__).resolve()), case, str(destination)]
    env = dict(os.environ, OMNI_KIT_ACCEPT_EULA="YES")
    if env.get("ROBO_ARCH_VISUALIZE") != "1":
        env.pop("DISPLAY", None)
        env.pop("WAYLAND_DISPLAY", None)
    log = destination / f"{case}.log"
    with log.open("w") as output:
        result = subprocess.run(
            command, env=env, stdout=output, stderr=subprocess.STDOUT, timeout=240
        )
    assert result.returncode == 0, (
        f"See {log}. Inspect the same case: ROBO_ARCH_VISUALIZE=1 "
        f"OMNI_KIT_ACCEPT_EULA=YES {' '.join(command)}"
    )


def run_case(case: str, destination: Path) -> None:
    """Native subprocess entry point, also usable for visual diagnosis."""
    from dataclasses import replace
    from functools import partial

    import numpy as np

    from robo_arch.core.config.loading import load_run
    from robo_arch.core.worlds.isaac.config import IsaacWorld
    from robo_arch.core.worlds.isaac.scenario import run_scenario as execute
    from robo_arch.scenarios.arm_tracking.control import parameters_for
    from robo_arch.scenarios.arm_tracking.evaluation import tracking_tasks
    from robo_arch.scenarios.arm_tracking.isaac import configure
    from robo_arch.scenarios.arm_tracking.plotting import plot_trace
    from robo_arch.scenarios.arm_tracking.run import run_scenario

    examples = {
        "ur7e": "scenario",
        "iiwa": "iiwa7",
        "bimanual": "bimanual",
        "desktop": "bimanual",
        "contact": "iiwa7_contact",
        "reset": "bimanual",
        "failure": "scenario",
    }
    live = os.environ.get("ROBO_ARCH_VISUALIZE") == "1"
    run = load_run(f"package://robo_arch/scenarios/arm_tracking/{examples[case]}.yaml")
    run = replace(
        run,
        sensors_enabled=case not in {"ur7e", "failure"},
        world_config=IsaacWorld.model_validate(
            {
                "num_envs": 2 if case == "reset" else 1,
                "physics": {"device": "cpu", "solver": "pgs"}
                if case == "desktop"
                else {},
                "visualization": {"mode": "live" if live else "off"},
            }
        ),
    )
    destination.mkdir(parents=True, exist_ok=True)
    trace_path = destination / f"{case}.npz"
    if case not in {"reset", "failure"}:
        result = run_scenario(run, metadata=destination / f"{case}.json")
        plot_trace(trace_path)
        assert result["success"], result
        return

    tasks = tracking_tasks(run.task.parameters)
    parameters = parameters_for(run, list(tasks))
    factory = partial(configure, run=run, parameters=parameters, tasks=tasks)
    counters = []

    def configure_counted(scene):
        commands = factory(scene)
        counts = {name: 0 for name in commands}
        counters.append(counts)

        def wrap(name, command):
            def counted(state, time):
                counts[name] += 1
                return command(state, time)

            return counted

        return {name: wrap(name, command) for name, command in commands.items()}

    reset_done = False

    def after_step(execution):
        nonlocal reset_done
        if execution.time < 0.5 or reset_done:
            return
        if case == "failure":
            raise RuntimeError("injected Lab failure after 0.5 seconds")
        # Root poses verify that cloned fixed joints moved with their environment;
        # joint-coordinate tracking alone cannot detect overlapping assemblies.
        for arm in execution.scene.native.articulations.values():
            positions = arm.data.root_pos_w.torch.cpu().numpy()
            np.testing.assert_allclose(
                positions[1] - positions[0], [3, 0, 0], atol=1e-6
            )
        stage = execution.scene.stage
        physics = stage.GetPrimAtPath(execution.scene.simulation.cfg.physics_prim_path)
        assert physics.GetAttribute("physxScene:invertCollisionGroupFilter").Get()
        groups = stage.GetPrimAtPath("/World/collisions").GetChildren()
        assert len(groups) == 3
        global_group = stage.GetPrimAtPath("/World/collisions/global_group")
        assert stage.GetPrimAtPath("/_world/ground/collision")
        included = set()
        for group in groups:
            if group != global_group:
                assert group.GetRelationship("physics:filteredGroups").GetTargets() == [
                    group.GetPath(),
                    global_group.GetPath(),
                ]
            included.update(
                str(path)
                for path in group.GetRelationship(
                    "collection:colliders:includes"
                ).GetTargets()
            )
        assert included == {"/World/envs/env_0", "/World/envs/env_1", "/_world/ground"}
        before = execution.sample()
        untouched_commands = execution.commands[1]
        untouched_counts = counters[1].copy()
        execution.reset([0])
        after = execution.sample()
        assert execution.commands[1] is untouched_commands
        assert counters[1] == untouched_counts
        assert execution.episode_times[0] == 0
        assert execution.episode_times[1] == execution.time
        assert len(counters) == 3
        assert all(value == 0 for value in counters[2].values())
        for key in before:
            if key.startswith("env_1/"):
                np.testing.assert_array_equal(before[key], after[key])
        for name, q in execution.initial.items():
            np.testing.assert_array_equal(after[f"env_0/{name}/q"], q)
            np.testing.assert_array_equal(after[f"env_0/{name}/v"], 0)
            np.testing.assert_array_equal(after[f"env_0/{name}/effort"], 0)
        for name in execution.scene.observations:
            assert np.isnan(after[f"env_0/{name}/wrench"]).all()
        reset_done = True

    if case == "failure":
        # This is a test entry point; production exceptions propagate unchanged.
        with pytest.raises(RuntimeError, match="injected Lab failure"):
            execute(
                run, configure=factory, trace_path=trace_path, after_step=after_step
            )
        with np.load(trace_path) as trace:
            assert 0 < trace["times"][-1] < run.duration
        plot_trace(trace_path)
        return
    result = execute(
        run, configure=configure_counted, trace_path=trace_path, after_step=after_step
    )
    plot_trace(trace_path)
    assert reset_done
    assert len(result["reset_events"]) == 1
    assert all(value > 0 for value in counters[2].values())
    for env in range(2):
        for name, task in tasks.items():
            q = result["trace"][f"env_{env}/{name}/q"][-1]
            assert np.max(np.abs(q - task.target)) < task.tolerance


if __name__ == "__main__":
    import sys

    if len(sys.argv) == 3:
        run_case(sys.argv[1], Path(sys.argv[2]))
    else:
        raise SystemExit(pytest.main([__file__]))
