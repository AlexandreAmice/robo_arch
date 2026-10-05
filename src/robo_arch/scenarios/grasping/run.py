"""Native contact evidence for mounted WSG manipulation and free objects."""

import argparse
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from robo_arch.core.config.loading import load_run
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.devices import load_definitions
from robo_arch.core.worlds.drake.scene import build_mechanism_model
from robo_arch.robot_system.ur7e_wsg50.control import Controller
from robo_arch.scenarios.grasping.reference import make_reference


def prepare(mode="grasp", world="drake", num_envs=1):
    from robo_arch.core.worlds.isaac.config import IsaacWorld

    run = load_run("package://robo_arch/scenarios/grasping/scenario.yaml")
    if world == "isaac":
        run = replace(
            run,
            world_config=IsaacWorld(
                ground=True,
                num_envs=num_envs,
                physics={"device": "cuda:0", "solver": "pgs"},
            ),
        )
    devices = resolve_devices(run.scene)
    model = build_mechanism_model(
        devices.mechanisms[0], load_definitions(run.scene, world)
    )
    reference, initial = make_reference(model, mode)
    robots = tuple(
        replace(
            robot, initial_positions=tuple(initial[list(model.indices[robot.name].q)])
        )
        for robot in run.robot_system.robots
    )
    run = replace(run, robot_system=replace(run.robot_system, robots=robots))
    if mode == "drop":
        from robo_arch.core.config.declarations import Pose

        run = replace(
            run,
            duration=1.5,
            objects=(replace(run.objects[0], pose=Pose(translation=(0.8, 0, 0.3))),),
        )
    return run, reference


def drake_configure(builder, scene, reference):
    from pydrake.systems.framework import BasicVector, LeafSystem
    from pydrake.systems.primitives import Demultiplexer

    class Control(LeafSystem):
        def __init__(self):
            super().__init__()
            self.arm = self.DeclareVectorInputPort("arm", 12)
            self.gripper = self.DeclareVectorInputPort("gripper", 4)
            self.command = Controller(scene.mechanism_models["arm"], reference)
            self.DeclareVectorOutputPort("effort", BasicVector(8), self.output)

        def output(self, context, output):
            arm, grip = self.arm.Eval(context), self.gripper.Eval(context)
            state = np.r_[arm[:6], grip[:2], arm[6:], grip[2:]]
            output.SetFromVector(self.command(state, context.get_time()))

    controller = builder.AddSystem(Control())
    for name in ("arm", "gripper"):
        builder.Connect(
            scene.plant.get_state_output_port(scene.robots[name]),
            controller.GetInputPort(name),
        )
    demux = builder.AddSystem(Demultiplexer([6, 2]))
    builder.Connect(controller.get_output_port(), demux.get_input_port())
    for index, name in enumerate(("arm", "gripper")):
        builder.Connect(
            demux.get_output_port(index),
            scene.plant.get_actuation_input_port(scene.robots[name]),
        )
    return {"block/state": scene.plant.get_state_output_port(scene.objects["block"])}


def execute(*, world="drake", mode="grasp", output: Path, num_envs=1):
    run, reference = prepare(mode, world, num_envs)
    output.mkdir(parents=True, exist_ok=True)
    if world == "drake":
        from robo_arch.core.worlds.drake.scenario import run_scenario

        result = run_scenario(
            run,
            configure=lambda builder, scene: drake_configure(builder, scene, reference),
            recording=output / "scene.html",
            trace_path=output / "trace.npz",
        )
    else:
        from robo_arch.core.worlds.isaac.scenario import run_scenario
        from robo_arch.scenarios.grasping.isaac import configure, sample_objects

        samples, times = [], []

        def after_step(execution):
            samples.append(sample_objects(execution.scene))
            times.append(execution.time)

        result = run_scenario(
            run,
            configure=lambda scene: configure(scene, reference),
            after_step=after_step,
            trace_path=output / "trace.npz",
        )
        result["trace"]["block/state"] = np.asarray(samples)
        result["trace"]["block/state/times"] = np.asarray(times)
        np.savez(output / "trace.npz", **result["trace"])
    trace = result["trace"]
    state = trace["block/state"]
    if state.ndim == 3:
        state = state[:, 0]
    # Both exports place xyz after a wxyz quaternion.
    z = state[:, 6]
    metrics = {
        "maximum_height_m": float(z.max()),
        "final_height_m": float(z[-1]),
        "horizontal_displacement_m": float(
            np.linalg.norm(state[-1, 4:6] - state[0, 4:6])
        ),
    }
    (output / "result.json").write_text(json.dumps(metrics, indent=2) + "\n")
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", choices=("drake", "isaac"), default="drake")
    parser.add_argument("--mode", choices=("grasp", "push", "drop"), default="grasp")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--num-envs", type=int, default=1)
    args = parser.parse_args()
    print(
        json.dumps(
            execute(
                world=args.world,
                mode=args.mode,
                output=args.output.resolve(),
                num_envs=args.num_envs,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
