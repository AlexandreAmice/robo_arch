"""Run the packaged arm-tracking scenario; SDK imports follow selection checks."""

import argparse
import json
import sys
import webbrowser
from dataclasses import replace
from pathlib import Path
from types import ModuleType

from robo_arch.core.config.loading import RunConfiguration, load_run
from robo_arch.core.config.parameters import Parameters
from robo_arch.core.controllers import definition
from robo_arch.core.worlds.registry import Registry, discover
from robo_arch.scenarios.arm_tracking.evaluation import TrackingTask, evaluate


def default_run() -> str:
    return "package://robo_arch/scenarios/arm_tracking/scenario.yaml"


def run_scenario(run: RunConfiguration, *, recording: Path | None = None) -> dict:
    """Run the task, optionally saving scene playback even after a runtime failure."""
    definitions = discover(run)
    if len(run.robots) != 1:
        raise ValueError("Arm tracking requires exactly one actuated robot")
    controller = definition(run.autonomy.controller)
    if run.world not in controller.IMPLEMENTATIONS:
        raise ValueError(f"Controller has no {run.world} implementation")
    parameters = controller.PARAMETERS.model_validate(run.autonomy.parameters)
    if run.task.type != "joint_tracking":
        raise ValueError(f"Unsupported task evaluator: {run.task.type}")
    task = TrackingTask.model_validate(run.task.parameters)
    if task.robot != run.robots[0].name:
        raise ValueError("Task must select the configured robot")
    if run.world == "isaac":
        return _run_isaac(run, definitions, controller, parameters, task, recording)
    if run.world != "drake":
        raise ValueError(f"No runner for world {run.world}")

    import numpy as np

    from robo_arch.scenarios.arm_tracking.drake import build_simulation

    meshcat = None
    if recording is not None:
        from pydrake.geometry import Meshcat, MeshcatParams

        recording = recording.resolve()
        recording.parent.mkdir(parents=True, exist_ok=True)
        meshcat = Meshcat(MeshcatParams(host="localhost"))
        meshcat.SetCameraPose([0.9, -0.9, 0.8], [0.25, 0.0, 0.3])
        meshcat.StartRecording()
    simulator, scene = build_simulation(
        run,
        definitions,
        parameters,
        desired_positions=task.target,
        meshcat=meshcat,
    )

    def positions() -> np.ndarray:
        context = scene.plant.GetMyContextFromRoot(simulator.get_context())
        return scene.plant.GetPositions(context, scene.robots[task.robot]).copy()

    def depth_image(name: str) -> np.ndarray:
        camera = scene.cameras[name]
        context = camera.GetMyContextFromRoot(simulator.get_context())
        return camera.depth_image_32F_output_port().Eval(context).data

    try:
        initial = positions()
        initial_depth = {
            sensor.name: int(np.isfinite(depth_image(sensor.name)).sum())
            for sensor in run.sensors
        }
        simulator.AdvanceTo(run.duration)
        if meshcat is not None:
            simulator.get_system().ForcedPublish(simulator.get_context())
        final = positions()
        if recording is not None:
            from pydrake.systems.sensors import ImageIo

            for name, camera in scene.cameras.items():
                image = camera.color_image_output_port().Eval(
                    camera.GetMyContextFromRoot(simulator.get_context())
                )
                destination = recording.with_name(
                    f"{recording.stem}_{name.replace('/', '__')}.png"
                )
                ImageIo().Save(image, destination)
                print(f"Camera {name}: {destination.as_uri()}", file=sys.stderr)
        return {
            "world": run.world,
            "duration_seconds": run.duration,
            "robot": task.robot,
            "initial_positions_rad": initial.tolist(),
            "final_positions_rad": final.tolist(),
            "initial_finite_depth_pixels": initial_depth,
            "final_finite_depth_pixels": {
                sensor.name: int(np.isfinite(depth_image(sensor.name)).sum())
                for sensor in run.sensors
            },
            **evaluate(task, tuple(final)),
        }
    finally:
        if meshcat is not None:
            meshcat.StopRecording()
            meshcat.PublishRecording()
            recording.write_text(meshcat.StaticHtml(), encoding="utf-8")
            print(f"Scene playback: {recording.as_uri()}", file=sys.stderr)


def _run_isaac(
    run: RunConfiguration,
    definitions: Registry,
    controller: ModuleType,
    parameters: Parameters,
    task: TrackingTask,
    recording: Path | None,
) -> dict:
    """Supply this task's reference to the same controller evaluated on the CPU."""
    import warnings

    import numpy as np

    from robo_arch.core.worlds.drake.scene import build_controller_model
    from robo_arch.core.worlds.isaac.simulation import run_scene

    if controller.EXECUTION["isaac"] != "tensor":
        warnings.warn(
            f"{run.autonomy.controller} in Isaac uses "
            f"{controller.EXECUTION['isaac']} execution; it has no tensor "
            "implementation and will require per-environment work in a batch.",
            RuntimeWarning,
            stacklevel=2,
        )
    robot = run.robots[0]
    robot_definition = definitions.robots[robot.model]
    model = build_controller_model(robot, robot_definition)
    target = np.asarray(task.target)
    if len(target) != model.num_positions() or not (
        np.all(target >= model.GetPositionLowerLimits())
        and np.all(target <= model.GetPositionUpperLimits())
    ):
        raise ValueError("Task target must match the robot's joints and limits")
    reference = np.r_[target, np.zeros_like(target)]
    command = controller.IMPLEMENTATIONS["isaac"].load()(
        model=model,
        parameters=parameters,
        joints=robot_definition.joints,
        desired_state=lambda time: reference,
    )
    trace_path = recording.with_suffix(".npz") if recording is not None else None
    if trace_path is not None:
        trace_path.unlink(missing_ok=True)
    try:
        trace = run_scene(run, definitions, command, trace_path=trace_path)
    finally:
        if trace_path is not None and trace_path.exists():
            from robo_arch.core.worlds.drake.visualization import replay_positions

            replay_positions(run, definitions, trace_path, recording)
            print(
                f"Isaac trajectory playback: {recording.resolve().as_uri()}",
                file=sys.stderr,
            )
    return {
        "world": run.world,
        "duration_seconds": float(trace["times"][-1]),
        "robot": task.robot,
        "initial_positions_rad": trace["positions"][0].tolist(),
        "final_positions_rad": trace["positions"][-1].tolist(),
        **evaluate(task, tuple(trace["positions"][-1])),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run", default=str(default_run()), help="Run file or package URI"
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Skip visualization unless --record is set",
    )
    parser.add_argument(
        "--record", type=Path, help="Save standalone HTML playback at this path"
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Save playback without opening a browser",
    )
    parser.add_argument(
        "--world", choices=("drake", "isaac"), help="Override the scenario world"
    )
    parser.add_argument(
        "--no-sensors",
        action="store_true",
        help="Explicitly omit sensors, e.g. for an initial Isaac physics run",
    )
    args = parser.parse_args()
    run = load_run(args.run)
    if args.world is not None:
        run = replace(run, world=args.world)
    if args.no_sensors:
        run = replace(run, sensors=())
    recording = args.record
    if recording is None and not args.headless:
        recording = Path(f"recordings/arm_tracking_{run.world}.html")
    result = run_scenario(run, recording=recording)
    print(json.dumps(result, indent=2))
    if recording is not None and not args.headless and not args.no_browser:
        webbrowser.open(recording.resolve().as_uri())
    if not result["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
