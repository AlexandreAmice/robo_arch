"""Run arm tracking with explicit native world and visualization settings."""

import argparse
import hashlib
import json
import os
import shlex
import sys
import uuid
import webbrowser
from dataclasses import asdict, replace
from importlib.metadata import distributions
from importlib.resources import files
from pathlib import Path

from pydantic import BaseModel, TypeAdapter

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.config.loading import load_run, load_world, resolve_resource
from robo_arch.core.config.worlds import DrakeWorld, IsaacWorld, parse_world
from robo_arch.core.controllers.joint_tracking.definition import JointTrackingParameters
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.devices import DeviceDefinitions, load_definitions
from robo_arch.scenarios.arm_tracking.evaluation import TrackingTask, evaluate


def default_run() -> str:
    return "package://robo_arch/scenarios/arm_tracking/scenario.yaml"


def _json_value(value: object) -> object:
    if isinstance(value, BaseModel):
        return value.model_dump(mode="json")
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Cannot serialize {type(value).__name__}")


def _inspection_command(run: RunConfiguration, metadata: Path) -> str:
    args = ["uv", "run", "--locked"]
    if run.world == "isaac":
        args.extend(["--project", "deployment/isaac"])
    args.extend(
        [
            "python",
            "-m",
            "robo_arch.scenarios.arm_tracking.run",
            "--inspect",
            str(metadata.resolve()),
            "--visualization",
            "live_and_record" if run.world == "drake" else "live",
        ]
    )
    return shlex.join(args)


def load_inspection(path: Path) -> RunConfiguration:
    """Restore resolved inputs, including test overrides, without reloading YAML.

    Device assets and executable code still come from the current installation.
    The report records configuration hashes and installed package versions.
    """
    report = json.loads(path.read_text(encoding="utf-8"))
    run = TypeAdapter(RunConfiguration).validate_python(report["configuration"])
    if not 0 < run.duration < float("inf"):
        raise ValueError("Inspection duration must be finite and positive")
    return run


def _prepare_report(run: RunConfiguration, destination: Path) -> dict:
    """Save the resolved world profile before execution, including on failure."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    profile = destination.with_suffix(".world.json").resolve()
    profile.write_text(
        run.world_config.model_dump_json(indent=2) + "\n", encoding="utf-8"
    )
    installed = {
        dist.metadata["Name"].lower().replace("_", "-"): dist.version
        for dist in distributions()
    }
    versions = {}
    for package in (
        "robo-arch",
        "drake",
        *(("isaacsim",) if run.world == "isaac" else ()),
    ):
        versions[package] = installed.get(package, "not installed")
    package_root = Path(str(files("robo_arch")))
    application_hashes = {
        path.relative_to(package_root).as_posix(): hashlib.sha256(
            path.read_bytes()
        ).hexdigest()
        for path in sorted(package_root.rglob("*"))
        if path.is_file()
        and not {"__pycache__", "tests"}.intersection(
            path.relative_to(package_root).parts
        )
        and path.name not in {"BUILD.bazel", "README.md"}
        and path.suffix not in {".pyc", ".pyo"}
    }
    return {
        "application_sha256": application_hashes,
        "run_id": str(uuid.uuid4()),
        "status": "starting",
        "configuration": asdict(run),
        "configuration_sha256": {
            str(path): (
                hashlib.sha256(path.read_bytes()).hexdigest()
                if path.is_file()
                else "unavailable"
            )
            for path in run.resources
        },
        "package_versions": versions,
        "inspection_command": _inspection_command(run, destination),
    }


def run_scenario(
    run: RunConfiguration,
    *,
    recording: Path | None = None,
    trace_path: Path | None = None,
    metadata: Path | None = None,
    keep_viewer_open: bool = False,
) -> dict:
    """Execute the configured world; optional files retain run and failure evidence.

    Supplying a recording path explicitly requests native recording. Live viewers
    remain open after execution only when requested by the interactive caller.
    """
    if recording is not None:
        payload = run.world_config.model_dump()
        mode = payload["visualization"]["mode"]
        payload["visualization"]["mode"] = (
            "live_and_record" if mode in {"live", "live_and_record"} else "record"
        )
        run = replace(run, world_config=parse_world(payload))
    visual = run.world_config.visualization
    if visual.mode in {"record", "live_and_record"} and recording is None:
        recording = Path(f"recordings/arm_tracking_{run.world}.html")
    if trace_path is not None and run.world != "isaac":
        raise ValueError("--trace currently records measured Isaac joint states only")
    if recording is not None:
        recording = recording.resolve()
        recording.parent.mkdir(parents=True, exist_ok=True)
    if metadata is None and recording is not None:
        metadata = recording.with_suffix(".json")
    report = _prepare_report(run, metadata) if metadata is not None else None
    try:
        devices = resolve_devices(run)
        definitions = load_definitions(run)
        if len(devices.robots) != 1:
            raise ValueError("Arm tracking requires exactly one actuated robot")
        if run.autonomy.controller != "joint_tracking":
            raise ValueError(
                f"Unsupported arm-tracking controller: {run.autonomy.controller}"
            )
        parameters = JointTrackingParameters.model_validate(run.autonomy.parameters)
        if run.task.type != "joint_tracking":
            raise ValueError(f"Unsupported task evaluator: {run.task.type}")
        task = TrackingTask.model_validate(run.task.parameters)
        if task.robot != devices.robots[0].name:
            raise ValueError("Task must select the configured robot")
        if isinstance(run.world_config, DrakeWorld):
            result = _run_drake(
                run, definitions, parameters, task, recording, keep_viewer_open
            )
        elif isinstance(run.world_config, IsaacWorld):
            result = _run_isaac(
                run,
                definitions,
                parameters,
                task,
                trace_path,
                keep_viewer_open,
            )
        else:
            raise ValueError("No hardware execution runner for arm tracking")
        result["world_configuration"] = run.world_config.model_dump(mode="json")
        if report is not None:
            report.update(
                status="passed" if result["success"] else "failed", result=result
            )
        return result
    finally:
        if report is not None:
            failure = sys.exception()
            if failure is not None:
                report.update(
                    status="error", error=f"{type(failure).__name__}: {failure}"
                )
            metadata.write_text(
                json.dumps(report, default=_json_value, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"Run metadata: {metadata.resolve().as_uri()}", file=sys.stderr)
            print(f"Inspect this run: {report['inspection_command']}", file=sys.stderr)


def _run_drake(
    run: RunConfiguration,
    definitions: DeviceDefinitions,
    parameters: JointTrackingParameters,
    task: TrackingTask,
    recording: Path | None,
    keep_viewer_open: bool,
) -> dict:
    import numpy as np

    from robo_arch.core.worlds.drake.visualization import (
        create_meshcat,
        hold_live,
        save_recording,
        start_recording,
    )
    from robo_arch.scenarios.arm_tracking.drake import build_simulation

    devices = resolve_devices(run)
    visual = run.world_config.visualization
    if recording is not None:
        recording.unlink(missing_ok=True)
    meshcat = create_meshcat(visual)
    if meshcat is not None:
        start_recording(meshcat, visual)
    try:
        simulator, scene = build_simulation(
            run, definitions, parameters, desired_positions=task.target, meshcat=meshcat
        )

        def positions() -> np.ndarray:
            context = scene.plant.GetMyContextFromRoot(simulator.get_context())
            return scene.plant.GetPositions(context, scene.robots[task.robot]).copy()

        def depth_image(name: str) -> np.ndarray:
            camera = scene.cameras[name]
            context = camera.GetMyContextFromRoot(simulator.get_context())
            return camera.depth_image_32F_output_port().Eval(context).data

        initial = positions()
        initial_depth = {
            sensor.name: int(np.isfinite(depth_image(sensor.name)).sum())
            for sensor in devices.sensors
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
        return {
            "world": run.world,
            "duration_seconds": run.duration,
            "robot": task.robot,
            "initial_positions_rad": initial.tolist(),
            "final_positions_rad": final.tolist(),
            "initial_finite_depth_pixels": initial_depth,
            "final_finite_depth_pixels": {
                sensor.name: int(np.isfinite(depth_image(sensor.name)).sum())
                for sensor in devices.sensors
            },
            **evaluate(task, tuple(final)),
        }
    finally:
        if meshcat is not None:
            if recording is not None:
                save_recording(meshcat, recording)
                print(f"Scene playback: {recording.as_uri()}", file=sys.stderr)
                if visual.mode == "record" and visual.open_browser:
                    webbrowser.open(recording.as_uri())
            if keep_viewer_open and visual.mode in {"live", "live_and_record"}:
                hold_live(meshcat)


def _run_isaac(
    run: RunConfiguration,
    definitions: DeviceDefinitions,
    parameters: JointTrackingParameters,
    task: TrackingTask,
    trace_path: Path | None,
    keep_viewer_open: bool,
) -> dict:
    """Use the same CPU control algorithm with measured Isaac joint state."""
    import warnings

    import numpy as np

    from robo_arch.core.controllers.joint_tracking.drake import make_policy
    from robo_arch.core.worlds.drake.scene import build_controller_model
    from robo_arch.core.worlds.isaac.simulation import run_scene

    transfers = (
        " State and commands cross the GPU boundary each step."
        if run.world_config.physics.device == "cuda:0"
        else ""
    )
    warnings.warn(
        "joint_tracking in Isaac evaluates Drake inverse dynamics on the CPU, "
        "with per-environment work in a batch." + transfers,
        RuntimeWarning,
        stacklevel=2,
    )
    robot = resolve_devices(run).robots[0]
    robot_definition = definitions.robots[robot.model]
    model = build_controller_model(robot, robot_definition)
    target = np.asarray(task.target)
    if len(target) != model.num_positions() or not (
        np.all(target >= model.GetPositionLowerLimits())
        and np.all(target <= model.GetPositionUpperLimits())
    ):
        raise ValueError("Task target must match the robot's joints and limits")
    reference = np.r_[target, np.zeros_like(target)]
    command = make_policy(
        model=model,
        parameters=parameters,
        joints=robot_definition.joints,
        desired_state=lambda time: reference,
    )
    if trace_path is not None:
        trace_path.unlink(missing_ok=True)
    trace = run_scene(
        run,
        definitions,
        command,
        trace_path=trace_path,
        keep_viewer_open=keep_viewer_open,
    )
    return {
        "world": run.world,
        "duration_seconds": float(trace["times"][-1]),
        "robot": task.robot,
        "initial_positions_rad": trace["positions"][0].tolist(),
        "final_positions_rad": trace["positions"][-1].tolist(),
        "physics_settings": trace["physics_settings"],
        **evaluate(task, tuple(trace["positions"][-1])),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group()
    inputs.add_argument(
        "--run", default=default_run(), help="Scenario file or package URI"
    )
    inputs.add_argument(
        "--inspect", type=Path, help="Restore resolved inputs from run metadata"
    )
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument(
        "--world",
        choices=("drake", "isaac", "real"),
        help="Replace the world configuration with native defaults",
    )
    selection.add_argument(
        "--world-config", help="Complete world configuration file or package URI"
    )
    viewing = parser.add_mutually_exclusive_group()
    viewing.add_argument(
        "--visualization", choices=("off", "live", "record", "live_and_record")
    )
    viewing.add_argument(
        "--headless", action="store_true", help="Disable the viewer; preserve sensors"
    )
    parser.add_argument(
        "--record", type=Path, help="Native recording output (Drake HTML)"
    )
    parser.add_argument(
        "--trace", type=Path, help="Measured Isaac joint-state NPZ output"
    )
    parser.add_argument(
        "--metadata", type=Path, help="Effective run configuration and result JSON"
    )
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument(
        "--no-sensors", action="store_true", help="Explicitly omit configured sensors"
    )
    args = parser.parse_args()
    run = load_inspection(args.inspect) if args.inspect else load_run(args.run)
    world = run.world_config
    if args.world is not None:
        world = parse_world({"type": args.world})
        run = replace(run, world_source=None)
    elif args.world_config is not None:
        world = load_world(args.world_config)
        source = (
            resolve_resource(args.world_config)
            if args.world_config.startswith("package:")
            else Path(args.world_config).resolve()
        )
        run = replace(run, world_source=source)
    payload = world.model_dump()
    if args.visualization is not None:
        payload["visualization"]["mode"] = args.visualization
    if args.headless:
        payload["visualization"]["mode"] = "record" if args.record else "off"
    if args.record is not None:
        if args.visualization == "off":
            parser.error("--record conflicts with --visualization off")
        mode = payload["visualization"]["mode"]
        payload["visualization"]["mode"] = (
            "live_and_record" if mode in {"live", "live_and_record"} else "record"
        )
    if args.no_browser and isinstance(world, DrakeWorld):
        payload["visualization"]["open_browser"] = False
    # Validate overrides as a complete native config, including unsupported modes.
    run = replace(run, world_config=parse_world(payload))
    if args.no_sensors:
        run = replace(run, sensors_enabled=False)
    destination = Path(os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR", "recordings"))
    metadata = args.metadata or (
        args.record.with_suffix(".json")
        if args.record
        else (
            args.inspect.with_name(args.inspect.stem + "_inspection.json")
            if args.inspect
            else destination / f"arm_tracking_{run.world}.json"
        )
    )
    recording = args.record
    if (
        args.inspect
        and recording is None
        and run.world_config.visualization.mode in {"record", "live_and_record"}
    ):
        recording = metadata.with_suffix(".html")
    result = run_scenario(
        run,
        recording=recording,
        trace_path=args.trace,
        metadata=metadata,
        keep_viewer_open=True,
    )
    print(json.dumps(result, indent=2))
    if not result["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
