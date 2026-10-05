"""Camera task composition over batched CUDA control and native Isaac physics."""

import math
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.controllers.cbf.isaac import build_filter
from robo_arch.core.controllers.selection import select_controller, tensor_policy
from robo_arch.core.worlds.drake.scene import build_controller_model
from robo_arch.core.worlds.isaac.scene import IsaacScene
from robo_arch.scenarios.camera_protection.reference import desired_state
from robo_arch.scenarios.camera_protection.setup import prepare


def configure(
    scene: IsaacScene,
    *,
    run: RunConfiguration,
    filtered: bool = True,
    description: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Build one independent nominal model and one Moreau workspace per batch."""
    setup = prepare(run, scene.devices, scene.definitions)
    control, task = setup.control, setup.task
    robot, definition, geometry = setup.robot, setup.definition, setup.geometry
    model = build_controller_model(
        robot, definition, sensors=scene.devices.sensors, definitions=scene.definitions
    )
    setup.validate_targets(
        model.GetPositionLowerLimits(), model.GetPositionUpperLimits()
    )
    cbf = build_filter(
        model=model,
        geometry=geometry,
        joints=definition.joints,
        parameters=control,
        batch_size=run.world_config.num_envs,
        device=run.world_config.physics.device,
        command_dtype=torch.float32,
    )
    if description is not None:
        description.update(
            setup.describe(
                pair_names=cbf.pair_names,
                velocity_bound_names=cbf.velocity_bound_names,
                qp_constraint_count=cbf.qp_constraint_count,
            )
        )
    initial = robot.initial_positions or definition.default_positions
    initial = torch.tensor(initial, device=cbf.device, dtype=cbf.dtype)
    cbf.validate_initial_state(
        torch.cat((initial, torch.zeros_like(initial)))[None, :]
        .expand(run.world_config.num_envs, -1)
        .contiguous()
    )
    selection = select_controller(
        run.world_config, "cbf", batched=True, sensors_enabled=run.sensors_enabled
    )
    if description is not None:
        description["controller_selection"] = selection.describe()
    nominal = tensor_policy(control.nominal_controller, cbf.model, control.nominal)
    unsafe = torch.tensor(task.unsafe_target, device=cbf.device, dtype=cbf.dtype)
    retreat = torch.tensor(task.retreat_target, device=cbf.device, dtype=cbf.dtype)

    class Command:
        diagnostics: dict[str, torch.Tensor]

        def __init__(self):
            self.diagnostics = {}
            self.statistics = {}

        def __call__(self, state: torch.Tensor, times: torch.Tensor) -> torch.Tensor:
            q, v = desired_state(
                times,
                initial,
                unsafe,
                retreat,
                retreat_time=task.retreat_time,
                transition_seconds=task.transition_seconds,
                namespace=torch,
            )
            evaluation = cbf.model.evaluate(state)
            effort = nominal.effort(state, q, v, evaluation=evaluation)
            self.diagnostics = {"nominal_effort": effort}
            if filtered:
                result = cbf.filter(state, effort, evaluation=evaluation)
                effort = result.effort
                self.diagnostics["cbf/diagnostics"] = result.diagnostics
            else:
                self.diagnostics["baseline/clearance"] = cbf.evaluate(
                    state, evaluation=evaluation
                ).clearance
            self.diagnostics["commanded_effort"] = effort
            count = cbf.constraint_count
            clearances = (
                self.diagnostics["cbf/diagnostics"][:, :count]
                if filtered
                else self.diagnostics["baseline/clearance"]
            )
            values = {"minimum_clearance_m": clearances.amin(dim=1)}
            if geometry.plane_pairs:
                values["minimum_ground_clearance_m"] = clearances[
                    :, len(geometry.pairs) :
                ].amin(dim=1)
            if filtered:
                values["minimum_cbf_residual"] = result.diagnostics[
                    :, 3 * count : 4 * count
                ].amin(dim=1)
                if result.velocity_slack.shape[1]:
                    values["minimum_joint_velocity_slack"] = result.velocity_slack.amin(
                        dim=1
                    )
                    values["minimum_velocity_cbf_residual"] = (
                        result.velocity_residual.amin(dim=1)
                    )
                correction = result.diagnostics[:, 5 * count]
                values["maximum_torque_correction_Nm"] = torch.where(
                    times < task.retreat_time, correction, torch.zeros_like(correction)
                )
            for name, value in values.items():
                previous = self.statistics.get(name, value)
                self.statistics[name] = (
                    torch.maximum(previous, value)
                    if name.startswith("maximum")
                    else torch.minimum(previous, value)
                )
            return effort

    return {control.robot: Command()}


def rollout(
    scene: IsaacScene,
    viewer: Any,
    *,
    run: RunConfiguration,
    filtered: bool = True,
    description: dict[str, Any] | None = None,
    trace_path: Path | None = None,
) -> dict[str, Any]:
    """Evaluate camera protection every step on the shared Lab tensor runtime.

    Exported traces may be sampled less often; controller statistics retain every
    evaluation, including the terminal state. Physics uses only Lab's scene and
    actuator buffers. Partial traces survive failures in control or stepping.
    """
    from robo_arch.core.worlds.isaac.batched import BatchedExecution
    from robo_arch.scenarios.camera_protection.configuration import TaskParameters

    task_parameters = TaskParameters.model_validate(run.task.parameters)
    execution = BatchedExecution(scene)
    commands = configure(scene, run=run, filtered=filtered, description=description)
    if set(commands) != set(execution.initial):
        raise ValueError("Commands must name exactly the configured robots")
    times: list[float] = []
    samples: dict[str, list[np.ndarray]] = {}

    def evaluate_commands() -> None:
        for name, arm in scene.native.articulations.items():
            state = torch.cat((arm.data.joint_pos.torch, arm.data.joint_vel.torch), -1)
            if not bool(torch.isfinite(state).all()):
                raise RuntimeError(f"Isaac returned invalid CUDA state for {name}")
            execution.set_effort(name, commands[name](state.double(), execution.times))

    def sample() -> None:
        values = {"environment_times": execution.times}
        for name, arm in scene.native.articulations.items():
            values[name + "/q"] = arm.data.joint_pos.torch
            values[name + "/v"] = arm.data.joint_vel.torch
            values[name + "/effort"] = execution.efforts[name]
        for command in commands.values():
            for name, value in command.diagnostics.items():
                if name in values or name == "times" or name.endswith("/times"):
                    raise ValueError(f"Conflicting diagnostic channel {name!r}")
                values[name] = value
        if samples and samples.keys() != values.keys():
            raise ValueError("Logged controller channels changed during execution")
        times.append(execution.time)
        for name, value in values.items():
            samples.setdefault(name, []).append(value.detach().cpu().numpy().copy())

    def trace() -> dict[str, np.ndarray]:
        result = {"times": np.asarray(times)}
        for name, values in samples.items():
            result[name] = np.asarray(values)
            result[name + "/times"] = result["times"]
        return result

    steps = math.ceil(run.duration / run.time_step)
    torch.cuda.synchronize()
    started = time.monotonic()
    if viewer is not None:
        viewer.wall_start = started
    try:
        for step in range(steps):
            evaluate_commands()
            if step % task_parameters.log_every_n_steps == 0:
                sample()
            execution.advance(min(run.time_step, run.duration - execution.time))
            if viewer is not None:
                viewer.update(execution.time, scene.world.target_realtime_rate)
        evaluate_commands()
        sample()
        torch.cuda.synchronize()
        elapsed = time.monotonic() - started
        return {
            "trace": trace(),
            "simulation_wall_seconds": elapsed,
            "realtime_rate": run.duration / elapsed,
            "environment_steps_per_second": scene.world.num_envs * steps / elapsed,
            "controller_statistics": {
                name: {
                    key: value.detach().cpu().numpy().copy()
                    for key, value in command.statistics.items()
                }
                for name, command in commands.items()
            },
        }
    finally:
        if trace_path is not None and times:
            trace_path.parent.mkdir(parents=True, exist_ok=True)
            np.savez(trace_path, **trace())
