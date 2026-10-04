"""Camera task composition over batched CUDA control and native Isaac physics."""

from typing import Any

import torch

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.controllers.cbf.isaac import build_filter
from robo_arch.core.controllers.joint_tracking.torch import TensorJointTracking
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
        batch_size=run.world_config.batch_size,
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
        .expand(run.world_config.batch_size, -1)
        .contiguous()
    )
    nominal = TensorJointTracking(cbf.model, control.nominal)
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
