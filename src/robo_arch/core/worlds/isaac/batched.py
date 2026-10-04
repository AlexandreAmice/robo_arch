"""Tensor effort execution for sensor-free, fixed-base Isaac Lab articulations."""

from collections.abc import Callable

import torch
import warp as wp
from torch import Tensor

from robo_arch.core.worlds.isaac.scene import IsaacScene, initialize_scene


class BatchedExecution:
    """Advance all environments with one tensor callback per robot.

    State views are borrowed until the next simulation write. This path supports
    the scene loader's stateless effort actuators, without sensor observations or
    externally applied wrenches. Episode/task state is owned by the scenario.
    """

    def __init__(self, scene: IsaacScene) -> None:
        if scene.observations:
            raise ValueError("Tensor execution does not support sensor observations")
        self.scene = scene
        device = scene.world.physics.device
        self.initial = {
            name: torch.as_tensor(q, device=device).repeat(scene.world.num_envs, 1)
            for name, q in initialize_scene(scene).items()
        }
        self.efforts = {name: torch.zeros_like(q) for name, q in self.initial.items()}
        self.time = 0.0
        scene.native.reset()
        self.reset(torch.ones(scene.world.num_envs, dtype=torch.bool, device=device))

    def reset(self, mask: Tensor) -> None:
        """Reset selected rows without copying masks or state through NumPy.

        PhysX internally compacts masks to indices; Newton consumes native masks.
        No actuator-wide reset is used: it would clear unselected target buffers
        in the pinned PhysX SDK. These effort actuators have no private memory.
        """
        if mask.shape != (self.scene.world.num_envs,) or mask.dtype != torch.bool:
            raise ValueError("Reset mask must contain one boolean per environment")
        if str(mask.device) != self.scene.world.physics.device:
            raise ValueError("Reset mask must be on the simulation device")
        native_mask = wp.from_torch(mask, dtype=wp.bool)
        for name, arm in self.scene.native.articulations.items():
            zero = torch.zeros_like(self.initial[name])
            arm.write_joint_state_to_sim_mask(
                position=self.initial[name], velocity=zero, env_mask=native_mask
            )
            arm.actuators.target_command.set_effort_mask(
                value=zero, env_mask=native_mask
            )
            self.efforts[name] = torch.where(mask[:, None], zero, self.efforts[name])

    def step(
        self, command: Callable[[str, Tensor, Tensor, Tensor], Tensor], dt: float
    ) -> None:
        """Evaluate tensor feedback and advance physics once, with no host samples."""
        for name, arm in self.scene.native.articulations.items():
            effort = command(
                name,
                arm.data.joint_pos.torch,
                arm.data.joint_vel.torch,
                arm.data.gravity_compensation_forces.torch,
            )
            if effort.shape != self.initial[name].shape:
                raise ValueError(f"Invalid tensor effort shape for {name}")
            self.efforts[name] = effort
            arm.actuators.target_command.set_effort_index(value=effort)
        self.scene.native.write_data_to_sim()
        self.scene.simulation.cfg.dt = dt
        self.scene.simulation.step(render=False)
        self.scene.native.update(dt)
        self.time += dt
