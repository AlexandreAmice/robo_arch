"""Tensor effort execution for sensor-free, fixed-base Isaac Lab articulations."""

from collections.abc import Callable

import torch
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
        self.environment = scene.environment
        self.efforts = self.environment.efforts
        self.times = self.environment.times

    @property
    def time(self):
        return self.environment.time

    @time.setter
    def time(self, value):
        self.environment.time = value

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
        ids = mask.nonzero(as_tuple=False).squeeze(-1).int()
        if len(ids):
            self.environment._reset_idx(ids)

    def configure_episodes(self, *, status, reset, observations=None):
        """Install task callbacks; Lab owns termination capture and physical reset."""
        self.environment.task_observations = observations
        self.environment.episode_status = status
        self.environment.after_reset = reset

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
            self.set_effort(name, effort)
        self.advance(dt)

    def set_effort(self, name: str, effort: Tensor) -> None:
        """Validate and copy one robot's command into Lab's actuator buffer.

        Commands are converted to the native state dtype on the simulation device.
        Finite checks synchronize a status scalar; state and effort stay on device.
        Controllers needing rounding guards must validate this native dtype.
        """
        expected = self.initial[name]
        if (
            not isinstance(effort, Tensor)
            or effort.shape != expected.shape
            or effort.device != expected.device
            or not bool(torch.isfinite(effort).all())
        ):
            raise ValueError(f"Invalid tensor effort for {name}")
        converted = effort.to(dtype=expected.dtype).contiguous()
        if not bool(torch.isfinite(converted).all()):
            raise ValueError(f"Effort overflows native dtype for {name}")
        self.efforts[name] = converted
        self.scene.native.articulations[name].actuators.target_command.set_effort_index(
            value=converted
        )

    def advance(self, dt: float) -> None:
        """Write buffered commands, step Lab once, and refresh its state buffers."""
        return self.environment.advance(dt)
