"""Native Lab owner for physical state, command cadence and episode lifecycle.

Import only after AppLauncher startup. Scenarios supply policy/task callbacks;
DirectRLEnv owns stepping, terminal observation capture and selective autoreset.
"""

from collections.abc import Callable
from pathlib import Path

import torch
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg

from robo_arch.core.worlds.isaac.backend import physics_config
from robo_arch.core.worlds.isaac.scene import (
    initialize_scene,
    populate_scene,
    reset_objects,
)


class Environment(DirectRLEnv):
    """One physical tick per control evaluation, on either pinned Lab backend.

    Decimation stays one so physics backends cannot hold a feedback command over
    hidden internal environment steps. A slower task/reference may hold its input
    while feedback is recomputed each tick. Duration must be an integer number
    of ticks; the native environment never mutates the integration step mid-run.
    """

    def __init__(self, configuration, world, *, directory: Path):
        self.configuration = configuration
        self.world = world
        self.directory = directory
        self.before_step: Callable[[], None] | None = None
        self.after_reset: Callable[[torch.Tensor], None] | None = None
        self.task_observations: Callable[[], dict[str, torch.Tensor]] | None = None
        self.episode_status: Callable[[], tuple[torch.Tensor, torch.Tensor]] | None = (
            None
        )
        super().__init__(
            DirectRLEnvCfg(
                seed=0,
                decimation=1,
                episode_length_s=1e12,
                compute_final_obs=True,
                action_space=0,
                observation_space=0,
                ui_window_class_type=None,
                scene=InteractiveSceneCfg(
                    num_envs=world.num_envs,
                    env_spacing=world.env_spacing,
                    replicate_physics=False,
                ),
                sim=SimulationCfg(
                    dt=world.physics.time_step,
                    device=world.physics.device,
                    physics=physics_config(world.physics),
                    use_fabric=False,
                    create_stage_in_memory=False,
                    use_newton_actuators=False,
                    visualizer_cfgs=[],
                ),
            )
        )
        self.render_enabled = False
        self.sim.set_setting("/app/player/playSimulations", False)
        self.initial = {
            name: torch.as_tensor(q, device=self.device).repeat(self.num_envs, 1)
            for name, q in initialize_scene(self.assembly).items()
        }
        self.efforts = {name: torch.zeros_like(q) for name, q in self.initial.items()}
        self.time = 0.0
        self.times = torch.zeros(self.num_envs, device=self.device, dtype=torch.float64)
        self._empty_action = torch.empty((self.num_envs, 0), device=self.device)
        self.reset()

    def _setup_scene(self):
        self.assembly = populate_scene(
            self.configuration,
            self.world,
            directory=self.directory,
            simulation=self.sim,
            native=self.scene,
        )
        self.assembly.environment = self

    def _pre_physics_step(self, action):
        self.extras.pop("final_obs", None)

    def _apply_action(self):
        if self.before_step is not None:
            self.before_step()
        for name, arm in self.scene.articulations.items():
            arm.actuators.target_command.set_effort_index(value=self.efforts[name])

    def _get_dones(self):
        self.time += self.step_dt
        self.times.add_(self.step_dt)
        if self.episode_status is not None:
            terminated, truncated = self.episode_status()
            expected = (self.num_envs,)
            if any(
                x.shape != expected
                or x.dtype != torch.bool
                or x.device != self.times.device
                for x in (terminated, truncated)
            ):
                raise ValueError(
                    "Task termination masks must be boolean rows on the simulation device"
                )
            return terminated, truncated
        return torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        ), torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    def _get_rewards(self):
        return torch.zeros(self.num_envs, device=self.device)

    def _get_observations(self):
        values = {
            "policy": torch.empty((self.num_envs, 0), device=self.device),
            "environment_times": self.times.clone(),
        }
        for name, arm in self.scene.articulations.items():
            values[name + "/q"] = arm.data.joint_pos.torch.clone()
            values[name + "/v"] = arm.data.joint_vel.torch.clone()
            values[name + "/effort"] = self.efforts[name].clone()
        for name, obj in self.scene.rigid_objects.items():
            values[name + "/pose"] = obj.data.root_link_pose_w.torch.clone()
            values[name + "/twist"] = obj.data.root_link_vel_w.torch.clone()
        if self.task_observations is not None:
            task = self.task_observations()
            if values.keys() & task.keys():
                raise ValueError("Task observation names overlap physical channels")
            values.update({name: value.clone() for name, value in task.items()})
        return values

    def _reset_idx(self, env_ids):
        # Native Lab resets its selected asset/sensor/event buffers first.
        super()._reset_idx(env_ids)
        for name, arm in self.scene.articulations.items():
            q = self.initial[name][env_ids]
            arm.write_joint_state_to_sim_index(
                position=q, velocity=torch.zeros_like(q), env_ids=env_ids
            )
            self.efforts[name][env_ids] = 0
            arm.actuators.target_command.set_effort_index(value=self.efforts[name])
        reset_objects(self.assembly, env_ids=env_ids)
        self.times[env_ids] = 0
        if self.after_reset is not None:
            self.after_reset(env_ids)

    def advance(self, dt: float):
        if abs(dt - self.physics_dt) > 1e-12:
            raise ValueError(
                "Native Isaac execution requires complete fixed physics ticks"
            )
        return self.step(self._empty_action)

    def close(self):
        self.before_step = None
        self.after_reset = None
        self.episode_status = None
        self.task_observations = None
        if hasattr(self, "assembly"):
            self.assembly.environment = None
        super().close()
