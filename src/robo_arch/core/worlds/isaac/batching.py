"""USD environment copies and native CUDA articulation control."""

from typing import Any

import numpy as np

from robo_arch.core.worlds.isaac.config import IsaacWorld
from robo_arch.core.worlds.isaac.scene import IsaacScene


def clone_environments(scene: IsaacScene, config: IsaacWorld) -> None:
    """Clone the assembled source with isolated collisions and translated anchors.

    USD references remap body relationships into each copy. Fixed joints attached
    to world additionally need their world-side frame translated explicitly.
    Joint states and nominal control geometry remain in source coordinates.
    """
    from pxr import Gf, UsdGeom, UsdPhysics

    stage = scene.stage
    paths = tuple(f"/_environments/env_{i:06d}" for i in range(config.batch_size))
    width = int(np.ceil(np.sqrt(config.batch_size)))
    origins = (
        np.array(
            [(i % width, i // width, 0) for i in range(config.batch_size)], dtype=float
        )
        * config.environment_spacing
    )
    source = paths[0]
    source_joints = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith(source + "/")
        and prim.IsA(UsdPhysics.FixedJoint)
        and not UsdPhysics.Joint(prim).GetBody0Rel().GetTargets()
    ]
    for path, origin in zip(paths[1:], origins[1:], strict=True):
        root = UsdGeom.Xform.Define(stage, path)
        root.GetPrim().GetReferences().AddInternalReference(source)
        root.AddTranslateOp().Set(Gf.Vec3d(*origin))
        for source_joint in source_joints:
            joint = UsdPhysics.Joint.Get(stage, path + source_joint[len(source) :])
            position = joint.GetLocalPos0Attr().Get()
            joint.GetLocalPos0Attr().Set(Gf.Vec3f(*(np.asarray(position) + origin)))

    # Native inverted filtering permits only self and global-ground groups.
    # This remains correct even for overlapping clone layouts or large fixtures.
    scene.physics.CreateInvertCollisionGroupFilterAttr(True)
    ground = None
    if config.ground:
        ground = UsdPhysics.CollisionGroup.Define(stage, "/_collisions/ground")
        ground.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(
            "/_world/ground"
        )
        ground.CreateFilteredGroupsRel().AddTarget(ground.GetPath())
    for i, path in enumerate(paths):
        group = UsdPhysics.CollisionGroup.Define(stage, f"/_collisions/env_{i:06d}")
        group.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(path)
        group.CreateFilteredGroupsRel().AddTarget(group.GetPath())
        if ground is not None:
            group.GetFilteredGroupsRel().AddTarget(ground.GetPath())
            ground.GetFilteredGroupsRel().AddTarget(group.GetPath())
    scene.environment_paths = paths
    scene.environment_origins = origins


class TorchArticulations:
    """One tensor view per robot, with independent environment clocks and reset.

    State/effort buffers stay on CUDA. Float64 controller inputs are cast from
    native float32 state on-device; accepted commands are cast back on-device.
    Sampling explicitly copies selected trace rows to the host. Finite-value
    checks synchronize only a boolean status, never the state/effort arrays.
    """

    def __init__(self, scene: IsaacScene, view: Any):
        import torch

        self.scene = scene
        self.view = view
        self.closed = False
        self.device = torch.device("cuda:0")
        if torch.device(view.device) != self.device:
            raise ValueError("Native tensor view must reside on CUDA device zero")
        self.count = len(scene.environment_paths)
        if not self.count:
            raise ValueError("Torch articulation control requires cloned environments")
        self.indices = torch.arange(self.count, device=self.device, dtype=torch.int32)
        self.times = torch.zeros(self.count, device=self.device, dtype=torch.float64)
        self.arms: dict[str, Any] = {}
        self.initial: dict[str, Any] = {}
        self.efforts: dict[str, Any] = {}
        initialized = False
        try:
            self._initialize()
            initialized = True
        finally:
            if not initialized:
                self.close()

    def _initialize(self) -> None:
        import torch

        scene = self.scene
        source = scene.environment_paths[0]
        for robot in scene.devices.robots:
            root = scene.roots[robot.name]
            paths = [path + root[len(source) :] for path in scene.environment_paths]
            arm = self.view.create_articulation_view(paths)
            self.arms[robot.name] = arm
            definition = scene.definitions.robots[robot.model]
            if arm.count != self.count or not arm.shared_metatype.fixed_base:
                raise ValueError(
                    f"Expected {self.count} fixed-base copies of {robot.name}"
                )
            if tuple(arm.shared_metatype.dof_names) != definition.joints:
                raise ValueError(f"Isaac joint order differs for {robot.name}")
            # Exact paths also make the environment-to-row association explicit.
            if list(arm.prim_paths) != paths:
                raise ValueError(
                    f"Isaac articulation ordering differs for {robot.name}"
                )
            initial = (
                definition.default_positions
                if robot.initial_positions is None
                else robot.initial_positions
            )
            q = torch.tensor(initial, device=self.device, dtype=torch.float32)
            if q.shape != (len(definition.joints),) or not bool(
                torch.isfinite(q).all()
            ):
                raise ValueError(f"Invalid initial positions for {robot.name}")
            limits = arm.get_dof_limits().to(device=self.device)
            if not bool(((q >= limits[..., 0]) & (q <= limits[..., 1])).all()):
                raise ValueError(f"Initial positions exceed limits for {robot.name}")
            self.initial[robot.name] = q.expand(self.count, -1).contiguous()
            self.efforts[robot.name] = torch.zeros_like(self.initial[robot.name])
        self.reset(self.indices)

    def close(self) -> None:
        """Release native GPU views before the pinned Kit plugins unload.

        Tensor API 110.3.2 exposes invalidation but no wrapper close method.
        Explicitly dropping its private backend references prevents late Python
        frame collection from calling GPU destructors after plugin shutdown.
        """
        if self.closed:
            return
        for arm in self.arms.values():
            arm._backend = None
        self.arms.clear()
        try:
            self.view.invalidate()
        finally:
            self.view._backend = None
            self.view = None
            self.closed = True

    def _require_open(self) -> None:
        if self.closed:
            raise RuntimeError("Isaac tensor runtime is closed")

    def reset(self, indices: Any) -> None:
        """Restore selected environments between steps; indices are CUDA int32.

        This resets native positions, velocities, commands and environment time.
        Stateful policies must reset their own corresponding entries separately.
        """
        import torch

        self._require_open()
        if (
            not isinstance(indices, torch.Tensor)
            or indices.device != self.device
            or indices.dtype != torch.int32
            or indices.ndim != 1
        ):
            raise ValueError(
                "Reset indices must be a one-dimensional CUDA int32 tensor"
            )
        if not bool(((indices >= 0) & (indices < self.count)).all()):
            raise ValueError("Reset indices are outside the environment batch")
        if torch.unique(indices).numel() != indices.numel():
            raise ValueError("Reset indices must be unique")
        if not indices.numel():
            return
        indices = indices.contiguous()
        for name, arm in self.arms.items():
            arm.set_dof_positions(self.initial[name], indices)
            arm.set_dof_velocities(torch.zeros_like(self.initial[name]), indices)
            self.efforts[name][indices] = 0
            arm.set_dof_actuation_forces(self.efforts[name], indices)
        self.times[indices] = 0

    def apply(self, commands: dict[str, Any]) -> None:
        import torch

        self._require_open()
        for name, arm in self.arms.items():
            state = torch.cat(
                (arm.get_dof_positions(), arm.get_dof_velocities()), dim=-1
            ).to(dtype=torch.float64)
            if state.device != self.device or not bool(torch.isfinite(state).all()):
                raise RuntimeError(f"Isaac returned invalid CUDA state for {name}")
            effort = commands[name](state, self.times)
            if (
                not isinstance(effort, torch.Tensor)
                or effort.device != self.device
                or effort.shape != self.efforts[name].shape
                or not bool(torch.isfinite(effort).all())
            ):
                raise ValueError(f"Controller returned invalid CUDA effort for {name}")
            self.efforts[name] = effort.to(dtype=torch.float32).contiguous()
            if not bool(torch.isfinite(self.efforts[name]).all()):
                raise ValueError(
                    f"Controller effort overflows native float32 for {name}"
                )
            arm.set_dof_actuation_forces(self.efforts[name], self.indices)

    def sample(self, commands: dict[str, Any]) -> dict[str, np.ndarray]:
        """Copy one measured state/command/diagnostics sample to owned host arrays."""
        import torch

        self._require_open()
        values = {"environment_times": self.times}
        for name, arm in self.arms.items():
            values[name + "/q"] = arm.get_dof_positions()
            values[name + "/v"] = arm.get_dof_velocities()
            values[name + "/effort"] = self.efforts[name]
        for command in commands.values():
            for name, value in getattr(command, "diagnostics", {}).items():
                if (
                    not name
                    or name in values
                    or name == "times"
                    or name.endswith("/times")
                ):
                    raise ValueError(f"Conflicting diagnostic channel {name!r}")
                if (
                    not isinstance(value, torch.Tensor)
                    or value.device != self.device
                    or value.ndim < 1
                    or value.shape[0] != self.count
                ):
                    raise ValueError(f"Invalid batched diagnostic tensor {name!r}")
                values[name] = value
        return {
            name: value.detach().cpu().numpy().copy() for name, value in values.items()
        }

    def link_poses(self) -> dict[str, np.ndarray]:
        """Copy native world link poses only when a viewer requests a frame.

        Values are owned host xyz/xyzw arrays, independent of control buffers.
        The simulation view uses global coordinates, including clone origins.
        """
        self._require_open()
        result = {}
        for arm in self.arms.values():
            poses = arm.get_link_transforms().detach().cpu().numpy()
            for paths, row in zip(arm.link_paths, poses, strict=True):
                result.update(
                    (path, pose.copy()) for path, pose in zip(paths, row, strict=True)
                )
        return result
