"""Extract independent device models from a finalized Drake plant."""

from collections.abc import Sequence

import numpy as np
import torch
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import PrismaticJoint, RevoluteJoint, WeldJoint

from robo_arch.core.controllers.dynamics.torch import Body, TensorModel


def build_tensor_model(
    model: MultibodyPlant,
    joints: tuple[str, ...],
    frames: tuple[str, ...],
    points: Sequence[Sequence[float]] | np.ndarray,
    device: str = "cuda:0",
) -> TensorModel:
    """Copy tree, inertias and query points into owned float64 device constants.

    Extraction accepts ordered scalar revolute/prismatic joints and welds with
    identity actuation, uniform gravity and joint damping. Unsupported joints,
    extra force elements, loops and floating bases fail here, before execution.
    The returned model retains no Drake plant, context or native array views.
    """
    if not model.is_finalized():
        raise ValueError("Tensor dynamics requires a finalized model")
    count = model.num_velocities()
    if (
        count == 0
        or model.num_positions() != count
        or model.num_actuated_dofs() != count
    ):
        raise ValueError("Tensor dynamics requires fixed-base fully actuated joints")
    actuators = [model.get_joint_actuator(i) for i in model.GetJointActuatorIndices()]
    if tuple(a.joint().name() for a in actuators) != joints or any(
        a.joint().num_positions() != 1
        or a.joint().num_velocities() != 1
        or a.joint().position_start() != i
        or a.joint().velocity_start() != i
        for i, a in enumerate(actuators)
    ):
        raise ValueError(
            "Tensor dynamics requires matching scalar q, v and actuator order"
        )
    if not np.array_equal(model.MakeActuationMatrix(), np.eye(count)):
        raise ValueError("Tensor dynamics requires identity actuation")
    if model.num_force_elements() != 1:
        raise ValueError("Tensor dynamics supports only gravity and joint damping")
    if model.num_constraints():
        raise ValueError("Tensor dynamics does not support constrained mechanisms")
    velocity_lower = tuple(model.GetVelocityLowerLimits())
    velocity_upper = tuple(model.GetVelocityUpperLimits())
    # Resolve shorthand "cuda" to its concrete device index during setup.
    tensor_device = torch.empty(0, device=device).device
    dtype = torch.float64

    def tensor(value) -> torch.Tensor:
        return torch.tensor(np.asarray(value), device=tensor_device, dtype=dtype)

    limits = tensor([a.effort_limit() for a in actuators])
    if not all(
        np.isfinite(a.effort_limit()) and a.effort_limit() > 0 for a in actuators
    ):
        raise ValueError("Tensor dynamics requires finite positive effort limits")
    damping = tensor([a.joint().default_damping() for a in actuators])
    rotor = tensor(
        [a.default_rotor_inertia() * a.default_gear_ratio() ** 2 for a in actuators]
    )
    pending = [model.get_joint(i) for i in model.GetJointIndices()]
    body_indices = {int(model.world_body().index()): 0}
    bodies = []
    while pending:
        ready = [j for j in pending if int(j.parent_body().index()) in body_indices]
        if not ready:
            raise ValueError("Tensor dynamics requires a world-rooted body tree")
        for joint in ready:
            pending.remove(joint)
            child = joint.child_body()
            if int(child.index()) in body_indices:
                raise ValueError("Tensor dynamics requires a tree without loops")
            parent_pose = joint.frame_on_parent().GetFixedPoseInBodyFrame()
            child_pose = joint.frame_on_child().GetFixedPoseInBodyFrame().inverse()
            dof = None
            axis = np.zeros(3)
            if isinstance(joint, RevoluteJoint):
                kind, axis, dof = (
                    "revolute",
                    joint.revolute_axis(),
                    joint.velocity_start(),
                )
            elif isinstance(joint, PrismaticJoint):
                kind, axis, dof = (
                    "prismatic",
                    joint.translation_axis(),
                    joint.velocity_start(),
                )
            elif isinstance(joint, WeldJoint):
                kind = "weld"
                parent_pose = parent_pose @ joint.X_FM()
            else:
                raise ValueError(
                    f"Unsupported tensor dynamics joint: {joint.type_name()}"
                )
            x, y, z = axis
            cross = [[0, -z, y], [z, 0, -x], [-y, x, 0]]
            spatial = child.default_spatial_inertia()
            com = spatial.get_com()
            inertia = spatial.Shift(com).CalcRotationalInertia().CopyToFullMatrix3()
            gravity = (
                model.gravity_field().gravity_vector()
                if model.is_gravity_enabled(child.model_instance())
                else np.zeros(3)
            )
            bodies.append(
                Body(
                    parent=body_indices[int(joint.parent_body().index())],
                    dof=dof,
                    kind=kind,
                    rotation_parent=tensor(parent_pose.rotation().matrix()),
                    translation_parent=tensor(parent_pose.translation()),
                    rotation_child=tensor(child_pose.rotation().matrix()),
                    translation_child=tensor(child_pose.translation()),
                    axis=tensor(axis),
                    axis_cross=tensor(cross),
                    mass=spatial.get_mass(),
                    com=tensor(com),
                    inertia=tensor(inertia),
                    gravity=tensor(gravity),
                )
            )
            body_indices[int(child.index())] = len(bodies)
    if len(body_indices) != model.num_bodies():
        raise ValueError(
            "Tensor dynamics requires every body welded or joined to world"
        )
    if len(frames) != len(points) or not frames:
        raise ValueError("Tensor dynamics requires one frame per query point")
    indices, local = [], []
    for name, point in zip(frames, points, strict=True):
        if name == "world":
            frame = model.world_frame()
        else:
            instance, frame_name = name.rsplit("/", 1)
            frame = model.GetFrameByName(
                frame_name, model.GetModelInstanceByName(instance)
            )
        point = np.asarray(point, dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError("Tensor dynamics query points must be finite triples")
        indices.append(body_indices[int(frame.body().index())])
        local.append(frame.GetFixedPoseInBodyFrame() @ point)
    return TensorModel(
        joints=joints,
        velocity_lower=velocity_lower,
        velocity_upper=velocity_upper,
        limits=limits,
        damping=damping,
        rotor=rotor,
        bodies=tuple(bodies),
        point_bodies=torch.tensor(indices, device=tensor_device, dtype=torch.long),
        points=tensor(local),
    )
