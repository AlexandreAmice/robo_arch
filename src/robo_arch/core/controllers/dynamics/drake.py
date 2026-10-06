"""Extract independent device models from a finalized Drake plant."""

from collections.abc import Sequence

import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import BodyIndex, PrismaticJoint, RevoluteJoint, WeldJoint

from robo_arch.core.controllers.dynamics.jax import NominalModel


def build_tensor_model(
    model: MultibodyPlant,
    joints: tuple[str, ...],
    frames: tuple[str, ...],
    points: Sequence[Sequence[float]] | np.ndarray,
    device: str = "cuda:0",
) -> NominalModel:
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
    import jax
    import jax.numpy as jnp
    from jaxsim.api.model import JaxSimModel
    from jaxsim.parsers.descriptions import (
        JointDescription,
        LinkDescription,
        ModelDescription,
    )

    jax.config.update("jax_enable_x64", True)
    limits = np.asarray([a.effort_limit() for a in actuators])
    if not np.all((~np.isnan(limits)) & (limits > 0)):
        raise ValueError("Tensor dynamics requires positive effort limits")
    damping = np.asarray([a.joint().default_damping() for a in actuators])
    rotor = np.asarray(
        [a.default_rotor_inertia() * a.default_gear_ratio() ** 2 for a in actuators]
    )
    gravity = np.asarray(model.gravity_field().gravity_vector())
    if not np.array_equal(gravity[:2], [0, 0]):
        raise ValueError("Shared dynamics supports world-z gravity only")
    if any(
        not model.is_gravity_enabled(model.get_body(BodyIndex(i)).model_instance())
        for i in range(1, model.num_bodies())
    ):
        raise ValueError("Shared dynamics requires gravity enabled for every body")
    links = [LinkDescription(name="world", mass=0.0, inertia=jnp.zeros((6, 6)))]
    descriptions = []
    body_poses = {0: np.eye(4)}
    pending = [model.get_joint(i) for i in model.GetJointIndices()]
    body_indices = {int(model.world_body().index()): 0}

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
            # JaxSim spatial vectors put linear before angular components.
            spatial = (
                child.default_spatial_inertia()
                .ReExpress(child_pose.rotation())
                .Shift(-child_pose.translation())
            )
            order = [3, 4, 5, 0, 1, 2]
            link = LinkDescription(
                name=f"body_{int(child.index())}",
                mass=spatial.get_mass(),
                inertia=jnp.asarray(spatial.CopyToFullMatrix6()[np.ix_(order, order)]),
                pose=jnp.eye(4),
            )
            descriptions.append(
                JointDescription(
                    name=joint.name()
                    if dof is not None
                    else f"fixed_{int(child.index())}",
                    parent=links[body_indices[int(joint.parent_body().index())]],
                    child=link,
                    jtype={"weld": 0, "revolute": 1, "prismatic": 2}[kind],
                    axis=axis if dof is not None else np.array([0.0, 0.0, 1.0]),
                    pose=jnp.asarray(
                        body_poses[int(joint.parent_body().index())]
                        @ parent_pose.GetAsMatrix4()
                    ),
                )
            )
            links.append(link)
            body_poses[int(child.index())] = child_pose.GetAsMatrix4()
            body_indices[int(child.index())] = len(links) - 1
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
        local_point = frame.GetFixedPoseInBodyFrame() @ point
        pose = body_poses[int(frame.body().index())]
        local.append(pose[:3, :3] @ local_point + pose[:3, 3])
    description = ModelDescription.build_model_from(
        name="nominal",
        links=links,
        joints=descriptions,
        base_link_name="world",
        fixed_base=True,
        considered_joints=joints,
    )
    native = JaxSimModel.build(description, gravity=float(gravity[2]))
    # Reduction preserves welded bodies as frames. Resolve each original body
    # through the maintained parser's transforms, never a second dynamics tree.
    from jaxsim.parsers.kinematic_graph import KinematicGraphTransforms

    transforms = KinematicGraphTransforms(description)
    parents, offsets = [], []
    for index, point in zip(indices, local, strict=True):
        name = links[index].name
        if name in native.link_names():
            parent, transform = name, np.eye(4)
        else:
            parent = description.frames_dict[name].parent_name
            transform = np.asarray(
                transforms.relative_transform(relative_to=parent, name=name)
            )
        parents.append(native.link_names().index(parent))
        offsets.append(transform[:3, :3] @ point + transform[:3, 3])
    return NominalModel(
        native=native,
        joints=joints,
        velocity_lower=velocity_lower,
        velocity_upper=velocity_upper,
        limits=limits,
        damping=damping,
        rotor=rotor,
        point_bodies=np.asarray(parents),
        points=np.asarray(offsets),
        device=device,
    )
