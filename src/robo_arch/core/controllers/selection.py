"""Bounded, SDK-independent world selection for the supported effort algorithms."""

import warnings
from dataclasses import asdict, dataclass

from robo_arch.core.contracts.commands import CommandCapabilities, CommandKind


@dataclass(frozen=True)
class ControllerSelection:
    """Actual implementation and numerical-model provenance for run metadata."""

    algorithm: str
    implementation: str
    numerical_model: str
    device: str
    batched: bool

    def describe(self) -> dict:
        return asdict(self)


def select_controller(
    world,
    algorithm: str,
    *,
    batched: bool = False,
    sensors_enabled: bool = False,
    capabilities: CommandCapabilities | None = None,
) -> ControllerSelection:
    """Resolve known implementations; never change the requested control law.

    Real transports supply capabilities explicitly. A trajectory-only transport
    cannot run an effort controller even if a driver accepts an effort field.
    """
    if algorithm not in {"joint_pd", "joint_tracking", "cbf"}:
        raise ValueError(f"Unsupported controller: {algorithm}")
    if capabilities is None:
        capabilities = CommandCapabilities(
            frozenset({CommandKind.JOINT_EFFORT})
            if world.type in {"drake", "isaac"}
            else frozenset()
        )
    capabilities.require(CommandKind.JOINT_EFFORT)
    if world.type not in {"drake", "isaac", "real"}:
        raise ValueError(f"Unsupported control world: {world.type}")
    device = world.physics.device if world.type == "isaac" else "cpu"
    if batched:
        if world.type != "isaac":
            raise ValueError("Batched controller execution requires Isaac")
        if sensors_enabled:
            raise ValueError("Tensor execution requires sensors_enabled: false")
        if algorithm == "cbf":
            if device != "cuda:0":
                raise ValueError("Batched CBF requires CUDA Moreau")
            if world.physics.backend != "physx":
                raise ValueError("Batched CBF currently supports only PhysX")
            if world.physics.solver != "pgs":
                raise ValueError(
                    "Batched CBF requires PGS: pinned TGS loses small imported "
                    "joint-position increments while reporting nonzero velocity"
                )
        implementation = "tensor_moreau" if algorithm == "cbf" else "tensor"
    else:
        implementation = "scalar_clarabel" if algorithm == "cbf" else "scalar"
        if world.type == "isaac":
            warnings.warn(
                "Scalar CPU controller in Isaac evaluates each environment separately; "
                "CUDA physics also transfers state and effort each step.",
                RuntimeWarning,
                stacklevel=2,
            )
        device = "cpu"
    return ControllerSelection(algorithm, implementation, "jaxsim", device, batched)


def scalar_policy(algorithm: str, **kwargs):
    """Construct the selected scalar law after capability resolution."""
    if algorithm == "joint_pd":
        from robo_arch.core.controllers.joint_pd.drake import make_policy
    elif algorithm == "joint_tracking":
        from robo_arch.core.controllers.joint_tracking.drake import make_policy
    else:
        raise ValueError(f"No standalone scalar policy for {algorithm}")
    return make_policy(**kwargs)


def scalar_system(algorithm: str, **kwargs):
    """Construct a Drake port adapter for the same scalar law."""
    if algorithm == "joint_pd":
        from robo_arch.core.controllers.joint_pd.drake import JointPdSystem

        return JointPdSystem(**kwargs)
    if algorithm == "joint_tracking":
        from robo_arch.core.controllers.joint_tracking.drake import build

        return build(**kwargs)
    raise ValueError(f"No scalar system for {algorithm}")


def tensor_policy(algorithm: str, model, parameters):
    """Construct a device-array law with the selected independent nominal model."""
    if algorithm == "joint_tracking":
        from robo_arch.core.controllers.joint_tracking.torch import TensorJointTracking

        return TensorJointTracking(model, parameters)
    if algorithm == "joint_pd":
        from robo_arch.core.controllers.joint_pd.tensor import TensorJointPd

        return TensorJointPd(model, parameters)
    raise ValueError(f"No standalone tensor policy for {algorithm}")
