"""Isaac effort-precision adapter and CUDA Moreau filter construction."""

from typing import Any

import torch

from robo_arch.core.controllers.cbf.assembly import ProtectionGeometry
from robo_arch.core.controllers.cbf.config import ProtectionParameters
from robo_arch.core.controllers.cbf.definition import CbfParameters
from robo_arch.core.controllers.cbf.tensor import TensorCbfFilter
from robo_arch.core.controllers.cbf.velocity import compile_velocity_bounds
from robo_arch.core.controllers.dynamics.drake import build_tensor_model


def build_filter(
    *,
    model: Any,
    joints: tuple[str, ...],
    geometry: ProtectionGeometry,
    parameters: ProtectionParameters,
    batch_size: int,
    device: str = "cuda:0",
    command_dtype: torch.dtype | None = torch.float32,
) -> TensorCbfFilter:
    """Compile independent model constants once and construct a CUDA Moreau filter.

    Drake is used only during model extraction; subsequent evaluation and solves
    use Torch/Moreau CUDA tensors. Native float32 effort guarding is enabled
    by default; command_dtype=None is only for numerical comparisons without
    native command conversion. No simulation state is read during extraction.
    """
    from robo_arch.core.controllers.cbf.moreau import MoreauProjection

    tensor_model = build_tensor_model(
        model,
        joints,
        frames=tuple(sphere.frame for sphere in geometry.spheres),
        points=tuple(sphere.center for sphere in geometry.spheres),
        device=device,
    )
    projection = MoreauProjection(
        count=tensor_model.count,
        constraint_count=len(geometry.pairs)
        + len(geometry.plane_pairs)
        + len(
            compile_velocity_bounds(
                joints, tensor_model.velocity_lower, tensor_model.velocity_upper
            ).names
        ),
        batch_size=batch_size,
        limits=(
            tensor_model.limits
            if command_dtype is None
            else tensor_model.limits * (1 - torch.finfo(command_dtype).eps)
        ),
        device=device,
        residual_tolerance=parameters.residual_tolerance,
    )
    return TensorCbfFilter(
        model=tensor_model,
        geometry=geometry,
        projection=projection,
        command_dtype=command_dtype,
        parameters=CbfParameters(
            alpha1=parameters.alpha1,
            alpha2=parameters.alpha2,
            velocity_limit_gain=parameters.velocity_limit_gain,
            residual_tolerance=parameters.residual_tolerance,
        ),
    )
