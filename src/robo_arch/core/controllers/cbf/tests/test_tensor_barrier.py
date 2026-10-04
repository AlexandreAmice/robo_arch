"""Cross-backend and batch parity of the shared barrier equations."""

from dataclasses import fields

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from robo_arch.core.controllers.cbf.barrier import (  # noqa: E402
    plane_constraints,
    sphere_constraints,
)
from robo_arch.core.controllers.cbf.definition import CbfParameters  # noqa: E402


@pytest.mark.parametrize("device", ["cpu", "cuda"])
@pytest.mark.parametrize("planes", [False, True])
def test_batch_rows_match_independent_numpy_rows(device, planes):
    if device == "cuda" and not torch.cuda.is_available():
        pytest.skip("CUDA is unavailable")
    rng = np.random.default_rng(2026)
    batch, pairs, joints = 4, 7, 6
    vectors = rng.normal(size=(batch, pairs, 3))
    jacobians = rng.normal(size=(batch, pairs, 3, joints))
    bias = rng.normal(size=(batch, pairs, 3))
    common = dict(
        velocity=rng.normal(size=(batch, joints)),
        acceleration_drift=rng.normal(size=(batch, joints)),
        acceleration_control=rng.normal(size=(batch, joints, joints)),
    )
    if planes:
        normal = rng.normal(size=(batch, pairs, 3))
        normal /= np.linalg.norm(normal, axis=-1, keepdims=True)
        inputs = dict(
            positions=vectors,
            jacobians=jacobians,
            bias_accelerations=bias,
            normals=normal,
            offsets_with_radius=rng.uniform(0.1, 0.5, (batch, pairs)),
            **common,
        )
        function = plane_constraints
    else:
        inputs = dict(
            displacement=vectors,
            relative_jacobian=jacobians,
            relative_bias_acceleration=bias,
            separation=rng.uniform(0.1, 0.5, (batch, pairs)),
            **common,
        )
        function = sphere_constraints
    parameters = CbfParameters(alpha1=3, alpha2=7)
    actual = function(
        **{key: torch.as_tensor(value, device=device) for key, value in inputs.items()},
        parameters=parameters,
        namespace=torch,
    )
    numpy_batch = function(**inputs, parameters=parameters)
    for index in range(batch):
        expected = function(
            **{key: value[index] for key, value in inputs.items()},
            parameters=parameters,
        )
        for field in fields(actual):
            tensor = getattr(actual, field.name)
            assert tensor.device.type == device
            np.testing.assert_allclose(
                tensor[index].cpu().numpy(), getattr(expected, field.name), atol=1e-11
            )
            np.testing.assert_allclose(
                getattr(numpy_batch, field.name)[index],
                getattr(expected, field.name),
                atol=1e-11,
            )
