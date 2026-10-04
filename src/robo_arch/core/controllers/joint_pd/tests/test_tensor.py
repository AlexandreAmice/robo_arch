"""Tensor/native parity, including broadcasting and saturation."""

import numpy as np
import pytest

from robo_arch.core.controllers.joint_pd.native import compute as native_compute


@pytest.fixture
def torch():
    return pytest.importorskip("torch")


@pytest.mark.parametrize("device", ["cpu", "cuda:0"])
@pytest.mark.parametrize("dtype", ["float32", "float64"])
def test_tensor_matches_native(device, dtype, torch):
    from robo_arch.core.controllers.joint_pd.tensor import compute

    if device == "cuda:0" and not torch.cuda.is_available():
        pytest.skip("CUDA unavailable")
    generator = np.random.default_rng(13)
    arrays = [generator.normal(size=(8, 6)) for _ in range(5)]
    kp, kd = np.arange(1, 7) * 20.0, np.arange(1, 7) * 2.0
    limits = np.arange(1, 7) * 3.0
    expected = np.stack(
        [
            np.clip(native_compute(*row, kp, kd), -limits, limits)
            for row in zip(*arrays, strict=True)
        ]
    )
    result = compute(
        *[
            torch.tensor(x, device=device, dtype=getattr(torch, dtype))
            for x in (*arrays, kp, kd, limits)
        ]
    )
    np.testing.assert_allclose(
        result.cpu().numpy(), expected, atol=2e-5 if dtype == "float32" else 1e-12
    )
    assert np.any(np.abs(expected) == limits)


def test_tensor_rejects_accidental_reference_broadcast(torch):
    from robo_arch.core.controllers.joint_pd.tensor import compute

    state = torch.zeros(4, 6)
    with pytest.raises(ValueError, match="shapes"):
        compute(state, state, state[0], state, state, state[0], state[0], state[0])


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
