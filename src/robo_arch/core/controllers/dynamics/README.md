# Batched nominal dynamics

The tensor model is exercised by the executable
[CBF composition example](../../../examples/cbf_filter.py) and the
[camera-protection benchmark](../../../scenarios/camera_protection/benchmark.py).

Read [`drake.py`](drake.py) for fixed-model validation and constant extraction,
then [`torch.py`](torch.py) for the SDK-independent batched kinematics and
dynamics. Their docstrings define supported joint/body structure, array shapes,
device ownership and failure reporting.

The model supports fixed-base scalar revolute/prismatic joints, welds, uniform
gravity, damping and reflected rotor inertia. It does not infer contact or other
external forces. `torch.compile` is opt-in and the first call includes compilation
and warmup. No single-arm speedup over scalar Drake is claimed.

Run CPU/CUDA parity checks in the optional vendor profile:

```sh
third_party/isaac/.venv/bin/python -m pytest -q \
  src/robo_arch/core/controllers/dynamics/tests
```

These tests compare the tensor implementation with independent Drake evaluations.
The core Bazel environment exposes the test sources but does not supply Torch or
validate CUDA execution.
