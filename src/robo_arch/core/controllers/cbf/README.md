# Sphere effort filter

Run the maintained example that resolves camera-protection profiles, builds an
independent controller model, validates its initial state and filters one nominal
effort command:

```sh
uv run python -m robo_arch.examples.cbf_filter
```

The example source is [`examples/cbf_filter.py`](../../../examples/cbf_filter.py).
It uses the same configuration, geometry and filter factories as the complete
camera-protection scenario rather than copying their equations.

## Read the implementation

| Responsibility | Source |
|---|---|
| SDK-independent sphere, plane and pair declarations | [`definition.py`](definition.py) |
| Protection YAML parameters | [`config.py`](config.py) |
| Asset profiles and physical-instance geometry | [`geometry.py`](geometry.py), [`assembly.py`](assembly.py) |
| Shared scalar/batched barrier rows | [`barrier.py`](barrier.py), [`velocity.py`](velocity.py) |
| Shared projection acceptance and diagnostics | [`filter.py`](filter.py) |
| Drake dynamics, Clarabel QP and native ports | [`drake.py`](drake.py) |
| Torch geometry and Moreau projection | [`tensor.py`](tensor.py), [`moreau.py`](moreau.py) |
| Isaac tensor-filter construction | [`isaac.py`](isaac.py) |

Units, shapes, ordering, ownership and failure contracts are rendered from these
docstrings in the [controller API catalogue](../../../../../docs/api/controllers.rst).

## Run complete scenarios

```sh
# Drake filtered and unfiltered comparison with Meshcat playback.
uv run src/robo_arch/scenarios/camera_protection/run.py --no-browser
uv run src/robo_arch/scenarios/camera_protection/run.py \
  --baseline --no-browser

# Optional batched CUDA/Moreau path in the independently locked Isaac profile.
uv sync --project third_party/isaac --locked --group cbf-gpu --group test
uv run src/robo_arch/scenarios/camera_protection/run.py \
  --world-config package://robo_arch/scenarios/camera_protection/isaac_gpu.yaml \
  --backend torch_moreau --batch-size 32 --no-browser
```

See the [scenario guide](../../../scenarios/camera_protection/README.md) for
artifacts, inspection, GPU measurements and native visual tests.

## Safety and compatibility

The filter uses nominal rigid-body dynamics and sampled state. It does not infer
contact forces, guarantee behavior between samples or provide a hardware
emergency stop. Geometry profiles are conservative approximations. Infeasible
QP, nonfinite inputs and rejected residuals raise `CbfFailure`; there is no
softened constraint, clipping after solve or nominal-command fallback.

The optional tensor path keeps numerical arrays on its selected device, but
status checks still synchronize with the host. Current Isaac camera protection
requires CUDA PhysX/PGS, float32 native commands and float64 model/QP calculations.
Newton and TGS are rejected for this scenario.

## Tests

```sh
uv run pytest src/robo_arch/core/controllers/cbf/tests
bazel test //src/robo_arch/core/controllers/cbf/...

third_party/isaac/.venv/bin/python -m pytest -q \
  src/robo_arch/core/controllers/cbf/tests/test_moreau.py \
  src/robo_arch/core/controllers/cbf/tests/test_tensor_barrier.py \
  src/robo_arch/core/controllers/cbf/tests/test_tensor_filter.py \
  src/robo_arch/core/controllers/dynamics/tests
```
