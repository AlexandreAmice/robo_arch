# World construction and execution

World packages turn the same loaded physical scene into native Drake, Isaac or
real-world runtime objects. Start with the executable tours:

```sh
# Load and namespace a nested system without a simulator SDK.
uv run python -m robo_arch.examples.configuration

# Construct a Drake plant, devices and independent controller models.
uv run python -m robo_arch.examples.drake_scene
```

## Read the implementation

| Question | Source |
|---|---|
| How are nested devices named and placed? | [`assembly.py`](assembly.py), [`devices.py`](devices.py) |
| How does a declaration become a Drake scene? | [`drake/scene.py`](drake/scene.py) |
| How does Drake build, run and retain logs? | [`drake/scenario.py`](drake/scenario.py) |
| How does Isaac construct and step native scenes? | [`isaac/scene.py`](isaac/scene.py), [`isaac/scenario.py`](isaac/scenario.py) |
| Where are native viewer lifecycles owned? | [`drake/visualization.py`](drake/visualization.py), [`isaac/visualization.py`](isaac/visualization.py), [`real/visualization.py`](real/visualization.py) |
| How does scenario autonomy connect? | [`../../scenarios/arm_tracking/drake.py`](../../scenarios/arm_tracking/drake.py), [`../../scenarios/arm_tracking/isaac.py`](../../scenarios/arm_tracking/isaac.py) |

The public declaration and configuration contracts are rendered in the
[API catalogue](../../../../docs/api/worlds.rst). Target ownership and lifecycle
requirements remain in the [architecture](../../../../docs/architecture.md) and
[build/layout guide](../../../../docs/build_and_layout.md).

## Current limits

- Drake and Isaac construct and execute declared scenes. Real execution raises
  `NotImplementedError`; the RViz launcher is observational only.
- Objects are fixed fixtures. Movable-object state and reset are not implemented.
- Scalar Isaac control crosses the CPU/GPU boundary. Tensor rollouts are explicit
  scenario code; batching alone does not imply GPU autonomy.
- D435 image generation is Drake-only. Newton retains mounted sensor bodies but
  rejects Mini45 observations because its pinned wrench sensor excludes the fixed
  sensing joint.
- Drake saves Meshcat playback. Isaac live viewing uses Storm and a final viewport
  capture; it does not provide RTX cameras, video or contact overlays.

Unsupported assets, observations and backend combinations fail before execution.
World switching never silently substitutes a controller or removes a device.

## Validate and inspect

```sh
uv run pytest src/robo_arch/core/worlds/tests \
  src/robo_arch/core/worlds/drake/tests
bazel test //src/robo_arch/core/worlds/... \
  //src/robo_arch/core/worlds/drake/...

# Native Isaac lifecycle, reset and failure checks require the vendor profile.
OMNI_KIT_ACCEPT_EULA=YES ROBO_ARCH_ISAAC_GPU_TEST=1 \
  third_party/isaac/.venv/bin/python -m pytest -q \
  src/robo_arch/core/worlds/isaac/tests/test_gpu_execution.py
```

When adding a world, follow the ownership and testability requirements in
[build and layout](../../../../docs/build_and_layout.md#testability). Keep its
SDK-independent settings in `config.py`, construction in `scene.py`, execution in
`scenario.py` and viewer lifecycle in `visualization.py`.
