# Camera protection

This scenario drives three arm-mounted D435 housings toward fixed obstacles and
then retreats. A bounded-effort CBF changes the nominal command to protect the
selected camera coverings from obstacles, nonexcluded robot links and the ground.

Start with the controller-only composition example, then run the complete Drake
scenario and its unfiltered comparison:

```sh
uv run python -m robo_arch.examples.cbf_filter
uv run src/robo_arch/scenarios/camera_protection/run.py --no-browser
uv run src/robo_arch/scenarios/camera_protection/run.py \
  --baseline --no-browser
uv run src/robo_arch/scenarios/camera_protection/run.py \
  --inspect recordings/camera_protection_filtered.json \
  --visualization live_and_record
```

Runs retain resolved inputs, results or failure snapshots, source/asset hashes,
NPZ traces, clearance/correction plots and native Meshcat HTML. The `protections`
layer contains the sphere covers and ground boundary and is hidden by default.
Failed or infeasible QPs stop execution; partial traces and playback remain
available.

## Read the implementation

| Responsibility | Source |
|---|---|
| Scenario/task parameters and supported selections | [`configuration.py`](configuration.py) |
| World-independent arm, task and geometry resolution | [`setup.py`](setup.py) |
| Shared approach/retreat reference | [`reference.py`](reference.py) |
| Native Drake ports and visualization overlays | [`drake.py`](drake.py) |
| Batched Isaac controller and rollout | [`isaac.py`](isaac.py) |
| Evaluation, reports and inspection | [`run.py`](run.py) |
| Measured trace rendering | [`plotting.py`](plotting.py) |
| Control-only CUDA measurement | [`benchmark.py`](benchmark.py) |

Reusable geometry, barrier and solver code is under
[`../../core/controllers/cbf/`](../../core/controllers/cbf). Its
[API catalogue](../../../../docs/api/controllers.rst) owns units, array layouts,
diagnostic ordering, ownership and failure contracts.

Each device or object owns its `protection.yaml`; the scenario selects physical
instances, protected instances, margins and explicit mounting exclusions. These
profiles are conservative sphere covers, not exact collision geometry. Cables,
brackets and measured mounting calibration are absent.

## Optional batched CUDA path

```sh
uv sync --project third_party/isaac --locked --group cbf-gpu --group test
uv run src/robo_arch/scenarios/camera_protection/run.py \
  --world-config package://robo_arch/scenarios/camera_protection/isaac_gpu.yaml \
  --backend torch_moreau --batch-size 32 --no-browser

uv run src/robo_arch/scenarios/camera_protection/benchmark.py
```

Add `--baseline` for the same batch without filtering and `--compile-model` to
include the compiled tensor model after separately reported warmup. One rejected
environment stops the entire batch. Model/barrier/QP calculations use float64;
the filter protects and rechecks the actual float32 effort sent to PhysX.

Current GPU execution requires CUDA PhysX/PGS with sensor observations disabled.
Newton is unsupported. Imported TGS lost low-speed position increments while
reporting nonzero velocity: five joints remained stationary for a commanded
20-microradian change and the scenario measured a 0.21 mm clearance violation.
PGS measured approximately 19–20 microradians, so unsupported solvers are rejected
before execution.

## Recorded local evidence

On an RTX 3060 Laptop with the pinned Isaac Lab PhysX/PGS profile, compiled
two-environment runs retained 8.23 µm clearance beyond the 10 mm obstacle margin
and 69.11 µm beyond the ground margin. Unfiltered comparisons reached −57.95 mm
and −26.17 mm. The six-second filtered obstacle run took 699.9 s of simulation
wall time on a heavily loaded workstation; this is correctness evidence, not
real-time performance.

The control-only float64 benchmark measured batches 1/32/128/512 at
39.7/58.6/137.6/487.1 ms per call in eager mode (25/546/930/1051 environment
commands/s). Compiled batches 1 and 32 measured 27.4 and 46.6 ms per call
(36 and 686 environment commands/s), excluding warmup. These are local throughput
measurements, not a hardware-independent capacity claim.

## Safety and tests

The CBF consumes privileged simulated state, fixed known obstacle poses and
nominal contact-free dynamics. Its sampled result does not guarantee separation
between steps, under model error or on hardware, and it is not an emergency stop.

```sh
# Drake visual tests use the same inputs as their automated cases.
ROBO_ARCH_VISUALIZE=1 uv run pytest \
  src/robo_arch/scenarios/camera_protection/tests/test_run.py -k default
ROBO_ARCH_VISUALIZE=1 uv run pytest \
  src/robo_arch/scenarios/camera_protection/tests/test_ground.py

# Native GPU cases retain traces, plots and resolved inputs.
ROBO_ARCH_NATIVE_ISAAC=1 OMNI_KIT_ACCEPT_EULA=YES \
  uv run --project third_party/isaac --group cbf-gpu --group test \
  python -m pytest -q -s \
  src/robo_arch/scenarios/camera_protection/tests/test_gpu_run.py
```
