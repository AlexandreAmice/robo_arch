# Batched joint reaching

Independent UR7e arms reach seeded joint targets and reset on success or timeout.
This is a simulation workload for tensor control, native batch stepping and
selective reset; it is not Cartesian planning, learning or hardware validation.

## Run, inspect and benchmark

```sh
uv run src/robo_arch/scenarios/batched_reaching/run.py \
  --backend newton --num-envs 16 --live --hold \
  --output recordings/batched_reaching/newton_live

uv run src/robo_arch/scenarios/batched_reaching/run.py \
  --inspect recordings/batched_reaching/newton_live.json --live --hold \
  --output recordings/batched_reaching/inspection

uv run src/robo_arch/scenarios/batched_reaching/benchmark.py
```

Choose `--backend physx` for PhysX/TGS. Omit `--live --hold` for headless
measurement. The run saves resolved JSON, sampled NPZ state/reference/velocity/
effort/episode data, plots and a final viewport capture when viewing is enabled.
Samples remain on the GPU until reporting.

The benchmark runs fresh sequential processes at 1, 16 and 64 environments, then
compares scalar native PD at 16. Initialization and warmup are excluded. Reported
latency percentiles are 100-step chunk means; rendering is disabled. Failed
measurements retain their logs and batch sizes are never reduced automatically.

## Read the implementation

| Responsibility | Source |
|---|---|
| Task and measurement schemas | [`config.py`](config.py) |
| Independent goals, clocks and masked reset | [`task.py`](task.py) |
| Tensor rollout, sampling and artifacts | [`run.py`](run.py) |
| Fresh-process scaling comparison | [`benchmark.py`](benchmark.py) |
| Native integration behavior | [`tests/test_native.py`](tests/test_native.py) |

The feedback law is [`../../core/controllers/joint_pd/tensor.py`](../../core/controllers/joint_pd/tensor.py);
shared Isaac stepping is in
[`../../core/worlds/isaac/batched.py`](../../core/worlds/isaac/batched.py).
Feedforward is explicitly simulator gravity, a privileged simulation input.

Newton uses MuJoCo Warp on CUDA. Its pinned wrench sensor cannot represent the
fixed Mini45 sensing joint, so this workload disables observations while retaining
the mounted physical bodies. PhysX internally compacts reset masks and may
synchronize; live display copies poses at rendering cadence.

## Recorded local measurement

On October 4, 2026, an RTX 3060 Laptop 6 GB with the pinned Lab 3.0 EA profile
measured the following single-sweep steady-state rates after warmup:

| Control / environments | PhysX env-steps/s | Newton env-steps/s |
|---|---:|---:|
| Tensor / 1 | 96 | 373 |
| Tensor / 16 | 1,296 | 1,969 |
| Tensor / 64 | 7,596 | 10,929 |
| Scalar / 16 | 3,025 | 5,259 |

All environments completed goals with zero timeouts. These shared-desktop
measurements have variable CPU load and do not establish a portable speedup or
capacity promise. The small PD calculation did not show a tensor-control speedup
at 16 environments.

## Tests

```sh
third_party/isaac/.venv/bin/python -m pytest -q \
  src/robo_arch/core/controllers/joint_pd/tests/test_tensor.py \
  src/robo_arch/scenarios/batched_reaching/tests/test_task.py
ROBO_ARCH_ISAAC_TESTS=1 third_party/isaac/.venv/bin/python -m pytest -q \
  src/robo_arch/scenarios/batched_reaching/tests/test_native.py
```

Set `ROBO_ARCH_VISUALIZE=1` to inspect the same native test inputs.
