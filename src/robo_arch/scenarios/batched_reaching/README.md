# Batched joint reaching

Independent UR7e arms reach seeded joint goals and reset on success or timeout.
The default is 16 environments in a grid, six seconds at 1 kHz, with goals within
0.15 rad of the declared nominal posture. Success requires every joint within
0.02 rad and below 0.05 rad/s for 0.1 s. Episodes time out after two seconds;
completed environments reset at the next 20 ms reset boundary.

The shared Torch PD implementation takes `[environment, joint]` tensors and
clips effort to URDF limits. Both engines use the same gains. Feedforward is
explicitly `simulator_gravity`: privileged model information supplied by Isaac
Lab, not the independent Drake model used by scalar arm tracking. The default
wrist gains accommodate explicit effort integration on both engines. This is
joint-space reaching, not Cartesian planning, learning or hardware validation.

## Run and inspect

Use the existing [Isaac vendor environment](../../../../third_party/isaac/README.md).
Run from the repository worktree:

```sh
uv run tools/dev.py run batched_reaching \
  --backend newton --num-envs 16 --live --hold \
  --output recordings/batched_reaching/newton_live
```

Choose `--backend physx` for PhysX/TGS. Omit `--live --hold` for headless
measurement; the helper handles display variables and selects the Isaac
environment from the scenario configuration. `--config` accepts a packaged run
resource; its physics settings also select PhysX/PGS or Newton solver tuning.
Newton uses MuJoCo Warp on CUDA; a CPU Newton configuration is rejected.
This example disables sensor observations. The pinned Newton joint-wrench
sensor cannot represent the fixed Mini45 sensing joint; physical mounted
bodies remain present.

Outputs are JSON configuration/results, sampled NPZ state/reference/velocity/
effort/episode-age/completion-count arrays, tracking plots and a live viewport
PNG. Run the exact resolved inputs again with:

```sh
uv run tools/dev.py run batched_reaching \
  --inspect recordings/batched_reaching/newton_live.json --live --hold \
  --output recordings/batched_reaching/inspection
```

Inspection uses current code and assets. `--sample-period`, `--sampled-envs`,
`--warmup-steps` and `--reset-period` configure measurement overhead; resolved
values are recorded. Four environments are sampled at 50 Hz by default. Samples
stay on the GPU until reporting; no per-step NumPy copy occurs in tensor mode.
PhysX's SDK internally compacts reset masks to indices, which can synchronize.
Live display copies poses at rendering cadence and is included in live timing.

## Measure scaling

```sh
uv run tools/dev.py benchmark batched_reaching
```

`--backends physx` or `--backends newton` restricts the comparison. By default,
this runs both backends at 1, 16 and 64 environments in fresh, sequential
processes, then compares scalar native PD at 16 environments. Scalar comparison
uses exactly the same targets and simulator gravity feedforward. Its CPU copies
and per-environment loop are intentional. Initialization and 200 warmup steps
are excluded from steady-state throughput. GPU synchronization brackets timing
chunks; reported latency percentiles are **100-step chunk means**, not individual
step latency percentiles. Rendering is off. Results and a throughput plot live
under `recordings/batched_reaching/benchmark/`.

Torch peak allocation excludes PhysX/Warp allocations; device-used memory
includes other processes. Batch sizes are never automatically reduced, and
failed measurements retain their logs. Timing is a local observation with no
hardware-independent speedup threshold.

### Local measurements

October 4, 2026, RTX 3060 Laptop 6 GB, pinned Lab 3.0 EA profile, three simulated
seconds per run after warmup. Every environment completed goals with zero
timeouts, including 320 PhysX and 323 Newton episodes at 64 environments.

| Control / environments | PhysX env-steps/s | Newton env-steps/s |
|---|---:|---:|
| Tensor / 1 | 96 | 373 |
| Tensor / 16 | 1,296 | 1,969 |
| Tensor / 64 | 7,596 | 10,929 |
| Scalar / 16 | 3,025 | 5,259 |

These are single sweeps on a shared desktop with variable CPU load. PhysX timings
were repeated after an unrelated CUDA job ended. The small PD calculation did
not show a tensor-control speedup at 16 environments in these runs; GPU-resident
control does not guarantee lower latency. The example establishes batched
execution and measured capacity, with timing variability limiting comparisons.
Live/headless replay on both engines matched all 151 recorded frames (four
sampled environments) and episode/error statistics for all 16 environments.
Reports under `recordings/batched_reaching/benchmark/` (Newton) and
`benchmark_physx/` (PhysX) retain inputs, versions and latency distributions.

## Tests

```sh
third_party/isaac/.venv/bin/python -m pytest -q \
  src/robo_arch/core/controllers/joint_pd/tests/test_tensor.py \
  src/robo_arch/scenarios/batched_reaching/tests/test_task.py
ROBO_ARCH_ISAAC_TESTS=1 .venv/bin/python -m pytest -q \
  src/robo_arch/scenarios/batched_reaching/tests/test_native.py
```

The native tests check root separation, backend collision isolation, selected
state/effort reset and repeated reaching. Set `ROBO_ARCH_VISUALIZE=1` to inspect
and capture the same test inputs. Numerical controller parity is tested against
the C++ implementation on CPU and CUDA; identical engine trajectories are not
required.
