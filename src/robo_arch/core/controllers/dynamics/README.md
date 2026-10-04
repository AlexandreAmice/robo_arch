# Batched nominal dynamics

`drake.build_tensor_model` extracts a fixed-base, fully actuated model once,
including welded bodies, mounted inertias, joint frames, damping, gravity and
reflected rotor inertia. Ordered scalar revolute/prismatic joints and identity
actuation are supported; unsupported joints, extra force elements, loops and
floating bases raise errors. The generic `torch.TensorModel` runtime imports no simulator SDK and retains no
Drake model or context. Its constructor takes owned device constants; the Drake
loader owns source-model validation and conversion.

`evaluate(state)` accepts float64 `[batch, 2*joints]` tensors in q/v order and
returns world-expressed point positions, velocity Jacobians, bias accelerations,
mass matrices and joint dynamics. `bias_force` includes Coriolis, gravity and
damping, so `effort = mass @ desired_acceleration + bias_force`. Constants and
outputs stay on the selected CPU/CUDA device. Loops traverse the body tree, not
environments; Torch batches the numerical operations. This implementation has
not established a speedup over scalar Drake for one arm.

`enable_compilation()` opts into `torch.compile` on the same equations. The
first call for a new shape incurs compilation/warmup; eager evaluation remains
the default. Compiled outputs are cloned outside the graph to preserve ownership
across subsequent calls. Use the camera scenario benchmark with `--compile-model`
to measure the complete control loop, including ownership copies and the QP.

`valid` is a per-environment device boolean covering finite input and successful
finite mass solves. Callers must reject invalid environments before applying
effort. The mass solve uses `solve_ex(check_errors=False)` to avoid implicit
CUDA synchronization; the caller decides when to surface failures on the host.
No contact or externally applied forces are inferred from the simulator.

Run the same parity tests in the optional vendor profile:

```sh
third_party/isaac/.venv/bin/python -m pytest -q \
  src/robo_arch/core/controllers/dynamics/tests
```

The tests compare independent Drake kinematics/dynamics on CPU and CUDA for a
batched mechanism with a rotated base, offset joint frames, mixed joint types,
joint damping, reflected rotor inertia and a welded payload. Bazel exposes the
test source as `gpu_tests`; execute it in the vendor profile above. The core
Bazel environment does not supply Torch and does not validate GPU execution.
