# Shared nominal dynamics

`drake.build_tensor_model` converts a finalized independent nominal model into
JaxSim's model description. JaxSim supplies rigid-body mass, bias and forward
kinematics; JAX differentiates the queried point positions for velocity
Jacobians and `Jdot v`. The same computation runs on CPU and CUDA. Drake remains
the canonical model loader and the independent parity oracle, not a second
runtime dynamics implementation.

The converter supports fixed-base, fully actuated trees with scalar revolute or
prismatic joints, welds, world-z gravity, viscous joint damping and reflected
rotor inertia. It converts body inertias and offset child joint frames into
JaxSim's joint-attached link convention. Weld reduction preserves payload mass
and query frames. It rejects floating bases, loop constraints, extra force
elements, per-instance gravity disabling and nonidentity actuation. Attached
actuated tools remain part of the full mechanism; their coordinates are not
replaced with a rigid payload. Contact and external forces are excluded.

`evaluate_numpy` takes float64 `[batch, 2*joints]` state on CPU.
`evaluate` accepts a Torch float64 tensor on the selected device and uses
DLPack for device transfer to JAX, then returns owned Torch arrays. Neither GPU
boundary copies state through NumPy. Both return positions, Jacobians, point bias
accelerations, mass, bias force and the affine effort-to-acceleration mapping.
Bias includes gravity and damping: `effort = mass @ acceleration + bias_force`.
Callers must check the returned per-row `valid` flag before applying effort.

JAX compilation is mandatory and specialized by model topology and batch shape;
there is no separate eager algorithm. First evaluation includes compilation.
Repeated owners of the same topology reuse the compiled function, but retained
outputs remain independent. CPU execution and GPU execution have different costs;
shared source does not establish a speedup. With Isaac's other GPU consumers,
set `XLA_PYTHON_CLIENT_PREALLOCATE=false` before launching to avoid JAX reserving
most device memory up front.

The `numerical` Python dependency group supplies JaxSim/JAX on CPU; the Isaac
`cbf-gpu` group supplies the matching CUDA plugin. Model conversion uses the
maintained in-memory parser API and does not invoke an SDF/URDF conversion CLI.
The model retains no Drake context or borrowed native views after conversion.

Run independent model/kinematics parity in the optional tensor environment:

```sh
third_party/isaac/.venv/bin/python -m pytest -q \
  src/robo_arch/core/controllers/dynamics/tests
```

Tests cover rotated bases, shifted joint frames, revolute/prismatic coordinates,
damping, rotor inertia, payloads, batch isolation and ownership across evaluations.
