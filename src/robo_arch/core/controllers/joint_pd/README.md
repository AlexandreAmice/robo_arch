# Native joint PD plus feedforward

`joint_pd_cc` is an independently usable C++23 library. It computes
`tau = feedforward + kp * (q_des - q) + kd * (v_des - v)` for ordered joint vectors.
Positions are radians, velocities rad/s and efforts N m; `kp` is N m/rad and `kd`
is N m s/rad. These are different units from `joint_tracking`'s acceleration gains.

The nanobind extension accepts equally sized, nonempty, contiguous **CPU float64**
vectors. Inputs are read-only borrows lasting only through the call, with no
implicit dtype/layout conversion. Output has independent capsule-owned storage.
The GIL remains held for this small stateless computation. Invalid dimensions,
nonfinite values, negative gains and overflow raise exceptions.

The Drake port adapter and the Isaac callable use the same extension. Each owns
an independent nominal Drake model context for gravity feedforward, including
mounted-device inertia; neither reads the simulation's dynamics context. The
adapters clip effort to model actuator limits. There is no integral term or
hidden reset state. Parameters and joint identities are checked at construction.

The local wheel targets CPython 3.12 on Linux x86-64 with host glibc. Nanobind and
the C++ runtime are linked statically with hidden archive symbols; only the Python
entry point is exported. This prevents symbol interposition with Kit's C++
libraries. No Drake C++ objects cross the binding boundary. The wheel is not a
portable manylinux release artifact.

```sh
uv run tools/dev.py native --profile drake
uv run tools/dev.py run --profile drake -- python -m robo_arch.scenarios.arm_tracking.run \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml
bazel test //src/robo_arch/core/controllers/joint_pd:joint_pd_test \
  //src/robo_arch/core/controllers/joint_pd:native_test
```

Use `--profile isaac` for the separately synchronized vendor environment. The
helper builds through Bazel, checks the interpreter, compares actual installed
extension bytes, installs without resolving dependencies and launches a fresh
process. Failed builds/installations stop launch. Ordinary Python edits need no
native rebuild. An exact `uv sync` may remove the development wheel; rerun the
helper afterward. Restart notebook kernels after changing native code.
