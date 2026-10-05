# Native joint PD plus feedforward

`joint_pd_cc` is an independently usable C++23 PD-plus-feedforward library.
The [joint-control API reference](../../../../../docs/api/controllers.rst#joint-control)
documents its equation, units, array contract, ownership and errors from the
[C++ declaration](joint_pd.h) and [Python facade](native.py). Gain validation lives
in [JointPdParameters](definition.py); its torque-feedback gains differ from
`joint_tracking`'s acceleration-feedback gains.

The Drake port adapter and the Isaac callable use the same extension. Each owns
an independent nominal Drake model context for gravity feedforward, including
mounted-device inertia; neither reads the simulation's dynamics context. The
adapters clip effort to model actuator limits. There is no integral term or
hidden reset state. Parameters and joint identities are checked at construction.

`tensor.compute` implements the same stateless feedback law on Torch CPU/CUDA
tensors, with leading batch dimensions and a final joint dimension. Gains and
effort limits broadcast; state/reference/feedforward shapes must match. Inputs
are borrowed and outputs own their storage. It clips effort to caller-provided
limits without host synchronization. The owner validates finite gains/limits
before execution and checks rollout state for failures. Native/tensor parity is
tested in float32 and float64. Torch remains in the vendor runtime profile.
Batched reaching supplies Isaac Lab gravity forces explicitly; the feedback
function itself has no simulator dependency or privileged state access.

The local wheel targets CPython 3.12 on Linux x86-64 with host glibc. Nanobind and
the C++ runtime are linked statically with hidden archive symbols; only the Python
entry point is exported. This prevents symbol interposition with Kit's C++
libraries. No Drake C++ objects cross the binding boundary. The wheel is not a
portable manylinux release artifact.

```sh
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml
bazel test //src/robo_arch/core/controllers/joint_pd:joint_pd_test \
  //src/robo_arch/core/controllers/joint_pd:native_test
```

Direct scripts select the environment from the effective world configuration,
build through Bazel, and refresh changed or missing native-wheel payloads before
starting a fresh process. Failed builds/installations stop launch. Exact
`uv sync` may remove the development wheel; the next direct launch restores it.
For IDEs and notebooks, use `uv run tools/native/install.py --profile drake`
(or `--profile isaac`) and restart the Python process after native changes.
