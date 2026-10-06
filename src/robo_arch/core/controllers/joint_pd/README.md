# Native joint PD plus feedforward

`joint_pd_cc` is an independently usable C++23 PD-plus-feedforward library.
The [joint-control API reference](../../../../../docs/api/controllers.rst#joint-control)
documents its equation, units, array contract, ownership and errors from the
[C++ declaration](joint_pd.h) and [Python facade](native.py). Gain validation lives
in [JointPdParameters](definition.py); its torque-feedback gains differ from
`joint_tracking`'s acceleration-feedback gains.

Supported scalar and tensor autonomy calls the authoritative array feedback law
in `core/controllers/feedback.py`. Independent JaxSim nominal models supply
gravity, including mounted-device inertia; no physical simulation dynamics are
queried. Adapters clip effort to model limits and validate joint identity/gains.
There is no integral term or hidden reset state.

`tensor.compute` supplies a Torch array boundary around that same law. Inputs are
borrowed, outputs own storage, and gains/limits broadcast across batch axes.
Batched reaching uses the same independent nominal gravity computation.

The standalone C++ library and `native.compute` binding are retained compatibility
APIs and build/binding examples; supported autonomy no longer selects them as an
alternative backend. Native/tensor parity tests in float32/float64 guard the
retained compatibility implementation against drift. Torch remains optional in
the vendor runtime profile.

The binding is included in the project aggregate `//tools/native:wheel`; this
package owns the C++ library and extension target. The local wheel targets
CPython 3.12 on Linux x86-64 with host glibc. Nanobind and
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
