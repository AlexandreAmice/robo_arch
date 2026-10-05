# Native joint PD plus feedforward

Run the controller through the mixed-arm scenario or its focused tests:

```sh
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml
bazel test //src/robo_arch/core/controllers/joint_pd:joint_pd_test \
  //src/robo_arch/core/controllers/joint_pd:native_test
```

Direct scenario execution refreshes the native wheel before importing runtimes.
For notebooks and IDEs, install it explicitly with
`uv run tools/native/install.py --profile drake` (or `--profile isaac`).

## Read the implementation

| Responsibility | Source |
|---|---|
| C++23 controller and canonical contract | [`joint_pd.h`](joint_pd.h), [`joint_pd.cc`](joint_pd.cc) |
| Thin nanobind array boundary | [`bindings.cc`](bindings.cc) |
| Python extension loading and Python-specific contract | [`native.py`](native.py) |
| Drake gravity model and native System adapter | [`drake.py`](drake.py) |
| Batched Torch feedback | [`tensor.py`](tensor.py) |
| Gain validation | [`definition.py`](definition.py) |

The generated [API reference](../../../../../docs/api/controllers.rst#joint-control)
owns equations, units, input layout, output ownership and errors. The C++ and
Torch paths share the stateless feedback law; their focused parity tests cover
float32 and float64 inputs.

The development wheel targets CPython 3.12 on Linux x86-64 with host glibc and is
not a portable manylinux artifact. No Drake C++ objects cross the binding. Restart
an already-running Python process after rebuilding an imported extension.
