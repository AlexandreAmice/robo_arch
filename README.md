# Robotics architecture

Local robotics development with shared autonomy across Drake and Isaac Lab. The
implemented examples cover UR7e and iiwa 7 assemblies, mounted D435 and Mini45
sensors, native viewing, batched simulation and effort control. Hardware
execution, a gripper and nut-on-pin manipulation remain future work.

## Set up development

Use the pinned uv version in `pyproject.toml` and Bazelisk from `.bazelversion`:

```sh
uv sync --locked
uv run tools/native/install.py --profile drake
uv run --group docs pytest
bazel test //src/robo_arch/... //tests/build:core //tests/build:cxx23 //tests/build:drake
uv run ruff check .
uv run ruff format --check .
uv lock --check
bazel run //:buildifier
```

Isaac uses its own locked environment:

```sh
uv sync --project third_party/isaac --locked
```

There are no hosted CI workflows or required remote checks. Select
`.venv/bin/python` in an IDE. To inspect declarations without Drake, use
`uv sync --locked --no-group drake`.

For C++ navigation, install the clangd extension and generate the ignored
compilation database after changing native sources, BUILD files or toolchains:

```sh
bazel run //:refresh_compile_commands
```

## Read executable examples

These examples are small programs that import the maintained implementation:

```sh
# Strict YAML loading and nested physical-instance resolution; no simulator SDK.
uv run python -m robo_arch.examples.configuration

# Native Drake scene construction in a caller-owned DiagramBuilder.
uv run python -m robo_arch.examples.drake_scene

# Camera-protection geometry, nominal dynamics and one accepted CBF command.
uv run python -m robo_arch.examples.cbf_filter
```

Read the corresponding source under [`src/robo_arch/examples/`](src/robo_arch/examples)
and follow its imports. Public contracts are rendered from source docstrings in
the [API catalogue](docs/api/index.rst).

## Run scenarios

Scenario scripts select the configured environment, refresh changed native code
and then launch. `--help` does not build or import a simulator.

```sh
# UR7e, physical D435 housing and ideal Drake RGB-D output.
uv run src/robo_arch/scenarios/arm_tracking/run.py

# iiwa 7 with native PD and wrist force/torque sensing.
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/iiwa7.yaml

# Mixed UR7e/iiwa assembly with independent controllers and sensors.
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml

# Deliberate Mini45 contact against a fixed block.
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/iiwa7_contact.yaml

# Batched UR7e reaching in Isaac with Newton/MuJoCo Warp.
uv run src/robo_arch/scenarios/batched_reaching/run.py \
  --backend newton --num-envs 16 --live --hold

# Filtered camera-protection motion in Drake.
uv run src/robo_arch/scenarios/camera_protection/run.py --no-browser
```

Use `--world isaac` or a complete `--world-config` on supported scenarios. A
desktop Isaac view can be selected with:

```sh
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml \
  --world-config package://robo_arch/core/worlds/isaac/desktop.yaml
```

Each run saves resolved inputs and versions, results or errors, source/asset
hashes and a copyable inspection command. Drake can save interactive Meshcat HTML;
Isaac live runs save a final viewport PNG. Use the generated command or pass
`--inspect <report.json>` to rerun the recorded inputs with current code.

For scenario-specific flags, artifacts, benchmarks and visual tests, read:

- [arm tracking](src/robo_arch/scenarios/arm_tracking/README.md)
- [batched reaching](src/robo_arch/scenarios/batched_reaching/README.md)
- [camera protection](src/robo_arch/scenarios/camera_protection/README.md)

The [documentation index](docs/README.md) points to architecture, build rules,
API contracts, model provenance and dependency compatibility. Isaac setup and
known vendor limits remain in [its dependency profile](third_party/isaac/README.md).
