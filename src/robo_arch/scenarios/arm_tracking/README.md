# Arm tracking examples

| Run resource | Assembly | Autonomy | Observation |
|---|---|---|---|
| `scenario.yaml` | UR7e + D435 | inverse dynamics | ideal Drake RGB-D |
| `iiwa7.yaml` | iiwa 7 + Mini45 | native PD + gravity | six-axis wrench |
| `bimanual.yaml` | UR7e + nested iiwa system | independent native PD | two named wrenches |
| `iiwa7_contact.yaml` | iiwa 7 + Mini45 + fixed block | obstructed native PD | wrench peak check |

## Run and inspect

```sh
uv run src/robo_arch/scenarios/arm_tracking/run.py
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml \
  --record recordings/bimanual_drake.html --no-browser
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --inspect recordings/bimanual_drake.json --visualization live_and_record
```

Add `--world isaac` to a supported run, or select a complete native profile with
`--world-config`. For two collision-isolated copies of the system:

```sh
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml \
  --world-config package://robo_arch/core/worlds/isaac/batch.yaml
```

`--no-sensors` disables observations while preserving mounted bodies, inertia and
collisions. Unsupported observations fail instead of being silently removed.

Runs save resolved JSON inputs/results, NPZ traces, diagnostic plots and an
inspection command. Channels use `<robot>/q`, `<robot>/v`, `<robot>/effort` and
`<sensor>/wrench`; the common time axis is seconds. Wrenches are
`[Fx,Fy,Fz,Tx,Ty,Tz]` in N and N m. Drake can save interactive Meshcat playback;
Isaac live viewing saves a final viewport PNG.

## Read the implementation

| Question | Source |
|---|---|
| How are YAML selections validated? | [`control.py`](control.py), [`evaluation.py`](evaluation.py) |
| How is autonomy wired into Drake? | [`drake.py`](drake.py) |
| How are scalar callbacks built for Isaac? | [`isaac.py`](isaac.py) |
| How are worlds dispatched and artifacts retained? | [`run.py`](run.py) |
| How are measured traces rendered? | [`plotting.py`](plotting.py) |
| What cases define expected behavior? | [`tests/test_run.py`](tests/test_run.py), [`tests/test_isaac_lab.py`](tests/test_isaac_lab.py) |

For the declaration-to-device path, run and read the
[configuration example](../../examples/configuration.py). Native world ownership
is shown in the [world guide](../../core/worlds/README.md).

The contact configuration demonstrates ideal simulation contact sensing, not
force-feedback manipulation or safe hardware motion. Isaac arm tracking is scalar
CPU autonomy and reports GPU transfers. D435 images remain Drake-only; Newton
requires sensor observations to be disabled for Mini45 assemblies.

## Native Isaac checks

```sh
uv sync --project third_party/isaac --locked --group test
ROBO_ARCH_ISAAC_TESTS=1 uv run pytest \
  src/robo_arch/scenarios/arm_tracking/tests/test_isaac_lab.py
```

Set `ROBO_ARCH_VISUALIZE=1` to run the same cases with native viewing and viewport
capture. Failures print a command that reconstructs the tested inputs.
