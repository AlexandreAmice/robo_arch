# World construction and execution

Every world has the same responsibility split:

| File | Responsibility |
|---|---|
| `config.py` | SDK-independent native physics/transport and viewer settings |
| `scene.py` | `build_scene(scene, config, ...)`: validate and construct physical instances in the native runtime |
| `scenario.py` | `run_scenario(run, ...)`: construct the scene, invoke scenario autonomy wiring, initialize, execute, collect traces and clean up |
| `visualization.py` | Native viewer wiring, publication and lifecycle |

Drake and Isaac implement construction and execution. Real exposes these entry
points but raises `NotImplementedError`; its RViz launcher remains independent
of hardware execution. Native helpers such as USD conversion need not have a
counterpart in every world.

## Inputs and ownership

The [configuration reference](../../../../docs/api/configuration.rst) owns the
loader and record contracts; the [device assembly reference](../../../../docs/api/worlds.rst)
owns name resolution, metadata checks and SDK-independent world settings.
Scene builders consume `load_run(...).scene` independently of task/autonomy and
load their own device definitions. Ordinary robot assets are parsed by the world;
specialized device behavior stays with the device. Disabling observations still
preserves sensor bodies, mounts, inertia and collisions.

A world receives ordinary Python scenario wiring through the required `configure`
argument. It does not import a concrete scenario or interpret task/controller
identifiers. The scenario supplies references and autonomy; the world owns native
execution. Evaluation and run reports stay with the scenario and consume the
returned `trace` and native diagnostic fields.

## Native entry points

Drake can build a physical scene into a caller-owned builder:

```python
from pydrake.systems.framework import DiagramBuilder
from robo_arch.core.worlds.drake.scene import build_scene

builder = DiagramBuilder()
scene = build_scene(run.scene, run.world_config, builder=builder)
```

`drake.scenario.build_simulation(run, configure=...)` constructs that scene,
invokes `configure(builder, scene)`, wires viewing, applies initial positions and
returns the native `(Simulator, DrakeScene)`. `run_scenario()` additionally owns
a single `Simulator.AdvanceTo(run.duration)` call, recording and optional viewer
hold. Meshcat publishes through its diagram events; `LogVectorOutput` systems
collect state, net actuation and wrench signals on native simulator steps. There
is no Python sampling loop or forced publication in simulation execution. A
scenario connects control using native ports; arm tracking's `drake.configure()`
is an example.

Illustration geometries tagged with `("meshcat", "accepting") = "protections"`
appear exclusively in `/drake/protections`, hidden by default and independently
toggleable in Meshcat's tree. Its live `protections α` slider adjusts opacity;
native HTML recordings retain the layer and motion but omit server-dependent
sliders. Untagged physical visuals remain in the standard illustration layer;
camera protection tags all sphere coverings and its ground boundary. Tags change
no collision or perception roles.

Drake wiring may return a mapping of diagnostic names to vector output ports,
or `None`. The runner logs these alongside the world signals; each diagnostic
has a separate `<name>/times` array, including when a failure interrupts logging.
Names must not overlap world channels or another channel's timestamps.
`build_simulation(..., initialize=False)` transfers initialization to the caller;
the runner uses this to preserve partial logs when initialization fails.

Isaac's `build_scene(run.scene, run.world_config, directory=...)` constructs an
unattached USD stage after Kit startup. The caller must keep the converted-asset
directory alive while the stage is in use. Its returned `IsaacScene` contains
articulation roots, resolved devices and definitions. `initialize_scene()` binds
native tensors/observations and applies initial positions and zero velocities
after PhysX attachment.

`isaac.scenario.run_scenario(run, configure=...)` owns Kit, temporary converted
assets, stage attachment, initialization, stepping and teardown. It invokes
`configure(scene)` to obtain effort callbacks keyed by configured robot name;
each receives `(state, time_seconds)` and returns joint efforts. Arm tracking's
`isaac.configure()` builds its selected controller against independent nominal
models. This effort interface currently supports fixed-base arms and scalar CPU
control, with explicit transfers when physics runs on the GPU.

Both execution functions accept `trace_path` and `keep_viewer_open`. Drake accepts
an HTML `recording` path; Isaac rejects recording and saves a final viewport PNG
beside the trace when live viewing is enabled. Returned `trace` dictionaries and
NPZ files contain owned NumPy arrays. Drake exports
its native logs; Isaac copies observations within its own PhysX loop. Partial
data is retained on stepping failures. Arm tracking owns its diagnostic plots.

## Physical limits

All objects are fixed fixtures; movable objects, initial object velocity and
batched/reset execution are not implemented. Isaac authors downward gravity of
9.81 m/s²; Drake uses its plant default. Shared assets do not guarantee identical
contact models or trajectories.

Drake and Isaac enable `world.ground: true` by default: an infinite static
collision plane at world z=0 with a 10×10 m gray visible surface. The ground has
nominal static/dynamic friction 0.8/0.6 and remains physical when visualization
is disabled. Set `world.ground: false` for an installation with its own floor.
Autonomy must explicitly include the floor in its constraints; the camera
protection scenario does so for every selected protected sphere.

Drake results include `simulation_wall_seconds` and `realtime_rate`, measured
around `Simulator.AdvanceTo`. The rate is simulated seconds per wall-clock
second; startup, HTML export and plotting are excluded. It includes controller
evaluation and scheduled logging/viewer work during stepping. A positive
`target_realtime_rate` only limits pacing; it cannot speed up computation.

Drake uses its native object parser. Isaac's `objects.py` explicitly supports only
SDF 1.7 fixtures with one unoffset link, one visual box and an identical collision
box. It checks the declared base frame, compares numeric dimensions and preserves
diffuse RGBA. Inertial data is validated but does not participate in static-fixture
dynamics. Additional geometry, offsets, contact properties and other unsupported
content fail before Kit startup through `run_scenario()`. This is a restricted
converter, not a general SDF importer. Extend asset support deliberately; do not
silently discard new physics or substitute a bounding box.

## Adding a world

1. Add SDK-independent settings in `core/worlds/<world>/config.py` and include the
   new type in `core/config/worlds.py:WorldConfiguration`. Keep complete profiles
   beside their owner and use `package://robo_arch/...` references. Update
   `RunConfiguration.time_step` if the world does not have simulated physics.
2. Implement `scene.py:build_scene()` against `SceneConfiguration` and native
   settings. Reuse device resolution and declarations; extend support
   checks in `devices.py` and implement native importers/adapters. Robot support
   follows the world's asset/behavior checks; `RobotDefinition` declares assets
   and joint/frame conventions. Declare sensor physical and observation support
   separately in `SensorDefinition`, and object support in `ObjectDefinition`.
   Preserve frames, joint order, mounts, geometry, inertia and material meaning;
   reject unsupported inputs and name deliberate approximations.
3. Implement `scenario.py:run_scenario()` against `RunConfiguration` with an
   explicit native `configure` function. Own startup, initialization, timing,
   tracing and cleanup; controllers own their internal state. Add physical reset
   only when supported. Hardware poses describe relationships, not commands to
   reposition devices. Keep task-specific references/evaluation in the scenario.
4. Implement `visualization.py` and connect it from the world execution path.
   Viewing must not change physics or sensor selection. Preserve inspectable
   partial results on failures. Document native input/output and lifecycle limits.
5. Add scenario wiring and dispatch. Arm tracking has native wiring in `drake.py`
   and `isaac.py`; `run.py` selects the world, evaluates traces and records inputs.
   Its CLI choices and inspection commands also need the new world. Never select
   another controller silently; warn about supported scalar/CPU execution costs.
6. Declare narrow Bazel targets and packaged resources. Put incompatible vendor
   dependencies in a separately locked `third_party/<world>/` profile. Test
   SDK-free rejection, independent scene construction, native settings and asset
   preservation, initialization and execution/failure cleanup. Exercise contact
   and reset when claimed. Compare controller outputs for matching inputs rather
   than requiring identical engine trajectories.

Follow the [architecture](../../../../docs/architecture.md#world-implementations)
and [testability requirements](../../../../docs/build_and_layout.md#testability),
including actual-run visualizations when requesting human inspection.
