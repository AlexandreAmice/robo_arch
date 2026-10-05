# Implementation plan: shared autonomy and physical composition

Open work only. Remove completed tasks once their durable usage and compatibility
information is in the owning documentation; Git and PRs retain implementation
history. Follow [architecture.md](architecture.md),
[build_and_layout.md](build_and_layout.md), and [AGENTS.md](../AGENTS.md).

Preserve the existing build and simulation baseline; commands and supported
behavior are documented in [README.md](../README.md) and the
[world construction guide](../src/robo_arch/core/worlds/README.md).
The tasks below are pending implementation and validation. Design documentation
and existing demonstrations do not close their acceptance gates.

## Architecture acceptance work

These gates precede further backend/demo expansion. Exact gripper, hardware
command interface and replacement numerical dependency remain open decisions.

**K1 — Shared numerical implementation and world selection**

Own `core/controllers/` and its controller-construction consumers. Establish one
maintained implementation of each control law and its nominal dynamics across
Drake, batched Isaac and deployment. Investigate a shared maintained dependency, including JAX-based kinematics,
or generated execution from authoritative code before choosing a replacement.
Assess a narrow native dynamics query if needed; document any inability to share
nominal dynamics and preserve explicit simulation/deployment model semantics.
The handwritten Torch dynamics tree is not the accepted long-term backend;
model extraction and parity tests do not remove its duplicated semantics.
Preserve a working shared CPU path with explicit costs during migration. Do not
claim GPU throughput or silently remove supported behavior to close this task.

Scenarios select autonomy and parameters; world construction resolves compatible
implementations using device commands, observation requirements and execution
settings. Remove scenario-local backend compatibility ladders and user-facing
duplicate backend selections. Record the effective selection; reject unsupported
combinations without substituting algorithms, disabling sensors or introducing
privileged model inputs. Keep native Python construction and ports, without a
universal factory context. Coordinate command semantics with H0 and compound
models with I2 before accepting a numerical dependency.

**Comments:**
Note it just might be worth having some abstract version that handles all worlds equally well, but it is an ideal we should strive for. If we can fix this by just choosing a better tool/library then we should, but if not we should document why we didn't. I think there are now JAX based kinematics that might be a good choice for helping with this. I am not sure how dynamics factors in, but simulators typically supply dynamics so it might be unavoidable to query the world for its dynamics. But maybe we can have that query be relatively narrow.

**I2 — Mounted actuated tools**

Own attachment declarations, resolution and physical assembly changes under
`core/config/`, `core/worlds/`, and the selected gripper/system packages. Resolve
robot/tool parents as instance-qualified frames, reject cycles and missing
frames, and preserve local transforms, actuator ownership and calibration
identity. World loaders must assemble moving tools into the physical mechanism;
adding a parent string while retaining a separate world-fixed articulation is
insufficient. Physical attachment topology is distinct from organizational
system nesting. A native articulation may contain several declared devices;
retain per-device state/command mappings and anchor only true physical roots.
Controller models must include the mounted device's configuration
and dynamics, not treat an actuated gripper as a permanently welded payload.

Prefer the manipulation project's WSG assets if their physics and licensing fit;
Robotiq is optional. Keep a model-neutral actuated test fixture available while
selection is unresolved, but do not count it as actual gripper support. The mounted-gripper
gate is mandatory before further backend/demo expansion. Exact model, command
mapping and any mimic/coupled-joint behavior must be established from the selected
device, not guessed or silently simplified.

Implement explicit per-instance hardware bindings and calibration profile
selection at device, assembly and scenario scope. Validate device identities,
mounting revisions, frames and conflicting effective transforms without importing
SDKs. Expose per-device observations/commands without fixing the autonomy stack.

**Comments:**
You can feel free to use something other than the robotiq gripper especially if the assets are more easily found. I think Tedrake's manipulation repo likes the wsg gripper so may as well do that

**O0 — Movable objects and contact-driven manipulation**

Own object declarations/assets and their scene-state integration. Distinguish
fixed fixtures from free objects, with explicit initial pose, velocity frames
and reset state. Support native free bodies in both worlds, including inertia,
collision geometry and per-environment identity. Implement drop/settle and robot
push evidence first; grasp/lift/release depends on I2. Object-state observations must identify any simulator ground truth rather than present it as deployment sensing.

**Comments:**

**X2 — Native Isaac lifecycle and scoped compatibility**

Own `core/worlds/isaac/` execution and scenario rollout migration. Split scene
population from runtime creation so the selected native Lab environment owns its
simulation context and scene lifecycle. Establish the pinned environment's
stepping, decimation, observation, terminal-state and selective-reset contracts
before replacing the existing loops. Native explicit stepping is legitimate;
duplicating lifecycle responsibilities across scenarios is the problem. Keep
task/reference/evaluation logic with scenarios and numerical algorithms with K1.

Unify command conversion/validation and lifecycle bookkeeping within the Isaac
integration. Move workload-specific capacity tuning into selected profiles and
scenario-specific sampling out of world defaults. Isolate unavoidable SDK
workarounds by affected version and test startup, failure and shutdown. Coordinate
scene ownership with I2/O0; no new universal scheduler or second runtime wrapper.

**Comments:**

**H0 — Deployment boundary without physical hardware**

Own shared ROS transport under `core/worlds/real/`, device-specific adapters with
their devices, and launch material under `deployment/`. First establish the
intended driver/controller versions and actual command/state interfaces for the
UR arm only; gripper and sensor drivers are outside this initial gate. Exercise
the same deployment adapter against supported ros2_control mock hardware and,
where applicable, URSim, following the [UR driver simulation support](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/simulation.html).
Do not invent a convenient torque driver to match the simulation examples. Keep
ROS outside ordinary autonomy and batched computation.
Add controllable transport failures and timestamps at the real process boundary.
Mock evidence covers software behavior, not physical timing, contact fidelity or
hardware safety. Calibration selections may use explicitly synthetic test values
without claiming measured calibration.

**Comments:**
Keep this one relatively narrow. Just the UR should be enough.

**P0 — Clean-machine reproduction and local validation**

Own root build/launch tooling, `tests/build/`, vendor environment manifests and
deployment packaging. Preserve uv resolution and Bazel native compilation.
Limit the initial OS scope to Ubuntu and macOS, with supported runtimes and
architectures explicit per OS; this does not imply Isaac support on macOS. Prefer
hermetic dependencies and supply a setup script for required host prerequisites.
Consider generating the support matrix from maintained dependency metadata.
Establish lock freshness checks,
resolved Kit extension inventory and host requirements. Provide one local
validation entry point with explicit core, Drake, vendor-numerical and native
integration suites; a requested suite must fail if providers are absent.
Exercise installed resource loading without editable imports. Source reproduction
and transfer of built artifacts are distinct claims and need distinct evidence.
No hosted CI, paid machines, driver changes or additional build system is implied.

**Comments:**
We can keep the support relatively narrow. Ubuntu and mac should be enough. Ideally, if we need any setup we should have a setup script similar to what is in Drake's build system, but ideally things are relatively hermetic. If we can autogenerate a support matrix that would be cool.

## Dependencies and ownership

K1, I2, H0 and P0 can proceed in parallel when assigned;
device choices and shared command/model contracts remain explicit dependencies.
X2 can establish native lifecycle ownership in parallel, but changes to scene
construction must be serialized with I2/O0. O0 declaration work can proceed beside
I2; grasping acceptance waits for the mounted gripper. Assign each shared file to
one implementation owner at a time, and send required edits to that owner.
The coordinator owns this plan and integrates the gates after relevant local
checks. Neither existing throughput results nor passing schema tests substitute
for the missing physical and deployment evidence.

**Comments:**

## Acceptance gates

| Gate                                  | Required evidence                                                                                                                                                                                                                                                                                                                                                                 |
| ------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Shared computation and selection (K1) | One maintained numerical implementation serves the claimed worlds/batch modes; changing only world/execution settings selects compatible adapters centrally. Matched-input/state and reset tests cover the same computation; no independent handwritten dynamics tree or scenario backend ladder remains in the accepted path. Unsupported combinations fail explicitly.          |
| Mounted gripper (I2)                  | A separately declared actuated gripper follows the arm flange in Drake and Isaac, with combined inertia/collision geometry and independently mapped commands. Two copies retain distinct identities, state and calibration selections. A visual housing or permanently welded payload does not qualify.                                                                           |
| Movable objects (O0)                  | Native drop/settle, contact-driven push, grasp/lift/release in both worlds; object pose/velocity and selective batch reset are exercised without teleporting or attaching the object to fake grasp success. Actual-run visualization accompanies physical acceptance evidence.                                                                                                    |
| Native execution (X2)                 | One Isaac lifecycle handles observations, command conversion, decimation and selective reset of physics, task and controller state. Resetting one environment preserves others. Startup/failure/shutdown and native viewing are exercised; scenarios do not own duplicate physics loops.                                                                                          |
| Hardware boundary (H0)                | The deployment adapter communicates through the intended ROS interfaces with realistic mock/vendor simulation processes. Tests cover names/units/frames, command rejection, stale observations, disconnect/reconnect, lifecycle and calibration identity without requiring a physical robot.                                                                                      |
| Portability and validation (P0)       | A clean second machine reproduces source builds/runs within the supported matrix without developer caches or local paths. Separately, transferred artifacts load and run without rebuilding where portability is claimed. Record OS, native ABI, GPU/driver and resolved vendor extensions; missing environments or unavailable machines leave the relevant evidence outstanding. |

Bazel CPU/Drake checks remain independent of `.venv`; vendor checks run in their
identified environment. Retain meaningful parity tests even after structural
sharing, measure rather than infer performance, and do not require matching
trajectories from different physics engines. No formal CI is required.

**Comments:**

## Deferred work

Keep these unresolved items after the architecture gates; they are not complete:

- Physical robot operation and measured device, mount and fixture calibration,
  after device/driver selection and the mock deployment gate. Synthetic profiles
  can validate selection semantics but are not measured calibration.
- Isaac camera observations, unsupported RTX/collision viewing, and actual RViz
  observations/TF rendering. Existing physics and fake viewer-process checks do
  not establish these capabilities; missing support must remain explicit.
- Complete nut-on-pin behavior and learned-policy training, after mounted-tool
  and movable-object acceptance and selection of actual part geometry and sensing.

Runtime and machine limitations stay in the [dependency notes](../third_party/compatibility.md)
and [Isaac profile](../third_party/isaac/README.md). Unavailable hardware or a
second machine leaves its required evidence outstanding; do not substitute a
passing mock or same-machine run.

**Comments:**

<!-- Add comments here. -->
