# Local Isaac environment

This isolated Python 3.12 environment runs the UR7e with Isaac SimulationApp,
configurable CPU/GPU PhysX and the same Drake inverse-dynamics controller used in the Drake run.
The robot adapter converts the canonical URDF with NVIDIA's USD converter; the
world loop supplies measured joint state and applies effort with drives disabled.

## Physics and native viewing

From the repository root, this automated physics check uses the GPU/TGS defaults:

```sh
uv sync --project deployment/isaac --locked
env -u DISPLAY -u WAYLAND_DISPLAY OMNI_KIT_ACCEPT_EULA=YES \
  deployment/isaac/.venv/bin/python -m robo_arch.scenarios.arm_tracking.run \
  --world isaac --no-sensors --headless \
  --trace recordings/arm_tracking_isaac.npz
```

`OMNI_KIT_ACCEPT_EULA=YES` accepts NVIDIA's runtime EULA. The JSON report retains
the effective configuration, results and inspection command; NPZ stores measured
joint positions and times, including partial traces after runtime errors. There is
no automatic Drake HTML playback. Native Isaac recording is unsupported.

For a complete Isaac profile, select `--world-config /absolute/path/world.yaml`
or a package URI. References inside YAML always use `package://robo_arch/...`:

```yaml
type: isaac
physics:
  time_step: 0.001
  solver: pgs                    # pgs or tgs; native PhysX scene solver
  device: cpu                    # cpu or cuda:0
visualization:
  type: isaac
  mode: "off"                     # off or live
  publish_period: 0.03333333333333333  # display seconds; independent of physics
  collision_geometry: false
```

CPU/PGS and GPU/TGS physics execution were verified. Native rendering remains
**experimental**: a desktop run advanced physics, but image capture did not
complete and shutdown blocked in native stage closure. No successful native
viewport/collision-overlay evidence is claimed. Its launch
path, on a desktop with `DISPLAY` or `WAYLAND_DISPLAY` set, is:

```sh
OMNI_KIT_ACCEPT_EULA=YES deployment/isaac/.venv/bin/python \
  -m robo_arch.scenarios.arm_tracking.run \
  --world isaac --no-sensors --visualization live
```

The viewer uses the same USD stage as PhysX. Collision overlay is requested with
`visualization.collision_geometry: true` in a live profile. It cannot show missing
UR7e collision geometry. Closing the viewport hides it; quitting Kit interrupts
execution. Mouse force interaction is disabled. This path needs successful render
validation on a supported graphics environment before routine visual inspection.

The initial runner supports one fixed-base arm and fixed single-link SDF boxes.
`--no-sensors` is required because the camera adapter is unavailable; turning the
viewer off does not change sensor selection. Control uses the same scalar CPU
inverse-dynamics controller as Drake and copies NumPy state/effort each step.
It is not a tensor-efficient batch implementation. The URDF converter runs in a
child process because its USD libraries conflict with Kit's in one process;
temporary converted assets are removed after the run.

## Compatibility and measured limits

On September 29, 2026, the 2-second, 2,000-step arm run on an RTX 3060 Laptop
(6 GB, driver 610.57.04; Ubuntu 24.04.4) reached a maximum final joint error of
0.000169 rad. The earlier geometry-replay workflow, including startup,
conversion, simulation and HTML generation, took 10.48 s;
peak RSS was 2.58 GiB. Observed GPU memory peaked at 396 MiB with 0.5-second
sampling, from 2 MiB idle. These are one-arm measurements, not batching capacity.

The vendor compatibility checker still rejects this GPU's VRAM. The actual
camera-free workload nevertheless runs. Enabling the native RTX camera extension
failed at extension resolution (`isaacsim.test.docstring` missing), followed by a
vendor teardown crash, before creating a camera. Camera support needs a separate
renderer/Replicator integration; no camera memory-capacity conclusion was obtained.

Isaac Sim is pinned to 6.1.0.0, PhysX/tensors to 110.3.2. `uv.lock` pins Python
packages; Kit downloads further extensions into its user cache, so this profile
is not offline/hermetic. Root dependencies remain independent. The special
OpenCV wheel uses an explicit NVIDIA index. The optional `probe.py`/`probe.kit`
remain a small independent GPU free-fall diagnostic:

```sh
env -u DISPLAY -u WAYLAND_DISPLAY OMNI_KIT_ACCEPT_EULA=YES \
  deployment/isaac/.venv/bin/python deployment/isaac/probe.py
```

References: [NVIDIA Python installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html),
[hardware requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html).
