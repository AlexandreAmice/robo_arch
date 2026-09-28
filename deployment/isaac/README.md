# Local Isaac feasibility (G0)

**Measured on 2026-09-28:** the camera-free GPU free-fall probe passes in the
Isaac-distributed Kit/PhysX runtime. The machine fails NVIDIA's compatibility
checker for VRAM. This is limited feasibility evidence on hardware below the
published minimum, not full Isaac Core/SimulationApp or robot validation.

## Environment and reproduction

This directory is an independent uv project; its `.venv` and `uv.lock` do not
modify the root development environment. Python is pinned to 3.12.13, Isaac Sim
to 6.1.0.0, and direct Kit extension dependencies in `probe.kit`. `packaging==26.0`
repairs an undeclared import dependency in the vendor compatibility checker.
Only the compatibility bundle is installed; the probe uses Kit's direct PhysX
API. There are no cameras, downloaded scene assets, robot models or controllers.

From the repository root, with uv 0.12.17:

```sh
cd deployment/isaac
uv sync --locked
mkdir -p .cache
env -u DISPLAY -u WAYLAND_DISPLAY OMNI_KIT_ACCEPT_EULA=YES \
  PYTHONUNBUFFERED=1 timeout --signal=TERM --kill-after=10s 180s \
  /usr/bin/time -v .venv/bin/python probe.py > .cache/probe.log 2>&1
```

`OMNI_KIT_ACCEPT_EULA=YES` accepts NVIDIA's runtime EULA. Unsetting display
variables prevents a vendor IOMMU warning dialog from blocking headless startup;
it does not disable IOMMU. The warning remains in the logs. No driver, kernel,
BIOS or power-governor settings were changed.

The Python lock contains hashes. Kit downloads additional extensions to its
normal user cache on first use; those transitive extension artifacts are not
covered by `uv.lock`. The observed runtime was Kit 110.3.0 (build `00c488ae`),
PhysX extension 110.3.2. This profile is not an offline/hermetic installation.
Full Core/SimulationApp imports are intentionally absent and remain untested.

The vendor checker can also be reproduced:

```sh
env -u DISPLAY -u WAYLAND_DISPLAY OMNI_KIT_ACCEPT_EULA=YES \
  PYTHONUNBUFFERED=1 timeout --signal=TERM --kill-after=10s 120s \
  /usr/bin/time -v .venv/bin/isaacsim isaacsim.exp.compatibility_check \
  --no-window --/app/quitAfter=10 --/app/settings/persistent=false \
  --/privacy/userConsent=false > .cache/compatibility.log 2>&1
```

Read its reported result: it exits **0 even when its checks fail**. The physics
probe instead raises on missing runtime, incorrect motion or absent GPU solver
allocation. No skipped check counts as validation.

## Observed compatibility and results

| Item | Local observation |
|---|---|
| OS / libc | Ubuntu 24.04.4, kernel 6.8.0-142, x86-64, glibc 2.39 |
| CPU / RAM | i9-11980HK, 8 cores / 16 threads, 62 GiB RAM |
| GPU / driver | RTX 3060 Laptop, 6144 MiB VRAM, NVIDIA 610.57.04 |
| Python / packages | CPython 3.12.13; `uv sync --locked` and `uv pip check --python .venv/bin/python` pass (61 installed packages) |
| Vendor checker | `System checking result: FAILED`; VRAM insufficient; driver, RTX GPU and OS checks pass |
| Numerical probe | Exit 0; 30 steps at 1/120 s; cube falls from 1 m to 0.6832187175750732 m |
| Expected result | Semi-implicit Euler: `1 - 9.81 * dt² * 30 * 31 / 2 = 0.68321875 m`; error < 1e-5 m |
| GPU evidence | GPU dynamics and broadphase requested; PhysX initializes CUDA device 0; scene statistics report 67,108,864-byte GPU heap and 22,091,944-byte GPU solver heap |
| Final warm run | 1.704 s startup, 2.20 s total process time, 2,587,612 KiB maximum RSS |
| GPU memory | 581 MiB maximum sampled by `nvidia-smi` every 100 ms during final run (device-wide, not a guaranteed instantaneous peak) |
| Disk | 72 GiB free initially; 55 GiB after concurrent B0/G0 work; this `.venv` occupies 351 MiB, excluding shared Kit/uv caches |

These startup measurements use warm extension/shader caches, are not throughput
benchmarks, and include no controller. The continuous-time free-fall solution is
0.6934375 m; the difference is the expected discrete integration error.

The checker uses a 10 GB VRAM threshold, while the current requirements page
lists 16 GB and an RTX 4080. This laptop fails both. The checker also sums storage
across mounts; the free-space figures above come from `df -h .` on the actual
installation filesystem. Other warnings include IOMMU, powersave CPU governor
and PCIe link width. No system settings were changed to suppress these warnings.

Initial attempts exposed two setup issues: the missing `packaging` import made
the checker extension fail despite exit 0, and a DISPLAY-enabled checker timed
out at 120 s waiting on the IOMMU dialog. A similarly blocked first physics
startup was terminated; the documented display-free commands then completed.
Ruff 0.16.9 lint (E4/E7/E9/F/I/UP/B, py312) and format checks pass for `probe.py`.

## Boundaries and downstream work

G0 establishes only this small GPU physics case. X0 still needs to validate its
chosen full API environment and arm workload, then satisfy S0/A0/K0 dependencies.
The 6 GB GPU remains unsupported by the published requirements; camera, complex
scene, batch capacity, controller reuse and hardware behavior are unvalidated.
No Isaac runtime expansion, fallback simulator or paid infrastructure was used.

References checked on 2026-09-28:
[NVIDIA requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html),
[Python installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html),
[compatibility checker](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html#isaac-sim-compatibility-checker),
[direct PhysX simulation](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.0/dev_guide/simulation_control/simulation_control.html).
