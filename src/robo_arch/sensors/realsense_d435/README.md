# RealSense D435

Mechanical geometry and nominal optical transforms are derived from
[realsense-ros](https://github.com/realsenseai/realsense-ros/tree/9a11121700cb4780e273e34141f6402fe184321d/realsense2_description),
revision `9a11121700cb4780e273e34141f6402fe184321d`. The Apache-2.0 notice is
retained in `THIRD_PARTY_LICENSES/`. The source D435 DAE is converted to OBJ with
trimesh 4.11.1. The same housing mesh supplies visual and convex-hull collision
geometry in both worlds; small openings are filled by the hull. Cables and a
physical mounting bracket are not modeled.

`mount` is the bottom screw frame. `depth_optical` and `color_optical` use +z
forward, +x right, +y down; the nominal RGB optical center is 15 mm from the depth
center. The body is approximately 90 × 25 × 25.05 mm. Its 72 g nominal mass comes
from the upstream description; inertia is an explicit uniform-box estimate,
not the upstream unreliable inertia tensor or a measured calibration.

Drake provides **ideal pinhole RGB-D rendering**, with the nominal separate RGB
and depth origins. The configured resolution/FOV are simulation choices (64×48
and 60° vertical by default), not calibrated D435 intrinsics. There is no stereo
matching, distortion, noise, latency, projector or hardware driver. Images are
not automatically registered across the two optical centers.

Isaac supports the physical housing but not image generation. Selecting this
sensor with observations enabled in Isaac is an error. `--no-sensors` disables
observations while retaining the housing, inertia and collisions.
