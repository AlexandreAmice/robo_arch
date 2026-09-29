# UR7e

`assets/model.urdf` is a flattened, nominal six-joint model derived from
[Universal Robots' description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/89bbe795f38a7ab00fb66fe8831dfff79dc99edf),
pinned to commit `89bbe795f38a7ab00fb66fe8831dfff79dc99edf`. The
upstream model is a ROS/Xacro template rather than a directly loadable URDF;
the local file avoids requiring ROS or Xacro at runtime. Joint transforms,
masses, centers of mass and inertia tensors come from
`config/ur7e/{default_kinematics,physical_parameters}.yaml`; joint limits come
from `config/ur7e/joint_limits.yaml`. Link/frame conventions follow
`urdf/ur_macro.xacro`.

This is an uncalibrated nominal model. The upstream base mass is itself marked
uncertain, and the elbow position range is restricted by upstream for planning.
The package adds effort transmissions so Drake creates the six joint actuators
and replaces upstream mesh visuals with approximate cylinders to keep this
small model self-contained. **There is no robot collision geometry:** this
asset supports dynamics/tracking examples, not contact or collision
validation. It contains no gripper, hardware driver, motor/friction model or
per-unit calibration.

The upstream BSD-3-Clause text applies to this derived model and is retained in
`THIRD_PARTY_LICENSES/Universal_Robots_ROS2_Description.txt`. It is a
third-party notice, not a license grant for the `robo_arch` project as a whole.

`drake.add_to_plant(plant, name=...)` loads an independently named instance.
The caller welds `base_link` and finalizes the plant. `definition.py` provides
joint order and a demonstration starting configuration without importing Drake.
