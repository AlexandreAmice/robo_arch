# UR7e

`assets/model.urdf` is a flattened nominal model derived from
[Universal Robots' description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/89bbe795f38a7ab00fb66fe8831dfff79dc99edf),
revision `89bbe795f38a7ab00fb66fe8831dfff79dc99edf`. The upstream BSD-3-Clause
notice is retained in `THIRD_PARTY_LICENSES/`.

Joint transforms, masses, centers of mass and inertias come from
`config/ur7e/{default_kinematics,physical_parameters}.yaml`; limits come from
`config/ur7e/joint_limits.yaml`. Frame conventions follow `urdf/ur_macro.xacro`.
The six effort transmissions preserve the ordering in `robot.yaml`.

Visual and **collision meshes** follow `config/ur7e/visual_parameters.yaml`,
including its exact offsets. That upstream UR7e configuration intentionally
selects the `meshes/ur5e/` assets. They are converted from DAE/STL to metre-scale
OBJ with trimesh 4.11.1, preserving coordinates and adding visual vertex normals.
The earlier cylinder visuals are removed.

Drake and PhysX use the convex hull of each source collision mesh. These are
model-specific link envelopes; concavities are filled, and the model is not a
manufacturing-clearance model. Native topology excludes adjacent and rigidly
connected links. Nonadjacent self-collision, sensor/environment collision and
inter-arm collision remain enabled.

The description is uncalibrated. Upstream marks the base mass uncertain and
restricts the elbow range for planning. There is no gripper, motor/friction model,
hardware driver or per-unit calibration. Mounted sensor inertias are included in
both the physical assembly and its separate controller dynamics model.

`robot.yaml` declares the asset and joint/frame conventions. Shared Drake loading
creates independent simulation and controller-model instances. Isaac uses the same
asset reference, composed with mounted devices before conversion. This robot
requires no Python factory or per-world wrapper.
