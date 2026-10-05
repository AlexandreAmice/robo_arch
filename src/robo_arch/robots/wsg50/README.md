# WSG 50 stock-tip model

The articulated WSG 050-110-P model derives from RobotLocomotion/models
revision `f85076b4d95a8bdfec70153015e723e92312e4e2`, `wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf`.
Upstream authored the meshes from Schunk catalog drawings rather than restricted
manufacturer CAD. The meshes converted from glTF to OBJ (baked scene transforms, explicit glTF Y-up to Z-up rotation, unchanged geometry)
and derived URDF retain the upstream
BSD-3-Clause notice in LICENSE. This single URDF feeds both world loaders.

The body and two prismatic finger frames, masses, joint limits, box/sphere
collision geometry and visual meshes follow that source. URDF positions are
relative to body, whose upstream model-frame translation was y=-0.049133 m.
The left finger rotation is expressed with exact pi rather than rounded pi.

The upstream finger diagonal inertia of 0.16 kg m² at 0.05 kg is replaced by a
named **uniform collision-box inertia approximation**: Ixx=m(y²+z²)/12 and cyclic
permutations, using each link's main collision box dimensions and source mass.
The same approximation supplies body inertia. Centers of mass remain at link
origins; these are nominal simulation values, not manufacturer identification.
Joint speed is limited to a nominal 0.2 m/s per finger, not a hardware claim.
The imported nominal 80 N per-joint effort bound is distinct from controller
command force. Stock-tip spheres preserve fingertip contact; no grasp weld is used.

The two-finger model has independent prismatic joints. A shared centering/width
controller represents mechanical synchronization as finite feedback, not an
exact kinematic constraint. Positive aperture travel is q_right-q_left; at zero
travel the source tip spheres leave approximately 5 mm physical clearance.
Mounted systems own the UR adapter transform and calibration identity.
