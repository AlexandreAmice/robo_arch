# KUKA LBR iiwa 7 R800

The packaged URDF is derived from `iiwa_description/urdf/iiwa7.xacro` in
[IFL-CAMP/iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack/tree/44f9d13c1b444d5dc9fd3e43ba60b7d3b2ea2bbb),
revision `44f9d13c1b444d5dc9fd3e43ba60b7d3b2ea2bbb`. Its BSD-2-Clause notice is
retained in `THIRD_PARTY_LICENSES/`. This is the **7 kg R800**, not the iiwa 14.

Joint transforms, inertias, position limits and link mesh offsets follow that
source. The seven actuators are ordered `iiwa_joint_1` through `iiwa_joint_7`;
`iiwa_link_0` is the base and `iiwa_link_ee` the tool frame. ROS/Gazebo tags and
Xacro dependencies are removed; effort transmissions are explicit. The source's
300 N m effort limits, 10 rad/s velocity limits and nominal inertias are simulation
parameters, not validated hardware capabilities.

Upstream visual and collision STL meshes are converted to metre-scale OBJ using
trimesh 4.11.1, preserving vertex coordinates and adding vertex normals. Drake
and PhysX use the convex hull of each collision mesh. This preserves the
model-specific link envelopes but fills concavities; it is not exact CAD contact.
Adjacent links are excluded by native joint topology; nonadjacent self-collision
and inter-arm collision remain enabled.

Both world adapters consume the same packaged model. Isaac converts the complete
arm plus mounted sensors in a subprocess, keeping its USD libraries separate
from Kit. No hardware driver or measured calibration is provided.
