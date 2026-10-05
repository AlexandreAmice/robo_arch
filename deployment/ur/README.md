# UR trajectory deployment

The installed `UrTrajectoryAdapter` uses the UR driver's
`FollowJointTrajectory` action and named `JointState` positions/velocities.
It requires an explicit installation binding and selected calibration profile;
synthetic calibration is valid for mock tests. Effort autonomy is rejected before
ROS imports. Default ROS effort fields are not treated as joint torques.

Run the same adapter against the actual UR7e driver and ros2_control mock process:

```sh
python3 -m tools.validation run --profile ros --suite ros
python3 deployment/ur/plot_trace.py recordings/ur_mock/trajectory.json
python3 -m http.server 8000 --directory recordings/ur_mock
```

Open `http://localhost:8000/trajectory.svg` for the measured joint trace. The six
process tests cover installed imports, trajectory/state round trips, cancellation,
stalled observations, controller rejection and cancellation of a goal accepted
after a client timeout. Reset reacquires state; it does not home the arm. A lost
connection cannot guarantee a remote stop.

The build installs an application wheel over a digest-pinned Jazzy base and the
232 exact Debian artifacts in `third_party/ros/packages.lock.json`. Artifact
SHA256 values are checked before installation. ROS uses its underlay's Python
3.12.3; application dependencies are exported from the root uv lock without
simulators. Docker build networking downloads inputs; tests run with no external
network. No host ROS packages, drivers or services are changed.

The selected driver is 3.8.0 and joint trajectory controller is 4.42.1. The vendor
launch file uses `/controller_manager`, so this mock profile runs at namespace
`/` in an isolated container. The adapter accepts explicit namespaces for other
installations. Exact package revisions and the base image are recorded in the
lock, rather than resolved at launch.

URSim and physical robot execution remain unvalidated. A URSim connection would
use the same action adapter with the driver's scaled trajectory controller;
URSim does not establish effort control. Gripper and sensor drivers are outside
this boundary. See the vendor's [mock trajectory instructions](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/jazzy/doc/ur_robot_driver/ur_robot_driver/doc/usage/move.html)
and [simulation limitations](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/jazzy/doc/ur_robot_driver/ur_robot_driver/doc/usage/simulation.html).
