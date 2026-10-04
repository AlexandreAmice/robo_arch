# ATI Mini45-R

This nominal, aluminum-plate model uses the dimensions in ATI drawing
[9230-05-1094 revision 20](https://www.ati-ia.com/app_content/Documents/9230-05-1094.auto.pdf):
45 mm outside diameter, 9.4 mm through-bore and 15.7 mm overall height, including
a 3.7 mm tool plate. Geometry is an original dimension-based model, not a
redistribution of ATI CAD. Bolt holes, cable and the small cable outlet are
omitted. Thirty-two convex annular sectors preserve the through-bore in both
simulators; the radial faceting error is under 0.11 mm.

`mount` is centered on the mounting face. `sensing` is centered 2 mm behind the
tool face; +z points toward the tool. `tool_flange` is the outer tool face.
The 91.7 g nominal total mass follows the
[ATI transducer manual](https://www.ati-ia.com/app_content/documents/9610-05-1031.pdf).
Mass division between mounting body and tool plate is proportional to their
modeled volumes; each inertia assumes a uniform annulus. These are explicit
inertial approximations, not measurements. System mounting transforms are nominal;
no fabricated adapter plates or measured mounting calibration are claimed.

The tool plate is a separate rigid body connected by `sensing_joint`. Both
backends read that joint's reaction, including gravity and inertia of the tool
side. Output is `[Fx, Fy, Fz, Tx, Ty, Tz]` in N and N m, at the sensing origin,
expressed in its frame, with **mounting side acting on tool side** sign. This is
ideal mechanical sensing: no electronics, filtering, bias, saturation or automatic
gravity compensation. The model has no hardware driver.

Drake reads joint reactions from its plant output. Isaac Lab uses
`JointWrenchSensor` with the incoming joint frame and resolves the sensed body
by **link name**. Samples are recorded after physics steps; time-zero and
just-reset wrench values are NaN until the solver steps from the initial state. Sensor bodies remain when
observations are disabled. The analytic test checks rotated gravity, an offset
payload and an external force/torque:

```sh
uv run pytest src/robo_arch/sensors/ati_mini45/tests
ROBO_ARCH_VISUALIZE=1 uv run pytest src/robo_arch/sensors/ati_mini45/tests -s
```

Isaac Lab's pinned Newton joint-wrench sensor excludes fixed joints, including
the Mini45 sensing joint. Newton observations are rejected before startup;
`sensors_enabled: false` retains the physical model without claiming a wrench
measurement. PhysX sensing remains supported.
