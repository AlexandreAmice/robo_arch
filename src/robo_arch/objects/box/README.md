# Box

Project-authored 50 mm cube, mass 0.1 kg, with uniform-density inertia.
`model.sdf` uses SI units and contains visual and collision geometry. Its body
frame `box` is at the center. The scenario chooses placement; both current
simulation worlds fix it to the world. Movable objects are not yet supported.
Isaac validates the supported SDF subset and preserves box geometry and diffuse color; see the
[world construction limits](../../core/worlds/README.md#physical-limits).
This is not a measured model of a particular physical part.
