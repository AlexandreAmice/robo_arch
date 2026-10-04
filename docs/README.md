# Documentation

Start with the [repository quickstart](../README.md) to install dependencies and
run an example. These guides describe the implemented code alongside the target
architecture; consult [implementation status](implementation_tasks.md) for the
boundary between them.

| Need | Authoritative location |
|---|---|
| System concepts, ownership and design constraints | [Architecture](architecture.md) |
| File placement, build, native development and documentation commands | [Build and layout](build_and_layout.md) |
| Shared function/class contracts, units, shapes and errors | [Generated API catalogue](api/index.rst), rendered from source docstrings |
| Constructing and extending a runtime | [World construction guide](../src/robo_arch/core/worlds/README.md) |
| Running and inspecting simulations | [Arm tracking](../src/robo_arch/scenarios/arm_tracking/README.md), [camera protection](../src/robo_arch/scenarios/camera_protection/README.md), [batched reaching](../src/robo_arch/scenarios/batched_reaching/README.md) |
| Controller examples, algorithms and runtime constraints | [Native PD](../src/robo_arch/core/controllers/joint_pd/README.md), [sphere effort filter](../src/robo_arch/core/controllers/cbf/README.md) |
| Dependency pins and supported environments | [Compatibility](../third_party/compatibility.md), [Isaac environment](../third_party/isaac/README.md) |
| Model provenance and approximations | [UR7e](../src/robo_arch/robots/ur7e/README.md), [iiwa 7](../src/robo_arch/robots/iiwa7/README.md), [D435](../src/robo_arch/sensors/realsense_d435/README.md), [Mini45](../src/robo_arch/sensors/ati_mini45/README.md), [box](../src/robo_arch/objects/box/README.md) |

The API catalogue has three sections: [configuration](api/configuration.rst),
[controllers and geometry](api/controllers.rst), and [device assembly/world
settings](api/worlds.rst). Follow the [build instructions](build_and_layout.md#api-documentation)
to render it locally. It is not a hosted site.

Maintain API descriptions in Python docstrings or public C++ Doxygen comments;
thin Python facades add only their Python-specific contract. Keep examples,
reasoning and operational guidance in the owning README, linking to the API
catalogue rather than maintaining another parameter list. New catalogue entries
need an explicit Bazel dependency and must import without simulator SDKs.

The [catalogue's coverage boundary](api/index.rst#coverage-boundary) identifies
remaining reference work. The current pass does not audit all SDK adapters,
device implementations or scenario APIs; their existing guides remain useful but
have not been verified by the SDK-independent documentation build.
