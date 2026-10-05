# Documentation

Start with the [repository quickstart](../README.md) to install dependencies and
run a scenario. For implementation, prefer the executable examples and the source
files they import over prose descriptions of current code.

| Need | Authoritative location |
|---|---|
| Load and inspect a declared system | [`examples/configuration.py`](../src/robo_arch/examples/configuration.py) |
| Construct a native Drake scene | [`examples/drake_scene.py`](../src/robo_arch/examples/drake_scene.py) |
| Compose and evaluate the effort CBF | [`examples/cbf_filter.py`](../src/robo_arch/examples/cbf_filter.py) |
| System concepts and design constraints | [Architecture](architecture.md) |
| File placement, builds and native development | [Build and layout](build_and_layout.md) |
| Public units, shapes, ownership and errors | [Generated API catalogue](api/index.rst) |
| Remaining architecture work | [Implementation plan](implementation_tasks.md) |
| Model provenance and approximations | Owner READMEs under [`robots/`](../src/robo_arch/robots), [`sensors/`](../src/robo_arch/sensors) and [`objects/`](../src/robo_arch/objects) |
| Dependency and runtime compatibility | [Compatibility](../third_party/compatibility.md) and [Isaac profile](../third_party/isaac/README.md) |

Scenario READMEs contain launch, inspection, benchmark and visual-test commands.
Their `run.py`, native adapter, task/evaluation and tests are the maintained
implementation examples. Package READMEs should link to those files rather than
restate their control flow.

Maintain public contracts in Python docstrings or public C++ declaration comments.
The API catalogue imports those sources without simulator SDKs and fails on
missing or duplicate selected entries. See
[API documentation](build_and_layout.md#api-documentation) for build commands and
the current coverage boundary.
