# Dependency baseline

B0 validation host: Ubuntu 24.04.4, Linux x86-64, glibc 2.39.

| Input/profile | Pin | Evidence and boundary |
|---|---|---|
| uv | 0.12.17 | Required by `pyproject.toml`; lock/export producer |
| Python | CPython 3.12.13 | `.python-version` and Bazel toolchain; shared source minimum 3.12 |
| Core | Pydantic 2.13.5, PyYAML 6.0.3 | Editable uv and Bazel use the same source and versions |
| Drake | 1.57.0 | Separate `.venv-drake`; import and minimal Diagram construction; no world integration |
| Bazel | 9.2.0 | Matches Drake v1.57.0 `.bazelversion` |
| C++ | LLVM 22.1.8, C++23 | `std::expected` compile/run passes with bundled libc++; glibc remains a host dependency |
| nanobind | 3.0.1 | Bazel library compiles; matches Drake v1.57.0 module pin; regular CPython ABI |
| Formatting | Ruff 0.16.9, clang-format 22.1.8, Buildifier 10.1.0 | Pinned in uv/Bzlmod; clang-format settings adapted from Drake v1.57.0 |
| Isaac | Separate vendor environment | See `deployment/isaac/`; never resolved into the core/Drake lock |

`uv.lock` is authoritative for core and Drake Python dependencies.
`tools/export_requirements.py` preserves uv's hashes and platform/Python markers;
`--check` compares without modifying files. `@pip` consumes the core/test export;
`@pip_drake` consumes the separate Drake/test export. Formatting packages stay out
of these runtime/test exports. `MODULE.bazel.lock` captures Bzlmod resolution.

Passed on this host: editable core pytest; the same core test plus C++23 and
Drake smoke tests through Bazel; `@nanobind//:nanobind` library build; pinned
Buildifier/Ruff/clang-format checks; deterministic export check including a
deliberately stale export; pure Python wheel build and isolated installed import.

B0 establishes Python-level Drake compatibility only. Equal nanobind versions do
not establish C++ interoperability with wheel-bound Drake objects. Native Drake
libraries, compiler/stdlib ABI, nanobind domain and ownership must be checked in
B1 before exchanging objects. No native development wheel exists yet. This
baseline is not a manylinux release build or a simulator integration. The first
LLVM fetch/extraction uses about 13 GB of cache space on this host.

Sources used when choosing pins:
[Drake v1.57.0 module](https://github.com/RobotLocomotion/drake/blob/v1.57.0/MODULE.bazel),
[Drake version](https://github.com/RobotLocomotion/drake/blob/v1.57.0/.bazelversion),
[Drake format settings](https://github.com/RobotLocomotion/drake/blob/v1.57.0/.clang-format),
[nanobind Bazel instructions](https://nanobind.readthedocs.io/en/latest/bazel.html),
[LLVM toolchains](https://github.com/bazel-contrib/toolchains_llvm/tree/v1.10.0),
[uv lock exports](https://docs.astral.sh/uv/concepts/projects/export/).
