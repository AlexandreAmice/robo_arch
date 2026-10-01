# Dependency compatibility

Development baseline: Ubuntu 24.04, Linux x86-64. The current native build uses
host glibc; it is not yet a portable manylinux release build.

| Dependency | Pinned version | Configuration |
|---|---|---|
| uv | 0.12.17 | `pyproject.toml` |
| CPython | 3.12.13 | `.python-version` and Bazel Python toolchain |
| Pydantic / PyYAML | 2.13.5 / 6.0.3 | `pyproject.toml` and `uv.lock` |
| Drake | 1.57.0 | Optional uv dependency group and explicit Bazel target dependencies |
| Bazel | 9.2.0 | `.bazelversion` |
| LLVM / C++ | 22.1.8 / C++23 | `MODULE.bazel` and `.bazelrc`; bundled libc++ |
| nanobind | 3.0.1 | `MODULE.bazel`; regular CPython ABI |
| Ruff / clang-format / Buildifier | 0.16.9 / 22.1.8 / 10.1.0 | uv and Bzlmod configuration |
| Hedron compile commands | `abb61a688167623088f8768cc9264798df6a9d10` | Development-only `git_override` in `MODULE.bazel` |

[Hedron](https://github.com/hedronvision/bazel-compile-commands-extractor) generates
the C++ editor database. Its upstream rules still use native Python/C++ rules
removed in Bazel 9. `hedron/bazel9.patch` declares the existing rules_python and
rules_cc dependencies and loads their rules explicitly. Bazel 9 also omits action
keys from its cache dump; the patch skips that optimization and lets Hedron
discover headers through preprocessing and its own header cache. Remove the
patch when an upstream pin supports these changes; see the
[editor setup](../README.md).

VS Code uses the toolchain's clangd, matching its libc++ headers. Do not add
`--query-driver` here: the extracted commands already carry the include paths,
and querying the wrapper promotes C system headers ahead of libc++'s wrappers.

`uv.lock` is authoritative for core and Drake Python dependencies. In
`MODULE.bazel`, rules_python's `pip.parse(uv_lock = "//:uv.lock", ...)` reads its
package versions, artifact URLs, hashes and resolution markers directly into
`@python_deps`. The extension is named `pip`; this path requires no requirements
export or separate dependency resolution. Our editable package remains a local
Bazel source target rather than an external wheel.

Direct ingestion exposes the whole lockfile, including development and Drake
packages; it does not select uv dependency groups. Each BUILD target declares
its actual dependencies through `@python_deps//:requirements.bzl`. Core targets
therefore have no Drake or formatter dependencies; the Drake test explicitly
depends on Drake. uv groups still select the separate development environments.
After editing dependencies, run `uv lock`, then the relevant Bazel tests to
refresh `MODULE.bazel.lock`. Use `uv lock --check` and Bazel's
`--lockfile_mode=error` to check committed locks without updating them.

Drake's Python package can be imported and used to construct a Diagram with this
setup. Passing Drake C++ objects between project extensions and pydrake still
requires a compatible native Drake build, compiler/standard-library ABI,
nanobind ABI/domain and ownership conventions. Matching nanobind version numbers
alone is insufficient. The native wheel and this interoperability are not yet
implemented.

Isaac uses an [independent dependency profile](isaac/README.md) rather than the
root Python dependency resolution. The initial LLVM download/extraction can
consume approximately 13 GB of cache storage.

Build references: [Drake dependencies](https://github.com/RobotLocomotion/drake/blob/v1.57.0/MODULE.bazel),
[nanobind with Bazel](https://nanobind.readthedocs.io/en/latest/bazel.html),
[LLVM toolchains](https://github.com/bazel-contrib/toolchains_llvm/tree/v1.10.0).
