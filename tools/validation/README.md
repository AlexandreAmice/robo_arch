# Local validation

`third_party/compatibility.toml` selects runtime profiles and explicit suites.
Versions remain in uv locks, interpreter pins and Bzlmod. The generated
[`support.md`](../../third_party/support.md) distinguishes configured platforms
from measured execution. Host setup checks never install OS packages or drivers.

```sh
python3 -m tools.validation setup --profile ubuntu --sync
python3 -m tools.validation run --profile ubuntu --suite core --suite native --suite drake --suite installed
python3 -m tools.validation matrix --check
```

Requested providers/interpreters must exist and match; a skipped selected test
fails the suite. Native compilation uses the existing Bazel aggregate. `--bazel`
also runs suites' declared independent Bazel targets. Isaac/tensor/Moreau suites
use the separately locked Isaac environment; ROS uses the
[isolated deployment image](../../deployment/ur/README.md).

For the configured macOS 15+ arm64 profile, use `--profile macos` and
`export UV_PYTHON="$(cat .python-version-macos)"` before ordinary `uv run`
commands. Drake 1.57 provides CPython 3.13 macOS wheels; the Linux and Isaac
profiles retain Python 3.12. Bazel selects the matching macOS toolchain/wheel tag.
Artifact selection was checked on Linux; macOS execution is still outstanding.

Source reproduction starts with a clean checkout and the setup/run commands
above. To test transfer separately, produce a wheel bundle:

```sh
python3 -m tools.validation artifact --profile ubuntu --output dist/transfer
# Copy the directory to another matching OS/architecture, then:
python3 transfer/verify_bundle.py
```

The verifier checks artifact hashes, creates an isolated environment, installs
hash-locked dependencies and produced wheels, then checks packaged YAML/meshes
and native imports from outside the checkout. It never invokes a compiler.
It requires network access for locked Python dependencies and a pinned interpreter
if absent. Linux artifacts use host glibc; they are not manylinux releases.
The local bundle test passed, but a same-machine temporary environment does not
establish transfer or clean-source reproduction on a second machine.
