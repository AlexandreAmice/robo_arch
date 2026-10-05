"""Package explicit Bazel extensions/resources for a declared CPython platform."""

import argparse
import base64
import csv
import hashlib
import io
import json
import zipfile
from pathlib import Path, PurePosixPath


def build_wheel(
    output: Path,
    extensions: list[tuple[str, Path]],
    resources: list[tuple[str, Path]],
    *,
    python: str = "3.12",
    platform: str = "linux_x86_64",
) -> None:
    """Assemble a deterministic private wheel without compiling or resolving inputs."""
    supported = {("3.12", "linux_x86_64"), ("3.13", "macosx_15_0_arm64")}
    if (python, platform) not in supported:
        raise ValueError(f"Unsupported native wheel profile: {python}, {platform}")
    tag = "cp" + python.replace(".", "")
    package = "robo_arch_native"
    entries = {f"{package}/__init__.py": b'"""Bazel-built native algorithms."""\n'}
    modules = []
    for module, source in extensions:
        parts = module.split(".")
        if len(parts) != 2 or parts[0] != package or not parts[1].isidentifier():
            raise ValueError(f"Extension must name a module within {package}: {module}")
        destination = module.replace(".", "/") + ".so"
        if destination in entries:
            raise ValueError(f"Duplicate wheel destination: {destination}")
        entries[destination] = source.read_bytes()
        modules.append(module)
    for destination, source in resources:
        path = PurePosixPath(destination)
        if (
            len(path.parts) < 2
            or path.parts[0] != package
            or ".." in path.parts
            or path.as_posix() != destination
        ):
            raise ValueError(
                f"Resource must be a relative {package}/ path: {destination}"
            )
        if destination in entries or destination == f"{package}/build.json":
            raise ValueError(f"Duplicate wheel destination: {destination}")
        entries[destination] = source.read_bytes()
    entries[f"{package}/build.json"] = json.dumps(
        {"extensions": sorted(modules), "python": python}, sort_keys=True
    ).encode()
    info = "robo_arch_native-0.1.0.dist-info"
    entries[f"{info}/METADATA"] = (
        "Metadata-Version: 2.1\nName: robo-arch-native\nVersion: 0.1.0\n"
        f"Requires-Python: =={python}.*\n"
    ).encode()
    entries[f"{info}/WHEEL"] = (
        "Wheel-Version: 1.0\nGenerator: robo-arch Bazel\n"
        f"Root-Is-Purelib: false\nTag: {tag}-{tag}-{platform}\n"
    ).encode()
    record = io.StringIO()
    writer = csv.writer(record, lineterminator="\n")
    for name, content in sorted(entries.items()):
        digest = base64.urlsafe_b64encode(hashlib.sha256(content).digest()).rstrip(b"=")
        writer.writerow([name, "sha256=" + digest.decode(), len(content)])
    writer.writerow([f"{info}/RECORD", "", ""])
    entries[f"{info}/RECORD"] = record.getvalue().encode()
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as wheel:
        for name, content in sorted(entries.items()):
            entry = zipfile.ZipInfo(name, date_time=(2020, 1, 1, 0, 0, 0))
            entry.compress_type = zipfile.ZIP_DEFLATED
            wheel.writestr(entry, content)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--python", default="3.12")
    parser.add_argument("--platform", default="linux_x86_64")
    parser.add_argument(
        "--extension", nargs=2, action="append", default=[], metavar=("MODULE", "FILE")
    )
    parser.add_argument(
        "--resource",
        nargs=2,
        action="append",
        default=[],
        metavar=("DESTINATION", "FILE"),
    )
    args = parser.parse_args()
    build_wheel(
        args.output,
        [(module, Path(source)) for module, source in args.extension],
        [(destination, Path(source)) for destination, source in args.resource],
        python=args.python,
        platform=args.platform,
    )


if __name__ == "__main__":
    main()
