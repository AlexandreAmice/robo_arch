"""Produce independently verifiable Python/native artifacts from Bazel and uv."""

import hashlib
import json
import platform
import shutil
import subprocess
from pathlib import Path

from tools.validation.profiles import ROOT, check_host, profile


def create(name: str, output: Path) -> None:
    record = profile(name)
    check_host(record)
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    subprocess.run(
        ["uv", "build", "--wheel", "--out-dir", str(output)], cwd=ROOT, check=True
    )
    subprocess.run(
        [
            "uv",
            "export",
            "--locked",
            "--no-default-groups",
            "--no-emit-project",
            "--output-file",
            str(output / "requirements.txt"),
        ],
        cwd=ROOT,
        check=True,
    )
    subprocess.run(
        ["bazel", "build", "--lockfile_mode=error", "//tools/native:wheel"],
        cwd=ROOT,
        check=True,
    )
    wheel = subprocess.check_output(
        ["bazel", "cquery", "//tools/native:wheel", "--output=files"],
        cwd=ROOT,
        text=True,
    ).strip()
    shutil.copyfile(ROOT / wheel, output / Path(wheel).name)
    for name in ("installed.py", "verify_bundle.py"):
        shutil.copyfile(ROOT / "tools/validation" / name, output / name)
    manifest = {
        "system": record["system"],
        "architecture": record["architecture"],
        "python": record["python_version"],
        "libc": platform.libc_ver(),
        "source_revision": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        "wheels": sorted(path.name for path in output.glob("*.whl")),
        "sha256": {
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(output.iterdir())
        },
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
