"""Validate a transferred wheel bundle; requires uv, never a source checkout/build."""

import hashlib
import json
import os
import platform
import subprocess
import tempfile
from pathlib import Path


def main() -> None:
    bundle = Path(__file__).resolve().parent
    manifest = json.loads((bundle / "manifest.json").read_text())
    if (platform.system(), platform.machine()) != (
        manifest["system"],
        manifest["architecture"],
    ):
        raise RuntimeError(
            "Bundle requires its declared OS/architecture; native wheels are not universal"
        )
    for name, digest in manifest["sha256"].items():
        if hashlib.sha256((bundle / name).read_bytes()).hexdigest() != digest:
            raise ValueError(f"Artifact differs from manifest: {name}")
    with tempfile.TemporaryDirectory(prefix="robo-artifact-") as directory:
        work = Path(directory)
        environment = work / "environment"
        subprocess.run(
            ["uv", "venv", "--python", manifest["python"], str(environment)], check=True
        )
        python = environment / "bin/python"
        subprocess.run(
            [
                "uv",
                "pip",
                "install",
                "--python",
                str(python),
                "--no-deps",
                "--require-hashes",
                "-r",
                str(bundle / "requirements.txt"),
            ],
            check=True,
        )
        subprocess.run(
            [
                "uv",
                "pip",
                "install",
                "--python",
                str(python),
                "--no-deps",
                *[str(bundle / name) for name in manifest["wheels"]],
            ],
            check=True,
        )
        env = os.environ.copy()
        env.pop("PYTHONPATH", None)
        env["PYTHONNOUSERSITE"] = "1"
        subprocess.run(
            [str(python), "-I", str(bundle / "installed.py")],
            cwd=work,
            env=env,
            check=True,
        )


if __name__ == "__main__":
    main()
