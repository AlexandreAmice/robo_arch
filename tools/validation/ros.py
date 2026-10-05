"""Build the pinned ROS underlay with an installed wheel, then run process tests."""

import json
import os
import subprocess

from tools.validation.profiles import ROOT

IMAGE = "robo-arch-ur:local"


def run(*, build: bool = True) -> None:
    output = ROOT / "build/ros"
    output.mkdir(parents=True, exist_ok=True)
    lock = json.loads((ROOT / "third_party/ros/packages.lock.json").read_text())
    if build:
        subprocess.run(["uv", "lock", "--check"], cwd=ROOT, check=True)
        subprocess.run(
            [
                "uv",
                "export",
                "--locked",
                "--no-default-groups",
                "--no-emit-project",
                "--format",
                "requirements-txt",
                "--output-file",
                str(output / "requirements.txt"),
            ],
            cwd=ROOT,
            check=True,
        )
        subprocess.run(
            ["uv", "build", "--wheel", "--out-dir", str(output)], cwd=ROOT, check=True
        )
        subprocess.run(
            [
                "docker",
                "build",
                "--network",
                "host",
                "--platform",
                lock["platform"],
                "--build-arg",
                "ROS_BASE=" + lock["base"],
                "-f",
                "deployment/ur/Dockerfile",
                "-t",
                IMAGE,
                ".",
            ],
            cwd=ROOT,
            check=True,
        )
    evidence = ROOT / "recordings/ur_mock"
    evidence.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "docker",
            "run",
            "--rm",
            "--network",
            "none",
            "--user",
            f"{os.getuid()}:{os.getgid()}",
            "-e",
            "HOME=/tmp",
            "-e",
            "ROBO_TRACE_DIR=/evidence",
            "-v",
            f"{ROOT / 'deployment/ur/tests'}:/checks:ro",
            "-v",
            f"{evidence}:/evidence",
            IMAGE,
            "python",
            "-m",
            "pytest",
            "-q",
            "-p",
            "no:cacheprovider",
            "/checks",
        ],
        cwd=ROOT,
        check=True,
    )


if __name__ == "__main__":
    run()
