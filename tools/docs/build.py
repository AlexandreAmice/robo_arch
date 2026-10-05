"""Build a strict API reference from isolated, installed project wheels."""

import argparse
import os
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NATIVE_PACKAGE = "tools/native"


def run(
    command: list[str], *, cwd: Path = ROOT, **kwargs
) -> subprocess.CompletedProcess:
    return subprocess.run(command, cwd=cwd, check=True, **kwargs)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "build/docs/html")
    args = parser.parse_args()
    output = args.output.resolve()
    run(["bazel", "build", f"//{NATIVE_PACKAGE}:wheel"])
    with tempfile.TemporaryDirectory(prefix="robo-arch-docs-") as temporary:
        work = Path(temporary)
        wheels = work / "wheels"
        run(["uv", "build", "--wheel", "--out-dir", str(wheels)])
        requirements = work / "requirements.txt"
        exported = run(
            [
                "uv",
                "export",
                "--locked",
                "--no-default-groups",
                "--group",
                "docs",
                "--no-emit-project",
                "--format",
                "requirements-txt",
            ],
            stdout=subprocess.PIPE,
        )
        requirements.write_bytes(exported.stdout)
        environment = work / "venv"
        import platform

        pin = (
            ".python-version-macos"
            if platform.system() == "Darwin"
            else ".python-version"
        )
        run(
            [
                "uv",
                "venv",
                "--python",
                (ROOT / pin).read_text().strip(),
                str(environment),
            ]
        )
        python = environment / "bin/python"
        run(
            [
                "uv",
                "pip",
                "install",
                "--python",
                str(python),
                "--require-hashes",
                "-r",
                str(requirements),
            ]
        )
        native_wheel = (
            ROOT
            / run(
                ["bazel", "cquery", f"//{NATIVE_PACKAGE}:wheel", "--output=files"],
                stdout=subprocess.PIPE,
                text=True,
            ).stdout.strip()
        )
        [project_wheel] = wheels.glob("*.whl")
        run(
            [
                "uv",
                "pip",
                "install",
                "--python",
                str(python),
                "--no-deps",
                str(project_wheel),
                str(native_wheel),
            ]
        )
        env = os.environ.copy()
        env.pop("PYTHONPATH", None)
        env["PYTHONNOUSERSITE"] = "1"
        run(
            [
                str(python),
                "-c",
                """
import sys
from pathlib import Path
import numpy as np
import robo_arch
from robo_arch_native._joint_pd import compute as bound_compute
from robo_arch.core.controllers.joint_pd._docstrings import JOINT_PD
from robo_arch.core.controllers.joint_pd.native import compute
assert Path(robo_arch.__file__).is_relative_to(sys.prefix)
assert JOINT_PD in bound_compute.__doc__
assert compute.__doc__.count(JOINT_PD) == 1
np.testing.assert_array_equal(compute(*(np.ones(2) for _ in range(7))), np.ones(2))
""",
            ],
            cwd=work,
            env=env,
        )
        run(
            [
                str(python),
                "-m",
                "sphinx",
                "-E",
                "-a",
                "-W",
                "--keep-going",
                "-b",
                "html",
                str(ROOT / "docs/api"),
                str(output),
            ],
            cwd=work,
            env=env,
        )
    print(f"API reference: {output / 'index.html'}")


if __name__ == "__main__":
    main()
