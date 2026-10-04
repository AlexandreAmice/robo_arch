"""Check the combined reference and errors using the real Sphinx reader."""

import os
import subprocess
import sys
from pathlib import Path

import pytest

from robo_arch.core.controllers.joint_pd.native import compute

ROOT = Path(__file__).absolute().parents[2]


def build_reference(tmp_path: Path, content: str) -> subprocess.CompletedProcess[str]:
    source = tmp_path / "source"
    source.mkdir()
    (source / "index.rst").write_text(content)
    (source / "conf.py").write_text((ROOT / "docs/api/conf.py").read_text())
    return subprocess.run(
        [
            sys.executable,
            "-m",
            "sphinx",
            "-W",
            "-b",
            "html",
            str(source),
            str(tmp_path / "html"),
        ],
        env=os.environ.copy(),
        text=True,
        capture_output=True,
    )


def test_combined_reference(tmp_path):
    result = build_reference(tmp_path, (ROOT / "docs/api/index.rst").read_text())
    assert result.returncode == 0, result.stdout + result.stderr
    html = (tmp_path / "html/index.html").read_text()
    assert "Compute stateless joint PD feedback plus feedforward effort." in html
    assert "Python array interface" in html
    assert "Resolve a validated application-package URI" in html
    assert "robo_arch_native" not in html


@pytest.mark.parametrize(
    "entry,diagnostic",
    [
        (
            ".. autofunction:: robo_arch.core.controllers.joint_pd.native.compute\n"
            * 2,
            "Duplicate public API entry",
        ),
        (
            ".. autofunction:: robo_arch.core.controllers.joint_pd.native.missing\n",
            "failed to import",
        ),
    ],
)
def test_bad_reference_fails(tmp_path, entry, diagnostic):
    result = build_reference(tmp_path, "API\n===\n\n" + entry)
    assert result.returncode != 0
    assert diagnostic in result.stdout + result.stderr


def test_facade_help_without_native_or_sdk():
    script = """
import sys
class BlockOptional:
    def find_spec(self, fullname, *args):
        if fullname.split('.')[0] in {'robo_arch_native', 'pydrake', 'isaacsim'}:
            raise AssertionError(f'Unexpected optional import: {fullname}')
sys.meta_path.insert(0, BlockOptional())
from robo_arch.core.controllers.joint_pd.native import compute
assert 'Compute stateless joint PD feedback' in compute.__doc__
assert 'Python array interface' in compute.__doc__
"""
    subprocess.run([sys.executable, "-c", script], check=True)
    assert compute.__doc__.count("Python array interface") == 1


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
