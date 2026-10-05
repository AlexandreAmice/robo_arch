"""Capture enabled Kit extension inputs from an already running pinned SDK.

Call ``capture(app, output)`` after the intended scene/extensions are loaded.
This diagnostic never launches Kit and does not turn downloaded extensions into
an offline distribution. Compare ``extensions`` between equivalent profiles;
host details are evidence, not universal compatibility requirements.
"""

import hashlib
import json
import platform
import subprocess
from pathlib import Path


def capture(app, output: str | Path) -> dict:
    manager = app.get_extension_manager()
    extensions = []
    for entry in manager.get_extensions():
        if not entry["enabled"]:
            continue
        identifier = entry["id"]
        directory = Path(manager.get_extension_path(identifier))
        config = directory / "config/extension.toml"
        extensions.append(
            {
                "id": identifier,
                "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest(),
            }
        )
    data = {
        "schema": 1,
        "kit": app.get_build_version(),
        "python": platform.python_version(),
        "host": {
            "system": platform.system(),
            "architecture": platform.machine(),
            "release": platform.release(),
        },
        "gpu": subprocess.check_output(
            [
                "nvidia-smi",
                "--query-gpu=name,driver_version,memory.total",
                "--format=csv,noheader",
            ],
            text=True,
        )
        .strip()
        .splitlines(),
        "extensions": sorted(extensions, key=lambda item: item["id"]),
    }
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(data, indent=2) + "\n")
    return data


def compare(expected: str | Path, actual: str | Path) -> None:
    """Reject drift in Kit version/enabled-extension configuration inputs."""
    left = json.loads(Path(expected).read_text())
    right = json.loads(Path(actual).read_text())
    for field in ("kit", "extensions"):
        if left[field] != right[field]:
            raise ValueError(f"Isaac runtime input drift: {field}")
