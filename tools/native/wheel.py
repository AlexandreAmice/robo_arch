"""Package the Bazel-built CPython 3.12 Linux extension as a local wheel."""

import argparse
import base64
import csv
import hashlib
import io
import json
import zipfile
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("extension", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    data = args.extension.read_bytes()
    info = "robo_arch_native-0.1.0.dist-info"
    entries = {
        "robo_arch_native/__init__.py": b'"""Bazel-built native algorithms."""\n',
        "robo_arch_native/_joint_pd.so": data,
        "robo_arch_native/build.json": json.dumps(
            {"extension_sha256": hashlib.sha256(data).hexdigest(), "python": "3.12"}
        ).encode(),
        f"{info}/METADATA": (
            b"Metadata-Version: 2.1\nName: robo-arch-native\nVersion: 0.1.0\n"
            b"Requires-Python: ==3.12.*\n"
        ),
        f"{info}/WHEEL": (
            b"Wheel-Version: 1.0\nGenerator: robo-arch Bazel\n"
            b"Root-Is-Purelib: false\nTag: cp312-cp312-linux_x86_64\n"
        ),
    }
    record = io.StringIO()
    writer = csv.writer(record, lineterminator="\n")
    for name, content in entries.items():
        digest = base64.urlsafe_b64encode(hashlib.sha256(content).digest()).rstrip(b"=")
        writer.writerow([name, "sha256=" + digest.decode(), len(content)])
    writer.writerow([f"{info}/RECORD", "", ""])
    entries[f"{info}/RECORD"] = record.getvalue().encode()
    with zipfile.ZipFile(args.output, "w", zipfile.ZIP_DEFLATED) as wheel:
        for name, content in entries.items():
            entry = zipfile.ZipInfo(name, date_time=(2020, 1, 1, 0, 0, 0))
            entry.compress_type = zipfile.ZIP_DEFLATED
            wheel.writestr(entry, content)


if __name__ == "__main__":
    main()
