"""Multiple native inputs and runtime resources form one reproducible wheel."""

import base64
import csv
import hashlib
import io
import json
import zipfile

import pytest

from tools.native.wheel import build_wheel


def test_multiple_extensions_resources_and_record(tmp_path):
    first, second, resource = (
        tmp_path / name for name in ("a.so", "b.so", "table.bin")
    )
    first.write_bytes(b"first extension")
    second.write_bytes(b"second extension")
    resource.write_bytes(b"runtime data")
    extensions = [
        ("robo_arch_native._first", first),
        ("robo_arch_native._second", second),
    ]
    resources = [("robo_arch_native/data/table.bin", resource)]
    wheel, repeated = tmp_path / "native.whl", tmp_path / "repeated.whl"
    build_wheel(wheel, extensions, resources)
    build_wheel(repeated, list(reversed(extensions)), resources)
    assert wheel.read_bytes() == repeated.read_bytes()
    with zipfile.ZipFile(wheel) as archive:
        assert archive.read("robo_arch_native/_first.so") == first.read_bytes()
        assert archive.read("robo_arch_native/_second.so") == second.read_bytes()
        assert archive.read(resources[0][0]) == resource.read_bytes()
        assert json.loads(archive.read("robo_arch_native/build.json"))[
            "extensions"
        ] == [module for module, _ in extensions]
        record = "robo_arch_native-0.1.0.dist-info/RECORD"
        rows = list(csv.reader(io.StringIO(archive.read(record).decode())))
        assert {row[0] for row in rows} == set(archive.namelist())
        for name, digest, size in rows:
            if name == record:
                assert (digest, size) == ("", "")
                continue
            data = archive.read(name)
            expected = base64.urlsafe_b64encode(hashlib.sha256(data).digest()).rstrip(
                b"="
            )
            assert digest == "sha256=" + expected.decode()
            assert int(size) == len(data)


@pytest.mark.parametrize(
    "destination",
    [
        "../outside",
        "/absolute",
        "robo_arch_native/../outside",
        "robo_arch_native/build.json",
    ],
)
def test_invalid_resource_destination(tmp_path, destination):
    resource = tmp_path / "data"
    resource.write_bytes(b"data")
    with pytest.raises(ValueError):
        build_wheel(tmp_path / "native.whl", [], [(destination, resource)])


def test_duplicate_extension_destination(tmp_path):
    source = tmp_path / "extension.so"
    source.write_bytes(b"native")
    extension = ("robo_arch_native._example", source)
    with pytest.raises(ValueError, match="Duplicate wheel destination"):
        build_wheel(tmp_path / "native.whl", [extension, extension], [])


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
