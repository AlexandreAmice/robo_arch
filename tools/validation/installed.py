"""Verify installed resources and native imports without any checkout search path."""

import importlib
import importlib.abc
import json
import sys
import xml.etree.ElementTree as ET
from importlib.resources import files
from pathlib import Path

import yaml


def main() -> None:
    class RejectSdk(importlib.abc.MetaPathFinder):
        def find_spec(self, fullname, path=None, target=None):
            if fullname.split(".")[0] in {"pydrake", "omni", "isaacsim", "rclpy"}:
                raise RuntimeError(
                    f"Resource inspection imported optional SDK: {fullname}"
                )

    sys.meta_path.insert(0, RejectSdk())
    import robo_arch_native

    import robo_arch

    package = Path(str(files("robo_arch"))).resolve()
    assert Path(robo_arch.__file__).is_relative_to(sys.prefix)
    assert Path(robo_arch_native.__file__).is_relative_to(sys.prefix)
    from robo_arch.core.config.loading import load_run, resolve_resource

    def references(value):
        if isinstance(value, dict):
            for child in value.values():
                references(child)
        elif isinstance(value, list):
            for child in value:
                references(child)
        elif isinstance(value, str) and value.startswith("package://"):
            if not resolve_resource(value).is_file():
                raise FileNotFoundError(value)

    declarations = list(package.rglob("*.yaml"))
    for path in declarations:
        references(yaml.safe_load(path.read_text()))
        if path.name == "scenario.yaml":
            load_run("package://robo_arch/" + path.relative_to(package).as_posix())
    meshes = 0
    for model in package.rglob("*.urdf"):
        for mesh in ET.parse(model).iter("mesh"):
            name = mesh.attrib["filename"]
            path = (
                resolve_resource(name)
                if name.startswith("package://")
                else (model.parent / name).resolve()
            )
            if not path.is_file() or not path.is_relative_to(package):
                raise FileNotFoundError(f"Unpackaged mesh {model}: {name}")
            meshes += 1
    inventory = json.loads(files("robo_arch_native").joinpath("build.json").read_text())
    for module in inventory["extensions"]:
        importlib.import_module(module)
    print(
        json.dumps(
            {
                "installed_yaml": len(declarations),
                "mesh_references": meshes,
                "native_modules": inventory["extensions"],
                "prefix": sys.prefix,
            }
        )
    )


if __name__ == "__main__":
    main()
