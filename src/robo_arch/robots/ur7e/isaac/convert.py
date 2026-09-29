"""Isolate URDF conversion from the running Kit process's USD libraries."""

import sys
from importlib.resources import as_file, files


def main() -> None:
    from urdf_usd_converter import Converter

    resource = files("robo_arch.robots.ur7e").joinpath("assets/model.urdf")
    with as_file(resource) as path:
        Converter(layer_structure=False, scene=False).convert(str(path), sys.argv[1])


if __name__ == "__main__":
    main()
