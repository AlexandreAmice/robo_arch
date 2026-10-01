"""Run USD conversion separately from Kit's incompatible USD libraries."""

import sys


def main() -> None:
    from urdf_usd_converter import Converter

    Converter(layer_structure=False, scene=False).convert(sys.argv[1], sys.argv[2])


if __name__ == "__main__":
    main()
