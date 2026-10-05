"""Install the captured ROS/Ubuntu package closure; never resolve new versions."""

import hashlib
import json
import subprocess
import sys
import tempfile
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


def main() -> None:
    lock = json.loads(Path(sys.argv[1]).read_text())
    with tempfile.TemporaryDirectory(prefix="ros-debs-") as temporary:
        directory = Path(temporary)

        def fetch(item: dict) -> Path:
            path = directory / (item["name"] + ".deb")
            with urllib.request.urlopen(item["url"], timeout=120) as response:
                path.write_bytes(response.read())
            if hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
                raise ValueError(f"Invalid package hash: {item['name']}")
            return path

        with ThreadPoolExecutor(max_workers=8) as pool:
            files = list(pool.map(fetch, lock["packages"]))
        subprocess.run(["dpkg", "--unpack", *map(str, files)], check=True)
        subprocess.run(["dpkg", "--configure", "-a"], check=True)
        for item in lock["packages"]:
            version = subprocess.check_output(
                ["dpkg-query", "-W", "-f=${Version}", item["name"]], text=True
            )
            if version != item["version"]:
                raise ValueError(f"Installed package differs from lock: {item['name']}")


if __name__ == "__main__":
    main()
