"""Render the actual ROS mock joint trace as a standalone SVG."""

import argparse
import html
import json
from pathlib import Path


def render(source: Path, output: Path) -> None:
    data = json.loads(source.read_text())
    observations = data["observations"]
    start = observations[0]["stamp"]
    times = [item["stamp"] - start for item in observations]
    values = [item["positions"][0] for item in observations]
    lower = min(*values, data["target"][0]) - 0.01
    upper = max(*values, data["target"][0]) + 0.01
    width, height = 720, 360

    def x(value):
        return 65 + 620 * value / max(times[-1], 0.001)

    def y(value):
        return 285 - 210 * (value - lower) / (upper - lower)

    points = " ".join(
        f"{x(t):.2f},{y(q):.2f}" for t, q in zip(times, values, strict=True)
    )
    target = y(data["target"][0])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="white"/>
<g font-family="sans-serif" fill="#172b4d">
<text x="65" y="28" font-size="18">UR7e: actual ROS mock observation</text>
<text x="65" y="50" font-size="12">{html.escape(data["identity"])} · ros2_control mock hardware · no physical motion</text>
<path d="M65 75 V285 H685" fill="none" stroke="#444"/>
<path d="M65 {target:.2f} H685" stroke="#b46516" stroke-dasharray="5 4"/>
<polyline points="{points}" fill="none" stroke="#1769aa" stroke-width="2"/>
<text x="65" y="310" font-size="12">0 s</text>
<text x="635" y="310" font-size="12">{times[-1]:.2f} s</text>
<text x="8" y="285" font-size="11">{lower:.3f}</text>
<text x="8" y="82" font-size="11">{upper:.3f}</text>
<text x="245" y="338" font-size="12">shoulder_pan_joint [rad] · dashed: goal position</text>
</g></svg>''')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument(
        "--output", type=Path, default=Path("recordings/ur_mock/trajectory.svg")
    )
    args = parser.parse_args()
    render(args.trace, args.output)
