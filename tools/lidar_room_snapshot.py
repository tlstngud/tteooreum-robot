#!/usr/bin/env python3

import argparse
import json
import math
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Iterable


REMOTE_CAPTURE_SCRIPT = r"""
set -eo pipefail

source /opt/ros/foxy/setup.bash >/dev/null 2>&1
source ~/ros_ws/install/setup.bash >/dev/null 2>&1

log_file="/tmp/sllidar_capture_$$.log"
timeout 30s ros2 run sllidar_ros2 sllidar_node --ros-args \
  -p channel_type:=serial \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baudrate:=256000 \
  -p frame_id:=laser \
  -p inverted:=false \
  -p angle_compensate:=true \
  -p scan_mode:=Sensitivity >"$log_file" 2>&1 &
node_pid=$!

cleanup() {
  kill "$node_pid" >/dev/null 2>&1 || true
  wait "$node_pid" >/dev/null 2>&1 || true
  rm -f "$log_file"
}
trap cleanup EXIT

sleep 3

python3 - <<'PY'
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanProbe(Node):
    def __init__(self):
        super().__init__('scan_probe')
        self.msg = None
        self.sub = self.create_subscription(LaserScan, '/scan', self._cb, 10)

    def _cb(self, msg):
        self.msg = msg


rclpy.init()
node = ScanProbe()
deadline = time.time() + 30.0
while time.time() < deadline and node.msg is None:
    rclpy.spin_once(node, timeout_sec=0.5)

if node.msg is None:
    print(json.dumps({"ok": False, "error": "no_scan"}))
else:
    msg = node.msg
    payload = {
        "ok": True,
        "angle_min": msg.angle_min,
        "angle_max": msg.angle_max,
        "angle_increment": msg.angle_increment,
        "range_min": msg.range_min,
        "range_max": msg.range_max,
        "ranges": [None if (not math.isfinite(v) or v <= 0.0) else v for v in msg.ranges],
        "intensities": list(msg.intensities[: len(msg.ranges)]),
    }
    print(json.dumps(payload))

node.destroy_node()
rclpy.shutdown()
PY
"""


def run(cmd: list[str], *, text: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, text=text, capture_output=True, check=False)


def ssh_capture(target: str, password: str) -> dict:
    cmd = [
        "sshpass",
        "-p",
        password,
        "ssh",
        "-o",
        "StrictHostKeyChecking=no",
        "-o",
        "UserKnownHostsFile=/dev/null",
        "-o",
        "ConnectTimeout=8",
        target,
        "bash",
        "-s",
    ]
    result = subprocess.run(
        cmd,
        input=REMOTE_CAPTURE_SCRIPT,
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip() or "SSH command failed")

    lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError("Remote capture produced no output")

    payload = json.loads(lines[-1])
    if not payload.get("ok"):
        raise RuntimeError(f"Remote LiDAR capture failed: {payload}")
    return payload


def polar_points(scan: dict) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    angle = scan["angle_min"]
    for value in scan["ranges"]:
        if value is not None:
            x = value * math.cos(angle)
            y = value * math.sin(angle)
            points.append((x, y))
        angle += scan["angle_increment"]
    return points


def chunk_segments(scan: dict, points: list[tuple[float, float]]) -> list[list[tuple[float, float]]]:
    segments: list[list[tuple[float, float]]] = []
    current: list[tuple[float, float]] = []
    point_iter = iter(points)
    prev_range = None
    prev_point = None
    for value in scan["ranges"]:
        if value is None:
            if len(current) > 1:
                segments.append(current)
            current = []
            prev_range = None
            prev_point = None
            continue

        point = next(point_iter)
        contiguous = True
        if prev_range is not None and prev_point is not None:
            if abs(value - prev_range) > 0.6:
                contiguous = False
            elif math.dist(point, prev_point) > 0.45:
                contiguous = False
        if not contiguous:
            if len(current) > 1:
                segments.append(current)
            current = [point]
        else:
            current.append(point)
        prev_range = value
        prev_point = point

    if len(current) > 1:
        segments.append(current)
    return segments


def sector_min(scan: dict, center_deg: float, half_width_deg: float = 20.0) -> float | None:
    center = math.radians(center_deg)
    half = math.radians(half_width_deg)
    best = None
    angle = scan["angle_min"]
    for value in scan["ranges"]:
        if value is not None:
            delta = math.atan2(math.sin(angle - center), math.cos(angle - center))
            if abs(delta) <= half and (best is None or value < best):
                best = value
        angle += scan["angle_increment"]
    return best


def summarize(scan: dict, points: list[tuple[float, float]]) -> dict:
    finite = [v for v in scan["ranges"] if v is not None]
    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    return {
        "sample_count": len(scan["ranges"]),
        "finite_count": len(finite),
        "min_range_m": round(min(finite), 3) if finite else None,
        "max_range_m": round(max(finite), 3) if finite else None,
        "visible_width_m": round(max(xs) - min(xs), 3) if xs else None,
        "visible_height_m": round(max(ys) - min(ys), 3) if ys else None,
        "front_min_m": round(sector_min(scan, 0.0) or 0.0, 3) if sector_min(scan, 0.0) else None,
        "left_min_m": round(sector_min(scan, 90.0) or 0.0, 3) if sector_min(scan, 90.0) else None,
        "rear_min_m": round(sector_min(scan, 180.0) or 0.0, 3) if sector_min(scan, 180.0) else None,
        "right_min_m": round(sector_min(scan, -90.0) or 0.0, 3) if sector_min(scan, -90.0) else None,
    }


def svg_map(scan: dict, points: list[tuple[float, float]], summary: dict) -> str:
    size = 1000
    cx = size / 2
    cy = size / 2
    margin = 70
    finite = [v for v in scan["ranges"] if v is not None]
    radius_m = min(max(finite) if finite else 6.0, 8.0)
    radius_m = max(radius_m, 1.0)
    scale = (size / 2 - margin) / radius_m

    def sx(x: float) -> float:
        return cx + x * scale

    def sy(y: float) -> float:
        return cy - y * scale

    rings = []
    ring_count = int(math.ceil(radius_m))
    for meter in range(1, ring_count + 1):
        r = meter * scale
        rings.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="none" stroke="#d8dee6" stroke-width="1"/>')
        rings.append(
            f'<text x="{cx + 8:.1f}" y="{cy - r + 16:.1f}" font-size="14" fill="#7a8694">{meter}m</text>'
        )

    segments_svg = []
    for segment in chunk_segments(scan, points):
        coords = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in segment)
        segments_svg.append(
            f'<polyline points="{coords}" fill="none" stroke="#1f5eff" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" opacity="0.75"/>'
        )

    dots_svg = []
    for x, y in points:
        dots_svg.append(f'<circle cx="{sx(x):.2f}" cy="{sy(y):.2f}" r="1.8" fill="#0f172a" opacity="0.85"/>')

    summary_lines = [
        f"표본 {summary['sample_count']}개 / 유효 {summary['finite_count']}개",
        f"최근접 {summary['min_range_m']}m / 최원 {summary['max_range_m']}m",
        f"전방 {summary['front_min_m']}m, 좌측 {summary['left_min_m']}m, 후방 {summary['rear_min_m']}m, 우측 {summary['right_min_m']}m",
        f"가시 폭 {summary['visible_width_m']}m / 가시 높이 {summary['visible_height_m']}m",
    ]

    legend_y = 42
    legend = []
    for line in summary_lines:
        legend.append(f'<text x="28" y="{legend_y}" font-size="22" fill="#0f172a">{line}</text>')
        legend_y += 28

    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{size}" height="{size}" viewBox="0 0 {size} {size}">
  <rect width="100%" height="100%" fill="#fbfcfe"/>
  <text x="28" y="24" font-size="18" fill="#5f6b7a">LiDAR snapshot from current robot position</text>
  {''.join(legend)}
  {''.join(rings)}
  <line x1="{cx:.1f}" y1="{margin/2:.1f}" x2="{cx:.1f}" y2="{size - margin/2:.1f}" stroke="#e5e9f0" stroke-width="1.5"/>
  <line x1="{margin/2:.1f}" y1="{cy:.1f}" x2="{size - margin/2:.1f}" y2="{cy:.1f}" stroke="#e5e9f0" stroke-width="1.5"/>
  <text x="{cx + 10:.1f}" y="{margin/2 + 20:.1f}" font-size="18" fill="#e11d48">좌측</text>
  <text x="{size - margin/2 - 70:.1f}" y="{cy - 10:.1f}" font-size="18" fill="#0f766e">전방</text>
  <text x="{cx + 10:.1f}" y="{size - margin/2 - 10:.1f}" font-size="18" fill="#e11d48">우측</text>
  <text x="{margin/2 + 8:.1f}" y="{cy - 10:.1f}" font-size="18" fill="#0f766e">후방</text>
  {''.join(segments_svg)}
  {''.join(dots_svg)}
  <circle cx="{cx:.1f}" cy="{cy:.1f}" r="12" fill="#ef4444"/>
  <path d="M {cx + 18:.1f} {cy:.1f} L {cx + 52:.1f} {cy - 12:.1f} L {cx + 52:.1f} {cy + 12:.1f} Z" fill="#ef4444"/>
</svg>
"""


def write_text(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture one LiDAR room snapshot from a SerBot over SSH.")
    parser.add_argument("--ssh-target", default="soda@172.16.140.52")
    parser.add_argument("--ssh-password", default=os.environ.get("SERBOT_PASSWORD", "soda"))
    parser.add_argument(
        "--output-dir",
        default=str(Path.cwd() / "artifacts" / "lidar"),
        help="Directory to store the raw scan JSON, summary, and SVG render.",
    )
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    stem = out_dir / f"room-scan-{stamp}"

    scan = ssh_capture(args.ssh_target, args.ssh_password)
    points = polar_points(scan)
    summary = summarize(scan, points)

    json_path = stem.with_suffix(".json")
    svg_path = stem.with_suffix(".svg")
    txt_path = stem.with_suffix(".txt")

    write_text(json_path, json.dumps(scan, ensure_ascii=False, indent=2))
    write_text(svg_path, svg_map(scan, points, summary))
    write_text(txt_path, json.dumps(summary, ensure_ascii=False, indent=2) + "\n")

    print(f"json={json_path}")
    print(f"svg={svg_path}")
    print(f"summary={txt_path}")
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
