"""Project an ASCII PCD point cloud onto a ROS-compatible 2D occupancy map."""

import argparse
import math
from pathlib import Path


def _read_ascii_pcd(path: Path):
    lines = path.read_text(encoding="utf-8").splitlines()
    data_index = next((i for i, line in enumerate(lines)
                       if line.strip().upper() == "DATA ASCII"), None)
    if data_index is None:
        raise ValueError("only DATA ascii PCD files are supported")
    fields = next((line.split()[1:] for line in lines[:data_index]
                   if line.startswith("FIELDS ")), None)
    if not fields or not all(name in fields for name in ("x", "y", "z")):
        raise ValueError("PCD must contain x, y and z fields")
    x_index = fields.index("x")
    y_index = fields.index("y")
    z_index = fields.index("z")
    points = []
    for line in lines[data_index + 1:]:
        values = line.split()
        if len(values) <= max(x_index, y_index, z_index):
            continue
        try:
            x = float(values[x_index])
            y = float(values[y_index])
            z = float(values[z_index])
        except ValueError:
            continue
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((x, y, z))
    return points


def _write_map(points, pgm_path: Path, yaml_path: Path, resolution: float,
               min_x: float, max_x: float, min_y: float, max_y: float,
               min_z: float, max_z: float):
    width = max(1, math.ceil((max_x - min_x) / resolution))
    height = max(1, math.ceil((max_y - min_y) / resolution))
    pixels = bytearray([255] * (width * height))
    for x, y, z in points:
        if not min_z <= z <= max_z:
            continue
        col = int((x - min_x) / resolution)
        row = int((y - min_y) / resolution)
        if 0 <= col < width and 0 <= row < height:
            pixels[(height - 1 - row) * width + col] = 0
    pgm_path.write_bytes(f"P5\n{width} {height}\n255\n".encode() + pixels)
    yaml_path.write_text(
        "image: %s\nresolution: %.6f\norigin: [%.6f, %.6f, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
        % (pgm_path.name, resolution, min_x, min_y),
        encoding="utf-8",
    )


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd")
    parser.add_argument("--output-dir", default=".")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--min-x", type=float, default=-10.0)
    parser.add_argument("--max-x", type=float, default=10.0)
    parser.add_argument("--min-y", type=float, default=-10.0)
    parser.add_argument("--max-y", type=float, default=10.0)
    parser.add_argument("--min-z", type=float, default=0.08)
    parser.add_argument("--max-z", type=float, default=0.60)
    args = parser.parse_args(argv)
    if (args.resolution <= 0 or args.max_x <= args.min_x or
            args.max_y <= args.min_y or args.max_z <= args.min_z):
        parser.error("invalid resolution or map bounds")
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(args.pcd).stem
    _write_map(_read_ascii_pcd(Path(args.pcd)), output_dir / f"{stem}.pgm",
               output_dir / f"{stem}.yaml", args.resolution, args.min_x,
               args.max_x, args.min_y, args.max_y, args.min_z, args.max_z)


if __name__ == "__main__":
    main()
