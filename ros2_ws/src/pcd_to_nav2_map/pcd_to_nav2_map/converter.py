#!/usr/bin/env python3
"""Convert an offline PCD/PLY point cloud into a Nav2 occupancy map."""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import warnings

import numpy as np

# Some Ubuntu Open3D builds import an optional SciPy module whose version check
# is unrelated to point-cloud I/O. Keep that warning out of normal CLI output.
warnings.filterwarnings(
    "ignore",
    message=r"A NumPy version .* is required for this version of SciPy.*",
    category=UserWarning,
)
try:
    import open3d as o3d
except ImportError as error:  # pragma: no cover - depends on host installation
    o3d = None
    OPEN3D_IMPORT_ERROR = error
else:
    OPEN3D_IMPORT_ERROR = None

from PIL import Image


UNKNOWN = np.uint8(205)
FREE = np.uint8(254)
OCCUPIED = np.uint8(0)


def positive_float(value):
    number = float(value)
    if number <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return number


def nonnegative_float(value):
    number = float(value)
    if number < 0.0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return number


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return number


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Convert a saved PCD/PLY map to Nav2 PGM/YAML files. "
            "The point cloud must already be level and expressed in the same "
            "map coordinates used by global localization."
        )
    )
    parser.add_argument("--input", required=True, help="Input .pcd or .ply file")
    parser.add_argument(
        "--output",
        help=(
            "Output prefix. By default, an input below FAST_LIO_ROS2 is saved "
            "as FAST_LIO_ROS2/map/<input_stem>"
        ),
    )
    parser.add_argument(
        "--resolution",
        type=positive_float,
        default=0.05,
        help="Map resolution in metres per pixel (default: 0.05)",
    )
    parser.add_argument(
        "--z-min",
        type=float,
        default=0.10,
        help="Minimum map-frame Z treated as an obstacle (default: 0.10)",
    )
    parser.add_argument(
        "--z-max",
        type=float,
        default=1.00,
        help="Maximum map-frame Z treated as an obstacle (default: 1.00)",
    )
    parser.add_argument(
        "--min-points-per-cell",
        type=positive_int,
        default=2,
        help="Minimum obstacle-slice points required to occupy a cell (default: 2)",
    )
    parser.add_argument(
        "--padding",
        type=nonnegative_float,
        default=0.50,
        help="Extra map border in metres (default: 0.50)",
    )
    parser.add_argument("--min-x", type=float, help="Optional map crop minimum X")
    parser.add_argument("--max-x", type=float, help="Optional map crop maximum X")
    parser.add_argument("--min-y", type=float, help="Optional map crop minimum Y")
    parser.add_argument("--max-y", type=float, help="Optional map crop maximum Y")
    parser.add_argument(
        "--free-space-mode",
        choices=("flood_fill", "observed", "bbox", "unknown"),
        default="flood_fill",
        help=(
            "Free-space inference: flood_fill from seed(s), observed low points, "
            "the complete bounding box, or no inferred free space "
            "(default: flood_fill)"
        ),
    )
    parser.add_argument(
        "--seed",
        nargs=2,
        type=float,
        action="append",
        metavar=("X", "Y"),
        help="Flood-fill seed in map coordinates; repeat for multiple regions",
    )
    parser.add_argument(
        "--flood-barrier-radius",
        type=nonnegative_float,
        default=0.25,
        help=(
            "Obstacle dilation in metres used only while flood filling; it "
            "closes small gaps in incomplete walls (default: 0.25)"
        ),
    )
    parser.add_argument(
        "--free-z-min",
        type=float,
        help="Observed-mode minimum Z; defaults to the cloud minimum",
    )
    parser.add_argument(
        "--free-z-max",
        type=float,
        help="Observed-mode maximum Z; defaults to --z-min",
    )
    parser.add_argument(
        "--min-free-points-per-cell",
        type=positive_int,
        default=1,
        help="Observed-mode minimum low points for a free cell (default: 1)",
    )
    parser.add_argument(
        "--radius-outlier-radius",
        type=nonnegative_float,
        default=0.0,
        help="Optional 3D radius outlier filter radius; 0 disables it",
    )
    parser.add_argument(
        "--radius-outlier-min-neighbors",
        type=positive_int,
        default=2,
        help="Minimum neighbours for radius filtering (default: 2)",
    )
    parser.add_argument(
        "--occupied-thresh",
        type=float,
        default=0.65,
        help="Nav2 YAML occupied threshold (default: 0.65)",
    )
    parser.add_argument(
        "--free-thresh",
        type=float,
        default=0.25,
        help="Nav2 YAML free threshold (default: 0.25)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite output files if they already exist",
    )
    return parser


def normalize_output_prefix(value):
    path = Path(value).expanduser().resolve()
    if path.suffix.lower() in (".pgm", ".yaml", ".yml", ".png", ".json"):
        path = path.with_suffix("")
    return path


def default_output_prefix(input_path):
    for parent in input_path.parents:
        if parent.name == "FAST_LIO_ROS2":
            return parent / "map" / input_path.stem
    return input_path.parent / "map" / input_path.stem


def validate_arguments(args, parser):
    input_path = Path(args.input).expanduser().resolve()
    if not input_path.is_file():
        parser.error("input file does not exist: {}".format(input_path))
    if input_path.suffix.lower() not in (".pcd", ".ply"):
        parser.error("input must be a .pcd or .ply file")
    if args.z_min >= args.z_max:
        parser.error("--z-min must be less than --z-max")
    if args.min_x is not None and args.max_x is not None and args.min_x >= args.max_x:
        parser.error("--min-x must be less than --max-x")
    if args.min_y is not None and args.max_y is not None and args.min_y >= args.max_y:
        parser.error("--min-y must be less than --max-y")
    if not 0.0 <= args.free_thresh <= 1.0:
        parser.error("--free-thresh must be in [0, 1]")
    if not 0.0 <= args.occupied_thresh <= 1.0:
        parser.error("--occupied-thresh must be in [0, 1]")
    if args.free_thresh >= args.occupied_thresh:
        parser.error("--free-thresh must be less than --occupied-thresh")
    if (
        args.free_z_min is not None
        and args.free_z_max is not None
        and args.free_z_min >= args.free_z_max
    ):
        parser.error("--free-z-min must be less than --free-z-max")
    output_prefix = (
        normalize_output_prefix(args.output)
        if args.output
        else default_output_prefix(input_path)
    )
    return input_path, output_prefix


def load_point_cloud(path, radius, minimum_neighbors):
    if o3d is None:
        raise RuntimeError("Open3D is not installed: {}".format(OPEN3D_IMPORT_ERROR))
    cloud = o3d.io.read_point_cloud(str(path))
    if cloud.is_empty():
        raise RuntimeError("Open3D could not read any points from {}".format(path))
    points = np.asarray(cloud.points, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        raise RuntimeError("the point cloud contains no finite XYZ points")

    original_count = len(points)
    if radius > 0.0:
        filtered = o3d.geometry.PointCloud()
        filtered.points = o3d.utility.Vector3dVector(points)
        _, indexes = filtered.remove_radius_outlier(
            nb_points=minimum_neighbors,
            radius=radius,
        )
        points = points[np.asarray(indexes, dtype=np.int64)]
        if not len(points):
            raise RuntimeError("radius filtering removed every point")
    return points, original_count


def crop_points(points, args):
    keep = np.ones(len(points), dtype=bool)
    if args.min_x is not None:
        keep &= points[:, 0] >= args.min_x
    if args.max_x is not None:
        keep &= points[:, 0] <= args.max_x
    if args.min_y is not None:
        keep &= points[:, 1] >= args.min_y
    if args.max_y is not None:
        keep &= points[:, 1] <= args.max_y
    cropped = points[keep]
    if not len(cropped):
        raise RuntimeError("crop bounds removed every point")
    return cropped


def grid_geometry(points, resolution, padding):
    origin_x = float(points[:, 0].min() - padding)
    origin_y = float(points[:, 1].min() - padding)
    maximum_x = float(points[:, 0].max() + padding)
    maximum_y = float(points[:, 1].max() + padding)
    width = int(math.floor((maximum_x - origin_x) / resolution)) + 1
    height = int(math.floor((maximum_y - origin_y) / resolution)) + 1
    if width <= 0 or height <= 0:
        raise RuntimeError("invalid map dimensions")
    if width * height > 500_000_000:
        raise RuntimeError(
            "map would contain {:,} cells; increase --resolution or crop the cloud".format(
                width * height
            )
        )
    return origin_x, origin_y, width, height


def point_counts(points, origin_x, origin_y, resolution, width, height):
    if not len(points):
        return np.zeros((height, width), dtype=np.int32)
    x_index = np.floor((points[:, 0] - origin_x) / resolution).astype(np.int64)
    y_index = np.floor((points[:, 1] - origin_y) / resolution).astype(np.int64)
    valid = (
        (x_index >= 0)
        & (x_index < width)
        & (y_index >= 0)
        & (y_index < height)
    )
    flat = y_index[valid] * width + x_index[valid]
    counts = np.bincount(flat, minlength=width * height)
    return counts.reshape((height, width)).astype(np.int32, copy=False)


def dilate(mask, radius):
    if radius <= 0:
        return mask.copy()
    height, width = mask.shape
    padded = np.pad(mask, radius, mode="constant", constant_values=False)
    result = np.zeros_like(mask)
    for y_offset in range(-radius, radius + 1):
        for x_offset in range(-radius, radius + 1):
            if x_offset * x_offset + y_offset * y_offset > radius * radius:
                continue
            y_start = radius + y_offset
            x_start = radius + x_offset
            result |= padded[y_start:y_start + height, x_start:x_start + width]
    return result


def nearest_open_cell(candidate, requested_x, requested_y):
    height, width = candidate.shape
    requested_x = min(max(requested_x, 0), width - 1)
    requested_y = min(max(requested_y, 0), height - 1)
    if candidate[requested_y, requested_x]:
        return requested_x, requested_y, False
    cells = np.argwhere(candidate)
    if not len(cells):
        raise RuntimeError("there are no non-obstacle cells available for flood filling")
    distance = (
        (cells[:, 1] - requested_x) * (cells[:, 1] - requested_x)
        + (cells[:, 0] - requested_y) * (cells[:, 0] - requested_y)
    )
    nearest = cells[int(np.argmin(distance))]
    return int(nearest[1]), int(nearest[0]), True


def restore_free_margin(free_core, occupied, radius):
    """Restore the inside margin hidden by the temporary flood barrier.

    Expansion is geodesic: it cannot jump across a genuinely occupied cell.
    The number of iterations is bounded by the same radius that closed wall
    gaps, so it also cannot flood an entire exterior region through such a gap.
    """
    if radius <= 0:
        return free_core
    restored = free_core.copy()
    frontier = free_core.copy()
    for _ in range(radius):
        neighbours = np.zeros_like(frontier)
        neighbours[1:, :] |= frontier[:-1, :]
        neighbours[:-1, :] |= frontier[1:, :]
        neighbours[:, 1:] |= frontier[:, :-1]
        neighbours[:, :-1] |= frontier[:, 1:]
        frontier = neighbours & ~occupied & ~restored
        if not frontier.any():
            break
        restored |= frontier
    return restored


def flood_fill_free(occupied, seeds, origin_x, origin_y, resolution, barrier_radius):
    barrier = dilate(occupied, barrier_radius)
    candidate = ~barrier
    free = np.zeros_like(occupied)
    queue = deque()
    adjusted_seeds = []

    for seed_x, seed_y in seeds:
        grid_x = int(math.floor((seed_x - origin_x) / resolution))
        grid_y = int(math.floor((seed_y - origin_y) / resolution))
        if grid_x < 0 or grid_x >= occupied.shape[1] or grid_y < 0 or grid_y >= occupied.shape[0]:
            raise RuntimeError(
                "flood-fill seed ({:.3f}, {:.3f}) is outside the output map".format(
                    seed_x, seed_y
                )
            )
        grid_x, grid_y, adjusted = nearest_open_cell(candidate, grid_x, grid_y)
        adjusted_seeds.append(
            {
                "requested": [float(seed_x), float(seed_y)],
                "used": [
                    origin_x + (grid_x + 0.5) * resolution,
                    origin_y + (grid_y + 0.5) * resolution,
                ],
                "adjusted": adjusted,
            }
        )
        if not free[grid_y, grid_x]:
            free[grid_y, grid_x] = True
            queue.append((grid_x, grid_y))

    height, width = occupied.shape
    while queue:
        x_index, y_index = queue.popleft()
        for next_x, next_y in (
            (x_index - 1, y_index),
            (x_index + 1, y_index),
            (x_index, y_index - 1),
            (x_index, y_index + 1),
        ):
            if (
                0 <= next_x < width
                and 0 <= next_y < height
                and candidate[next_y, next_x]
                and not free[next_y, next_x]
            ):
                free[next_y, next_x] = True
                queue.append((next_x, next_y))

    # The dilated obstacle mask above is only a topological aid for closing
    # incomplete walls. Restore reachable non-obstacle cells on the inside so
    # it does not appear as an artificial unknown ring around every obstacle.
    free = restore_free_margin(free, occupied, barrier_radius)

    touches_boundary = bool(
        free[0, :].any()
        or free[-1, :].any()
        or free[:, 0].any()
        or free[:, -1].any()
    )
    return free, adjusted_seeds, touches_boundary


def write_pgm(path, occupancy_image):
    height, width = occupancy_image.shape
    with path.open("wb") as output:
        output.write("P5\n{} {}\n255\n".format(width, height).encode("ascii"))
        output.write(np.flipud(occupancy_image).tobytes(order="C"))


def write_yaml(path, pgm_path, resolution, origin_x, origin_y, args):
    content = (
        "image: {}\n"
        "mode: trinary\n"
        "resolution: {:.9g}\n"
        "origin: [{:.9g}, {:.9g}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: {:.9g}\n"
        "free_thresh: {:.9g}\n"
    ).format(
        pgm_path.name,
        resolution,
        origin_x,
        origin_y,
        args.occupied_thresh,
        args.free_thresh,
    )
    path.write_text(content, encoding="utf-8")


def write_preview(path, occupancy_image):
    preview = np.empty(occupancy_image.shape + (3,), dtype=np.uint8)
    preview[occupancy_image == UNKNOWN] = (128, 128, 128)
    preview[occupancy_image == FREE] = (255, 255, 255)
    preview[occupancy_image == OCCUPIED] = (0, 0, 0)
    Image.fromarray(np.flipud(preview), mode="RGB").save(path)


def convert(args, input_path, output_prefix):
    output_paths = {
        "pgm": output_prefix.with_suffix(".pgm"),
        "yaml": output_prefix.with_suffix(".yaml"),
        "preview": output_prefix.with_name(output_prefix.name + "_preview.png"),
        "stats": output_prefix.with_name(output_prefix.name + "_stats.json"),
    }
    existing = [str(path) for path in output_paths.values() if path.exists()]
    if existing and not args.force:
        raise RuntimeError(
            "output file(s) already exist; use --force to overwrite: {}".format(
                ", ".join(existing)
            )
        )
    output_prefix.parent.mkdir(parents=True, exist_ok=True)

    points, original_count = load_point_cloud(
        input_path,
        args.radius_outlier_radius,
        args.radius_outlier_min_neighbors,
    )
    filtered_count = len(points)
    points = crop_points(points, args)
    cropped_count = len(points)

    origin_x, origin_y, width, height = grid_geometry(
        points,
        args.resolution,
        args.padding,
    )
    obstacle_points = points[
        (points[:, 2] >= args.z_min) & (points[:, 2] <= args.z_max)
    ]
    obstacle_counts = point_counts(
        obstacle_points,
        origin_x,
        origin_y,
        args.resolution,
        width,
        height,
    )
    occupied = obstacle_counts >= args.min_points_per_cell

    seeds = args.seed if args.seed else [(0.0, 0.0)]
    adjusted_seeds = []
    flood_touches_boundary = False
    if args.free_space_mode == "flood_fill":
        barrier_cells = int(math.ceil(args.flood_barrier_radius / args.resolution))
        free, adjusted_seeds, flood_touches_boundary = flood_fill_free(
            occupied,
            seeds,
            origin_x,
            origin_y,
            args.resolution,
            barrier_cells,
        )
    elif args.free_space_mode == "bbox":
        free = ~occupied
    elif args.free_space_mode == "observed":
        free_z_min = (
            args.free_z_min
            if args.free_z_min is not None
            else float(points[:, 2].min()) - np.finfo(np.float64).eps
        )
        free_z_max = args.free_z_max if args.free_z_max is not None else args.z_min
        if free_z_min >= free_z_max:
            raise RuntimeError("observed free-space Z range is empty")
        free_points = points[
            (points[:, 2] >= free_z_min) & (points[:, 2] < free_z_max)
        ]
        free_counts = point_counts(
            free_points,
            origin_x,
            origin_y,
            args.resolution,
            width,
            height,
        )
        free = free_counts >= args.min_free_points_per_cell
        free &= ~occupied
    else:
        free = np.zeros_like(occupied)

    occupancy_image = np.full((height, width), UNKNOWN, dtype=np.uint8)
    occupancy_image[free] = FREE
    occupancy_image[occupied] = OCCUPIED

    write_pgm(output_paths["pgm"], occupancy_image)
    write_yaml(
        output_paths["yaml"],
        output_paths["pgm"],
        args.resolution,
        origin_x,
        origin_y,
        args,
    )
    write_preview(output_paths["preview"], occupancy_image)

    total_cells = width * height
    occupied_cells = int(occupied.sum())
    free_cells = int(free.sum())
    unknown_cells = int(total_cells - occupied_cells - free_cells)
    stats = {
        "input": str(input_path),
        "outputs": {name: str(path) for name, path in output_paths.items()},
        "points": {
            "input": original_count,
            "after_radius_filter": filtered_count,
            "after_crop": cropped_count,
            "obstacle_slice": int(len(obstacle_points)),
        },
        "map": {
            "resolution": args.resolution,
            "origin": [origin_x, origin_y, 0.0],
            "width": width,
            "height": height,
            "occupied_cells": occupied_cells,
            "free_cells": free_cells,
            "unknown_cells": unknown_cells,
            "occupied_percent": 100.0 * occupied_cells / total_cells,
            "free_percent": 100.0 * free_cells / total_cells,
            "unknown_percent": 100.0 * unknown_cells / total_cells,
        },
        "parameters": {
            "z_min": args.z_min,
            "z_max": args.z_max,
            "min_points_per_cell": args.min_points_per_cell,
            "padding": args.padding,
            "crop": {
                "min_x": args.min_x,
                "max_x": args.max_x,
                "min_y": args.min_y,
                "max_y": args.max_y,
            },
            "free_space_mode": args.free_space_mode,
            "flood_barrier_radius_metres": args.flood_barrier_radius,
            "flood_barrier_radius_cells": (
                int(math.ceil(args.flood_barrier_radius / args.resolution))
                if args.free_space_mode == "flood_fill"
                else 0
            ),
            "seeds": adjusted_seeds,
            "flood_touches_boundary": flood_touches_boundary,
            "radius_outlier_radius": args.radius_outlier_radius,
            "radius_outlier_min_neighbors": args.radius_outlier_min_neighbors,
        },
    }
    output_paths["stats"].write_text(
        json.dumps(stats, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return stats


def main(argv=None):
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    input_path, output_prefix = validate_arguments(args, parser)
    try:
        stats = convert(args, input_path, output_prefix)
    except (OSError, RuntimeError, ValueError) as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 1

    map_stats = stats["map"]
    print("Map conversion completed")
    print("  YAML: {}".format(stats["outputs"]["yaml"]))
    print("  PGM:  {}".format(stats["outputs"]["pgm"]))
    print("  Preview: {}".format(stats["outputs"]["preview"]))
    print(
        "  Size: {} x {} cells at {:.3f} m/cell".format(
            map_stats["width"],
            map_stats["height"],
            map_stats["resolution"],
        )
    )
    print(
        "  Occupied/free/unknown: {:.1f}% / {:.1f}% / {:.1f}%".format(
            map_stats["occupied_percent"],
            map_stats["free_percent"],
            map_stats["unknown_percent"],
        )
    )
    if stats["parameters"]["flood_touches_boundary"]:
        print(
            "WARNING: flood-filled free space reaches the map boundary. "
            "Inspect the preview and use crop bounds if unmapped exterior was marked free.",
            file=sys.stderr,
        )
    if any(seed["adjusted"] for seed in stats["parameters"]["seeds"]):
        print(
            "WARNING: at least one flood seed was on an obstacle and was moved "
            "to the nearest open cell; inspect the stats JSON.",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
