#!/usr/bin/env python3
"""Flatten the colored MID-360 Collada mesh for Gazebo Classic.

Run through Blender so the source scene hierarchy is baked into one mesh while
its embedded materials are retained.  The uncolored simulator mesh is used as
the authoritative size and origin, keeping the visual aligned with the lidar
and collision frames.
"""

import argparse
import sys
from pathlib import Path

import bpy
from mathutils import Vector


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--baseline", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args(argv)


def imported_meshes(path):
    before = set(bpy.data.objects)
    result = bpy.ops.wm.collada_import(filepath=str(path))
    if "FINISHED" not in result:
        raise RuntimeError(f"Failed to import {path}")
    return [obj for obj in bpy.data.objects if obj not in before and obj.type == "MESH"]


def world_bounds(objects):
    points = [obj.matrix_world @ vertex.co for obj in objects for vertex in obj.data.vertices]
    if not points:
        raise RuntimeError("Imported Collada scene contains no mesh vertices")
    lower = Vector(tuple(min(point[index] for point in points) for index in range(3)))
    upper = Vector(tuple(max(point[index] for point in points) for index in range(3)))
    return lower, upper


def detach_and_join(objects):
    for obj in objects:
        world_matrix = obj.matrix_world.copy()
        obj.parent = None
        obj.matrix_world = world_matrix

    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    bpy.ops.object.join()
    result = bpy.context.view_layer.objects.active
    result.name = "livox_mid360_visual"
    return result


def main():
    args = parse_args()
    source = Path(args.input).resolve()
    baseline = Path(args.baseline).resolve()
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)

    colored_objects = imported_meshes(source)
    colored_mesh = detach_and_join(colored_objects)
    source_lower, source_upper = world_bounds([colored_mesh])

    baseline_objects = imported_meshes(baseline)
    target_lower, target_upper = world_bounds(baseline_objects)
    for obj in baseline_objects:
        bpy.data.objects.remove(obj, do_unlink=True)

    source_size = source_upper - source_lower
    target_size = target_upper - target_lower
    axis_scales = [target_size[index] / source_size[index] for index in range(3)]
    scale = sum(axis_scales) / 3.0
    if max(abs(value - scale) for value in axis_scales) > 1e-4:
        raise RuntimeError(f"Meshes are not uniformly scaled: {axis_scales}")

    source_center = (source_lower + source_upper) * 0.5
    target_center = (target_lower + target_upper) * 0.5
    for vertex in colored_mesh.data.vertices:
        vertex.co = (vertex.co - source_center) * scale + target_center

    bpy.ops.object.select_all(action="DESELECT")
    colored_mesh.select_set(True)
    bpy.context.view_layer.objects.active = colored_mesh
    bpy.ops.wm.collada_export(
        filepath=str(output),
        selected=True,
        apply_modifiers=True,
        triangulate=True,
    )

    final_lower, final_upper = world_bounds([colored_mesh])
    print(f"Wrote {output}")
    print(f"Bounds: {tuple(final_lower)} .. {tuple(final_upper)}")
    print(f"Materials: {[slot.material.name for slot in colored_mesh.material_slots]}")


if __name__ == "__main__":
    main()
