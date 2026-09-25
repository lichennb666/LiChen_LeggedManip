#!/usr/bin/env python3
"""Install the small official VBC object/PointNet++ reproduction subset.

The upstream release stores one ``features.npy`` beside each object's URDF
and meshes. This script copies the selected objects from an existing clone,
or creates a temporary shallow clone when ``--source-repo`` is omitted.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np


UPSTREAM_URL = "https://github.com/Ericonaldo/visual_wholebody.git"
UPSTREAM_REF = "main"
OBJECT_NAMES = ("plate_holder", "glue_1", "blue_cup", "clear_box")
UPSTREAM_ROOT = "high-level/data/asset/obj_set"


def _run(*args: str, cwd: Path | None = None) -> str:
    return subprocess.check_output(args, cwd=cwd, text=True).strip()


def _copy_from_git(repo: Path, destination: Path) -> None:
    for object_name in OBJECT_NAMES:
        source_prefix = f"{UPSTREAM_ROOT}/{object_name}"
        files = _run(
            "git", "ls-tree", "-r", "--name-only", UPSTREAM_REF, source_prefix, cwd=repo
        ).splitlines()
        if not files:
            raise RuntimeError(f"Object '{object_name}' is missing from {repo}@{UPSTREAM_REF}")
        for source_path in files:
            relative_path = Path(source_path).relative_to(UPSTREAM_ROOT)
            output_path = destination / relative_path
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_bytes(
                subprocess.check_output(
                    ["git", "show", f"{UPSTREAM_REF}:{source_path}"], cwd=repo
                )
            )


def _validate(destination: Path) -> None:
    for object_name in OBJECT_NAMES:
        object_dir = destination / object_name
        for filename in ("model.urdf", "collision.obj", "textured.obj", "features.npy"):
            path = object_dir / filename
            if not path.is_file():
                raise FileNotFoundError(path)
        feature = np.asarray(np.load(object_dir / "features.npy"), dtype=np.float32).reshape(-1)
        if feature.shape != (1024,):
            raise ValueError(f"{object_name}: expected 1024-D feature, got {feature.shape}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-repo", type=Path)
    parser.add_argument(
        "--destination",
        type=Path,
        default=(
            Path(__file__).resolve().parents[1]
            / "source/LeggedManip_Lab/LeggedManip_Lab/assets/vbc_objects"
        ),
    )
    args = parser.parse_args()

    temporary_root: Path | None = None
    source_repo = args.source_repo
    if source_repo is None:
        temporary_root = Path(tempfile.mkdtemp(prefix="vbc-assets-"))
        source_repo = temporary_root / "visual_wholebody"
        subprocess.check_call(
            ["git", "clone", "--depth", "1", UPSTREAM_URL, str(source_repo)]
        )
    try:
        _copy_from_git(source_repo.resolve(), args.destination.resolve())
        _validate(args.destination.resolve())
        print(
            f"Installed {len(OBJECT_NAMES)} VBC objects and 1024-D features to "
            f"{args.destination.resolve()}"
        )
    finally:
        if temporary_root is not None:
            shutil.rmtree(temporary_root)


if __name__ == "__main__":
    main()
