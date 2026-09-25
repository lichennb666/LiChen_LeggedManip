# VBC object assets

Run `python3 scripts/prepare_vbc_object_assets.py` before the paper-aligned
VBC tasks. The script installs a four-object subset from the official
[`visual_wholebody`](https://github.com/Ericonaldo/visual_wholebody) release:

- `plate_holder`
- `glue_1`
- `blue_cup`
- `clear_box`

Each directory contains the upstream URDF/meshes/textures and its frozen
1024-D PointNet++ `features.npy`. The environment assigns object `i % 4` to
Isaac environment `i`, and uses the same mapping for the feature table.

The files retain their upstream licensing/provenance. Do not regenerate only
some feature files with a different encoder: all shape codes used by one
teacher must belong to the same embedding space.
