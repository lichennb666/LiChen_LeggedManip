#!/usr/bin/env python3
"""Fail-fast check for active prismatic gripper joints in an Isaac USD."""

from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("usd_path")
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()

    # PXR bindings are loaded by Kit.  Launching through AppLauncher makes
    # this a real Isaac-Sim/USD check instead of a host-side text inspection.
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    from pxr import Usd, UsdPhysics  # noqa: PLC0415

    try:
        stage = Usd.Stage.Open(args.usd_path)
        if stage is None:
            raise RuntimeError(f"cannot open USD: {args.usd_path}")

        matches = []
        for prim in stage.Traverse():
            name = prim.GetName()
            if name in {"joint7", "joint8"}:
                matches.append(
                    (
                        str(prim.GetPath()),
                        prim.GetTypeName(),
                        prim.IsA(UsdPhysics.PrismaticJoint),
                        prim.GetAppliedSchemas(),
                    )
                )

        if len(matches) != 2 or any(
            kind != "PhysicsPrismaticJoint" or not is_prismatic
            for _, kind, is_prismatic, _ in matches
        ):
            raise RuntimeError(
                "active joint7/joint8 check failed; found "
                f"{matches}. The old USD is expected to fail because it contains only fixed gripper visuals."
            )

        print("[VBC] active physical gripper joints:", flush=True)
        for row in matches:
            print(
                f"  path={row[0]} type={row[1]} is_prismatic={row[2]} schemas={row[3]}",
                flush=True,
            )
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
