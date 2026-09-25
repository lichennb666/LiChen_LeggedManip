#!/usr/bin/env python3
"""Plot selected TensorBoard scalars from an RSL-RL run to a PNG.

用法:
  # 最新一次 WBC 训练
  python scripts/plot_metrics.py --exp go2_piper_wbc

  # 指定 run 目录
  python scripts/plot_metrics.py --dir logs/rsl_rl/go2_piper_wbc/2026-09-20_01-23-58

  # 换输出文件 / 只看某些关键字
  python scripts/plot_metrics.py --exp go2_piper_vbc_shape_teacher \
    --out /tmp/vbc.png --keys Metrics Loss Train

默认会在 logs/rsl_rl/<exp>/ 下找最新的 events.out.tfevents*。
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_ROOT = os.path.join(ROOT, "logs", "rsl_rl")

DEFAULT_KEYS = ("Metrics/", "Loss/value", "Loss/learning_rate", "Train/mean_reward", "Policy/mean_std")


def newest_run(exp: str) -> str:
    base = os.path.join(LOG_ROOT, exp) if exp else LOG_ROOT
    if exp:
        runs = glob.glob(os.path.join(base, "*", ""))
    else:
        runs = glob.glob(os.path.join(base, "*", "*", ""))
    runs = [r for r in runs if os.path.isdir(r) and "exported" not in r]
    if not runs:
        sys.exit(f"找不到 run 目录: {base}")
    return max(runs, key=os.path.getmtime)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--exp", default="", help="experiment name under logs/rsl_rl (取最新 run)")
    ap.add_argument("--dir", default="", help="直接指定 run 目录")
    ap.add_argument("--out", default="", help="输出 PNG (默认 /tmp/<run>_metrics.png)")
    ap.add_argument("--keys", nargs="*", default=list(DEFAULT_KEYS), help="只画名字里含这些关键字的标量")
    args = ap.parse_args()

    run_dir = args.dir or newest_run(args.exp)
    events = sorted(glob.glob(os.path.join(run_dir, "events.out.tfevents*")))
    if not events:
        sys.exit(f"run 里没有 events 文件: {run_dir}")
    ea = EventAccumulator(events[-1], size_guidance={"scalars": 0})
    ea.Reload()
    tags = [t for t in ea.Tags().get("scalars", []) if any(k in t for k in args.keys)]
    tags.sort()
    if not tags:
        sys.exit("没有匹配的标量")

    ncols = 2
    nrows = (len(tags) + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(13, 2.6 * nrows), squeeze=False)
    for ax, tag in zip(axes.flat, tags):
        data = ea.Scalars(tag)
        steps = np.array([d.step for d in data], float)
        vals = np.array([d.value for d in data], float)
        ax.plot(steps, vals, lw=1.2)
        ax.set_title(tag, fontsize=9)
        ax.grid(alpha=0.3)
        ax.tick_params(labelsize=8)
        last = vals[-1]
        ax.axhline(last, color="r", ls=":", lw=0.8)
        ax.text(0.99, 0.05, f"last={last:.4g}", transform=ax.transAxes,
                ha="right", va="bottom", fontsize=8, color="r")
    for ax in axes.flat[len(tags):]:
        ax.axis("off")
    fig.suptitle(f"{os.path.basename(run_dir.rstrip('/'))}  |  {len(tags)} scalars", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    out = args.out or os.path.join("/tmp", os.path.basename(run_dir.rstrip("/")) + "_metrics.png")
    fig.savefig(out, dpi=130)
    print(f"已保存: {out}")
    # 顺便打印关键标量的首/末值
    for tag in tags:
        v = np.array([d.value for d in ea.Scalars(tag)], float)
        print(f"  {tag:46s} first={v[0]: .4g} last={v[-1]: .4g} min={v.min(): .4g} max={v.max(): .4g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
