#!/usr/bin/env python3
"""Conservatively remove thin LiDAR ray artifacts from a ROS occupancy map.

Only free cells are changed, and rejected cells become unknown. Occupied cells
are never invented or erased. This is intentionally suitable for a robot
navigation map where uncertain glass/door observations must not become
confident free space.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage


UNKNOWN = np.uint8(205)
FREE = np.uint8(254)


def disk(radius: int) -> np.ndarray:
    yy, xx = np.ogrid[-radius : radius + 1, -radius : radius + 1]
    return xx * xx + yy * yy <= radius * radius


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--preview", type=Path, required=True)
    parser.add_argument(
        "--radius-px",
        type=int,
        default=4,
        help="Minimum supported free-space half-width (4 px = 0.20 m).",
    )
    parser.add_argument(
        "--min-component-px",
        type=int,
        default=12,
        help="Remove isolated free components smaller than this area.",
    )
    args = parser.parse_args()

    source = np.asarray(Image.open(args.input).convert("L"))
    free = source == FREE
    opened = ndimage.binary_opening(free, structure=disk(args.radius_px))

    labels, count = ndimage.label(opened)
    sizes = np.bincount(labels.ravel(), minlength=count + 1)
    supported = opened & (sizes[labels] >= args.min_component_px)
    rejected = free & ~supported

    cleaned = source.copy()
    cleaned[rejected] = UNKNOWN

    # Preview: retained map in grayscale; rejected free cells in orange.
    preview = np.repeat(cleaned[:, :, None], 3, axis=2)
    preview[rejected] = (255, 110, 0)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.preview.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(cleaned).save(args.output)
    Image.fromarray(preview).resize(
        (source.shape[1] * 3, source.shape[0] * 3), Image.Resampling.NEAREST
    ).save(args.preview)

    print(
        f"radius_px={args.radius_px} original_free={free.sum()} "
        f"retained_free={supported.sum()} rejected_free={rejected.sum()} "
        f"rejected_ratio={rejected.sum() / free.sum():.4f}"
    )


if __name__ == "__main__":
    main()
