#!/usr/bin/env python3
"""Apply a hand-drawn red/blue annotation to a ROS occupancy map.

Dense red hatching marks free space that should become unknown. Blue strokes
protect confirmed openings. Thin red boundary strokes alone do not create
occupied cells, so the tool cannot accidentally hallucinate a wall.
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
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--annotation", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--preview", type=Path, required=True)
    parser.add_argument(
        "--crop",
        type=int,
        nargs=4,
        metavar=("X", "Y", "WIDTH", "HEIGHT"),
        required=True,
    )
    parser.add_argument("--density-radius-px", type=int, default=5)
    parser.add_argument("--min-red-pixels", type=int, default=16)
    parser.add_argument("--blue-protect-radius-px", type=int, default=4)
    args = parser.parse_args()

    source = np.asarray(Image.open(args.map).convert("L"))
    screenshot = Image.open(args.annotation).convert("RGB")
    x, y, width, height = args.crop
    crop = np.asarray(screenshot.crop((x, y, x + width, y + height)))

    red = (
        (crop[:, :, 0] > 220)
        & (crop[:, :, 1] < 100)
        & (crop[:, :, 2] < 110)
    )
    blue = (
        (crop[:, :, 2] > 150)
        & (crop[:, :, 0] < 120)
        & (crop[:, :, 1] < 170)
    )

    target_size = (source.shape[1], source.shape[0])
    red_scaled = np.asarray(
        Image.fromarray(red.astype(np.uint8) * 255).resize(
            target_size, Image.Resampling.BOX
        )
    )
    blue_scaled = np.asarray(
        Image.fromarray(blue.astype(np.uint8) * 255).resize(
            target_size, Image.Resampling.BOX
        )
    )
    red_presence = red_scaled >= 32
    blue_presence = blue_scaled >= 32

    support = ndimage.convolve(
        red_presence.astype(np.uint16),
        disk(args.density_radius_px).astype(np.uint16),
        mode="constant",
    )
    dense_red = support >= args.min_red_pixels
    dense_red = ndimage.binary_closing(dense_red, structure=disk(2))
    blue_protected = ndimage.binary_dilation(
        blue_presence, structure=disk(args.blue_protect_radius_px)
    )

    rejected = (source == FREE) & dense_red & ~blue_protected
    result = source.copy()
    result[rejected] = UNKNOWN

    preview = np.repeat(result[:, :, None], 3, axis=2)
    preview[rejected] = (255, 45, 45)
    preview[blue_protected & (source == FREE)] = (0, 120, 255)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.preview.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(result).save(args.output)
    Image.fromarray(preview).resize(
        (source.shape[1] * 3, source.shape[0] * 3), Image.Resampling.NEAREST
    ).save(args.preview)
    print(
        f"red_pixels={red.sum()} blue_pixels={blue.sum()} "
        f"rejected_free={rejected.sum()} "
        f"blue_protected_free={(blue_protected & (source == FREE)).sum()}"
    )


if __name__ == "__main__":
    main()
