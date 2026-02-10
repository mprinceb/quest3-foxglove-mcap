"""Quest capture dataset helpers shared across SLAM/VIO modules."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def load_json(path: Path) -> dict[str, Any]:
    with path.open() as f:
        return json.load(f)


def load_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        return list(reader)


def list_ts_files(folder: Path, suffix: str) -> dict[int, Path]:
    out: dict[int, Path] = {}
    for p in folder.glob(f"*.{suffix}"):
        try:
            ts = int(p.stem)
        except ValueError:
            continue
        out[ts] = p
    return out


def ms_to_ns(ts_ms: int) -> int:
    return ts_ms * 1_000_000


def yuv420_888_to_y(yuv_path: Path, fmt: dict[str, Any]) -> np.ndarray:
    """Read Y plane from Android YUV_420_888 capture as mono8 image."""
    width = int(fmt["width"])
    height = int(fmt["height"])
    y_row_stride = int(fmt["planes"][0]["rowStride"])
    y_size = int(fmt["planes"][0]["bufferSize"])

    data = np.fromfile(yuv_path, np.uint8)
    if data.size < y_size:
        raise ValueError(f"YUV file too small: {yuv_path} ({data.size} < {y_size})")

    y = data[:y_size].reshape((height, y_row_stride))[:, :width]
    return y


def yuv420_888_to_bgr(yuv_path: Path, fmt: dict[str, Any]) -> np.ndarray:
    """Decode Android YUV_420_888 capture to BGR image."""
    width = int(fmt["width"])
    height = int(fmt["height"])
    y_row_stride = int(fmt["planes"][0]["rowStride"])
    uv_row_stride = int(fmt["planes"][1]["rowStride"])
    uv_pixel_stride = int(fmt["planes"][1]["pixelStride"])
    y_plane_size = int(fmt["planes"][0]["bufferSize"])
    u_plane_size = int(fmt["planes"][1]["bufferSize"])
    v_plane_size = int(fmt["planes"][2]["bufferSize"])

    y_size = y_plane_size
    uv_size = min(u_plane_size, v_plane_size)
    uv_expected = uv_row_stride * (height // 2)

    data = np.fromfile(yuv_path, np.uint8)
    if data.size < y_size + 2 * uv_size:
        raise ValueError(f"YUV file too small: {yuv_path}")

    y = data[:y_size].reshape((height, y_row_stride))[:, :width]
    u_flat = data[y_size:y_size + uv_size]
    v_flat = data[y_size + uv_size:y_size + 2 * uv_size]

    if u_flat.size < uv_expected:
        u_flat = np.pad(u_flat, (0, uv_expected - u_flat.size), mode="edge")
    if v_flat.size < uv_expected:
        v_flat = np.pad(v_flat, (0, uv_expected - v_flat.size), mode="edge")

    u = u_flat.reshape((height // 2, uv_row_stride))
    v = v_flat.reshape((height // 2, uv_row_stride))

    if uv_pixel_stride == 2:
        u = u[:, ::2]
        v = v[:, ::2]
    else:
        u = u[:, : width // 2]
        v = v[:, : width // 2]

    i420 = np.concatenate((y.flatten(), u.flatten(), v.flatten()))
    i420 = i420.reshape((height * 3 // 2, width))
    return cv2.cvtColor(i420, cv2.COLOR_YUV2BGR_I420)


def load_depth_raw(raw_path: Path, width: int, height: int) -> np.ndarray | None:
    depth = np.fromfile(raw_path, dtype=np.uint16)
    if depth.size < width * height:
        return None
    return depth[: width * height].reshape((height, width))
