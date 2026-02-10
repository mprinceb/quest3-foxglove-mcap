#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <session_dir> [out_tum]"
  exit 1
fi

SESSION_DIR="$(realpath "$1")"
OUT_TUM="${2:-$(pwd)/slam/results/depth_pnp_vo_$(basename "$SESSION_DIR").tum.txt}"
OUT_TUM="$(realpath "$OUT_TUM")"

uv run python slam/baselines/depth_pnp_vo.py \
  --session-dir "$SESSION_DIR" \
  --out-tum "$OUT_TUM"
