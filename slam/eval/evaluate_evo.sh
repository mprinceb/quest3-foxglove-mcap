#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <ref_tum.txt> <est_tum.txt> [out_dir]"
  exit 1
fi

REF="$(realpath "$1")"
EST="$(realpath "$2")"
OUT_DIR="${3:-$(pwd)/evo_results}"
OUT_DIR="$(realpath "$OUT_DIR")"
mkdir -p "$OUT_DIR"

if ! command -v evo_ape >/dev/null || ! command -v evo_rpe >/dev/null; then
  echo "evo CLI not found. Install with: pip install evo --upgrade"
  exit 2
fi

set -x
evo_ape tum "$REF" "$EST" -va --plot --save_results "$OUT_DIR/ape.zip" > "$OUT_DIR/ape.txt"
evo_rpe tum "$REF" "$EST" -va --plot --save_results "$OUT_DIR/rpe.zip" > "$OUT_DIR/rpe.txt"
set +x

echo "evo evaluation complete: $OUT_DIR"
