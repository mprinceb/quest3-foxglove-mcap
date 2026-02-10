#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <export_dir> [config_yaml] [results_dir]"
  echo "  export_dir: path produced by slam/adapters/export_euroc.py"
  exit 1
fi

EXPORT_DIR="$(realpath "$1")"
CONFIG_YAML="${2:-$(realpath "$(dirname "$0")/../config/orbslam3_quest3_stereo.yaml")}" 
RESULTS_DIR="${3:-$EXPORT_DIR/orbslam3_stereo_results}"
RESULTS_DIR="$(realpath "$RESULTS_DIR")"

SEQ_DIR="$EXPORT_DIR/mav0"
TIMES_FILE="$SEQ_DIR/meta/orbslam_times.txt"

if [[ ! -d "$SEQ_DIR/cam0/data" || ! -d "$SEQ_DIR/cam1/data" ]]; then
  echo "Expected EuRoC-style stereo dirs under $SEQ_DIR"
  exit 2
fi

if [[ -z "${ORB_SLAM3_ROOT:-}" && -z "${ORB_SLAM3_STEREO_BIN:-}" ]]; then
  echo "Set ORB_SLAM3_ROOT or ORB_SLAM3_STEREO_BIN before running."
  exit 3
fi

STEREO_BIN="${ORB_SLAM3_STEREO_BIN:-${ORB_SLAM3_ROOT}/Examples/Stereo/stereo_euroc}"
VOCAB_FILE="${ORB_VOCAB:-${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt}"

if [[ ! -x "$STEREO_BIN" ]]; then
  echo "ORB-SLAM3 stereo binary not found/executable: $STEREO_BIN"
  exit 4
fi

if [[ ! -f "$VOCAB_FILE" ]]; then
  echo "ORB vocabulary file not found: $VOCAB_FILE"
  exit 5
fi

if [[ ! -f "$CONFIG_YAML" ]]; then
  echo "Config file not found: $CONFIG_YAML"
  exit 6
fi

if [[ ! -f "$TIMES_FILE" ]]; then
  echo "Times file not found: $TIMES_FILE"
  exit 7
fi

mkdir -p "$RESULTS_DIR"

pushd "$RESULTS_DIR" >/dev/null

set -x
"$STEREO_BIN" "$VOCAB_FILE" "$CONFIG_YAML" "$SEQ_DIR" "$TIMES_FILE" | tee run.log
set +x

for f in CameraTrajectory.txt KeyFrameTrajectory.txt; do
  if [[ -f "$f" ]]; then
    cp "$f" "${f%.txt}.tum.txt"
  fi
done

popd >/dev/null

echo "ORB-SLAM3 stereo run complete. Results: $RESULTS_DIR"
