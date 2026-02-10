#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <export_dir> [config_yaml] [results_dir]"
  exit 1
fi

EXPORT_DIR="$(realpath "$1")"
CONFIG_YAML="${2:-$(realpath "$(dirname "$0")/../config/orbslam3_quest3_stereo.yaml")}" 
RESULTS_DIR="${3:-$EXPORT_DIR/orbslam3_stereo_inertial_results}"
RESULTS_DIR="$(realpath "$RESULTS_DIR")"

SEQ_DIR="$EXPORT_DIR/mav0"
TIMES_FILE="$SEQ_DIR/meta/orbslam_times.txt"

if [[ -z "${ORB_SLAM3_ROOT:-}" && -z "${ORB_SLAM3_STEREO_INERTIAL_BIN:-}" ]]; then
  echo "Set ORB_SLAM3_ROOT or ORB_SLAM3_STEREO_INERTIAL_BIN before running."
  exit 3
fi

BIN="${ORB_SLAM3_STEREO_INERTIAL_BIN:-${ORB_SLAM3_ROOT}/Examples/Stereo-Inertial/stereo_inertial_euroc}"
VOCAB_FILE="${ORB_VOCAB:-${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt}"

for p in "$BIN" "$VOCAB_FILE" "$CONFIG_YAML" "$TIMES_FILE"; do
  if [[ ! -e "$p" ]]; then
    echo "Missing required file: $p"
    exit 4
  fi
done

mkdir -p "$RESULTS_DIR"
pushd "$RESULTS_DIR" >/dev/null

set -x
"$BIN" "$VOCAB_FILE" "$CONFIG_YAML" "$SEQ_DIR" "$TIMES_FILE" | tee run.log
set +x

for f in CameraTrajectory.txt KeyFrameTrajectory.txt; do
  if [[ -f "$f" ]]; then
    cp "$f" "${f%.txt}.tum.txt"
  fi
done

popd >/dev/null

echo "ORB-SLAM3 stereo-inertial run complete. Results: $RESULTS_DIR"
