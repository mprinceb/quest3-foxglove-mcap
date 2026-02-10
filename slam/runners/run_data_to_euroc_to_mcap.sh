#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <session_dir> [extra pipeline args...]"
  echo "Example: $0 data/20260209_173413 --include-depth"
  exit 1
fi

SESSION_DIR="$1"
shift

uv run python slam/pipeline_data_to_mcap.py --session-dir "$SESSION_DIR" "$@"
