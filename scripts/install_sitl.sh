#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${1:-$repo_dir/PX4-Autopilot}"

"$repo_dir/scripts/install_main.sh" "$px4_dir"

sitl_board="$px4_dir/boards/px4/sitl/default.px4board"
if [[ ! -f "$sitl_board" ]]; then
    echo "PX4 SITL board config not found: $sitl_board" >&2
    exit 1
fi

if ! grep -qxF 'CONFIG_MODULES_L1_ADAPTIVE_CONTROL=y' "$sitl_board"; then
    echo 'CONFIG_MODULES_L1_ADAPTIVE_CONTROL=y' >> "$sitl_board"
fi

echo "SITL integration installed."
echo "Build/run with: make -C $px4_dir px4_sitl gz_x500"
