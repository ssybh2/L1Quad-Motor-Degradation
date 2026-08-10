#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${1:-$repo_dir/PX4-Autopilot}"
expected_px4_commit="d6f12ad1c4f70ad3230afd7d86e971421e02fef4"

if [[ ! -d "$px4_dir/src/modules" ]]; then
    echo "PX4 source not found: $px4_dir" >&2
    echo "Run: git submodule update --init --recursive" >&2
    exit 1
fi

actual_px4_commit="$(git -C "$px4_dir" rev-parse HEAD)"

if [[ "$actual_px4_commit" != "$expected_px4_commit" ]]; then
    echo "Unsupported PX4 revision: $actual_px4_commit" >&2
    echo "Expected PX4 v1.17.0 base: $expected_px4_commit" >&2
    exit 1
fi

copy_tree() {
    mkdir -p "$2"
    cp -a "$1/." "$2/"
}

apply_once() {
    if git -C "$1" apply --reverse --check "$2" >/dev/null 2>&1; then
        echo "Already applied: $(basename "$2")"
    else
        git -C "$1" apply --check "$2"
        git -C "$1" apply "$2"
    fi
}

copy_tree "$repo_dir/l1_adaptive_control" "$px4_dir/src/modules/l1_adaptive_control"
apply_once "$px4_dir" "$repo_dir/patches/px4-v1.17.0-l1-failure-mode.patch"
apply_once "$px4_dir" "$repo_dir/patches/px4-v1.17.0-motor-degradation.patch"
apply_once "$px4_dir" "$repo_dir/patches/px4-v1.17.0-fmu-v6c-integration.patch"

echo "Installed. Build with: cd $px4_dir && make px4_fmu-v6c_default"
