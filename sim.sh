#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$repo_dir"

mode="${1:-gui}"

if ! command -v docker >/dev/null 2>&1; then
    echo "Docker is not installed. Install Docker Engine + Compose plugin first." >&2
    exit 1
fi

if ! docker compose version >/dev/null 2>&1; then
    echo "Docker Compose plugin is not available (expected: docker compose)." >&2
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    echo "Docker daemon is unavailable or your user lacks permission." >&2
    echo "Try: sudo usermod -aG docker $USER  (then log out/in)" >&2
    exit 1
fi

if [[ ! -d PX4-Autopilot/src ]]; then
    echo "Initializing PX4 submodule..."
    git submodule update --init --recursive
fi

export LOCAL_UID="$(id -u)"
export LOCAL_GID="$(id -g)"

container_setup='mkdir -p "$HOME"; git config --global --add safe.directory "*"; ./scripts/install_sitl.sh ./PX4-Autopilot'

run_in_container() {
    docker compose run --rm sitl bash -lc "$1"
}

case "$mode" in
    build|image)
        docker compose build
        ;;

    compile)
        docker compose build
        run_in_container "$container_setup; make -C ./PX4-Autopilot px4_sitl_default -j\$(nproc); test -f ./PX4-Autopilot/build/px4_sitl_default/src/modules/l1_adaptive_control/libmodules__l1_adaptive_control.a; echo 'DSun/L1 SITL compile OK'"
        ;;

    headless)
        docker compose build
        run_in_container "$container_setup; HEADLESS=1 make -C ./PX4-Autopilot px4_sitl gz_x500"
        ;;

    gui)
        if [[ -z "${DISPLAY:-}" ]]; then
            echo "DISPLAY is not set. Use './sim.sh headless' or start this from your desktop session." >&2
            exit 1
        fi

        if command -v xhost >/dev/null 2>&1; then
            xhost +local: >/dev/null
            trap 'xhost -local: >/dev/null 2>&1 || true' EXIT
        else
            echo "Warning: xhost not found. If Gazebo GUI cannot open, install x11-xserver-utils." >&2
        fi

        docker compose build
        run_in_container "$container_setup; make -C ./PX4-Autopilot px4_sitl gz_x500"
        ;;

    shell)
        docker compose build
        run_in_container 'mkdir -p "$HOME"; git config --global --add safe.directory "*"; exec bash'
        ;;

    clean)
        docker compose run --rm sitl bash -lc 'rm -rf ./PX4-Autopilot/build/px4_sitl_default'
        ;;

    *)
        cat >&2 <<'EOF'
Usage: ./sim.sh [gui|headless|compile|build|shell|clean]

  gui       Build image if needed, install DSun/L1 into PX4, launch Gazebo X500 GUI (default)
  headless  Same simulation without Gazebo GUI
  compile   Compile px4_sitl_default and verify the L1 module library exists
  build     Build only the Docker image
  shell     Open a shell inside the development container
  clean     Remove the PX4 SITL build directory
EOF
        exit 2
        ;;
esac
