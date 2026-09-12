#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run this once with sudo: sudo ./scripts/bootstrap_docker_ubuntu22.sh" >&2
    exit 1
fi

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
    echo "This helper is intended for Ubuntu 22.04 (detected: ${PRETTY_NAME:-unknown})." >&2
    exit 1
fi

apt-get update
apt-get install -y git docker.io docker-compose-v2 x11-xserver-utils
systemctl enable --now docker

target_user="${SUDO_USER:-}"
if [[ -n "$target_user" && "$target_user" != "root" ]]; then
    usermod -aG docker "$target_user"
    echo
    echo "Added $target_user to the docker group. Log out and back in once before using ./sim.sh."
fi

echo

echo "Docker installation complete."
docker --version
docker compose version
