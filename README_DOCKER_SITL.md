# Docker SITL Quick Reference

This branch provides a reproducible Ubuntu 22.04 Docker environment for the DSun geometric controller, L1 adaptive controller, PX4 v1.17, Gazebo Harmonic and the X500 model.

For the full walkthrough, including QGroundControl installation and troubleshooting, see [`README.md`](README.md).

## 1. Clone the branch

```bash
git clone --branch dsun-sitl-docker-gz-partition-fix --recurse-submodules \
  https://github.com/ssybh2/L1Quad-Motor-Degradation.git
cd L1Quad-Motor-Degradation
```

## 2. One-time Ubuntu 22.04 host setup

```bash
chmod +x ./scripts/bootstrap_docker_ubuntu22.sh ./sim.sh
sudo ./scripts/bootstrap_docker_ubuntu22.sh
```

Log out and back in once after the script adds your user to the `docker` group.

Verify:

```bash
docker --version
docker compose version
docker info
```

## 3. Build and compile

```bash
./sim.sh build
./sim.sh compile
```

A successful compile ends with:

```text
DSun/L1 SITL compile OK
```

The image is pinned to:

```text
px4io/px4-dev-simulation-jammy:2024-05-18
```

The project source is bind-mounted into the container, so normal controller edits do not require rebuilding the Docker image.

## 4. QGroundControl on Ubuntu 22.04

Run QGroundControl on the Ubuntu host, not inside the SITL container.

Current QGroundControl 5.1 Linux AppImages require Ubuntu 24.04 or newer. For this Ubuntu 22.04 workflow, use the latest QGroundControl **5.0.x** Linux x86_64 AppImage.

Official downloads and instructions:

- https://github.com/mavlink/qgroundcontrol/releases
- https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html

Install the Ubuntu 22.04 runtime dependencies:

```bash
sudo apt update
sudo apt install -y \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-libav \
  gstreamer1.0-gl \
  libfuse2 \
  libxcb-xinerama0 \
  libxkbcommon-x11-0 \
  libxcb-cursor-dev
```

Then make the downloaded AppImage executable and launch it:

```bash
cd ~/Downloads
chmod +x ./QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage
```

For pure SITL, serial-port setup is not required. If QGC will later connect to a Pixhawk over USB:

```bash
sudo usermod -aG dialout "$(id -un)"
```

Log out and back in once after changing group membership.

## 5. Launch Gazebo X500

GUI:

```bash
./sim.sh gui
```

Headless:

```bash
./sim.sh headless
```

The Docker service uses host networking. QGroundControl on the same Ubuntu host should normally discover PX4 SITL automatically without creating a manual UDP link.

## 6. First simulation settings

In the PX4 shell:

```text
l1_adaptive_control status
param set L1_MOT1_SCALE 1.0
param set L1_ADAPT_EN 0
```

The repository controller defaults were tuned for a different vehicle than Gazebo X500. Before controller evaluation, set:

```text
L1_MASS
L1_JXX
L1_JYY
L1_JZZ
```

to values matching the simulated vehicle.

Recommended experiment order:

```text
1. Native PX4 X500 flight
2. DSun, Motor 1 = 100%, L1 OFF
3. DSun, Motor 1 = 95/90/80%, L1 OFF
4. Repeat the same degradation levels with L1 ON
5. Compare logs and tracking errors
```

Motor degradation:

```text
param set L1_MOT1_SCALE 1.00
param set L1_MOT1_SCALE 0.80
```

L1 augmentation:

```text
param set L1_ADAPT_EN 0
param set L1_ADAPT_EN 1
```

## 7. Useful commands

```bash
./sim.sh gui       # Gazebo GUI SITL
./sim.sh headless  # headless SITL
./sim.sh compile   # compile-only verification
./sim.sh build     # rebuild Docker image
./sim.sh shell     # development shell
./sim.sh clean     # remove PX4 SITL build directory
```

## 8. Common checks

If QGC does not connect, run in the PX4 shell:

```text
mavlink status
```

and on the host:

```bash
ss -lunp | grep -E '14540|14550|14580'
```

If Gazebo GUI does not open:

```bash
echo "$DISPLAY"
sudo apt install -y x11-xserver-utils
```

If the PX4 submodule is missing:

```bash
git submodule update --init --recursive
```

This branch sets `GZ_PARTITION=l1quad_sitl` in `docker-compose.yml` to avoid Gazebo Transport discovery conflicts in the Docker workflow.
