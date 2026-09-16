# DSun + L1 Adaptive Control — PX4 v1.17 Gazebo SITL with Docker

This branch provides a reproducible **Ubuntu 22.04 + Docker + PX4 v1.17 + Gazebo Harmonic** simulation environment for the DSun geometric controller and L1 adaptive augmentation.

The intended workflow is:

```text
Ubuntu 22.04 host
  ├── QGroundControl 5.0.x
  └── Docker
       └── PX4 v1.17 SITL + Gazebo Harmonic
            └── X500 + DSun geometric controller + L1 augmentation
```

PX4 state estimates are consumed by the custom controller and then passed through PX4's normal control-allocation path:

```text
PX4 state estimates
  → trajectory / manual input
  → DSun geometric controller
  → L1 adaptive augmentation
  → thrust / torque setpoints
  → PX4 control allocator
  → simulated motors
```

## Tested branch

This README describes:

```text
dsun-sitl-docker-gz-partition-fix
```

This branch includes the Gazebo Transport `GZ_PARTITION` fix required for reliable Docker SITL discovery.

## Host requirements

Recommended host configuration:

- Ubuntu **22.04 LTS Desktop**, x86_64.
- Internet access for the first Docker image build.
- At least 20 GB free disk space recommended.
- A desktop/X11 or XWayland session for Gazebo GUI mode.
- QGroundControl running on the **host**, not inside the SITL container.

The Docker workflow uses the pinned image:

```text
px4io/px4-dev-simulation-jammy:2024-05-18
```

Gazebo Harmonic is installed inside the development image by `Dockerfile.sitl`.

---

# 1. Clone the SITL Docker branch

```bash
git clone --branch dsun-sitl-docker-gz-partition-fix --recurse-submodules \
  https://github.com/ssybh2/L1Quad-Motor-Degradation.git

cd L1Quad-Motor-Degradation
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

The pinned `PX4-Autopilot` submodule is the PX4 v1.17 development base used by this branch.

---

# 2. Install Docker on Ubuntu 22.04

A host bootstrap script is included. Run it once:

```bash
chmod +x ./scripts/bootstrap_docker_ubuntu22.sh ./sim.sh
sudo ./scripts/bootstrap_docker_ubuntu22.sh
```

The script installs:

```text
git
docker.io
docker-compose-v2
x11-xserver-utils
```

It also enables the Docker service and adds the current user to the `docker` group.

**Log out of Ubuntu and log back in once after running the script.**

Then verify Docker access without `sudo`:

```bash
docker --version
docker compose version
docker info
```

If `docker info` reports a permission error, confirm that the current user belongs to the Docker group:

```bash
groups
```

---

# 3. Build the SITL development image

From the repository root:

```bash
./sim.sh build
```

The first build downloads the PX4 Jammy development image and installs Gazebo Harmonic inside it, so it requires network access.

Normal source-code changes under this repository do **not** require rebuilding the Docker image because the project directory is bind-mounted into the container.

---

# 4. Compile the PX4 SITL target

Before launching the simulator, run the compile-only check:

```bash
./sim.sh compile
```

This command:

1. installs the repository integration into the pinned PX4 source tree;
2. enables the L1 adaptive-control module for SITL;
3. builds `px4_sitl_default`;
4. verifies that the L1 module library was produced.

A successful run ends with:

```text
DSun/L1 SITL compile OK
```

---

# 5. Install QGroundControl on Ubuntu 22.04

## Important QGC version note

Current QGroundControl 5.1 Linux AppImages require Ubuntu 24.04 or newer. Because this branch intentionally uses **Ubuntu 22.04**, the simplest host setup is to use the latest **QGroundControl 5.0.x** Linux x86_64 AppImage.

Official releases:

- https://github.com/mavlink/qgroundcontrol/releases
- https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html

Choose a **5.0.x stable release** and download the Linux `x86_64` AppImage. Do not choose the 5.1 AppImage for a stock Ubuntu 22.04 host.

## Install the QGC runtime dependencies

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

For pure SITL, serial-port access is not required. If the same QGC installation will later be used with a Pixhawk over USB, also run:

```bash
sudo usermod -aG dialout "$(id -un)"
```

Then log out and back in once.

`ModemManager` can claim real serial devices. It is not necessary to disable it for pure SITL, but for USB flight-controller work it can be stopped with:

```bash
sudo systemctl mask --now ModemManager.service
```

## Run QGroundControl

Assuming the downloaded file is in `~/Downloads`:

```bash
cd ~/Downloads
chmod +x ./QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage
```

You may also double-click the AppImage after making it executable.

Keep QGroundControl running on the Ubuntu host while PX4 SITL runs in Docker.

---

# 6. Launch Gazebo X500 SITL

## GUI mode

From the repository root:

```bash
./sim.sh gui
```

`sim.sh` automatically:

- checks Docker and Docker Compose;
- initializes the PX4 submodule if required;
- installs the project integration into PX4;
- builds the Docker image if needed;
- forwards the host display into the container;
- launches `make px4_sitl gz_x500`.

You should see:

1. a PX4 shell in the terminal;
2. Gazebo Harmonic with an X500 vehicle;
3. QGroundControl automatically detect the PX4 SITL vehicle.

The container uses host networking, so QGC normally requires **no manual UDP link configuration**.

## Headless mode

For CI, remote machines, or systems without a desktop session:

```bash
./sim.sh headless
```

---

# 7. Verify the simulation

In the PX4 shell, verify that the custom module is running:

```text
l1_adaptive_control status
```

You can also inspect MAVLink if QGC does not connect:

```text
mavlink status
```

Recommended initial parameters:

```text
param set L1_MOT1_SCALE 1.0
param set L1_ADAPT_EN 0
```

This starts with:

- Motor 1 at 100% output;
- L1 adaptive augmentation disabled;
- DSun geometric-control baseline only.

The controller defaults were tuned for a vehicle different from the Gazebo X500. Before evaluating controller performance, set the simulated vehicle mass and inertia parameters to match the model:

```text
L1_MASS
L1_JXX
L1_JYY
L1_JZZ
```

---

# 8. Recommended experiment sequence

Use the following order so that controller behavior and motor-degradation effects can be separated cleanly:

```text
1. Native PX4 X500 baseline
2. DSun controller, Motor 1 = 100%, L1 OFF
3. DSun controller, Motor 1 = 95%, L1 OFF
4. DSun controller, Motor 1 = 90%, L1 OFF
5. DSun controller, Motor 1 = 80%, L1 OFF
6. Repeat the same degradation levels with L1 ON
7. Compare logs, attitude response, position tracking and control effort
```

Motor 1 degradation:

```text
param set L1_MOT1_SCALE 1.00   # no degradation
param set L1_MOT1_SCALE 0.95   # 95% actuator command remains
param set L1_MOT1_SCALE 0.90   # 90% actuator command remains
param set L1_MOT1_SCALE 0.80   # 80% actuator command remains
```

L1 augmentation:

```text
param set L1_ADAPT_EN 0        # DSun baseline
param set L1_ADAPT_EN 1        # DSun + L1 adaptive augmentation
```

---

# 9. QGroundControl connection troubleshooting

## QGC opens but does not detect PX4 SITL

First confirm that QGC is running on the **host** and SITL is still running in the Docker terminal.

Inside the PX4 shell:

```text
mavlink status
```

On the Ubuntu host, you can inspect UDP sockets with:

```bash
ss -lunp | grep -E '14540|14550|14580'
```

Then check:

- only one QGC instance is running;
- `docker compose` is using `network_mode: host` from this repository;
- a restrictive host firewall is not blocking local UDP traffic;
- another simulator is not already using the same MAVLink ports.

In normal use, no custom QGC communication link needs to be created manually.

## QGC AppImage does not start on Ubuntu 22.04

Check the version first. QGC 5.1 Linux AppImages target Ubuntu 24.04 or newer. For this Ubuntu 22.04 workflow, use a **QGC 5.0.x** AppImage.

Also verify:

```bash
sudo apt install -y libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev
```

---

# 10. Gazebo GUI troubleshooting

## `DISPLAY is not set`

GUI mode must be started from an Ubuntu desktop session:

```bash
echo "$DISPLAY"
```

If the variable is empty, use:

```bash
./sim.sh headless
```

or start the command from a graphical terminal session.

## Gazebo window cannot open

Install the host X11 helper if necessary:

```bash
sudo apt install -y x11-xserver-utils
```

`sim.sh gui` automatically performs the temporary `xhost` permission change needed by the container and restores it when the script exits.

## Gazebo Transport discovery problems

Use this branch:

```text
dsun-sitl-docker-gz-partition-fix
```

Its `docker-compose.yml` sets:

```text
GZ_PARTITION=l1quad_sitl
```

which prevents the Gazebo Transport discovery issue that affected the earlier Docker branch.

---

# 11. Useful commands

```bash
./sim.sh gui       # launch PX4 SITL + Gazebo X500 GUI
./sim.sh headless  # launch without Gazebo GUI
./sim.sh compile   # compile px4_sitl_default and verify L1 module
./sim.sh build     # rebuild only the Docker image
./sim.sh shell     # open an interactive development container
./sim.sh clean     # remove PX4 SITL build output
```

If the PX4 submodule becomes incomplete:

```bash
git submodule update --init --recursive
```

To completely rebuild the SITL target:

```bash
./sim.sh clean
./sim.sh compile
```

---

# Repository layout

```text
./Dockerfile.sitl                  Ubuntu 22.04 PX4/Gazebo development image
./docker-compose.yml               host networking, X11 and GZ_PARTITION setup
./sim.sh                           main build/compile/run entry point
./PX4-Autopilot/                   pinned PX4 v1.17 submodule
./l1_adaptive_control/             DSun/L1 controller module
./gz_plugins/                      Gazebo integration/plugins
./scripts/install_sitl.sh          installs project changes into PX4
./scripts/bootstrap_docker_ubuntu22.sh
./README_DOCKER_SITL.md            shorter Docker SITL reference
```

# References

- [Original L1Quad repository](https://github.com/sigma-pi/L1Quad)
- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
- [QGroundControl releases](https://github.com/mavlink/qgroundcontrol/releases)
- [QGroundControl 5.0 Ubuntu installation guide](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html)

# Safety

This repository contains research flight-control software. Complete SITL and propeller-free hardware tests before real flight. Do not introduce simulated or real motor degradation until the baseline vehicle and controller are stable.
