# DSun + L1 PX4 SITL with Docker

This branch provides a reproducible Ubuntu 22.04 Docker workflow for the custom DSun geometric controller, L1 adaptive controller, PX4 v1.17 base commit, and Gazebo X500 SITL.

## 1. One-time host setup (Ubuntu 22.04)

```bash
sudo ./scripts/bootstrap_docker_ubuntu22.sh
```

Log out and back in once after the script adds your user to the `docker` group.

## 2. Build the development image

```bash
./sim.sh build
```

The image is pinned to:

```text
px4io/px4-dev-simulation-jammy:2024-05-18
```

Project source is bind-mounted into the container, so normal controller edits do not require rebuilding the Docker image.

## 3. Compile-only check

```bash
./sim.sh compile
```

This installs the project integration into the pinned PX4 submodule, enables `CONFIG_MODULES_L1_ADAPTIVE_CONTROL=y` for SITL, builds `px4_sitl_default`, and verifies that the L1 module library was produced.

## 4. Launch Gazebo X500

GUI:

```bash
./sim.sh gui
```

Headless:

```bash
./sim.sh headless
```

On the same Ubuntu host, QGroundControl can normally connect to PX4 SITL through the host network automatically.

## 5. First simulation settings

In the PX4 shell, verify the module and start with no motor degradation and no L1 augmentation:

```text
l1_adaptive_control status
param set L1_MOT1_SCALE 1.0
param set L1_ADAPT_EN 0
```

The repository controller defaults were tuned for a different vehicle than Gazebo X500. Before evaluating DSun performance, set `L1_MASS`, `L1_JXX`, `L1_JYY`, and `L1_JZZ` to values that match the simulated vehicle model.

Recommended experiment order:

```text
1. Native PX4 X500 flight
2. DSun, Motor 1 = 100%, L1 OFF
3. DSun, Motor 1 = 95/90/80%, L1 OFF
4. Same motor degradation with L1 ON
5. Compare logs and tracking errors
```

Motor 1 degradation is controlled by:

```text
L1_MOT1_SCALE = 1.00   # no degradation
L1_MOT1_SCALE = 0.80   # 80% actuator command remains
```

L1 augmentation is controlled by:

```text
L1_ADAPT_EN = 0        # DSun baseline only
L1_ADAPT_EN = 1        # DSun + L1 adaptive augmentation
```

## Utility commands

```bash
./sim.sh shell      # development shell inside the container
./sim.sh clean      # remove PX4 SITL build directory
./sim.sh build      # rebuild Docker image
./sim.sh compile    # compile-only CI-style verification
```
