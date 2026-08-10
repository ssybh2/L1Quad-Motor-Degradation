# L1Quad Controller for PX4 v1.17.0

PX4 v1.17.0 port of the L1 adaptive geometric controller from
[L1Quad](https://github.com/sigma-pi/L1Quad).

The original ArduPilot implementation computes motor commands directly. This
port integrates the controller with PX4's estimator, uORB setpoints and control
allocator:

```text
PX4 state estimates
  → trajectory and manual input
  → geometric controller
  → L1 adaptive augmentation
  → thrust/torque setpoints
  → PX4 control allocator
  → motors
```

## Branches

- `main`: real-hardware baseline and verified L1 failure flight modes.
- `hitl`: Pixhawk 6C Mini SIH/jMAVSim hardware-in-the-loop configuration.
- `sitl`: Gazebo Harmonic, keyboard, QGC Joystick and simulated motor failure.

## Features

- L1 adaptive geometric controller.
- RC, QGC Joystick and SITL keyboard input.
- Configurable Motor 1 output degradation in L1 failure modes.
- Yaw control retained during partial Motor 1 degradation.
- `L1 Failure`: Position-based mode that holds position and altitude.
- `L1 Altitude Failure`: Altitude-based mode with manual roll/pitch input.

## Repository Layout

```text
./l1_adaptive_control/             L1 controller module
./l1_keyboard_throttle/            SITL keyboard input
./patches/                         PX4 v1.17.0 integration patches
./scripts/                         setup helpers
./firmware/pixhawk6cmini/          prebuilt Pixhawk 6C Mini firmware
./指导.txt                          operating guide
```

This repository contains the port and its integration files, not a complete
copy of PX4.

## Build the Main Firmware

Clone the real-hardware branch and initialize its pinned PX4 source tree:

```sh
git clone --branch main --recurse-submodules \
  https://github.com/Edwin-Shao/L1Quad-Motor-Degradation.git
cd L1Quad-Motor-Degradation
```

Install the controller and PX4 integration changes, then build the Pixhawk 6C
Mini firmware:

```sh
./scripts/install_main.sh ./PX4-Autopilot
make -C ./PX4-Autopilot px4_fmu-v6c_default
```

The custom firmware is generated at:

```text
./PX4-Autopilot/build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
```

## References

- [Original L1Quad repository](https://github.com/sigma-pi/L1Quad)
- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
- Target PX4 release: `v1.17.0`

## Suggested Workspace Layout

The documentation assumes the terminal is opened in a workspace containing:

```text
./
├── Ardupoilt-to-PX4/
├── PX4-Autopilot-v1.17.0/
├── PX4-Autopilot-v1.17.0-HITL/
├── PX4-Autopilot-v1.17.0-MAIN/
└── L1Quad-firmware/
```

The workspace may be located anywhere. Commands use only relative paths.

## Prebuilt Firmware

The following files target Pixhawk 6C Mini:

```text
./firmware/pixhawk6cmini/L1Quad-main-pixhawk6cmini.px4
./firmware/pixhawk6cmini/L1Quad-hitl-pixhawk6cmini.px4
./firmware/pixhawk6cmini/L1Quad-sitl-pixhawk6cmini.px4
```

Use `main` for real-hardware preparation and `hitl` for SIH/jMAVSim. The
`sitl` file is a Pixhawk-compatible snapshot of the SITL branch; Gazebo SITL
itself runs as a host program.

To flash a file, open QGroundControl and select:

```text
Vehicle Setup → Firmware → Advanced settings → Custom firmware file
```

Remove all propellers before flashing or testing custom firmware.

See [`指导.txt`](指导.txt) for build, SITL/HITL operation and troubleshooting.

## Safety

This is research flight-control software. Complete SITL and propeller-free
HITL tests before real flight. Enter a failure mode only after a stable normal
takeoff.
