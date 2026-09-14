# L1Quad ModeAdaptive Source-Equivalent Port for PX4 v1.17.0

This branch ports the original L1Quad `ModeAdaptive` control path to PX4 v1.17.0 while keeping the original numerical operations separated into modules. PX4 is used for state acquisition, flight-mode integration and actuator transport; the control math follows the original L1Quad source order.

Original source:

- `sigma-pi/L1Quad/L1AC_customization/ArduCopter/mode_adaptive.cpp`
- `sigma-pi/L1Quad/L1AC_customization/ArduCopter/ACRL_trajectories.cpp`

## Source-equivalent control path

```text
PX4 state estimates
  -> original ACRL trajectory equations
  -> original geometricController equations
  -> original fixed-dt L1AdaptiveAugmentation equations
  -> u_b + u_ad
  -> original motorMixing()
  -> original iterativeMotorMixing() twice
  -> original [0, 100] motor saturation
  -> optional external Motor 1 degradation
  -> actuator_motors [0, 1]
  -> PX4 PWM/DShot/CAN output driver
```

`control_allocator` is deliberately disabled while either L1 failure navigation state is active. The L1 module publishes `actuator_motors` directly so PX4 does not replace the original nonlinear `motorMixing()` calculation.

## Module mapping

```text
l1_adaptive_control/
├── ACRLTrajectories.cpp/.hpp       original ACRL trajectory functions
├── TrajectoryGenerator.cpp/.hpp    ModeAdaptive::run trajectory selection/landing orchestration
├── GeometricController.cpp/.hpp    ModeAdaptive::geometricController
├── L1AdaptiveAugmentation.cpp/.hpp ModeAdaptive::L1AdaptiveAugmentation
├── MotorMixer.cpp/.hpp             motorMixing, iterativeMotorMixing, mat4Inv
├── L1AdaptiveControl.cpp/.hpp       PX4 state/mode/uORB adapter only
└── *Test.cpp                       source-equivalence regression tests
```

The controller loop is scheduled at 400 Hz because the original L1 predictor is explicitly discretized with `dt = 0.0025 s`. The port does not use timestamp-derived dynamic `dt` and does not add the previous custom L1 thrust/moment clipping.

## Motor degradation

Motor degradation is intentionally outside the original controller equations:

```text
original motorMixing result -> L1_MOT1_SCALE -> actuator_motors
```

`L1_MOT1_SCALE=1.0` is the source-equivalent default. Set it below one only when intentionally injecting Motor 1 degradation, for example:

```sh
param set L1_MOT1_SCALE 0.8
```

This scales the Motor 1 command after the original mixer; it is not fed back into the baseline geometric controller, the L1 predictor equations or the mixer calculation.

## Source parameters

The important PX4 parameters map to the original L1Quad parameters:

```text
L1_CIR_RADIUS  <-> CIRCRADIUSX   default 2.0 m
L1_CIR_RAD_Y   <-> CIRCRADIUSY   default 1.0 m
L1_CIR_SPEED   <-> CIRCSPEED     default 0.3 m/s
L1_TRAJ_IDX    <-> TRAJINDEX     default 0
L1_LAND_FLAG   <-> LANDFLAG      default 0
L1_ADAPT_EN    <-> L1ENABLE      default 0
L1_AS_V        <-> ASV           default -5
L1_AS_OMEGA    <-> ASOMEGA       default -10
L1_Q1_THR      <-> CTOFFQ1THRUST default 10
L1_Q1_MOM      <-> CTOFFQ1MOMENT default 10
L1_Q2_MOM      <-> CTOFFQ2MOMENT default 2
```

The real-aircraft build uses the original `REAL_OR_SITL=1` mass, inertia, hand-computed inertia inverse and motor-model constants. Unit tests use the original SITL profile unless explicitly overridden.

## PX4 output mapping

The original ArduPilot source sends:

```text
PWM = 1000 + 10 * motorPWM
```

where `motorPWM` is limited to `[0, 100]`. This port keeps that motor command calculation unchanged, then performs the PX4 interface conversion `motorPWM / 100` into `actuator_motors`.

For a PWM ESC setup that should reproduce the original 1000-2000 us endpoint mapping, configure the four motor outputs to 1000/2000 us and disable PX4's additional thrust-model inversion:

```sh
param set THR_MDL_FAC 0
param set PWM_MAIN_MIN1 1000
param set PWM_MAIN_MIN2 1000
param set PWM_MAIN_MIN3 1000
param set PWM_MAIN_MIN4 1000
param set PWM_MAIN_MAX1 2000
param set PWM_MAIN_MAX2 2000
param set PWM_MAIN_MAX3 2000
param set PWM_MAIN_MAX4 2000
```

Use the corresponding `PWM_AUX_MINn/MAXn` parameters when the motors are assigned to AUX outputs. DShot/CAN transports use their native output mapping rather than PWM microseconds.

## PX4 integration

The repository keeps the existing PX4 v1.17 custom navigation states:

- `L1 Failure`
- `L1 Altitude Failure`

On this source-equivalent branch both entries run the same original ModeAdaptive core. The previous manual-tilt/manual-height controller modifications are not part of the source-equivalent numerical path.

The legacy `patches/px4-v1.17.0-motor-degradation.patch` file is retained for history but `scripts/install_main.sh` no longer applies it. Motor degradation is now performed after the original mixer inside the L1 module.

## Build

Clone this branch and initialize the pinned PX4 source tree:

```sh
git clone --branch refactor/mode-adaptive-source-equivalent --recurse-submodules \
  https://github.com/ssybh2/L1Quad-Motor-Degradation.git
cd L1Quad-Motor-Degradation
```

Install the module and PX4 integration patches, then build Pixhawk 6C Mini firmware:

```sh
./scripts/install_main.sh ./PX4-Autopilot
make -C ./PX4-Autopilot px4_fmu-v6c_default
```

The generated firmware is expected at:

```text
./PX4-Autopilot/build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
```

## Regression tests

The module includes regression tests that lock important source behavior:

- original ACRL takeoff polynomial and fixed-yaw circle values;
- original geometric-controller hover/yaw behavior;
- L1 `dt=0.0025`, source initialization and re-entry state semantics, and absence of custom adaptive clipping;
- original three-iteration nonlinear motor mixer values and X-layout signs.

## Repository layout

```text
./l1_adaptive_control/             source-equivalent controller modules
./l1_keyboard_throttle/            legacy/SITL keyboard support
./patches/                         PX4 v1.17 integration patches
./scripts/                         installation helpers
./firmware/pixhawk6cmini/          existing prebuilt firmware snapshots
./指导.txt                          existing operating guide
```

Existing prebuilt firmware files were produced from earlier branches and should not be treated as builds of this source-equivalent branch until rebuilt from this branch.

## Safety

This is research flight-control software. Verify compilation and all regression tests, then complete SITL and propeller-free HITL/bench testing before any real flight. Do not enable motor degradation during initial bring-up.
