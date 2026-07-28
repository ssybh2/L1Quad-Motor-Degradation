/**
 * Automatically start the L1 failure-mode controller
 *
 * The controller remains idle outside the L1 position and altitude
 * failure modes.
 *
 * @boolean
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_INT32(L1_FAIL_EN, 1);

/**
 * Motor 1 output scale in L1 failure modes
 *
 * 1.0 keeps the full actuator command, 0.8 keeps 80 percent, and 0.0
 * commands no output. Actual thrust depends on the ESC, motor and propeller
 * response and must be calibrated before real-flight testing.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_MOT1_SCALE, 0.8f);

/**
 * Circle trajectory radius
 *
 * The new value is applied while the module is running. If it changes during
 * a circle, the trajectory smoothly transitions to the new radius.
 *
 * @unit m
 * @min 0.2
 * @max 20.0
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_CIR_RADIUS, 1.0f);
