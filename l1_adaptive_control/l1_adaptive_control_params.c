/**
 * Automatically start the L1 failure-mode controller
 *
 * The module remains idle outside the two L1 failure navigation states.
 *
 * @boolean
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_INT32(L1_FAIL_EN, 1);

/**
 * Motor 1 command scale after the original motor mixer
 *
 * 1.0 is source-equivalent. Values below 1.0 inject actuator degradation
 * after the original ModeAdaptive motorMixing calculation.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_MOT1_SCALE, 1.0f);

/**
 * Circle or figure-eight X radius
 *
 * Maps to the original CIRCRADIUSX parameter.
 *
 * @unit m
 * @min 0.1
 * @max 20.0
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_CIR_RADIUS, 2.0f);

/**
 * Figure-eight Y radius
 *
 * Maps to the original CIRCRADIUSY parameter.
 *
 * @unit m
 * @min 0.1
 * @max 20.0
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_CIR_RAD_Y, 1.0f);

/**
 * Final trajectory tangent speed
 *
 * Maps to the original CIRCSPEED parameter.
 *
 * @unit m/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_CIR_SPEED, 0.3f);

/**
 * Original trajectory index
 *
 * 0 uses the source default hover behavior, 1 is variable-yaw circle,
 * 2 is fixed-yaw circle, 3 is fixed-yaw figure eight, and 4 is tilted
 * figure eight.
 *
 * @min 0
 * @max 4
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_INT32(L1_TRAJ_IDX, 0);

/**
 * Original landing flag
 *
 * Maps to the original LANDFLAG parameter.
 *
 * @boolean
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_INT32(L1_LAND_FLAG, 0);

/**
 * Position gain X
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_KPX, 14.0f);

/**
 * Position gain Y
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_KPY, 15.0f);

/**
 * Position gain Z
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_KPZ, 15.0f);

/**
 * Velocity gain X
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.05
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_KVX, 1.5f);

/**
 * Velocity gain Y
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.05
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_KVY, 0.9f);

/**
 * Velocity gain Z
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.05
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_KVZ, 1.1f);

/**
 * Rotation gain X
 *
 * @min 0.0
 * @max 20.0
 * @decimal 4
 * @increment 0.01
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_KRX, 0.55f);

/**
 * Rotation gain Y
 *
 * @min 0.0
 * @max 20.0
 * @decimal 4
 * @increment 0.01
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_KRY, 0.35f);

/**
 * Rotation gain Z
 *
 * @min 0.0
 * @max 20.0
 * @decimal 4
 * @increment 0.01
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_KRZ, 0.15f);

/**
 * Angular velocity gain X
 *
 * @min 0.0
 * @max 5.0
 * @decimal 5
 * @increment 0.001
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_KOX, 0.035f);

/**
 * Angular velocity gain Y
 *
 * @min 0.0
 * @max 5.0
 * @decimal 5
 * @increment 0.001
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_KOY, 0.03f);

/**
 * Angular velocity gain Z
 *
 * @min 0.0
 * @max 5.0
 * @decimal 5
 * @increment 0.001
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_KOZ, 0.004f);

/**
 * Enable original L1 adaptive augmentation
 *
 * Maps to the original L1ENABLE parameter.
 *
 * @boolean
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_INT32(L1_ADAPT_EN, 0);

/**
 * L1 translational predictor pole
 *
 * Maps to the original ASV parameter.
 *
 * @min -100.0
 * @max -0.1
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_AS_V, -5.0f);

/**
 * L1 angular predictor pole
 *
 * Maps to the original ASOMEGA parameter.
 *
 * @min -100.0
 * @max -0.1
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_AS_OMEGA, -10.0f);

/**
 * L1 thrust first-filter cutoff
 *
 * Maps to the original CTOFFQ1THRUST parameter.
 *
 * @unit rad/s
 * @min 0.1
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_Q1_THR, 10.0f);

/**
 * L1 moment first-filter cutoff
 *
 * Maps to the original CTOFFQ1MOMENT parameter.
 *
 * @unit rad/s
 * @min 0.1
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_Q1_MOM, 10.0f);

/**
 * L1 moment second-filter cutoff
 *
 * Maps to the original CTOFFQ2MOMENT parameter.
 *
 * @unit rad/s
 * @min 0.1
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_Q2_MOM, 2.0f);
