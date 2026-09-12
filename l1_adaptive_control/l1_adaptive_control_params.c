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

/**
 * Circle tangential speed
 *
 * @unit m/s
 * @min 0.05
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_CIR_SPEED, 0.5f);

/**
 * Circle transition duration
 *
 * @unit s
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_CIR_TRANS, 2.0f);

/**
 * Automatic takeoff height
 *
 * @unit m
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_TKOFF_HGT, 1.0f);

/**
 * Automatic takeoff duration
 *
 * @unit s
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_TKOFF_T, 2.0f);

/**
 * Manual height stick deadzone
 *
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_MAN_DZ, 0.10f);

/**
 * Manual maximum climb rate
 *
 * @unit m/s
 * @min 0.05
 * @max 3.0
 * @decimal 2
 * @increment 0.05
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_MAN_VZ, 0.3f);

/**
 * Manual minimum takeoff height
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_MAN_HMIN, 0.5f);

/**
 * Manual maximum height
 *
 * @min 0.1
 * @max 50.0
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_MAN_HMAX, 2.0f);

/**
 * Manual input timeout
 *
 * @unit s
 * @min 0.05
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_MAN_TOUT, 0.5f);

/**
 * Vehicle mass
 *
 * @unit kg
 * @min 0.1
 * @max 20.0
 * @decimal 3
 * @increment 0.01
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_MASS, 0.62f);

/**
 * Roll moment of inertia
 *
 * @unit kg m^2
 * @min 0.00001
 * @max 1.0
 * @decimal 6
 * @increment 0.000001
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_JXX, 0.002016f);

/**
 * Pitch moment of inertia
 *
 * @unit kg m^2
 * @min 0.00001
 * @max 1.0
 * @decimal 6
 * @increment 0.000001
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_JYY, 0.001827f);

/**
 * Yaw moment of inertia
 *
 * @unit kg m^2
 * @min 0.00001
 * @max 1.0
 * @decimal 6
 * @increment 0.000001
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_JZZ, 0.00322f);

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
 * Enable L1 adaptive augmentation
 *
 * @boolean
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_INT32(L1_ADAPT_EN, 0);

/**
 * L1 velocity predictor pole
 *
 * @min -100.0
 * @max -0.1
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_AS_V, -5.0f);

/**
 * L1 angular rate predictor pole
 *
 * @min -100.0
 * @max -0.1
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_AS_OMEGA, -10.0f);

/**
 * L1 thrust first filter cutoff
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
 * L1 moment first filter cutoff
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
 * L1 moment second filter cutoff
 *
 * @unit rad/s
 * @min 0.1
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_Q2_MOM, 2.0f);

/**
 * L1 thrust limit as weight fraction
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_A_THR_FRAC, 0.35f);

/**
 * L1 roll and pitch moment limit
 *
 * @unit Nm
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_A_RP_MAX, 0.35f);

/**
 * L1 yaw moment limit
 *
 * @unit Nm
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_A_YAW_MAX, 0.20f);

/**
 * Motor thrust curve quadratic coefficient
 *
 * @min 0.0
 * @max 0.1
 * @decimal 7
 * @increment 0.000001
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_MOT_K2, 0.0009251f);

/**
 * Motor thrust curve linear coefficient
 *
 * @min 0.0
 * @max 1.0
 * @decimal 7
 * @increment 0.000001
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_MOT_K1, 0.021145f);

/**
 * Motor model maximum command
 *
 * @min 1.0
 * @max 1000.0
 * @decimal 1
 * @increment 1.0
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_MOT_CMDMAX, 100.0f);

/**
 * Roll motor arm
 *
 * @unit m
 * @min 0.01
 * @max 2.0
 * @decimal 4
 * @increment 0.001
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_ARM_ROLL, 0.175f);

/**
 * Pitch motor arm
 *
 * @unit m
 * @min 0.01
 * @max 2.0
 * @decimal 4
 * @increment 0.001
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_ARM_PITCH, 0.131f);

/**
 * Maximum yaw moment
 *
 * @unit Nm
 * @min 0.01
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_YAW_MMAX, 1.0f);
