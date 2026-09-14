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
 * Motor 1 command scale applied after the original ModeAdaptive motor mixer
 *
 * 1.0 is source-equivalent (no degradation). Values below 1.0 inject an
 * external Motor 1 degradation after the original nonlinear mixer.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1_MOT1_SCALE, 1.0f);

/** Original CIRCRADIUSX parameter. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_CIR_RADIUS, 2.0f);

/** Original CIRCRADIUSY parameter. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_CIR_RAD_Y, 1.0f);

/** Original CIRCSPEED parameter. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_CIR_SPEED, 0.3f);

/** Original TRAJINDEX parameter: 0 hover, 1/2 circle, 3/4 figure-eight. @group L1 Adaptive Control */
PARAM_DEFINE_INT32(L1_TRAJ_IDX, 0);

/** Original LANDFLAG parameter. @boolean @group L1 Adaptive Control */
PARAM_DEFINE_INT32(L1_LAND_FLAG, 0);

/** Original real-aircraft GeoCtrl_Kpx default. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_KPX, 14.0f);
/** Original real-aircraft GeoCtrl_Kpy default. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_KPY, 15.0f);
/** Original real-aircraft GeoCtrl_Kpz default. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_KPZ, 15.0f);
/** Original real-aircraft GeoCtrl_Kvx default. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_KVX, 1.5f);
/** Original real-aircraft GeoCtrl_Kvy default. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_KVY, 0.9f);
/** Original real-aircraft GeoCtrl_Kvz default. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_KVZ, 1.1f);
/** Original real-aircraft GeoCtrl_KRx default. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_KRX, 0.55f);
/** Original real-aircraft GeoCtrl_KRy default. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_KRY, 0.35f);
/** Original real-aircraft GeoCtrl_KRz default. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_KRZ, 0.15f);
/** Original real-aircraft GeoCtrl_KOx default. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_KOX, 0.035f);
/** Original real-aircraft GeoCtrl_KOy default. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_KOY, 0.03f);
/** Original real-aircraft GeoCtrl_KOz default. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_KOZ, 0.004f);

/** Original L1ENABLE parameter. @boolean @group L1 Adaptive Control */
PARAM_DEFINE_INT32(L1_ADAPT_EN, 0);
/** Original ASV parameter. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_AS_V, -5.0f);
/** Original ASOMEGA parameter. @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_AS_OMEGA, -10.0f);
/** Original CTOFFQ1THRUST parameter. @unit rad/s @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_Q1_THR, 10.0f);
/** Original CTOFFQ1MOMENT parameter. @unit rad/s @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_Q1_MOM, 10.0f);
/** Original CTOFFQ2MOMENT parameter. @unit rad/s @group L1 Adaptive Control */
PARAM_DEFINE_FLOAT(L1_Q2_MOM, 2.0f);
