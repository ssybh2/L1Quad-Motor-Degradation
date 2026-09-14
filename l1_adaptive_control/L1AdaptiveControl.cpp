#include "L1AdaptiveControl.hpp"

#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

#include <math.h>
#include <string.h>

namespace
{

float yaw_from_quat_body_to_ned(const float q[4])
{
	return matrix::Eulerf(matrix::Quatf(q)).psi();
}

const char *trajectory_name(uint8_t index)
{
	switch (index) {
	case 1: return "circle_variable_yaw";
	case 2: return "circle_fixed_yaw";
	case 3: return "figure8_fixed_yaw";
	case 4: return "figure8_tilted";
	default: return "hover(source default)";
	}
}

} // namespace

L1AdaptiveControl::L1AdaptiveControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();
	apply_parameter_values();
}

L1AdaptiveControl::~L1AdaptiveControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool L1AdaptiveControl::init()
{
	PX4_INFO("ModeAdaptive source-equivalent controller init (%s)", REAL_OR_SITL ? "REAL" : "SITL");

	// ModeAdaptive::L1AdaptiveAugmentation() is explicitly discretized at
	// dt = 0.0025 s, so run the complete source-equivalent path at 400 Hz.
	ScheduleOnInterval(2500);
	return true;
}

void L1AdaptiveControl::apply_parameter_values()
{
	GeometricController::Parameters geometric_parameters{};
	L1AdaptiveAugmentation::Parameters l1_parameters{};
	MotorMixer::Parameters mixer_parameters{};

#if (!REAL_OR_SITL)
	geometric_parameters.mass_kg = 3.f;
	geometric_parameters.inertia_kg_m2[0] = 0.023f;
	geometric_parameters.inertia_kg_m2[1] = 0.023f;
	geometric_parameters.inertia_kg_m2[2] = 0.0459f;

	l1_parameters.mass_kg = 3.f;
	l1_parameters.inertia_kg_m2[0] = 0.023f;
	l1_parameters.inertia_kg_m2[1] = 0.023f;
	l1_parameters.inertia_kg_m2[2] = 0.0459f;
	l1_parameters.inertia_inverse[0] = 43.478f;
	l1_parameters.inertia_inverse[1] = 43.478f;
	l1_parameters.inertia_inverse[2] = 21.786f;
	mixer_parameters.real_vehicle = false;
#else
	// These are the literal REAL_OR_SITL constants in L1Quad mode.h.
	geometric_parameters.mass_kg = 0.62f;
	geometric_parameters.inertia_kg_m2[0] = 0.002016f;
	geometric_parameters.inertia_kg_m2[1] = 0.001827f;
	geometric_parameters.inertia_kg_m2[2] = 0.00322f;

	l1_parameters.mass_kg = 0.62f;
	l1_parameters.inertia_kg_m2[0] = 0.002016f;
	l1_parameters.inertia_kg_m2[1] = 0.001827f;
	l1_parameters.inertia_kg_m2[2] = 0.00322f;
	l1_parameters.inertia_inverse[0] = 496.03f;
	l1_parameters.inertia_inverse[1] = 547.345f;
	l1_parameters.inertia_inverse[2] = 310.559f;
	mixer_parameters.real_vehicle = true;
#endif

	// The original controller reads these gains directly from g.GeoCtrl_*.
	geometric_parameters.position_gain[0] = _param_l1_kpx.get();
	geometric_parameters.position_gain[1] = _param_l1_kpy.get();
	geometric_parameters.position_gain[2] = _param_l1_kpz.get();
	geometric_parameters.velocity_gain[0] = _param_l1_kvx.get();
	geometric_parameters.velocity_gain[1] = _param_l1_kvy.get();
	geometric_parameters.velocity_gain[2] = _param_l1_kvz.get();
	geometric_parameters.rotation_gain[0] = _param_l1_krx.get();
	geometric_parameters.rotation_gain[1] = _param_l1_kry.get();
	geometric_parameters.rotation_gain[2] = _param_l1_krz.get();
	geometric_parameters.angular_velocity_gain[0] = _param_l1_kox.get();
	geometric_parameters.angular_velocity_gain[1] = _param_l1_koy.get();
	geometric_parameters.angular_velocity_gain[2] = _param_l1_koz.get();

	l1_parameters.as_v = _param_l1_as_v.get();
	l1_parameters.as_omega = _param_l1_as_omega.get();
	l1_parameters.cutoff_q1_thrust = _param_l1_q1_thr.get();
	l1_parameters.cutoff_q1_moment = _param_l1_q1_mom.get();
	l1_parameters.cutoff_q2_moment = _param_l1_q2_mom.get();
	l1_parameters.l1_enable = static_cast<int8_t>(_param_l1_adapt_en.get() != 0);

	_geometric_controller.set_parameters(geometric_parameters);
	_l1_adaptive_augmentation.set_parameters(l1_parameters);
	_motor_mixer.set_parameters(mixer_parameters);

	// ModeAdaptive::init() snapshots trajectory index/radius/speed.  Do not
	// change an active source trajectory just because PX4 parameters refresh.
	if (!_failure_mode_selected) {
		TrajectoryGenerator::Parameters trajectory_parameters{};
		trajectory_parameters.radius_x = _param_l1_cir_radius.get();
		trajectory_parameters.radius_y = _param_l1_cir_rad_y.get();
		trajectory_parameters.target_speed = _param_l1_cir_speed.get();
		trajectory_parameters.trajectory_index = static_cast<uint8_t>(math::constrain(_param_l1_traj_idx.get(), 0, 4));
		_trajectory_generator.set_parameters(trajectory_parameters);
	}
}

void L1AdaptiveControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update{};
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
		apply_parameter_values();
	}

	update_subscriptions();
	update_internal_state();
	update_failure_mode();

	// The module is started with PX4, but owns motor outputs only while one of
	// the two L1 failure navigation states is selected.
	if (!_failure_mode_selected) {
		perf_end(_loop_perf);
		return;
	}

	update_trajectory_input();
	run_trajectory_generator();
	update_controller_input();
	run_geometric_controller();
	run_l1_adaptive_augmentation();
	run_motor_mixer();
	apply_motor_degradation();
	publish_motor_commands();

	const hrt_abstime now_us = hrt_absolute_time();
	if (now_us - _last_print_us > 1000000) {
		print_debug_info();
		_last_print_us = now_us;
	}

	perf_end(_loop_perf);
}

void L1AdaptiveControl::update_subscriptions()
{
	if (_vehicle_local_position_sub.update(&_vehicle_local_position)) {
		_has_local_position = true;
	}

	if (_vehicle_attitude_sub.update(&_vehicle_attitude)) {
		_has_attitude = true;
	}

	if (_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity)) {
		_has_angular_velocity = true;
	}

	if (_vehicle_status_sub.update(&_vehicle_status)) {
		_has_vehicle_status = true;
	}
}

void L1AdaptiveControl::update_internal_state()
{
	_state.timestamp_us = hrt_absolute_time();

	if (_has_vehicle_status) {
		_state.arming_state = _vehicle_status.arming_state;
		_state.nav_state = _vehicle_status.nav_state;
		_state.failsafe = _vehicle_status.failsafe;
		_state.armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	}

	if (_has_local_position) {
		_state.position_ned[0] = _vehicle_local_position.x;
		_state.position_ned[1] = _vehicle_local_position.y;
		_state.position_ned[2] = _vehicle_local_position.z;
		_state.velocity_ned[0] = _vehicle_local_position.vx;
		_state.velocity_ned[1] = _vehicle_local_position.vy;
		_state.velocity_ned[2] = _vehicle_local_position.vz;
		_state.position_valid = _vehicle_local_position.xy_valid && _vehicle_local_position.z_valid;
		_state.velocity_valid = _vehicle_local_position.v_xy_valid && _vehicle_local_position.v_z_valid;
	}

	if (_has_attitude) {
		for (int i = 0; i < 4; i++) {
			_state.quat_body_to_ned[i] = _vehicle_attitude.q[i];
		}
		_state.attitude_valid = true;
	}

	if (_has_angular_velocity) {
		for (int i = 0; i < 3; i++) {
			_state.angular_velocity_body[i] = _vehicle_angular_velocity.xyz[i];
		}
		_state.angular_velocity_valid = true;
	}

	_state_valid_for_control = _state.position_valid
				   && _state.velocity_valid
				   && _state.attitude_valid
				   && _state.angular_velocity_valid
				   && !_state.failsafe;
}

void L1AdaptiveControl::update_failure_mode()
{
	const bool selected = _state.nav_state == vehicle_status_s::NAVIGATION_STATE_L1_FAILURE
			      || _state.nav_state == vehicle_status_s::NAVIGATION_STATE_L1_FAILURE_ALT;

	if (selected == _failure_mode_selected) {
		return;
	}

	_failure_mode_selected = selected;

	// Equivalent to ModeAdaptive::init() on entry: reset the run-local landing
	// and adaptive states.  L1AdaptiveAugmentation::reset() intentionally keeps
	// the source sigma estimates because upstream init() does not clear them.
	_trajectory_generator.reset();
	_l1_adaptive_augmentation.reset();
	_motor_mix_executed = false;
	_motor_command = MotorMixer::MotorCommand{};
	_degraded_motor_command = MotorMixer::MotorCommand{};

	if (selected) {
		// Snapshot the same trajectory parameters that ModeAdaptive::init() reads.
		TrajectoryGenerator::Parameters trajectory_parameters{};
		trajectory_parameters.radius_x = _param_l1_cir_radius.get();
		trajectory_parameters.radius_y = _param_l1_cir_rad_y.get();
		trajectory_parameters.target_speed = _param_l1_cir_speed.get();
		trajectory_parameters.trajectory_index = static_cast<uint8_t>(math::constrain(_param_l1_traj_idx.get(), 0, 4));
		_trajectory_generator.set_parameters(trajectory_parameters);
	}

	PX4_WARN("ModeAdaptive source path %s", selected ? "active" : "inactive");
}

void L1AdaptiveControl::update_trajectory_input()
{
	_trajectory_input = TrajectoryGenerator::Input{};
	_trajectory_input.timestamp_us = _state.timestamp_us;

	for (int i = 0; i < 3; i++) {
		_trajectory_input.current_position_ned[i] = _state.position_ned[i];
		_trajectory_input.current_velocity_ned[i] = _state.velocity_ned[i];
	}

	_trajectory_input.current_yaw = yaw_from_quat_body_to_ned(_state.quat_body_to_ned);
	_trajectory_input.land_flag = _param_l1_land_flag.get() != 0;
	_trajectory_input.state_valid_for_control = _state_valid_for_control;
	_trajectory_input.armed = _state.armed;
	_trajectory_input.failsafe = _state.failsafe;
}

void L1AdaptiveControl::run_trajectory_generator()
{
	_trajectory_update_executed = _trajectory_generator.update(_trajectory_input, _trajectory_output);
}

void L1AdaptiveControl::update_controller_input()
{
	_controller_input = GeometricController::Input{};
	_controller_input.timestamp_us = _state.timestamp_us;

	for (int i = 0; i < 3; i++) {
		_controller_input.position_ned[i] = _state.position_ned[i];
		_controller_input.velocity_ned[i] = _state.velocity_ned[i];
		_controller_input.angular_velocity_body[i] = _state.angular_velocity_body[i];
		_controller_input.target_position_ned[i] = _trajectory_output.position_ned[i];
		_controller_input.target_velocity_ned[i] = _trajectory_output.velocity_ned[i];
		_controller_input.target_acceleration_ned[i] = _trajectory_output.acceleration_ned[i];
		_controller_input.target_jerk_ned[i] = _trajectory_output.jerk_ned[i];
		_controller_input.target_snap_ned[i] = _trajectory_output.snap_ned[i];
	}

	for (int i = 0; i < 4; i++) {
		_controller_input.quat_body_to_ned[i] = _state.quat_body_to_ned[i];
	}

	for (int i = 0; i < 2; i++) {
		_controller_input.target_yaw[i] = _trajectory_output.yaw[i];
		_controller_input.target_yaw_dot[i] = _trajectory_output.yaw_dot[i];
		_controller_input.target_yaw_ddot[i] = _trajectory_output.yaw_ddot[i];
	}

	_controller_input.state_valid_for_control = _state_valid_for_control && _trajectory_output.valid;
	_controller_input.armed = _state.armed;
	_controller_input.failsafe = _state.failsafe;
	_controller_input.nav_state = _state.nav_state;
}

void L1AdaptiveControl::run_geometric_controller()
{
	_geometric_update_executed = _geometric_controller.update(_controller_input, _geometric_output);
	_baseline_thrust_moment[0] = _geometric_output.target_thrust;
	_baseline_thrust_moment[1] = _geometric_output.M[0];
	_baseline_thrust_moment[2] = _geometric_output.M[1];
	_baseline_thrust_moment[3] = _geometric_output.M[2];
}

void L1AdaptiveControl::run_l1_adaptive_augmentation()
{
	L1AdaptiveAugmentation::Input l1_input{};
	l1_input.timestamp_us = _state.timestamp_us;
	l1_input.baseline_valid = _geometric_output.valid;
	l1_input.state_valid = _state_valid_for_control;
	l1_input.armed = _state.armed;
	l1_input.failsafe = _state.failsafe;

	for (int i = 0; i < 3; i++) {
		l1_input.velocity_ned[i] = _state.velocity_ned[i];
		l1_input.angular_velocity_body[i] = _state.angular_velocity_body[i];
	}

	for (int i = 0; i < 4; i++) {
		l1_input.quat_body_to_ned[i] = _state.quat_body_to_ned[i];
		l1_input.baseline_thrust_moment[i] = _baseline_thrust_moment[i];
	}

	_l1_update_executed = _l1_adaptive_augmentation.update(l1_input, _l1_output);

	for (int i = 0; i < 4; i++) {
		_combined_thrust_moment[i] = _baseline_thrust_moment[i] + _l1_output.adaptive_thrust_moment[i];
	}
}

void L1AdaptiveControl::run_motor_mixer()
{
	_motor_mix_executed = false;
	_motor_command = MotorMixer::MotorCommand{};

	if (!_state_valid_for_control || !_state.armed || _state.failsafe
	    || !_geometric_output.valid || !_l1_update_executed) {
		return;
	}

	MotorMixer::ThrustMoment thrust_moment_cmd{};
	for (int i = 0; i < 4; i++) {
		thrust_moment_cmd(i) = _combined_thrust_moment[i];
	}

	_motor_mix_executed = _motor_mixer.mix(thrust_moment_cmd, _motor_command);
	if (!_motor_mix_executed) {
		return;
	}

	// Literal source saturation after motorMixing().
	for (int i = 0; i < 4; i++) {
		if (_motor_command(i) < 0.f) {
			_motor_command(i) = 0.f;
		} else if (_motor_command(i) > 100.f) {
			_motor_command(i) = 100.f;
		}
	}

	if (_trajectory_output.landing_complete) {
		for (int i = 0; i < 4; i++) {
			_motor_command(i) = 1.f;
		}
	}
}

void L1AdaptiveControl::apply_motor_degradation()
{
	_degraded_motor_command = _motor_command;

	if (!_motor_mix_executed) {
		return;
	}

	// Fault injection is deliberately outside the ModeAdaptive control math.
	// This represents actuator degradation after the original nonlinear mixer.
	const float motor_1_scale = math::constrain(_param_l1_mot1_scale.get(), 0.f, 1.f);
	_degraded_motor_command(0) *= motor_1_scale;
}

void L1AdaptiveControl::publish_motor_commands()
{
	_actuator_motors_published = false;

	if (!_failure_mode_selected) {
		return;
	}

	actuator_motors_s actuator_motors{};
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;
	actuator_motors.reversible_flags = 0;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		actuator_motors.control[i] = NAN;
	}

	if (_motor_mix_executed && _state.armed && !_state.failsafe) {
		for (int i = 0; i < 4; i++) {
			// ArduPilot source maps 0..100 to 1000..2000 us.  actuator_motors
			// carries the same final motor command through PX4 as 0..1.
			_published_motor_control[i] = _degraded_motor_command(i) * 0.01f;
			actuator_motors.control[i] = _published_motor_control[i];
		}
	}

	_actuator_motors_pub.publish(actuator_motors);
	_actuator_motors_published = true;
	_actuator_publish_count++;
}

void L1AdaptiveControl::print_debug_info()
{
	PX4_INFO("L1src | armed=%d valid=%d traj=%s t=%.2f adapt=%d scale=%.2f",
		 (int)_state.armed, (int)_state_valid_for_control,
		 trajectory_name(_trajectory_generator.parameters().trajectory_index),
		 (double)_trajectory_output.time_in_this_run_s,
		 (int)(_param_l1_adapt_en.get() != 0),
		 (double)_param_l1_mot1_scale.get());

	PX4_INFO("  ub=[%.2f %.3f %.3f %.3f] uad=[%.2f %.3f %.3f %.3f]",
		 (double)_baseline_thrust_moment[0], (double)_baseline_thrust_moment[1],
		 (double)_baseline_thrust_moment[2], (double)_baseline_thrust_moment[3],
		 (double)_l1_output.adaptive_thrust_moment[0], (double)_l1_output.adaptive_thrust_moment[1],
		 (double)_l1_output.adaptive_thrust_moment[2], (double)_l1_output.adaptive_thrust_moment[3]);

	PX4_INFO("  source_motor%%=[%.2f %.2f %.2f %.2f] degraded=[%.2f %.2f %.2f %.2f]",
		 (double)_motor_command(0), (double)_motor_command(1), (double)_motor_command(2), (double)_motor_command(3),
		 (double)_degraded_motor_command(0), (double)_degraded_motor_command(1),
		 (double)_degraded_motor_command(2), (double)_degraded_motor_command(3));
}

int L1AdaptiveControl::task_spawn(int argc, char *argv[])
{
	L1AdaptiveControl *instance = new L1AdaptiveControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int L1AdaptiveControl::print_status()
{
	PX4_INFO("ModeAdaptive source-equivalent PX4 port");
	PX4_INFO("  profile: %s, loop: 400 Hz", REAL_OR_SITL ? "REAL" : "SITL");
	PX4_INFO("  selected=%d armed=%d failsafe=%d valid=%d",
		 (int)_failure_mode_selected, (int)_state.armed, (int)_state.failsafe, (int)_state_valid_for_control);
	PX4_INFO("  trajectory=%s radius=[%.2f %.2f] speed=%.2f land=%d",
		 trajectory_name(_trajectory_generator.parameters().trajectory_index),
		 (double)_trajectory_generator.parameters().radius_x,
		 (double)_trajectory_generator.parameters().radius_y,
		 (double)_trajectory_generator.parameters().target_speed,
		 (int)(_param_l1_land_flag.get() != 0));
	PX4_INFO("  motor1 degradation scale=%.2f", (double)_param_l1_mot1_scale.get());
	PX4_INFO("  actuator_motors publish=%d count=%u", (int)_actuator_motors_published, (unsigned)_actuator_publish_count);
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int L1AdaptiveControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int L1AdaptiveControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
PX4 v1.17 wrapper around the original L1Quad ModeAdaptive control path.
The numerical path is kept in source order:
ACRL trajectory -> geometricController -> L1AdaptiveAugmentation -> motorMixing -> actuator_motors.
Motor 1 degradation is applied only after the original mixer.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("l1_adaptive_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int l1_adaptive_control_main(int argc, char *argv[])
{
	return L1AdaptiveControl::main(argc, argv);
}
