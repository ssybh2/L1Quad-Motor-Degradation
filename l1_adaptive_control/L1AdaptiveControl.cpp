#include "L1AdaptiveControl.hpp"

#include <math.h>
#include <mathlib/mathlib.h>
#include <string.h>

namespace
{

static constexpr float GRAVITY_MSS = 9.80665f;

float dot3(const float a[3], const float b[3])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void cross3(const float a[3], const float b[3], float out[3])
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

void quat_to_rotation_matrix_body_to_ned(const float q[4], float R[3][3])
{
	const float w = q[0];
	const float x = q[1];
	const float y = q[2];
	const float z = q[3];

	R[0][0] = 1.0f - 2.0f * (y * y + z * z);
	R[0][1] = 2.0f * (x * y - w * z);
	R[0][2] = 2.0f * (x * z + w * y);

	R[1][0] = 2.0f * (x * y + w * z);
	R[1][1] = 1.0f - 2.0f * (x * x + z * z);
	R[1][2] = 2.0f * (y * z - w * x);

	R[2][0] = 2.0f * (x * z - w * y);
	R[2][1] = 2.0f * (y * z + w * x);
	R[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

float yaw_from_quat_body_to_ned(const float q[4])
{
	float R[3][3]{};
	quat_to_rotation_matrix_body_to_ned(q, R);
	return atan2f(R[1][0], R[0][0]);
}

void get_matrix_column(const float R[3][3], int col, float out[3])
{
	out[0] = R[0][col];
	out[1] = R[1][col];
	out[2] = R[2][col];
}

void copy3(const float in[3], float out[3])
{
	out[0] = in[0];
	out[1] = in[1];
	out[2] = in[2];
}

float phi_inverse_mu(float prediction_error, float as_value, float dt)
{
	const float exp_as_dt = expf(as_value * dt);
	const float denominator = exp_as_dt - 1.0f;

	if (fabsf(denominator) < 1e-5f || fabsf(as_value) < 1e-5f) {
		return 0.f;
	}

	return prediction_error / denominator * as_value * exp_as_dt;
}

}

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
PX4_INFO("L1 adaptive control init");

// 250 Hz control loop. This matches the PX4 control allocation path better than
// the earlier debug-only 10 Hz loop while keeping the module independent.
ScheduleOnInterval(4_ms);

return true;
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

apply_trajectory_command();
update_trajectory_input();
run_trajectory_generator();

update_controller_input();
run_geometric_controller();
run_l1_adaptive_augmentation();
publish_control_setpoints();

const hrt_abstime now_us = hrt_absolute_time();

if (now_us - _last_print_us > 1000000) {
print_debug_info();
_last_print_us = now_us;
}

perf_end(_loop_perf);
}

void L1AdaptiveControl::apply_parameter_values()
{
_trajectory_generator.set_takeoff_height_m(_param_l1_tkoff_hgt.get());
_trajectory_generator.set_takeoff_duration_s(_param_l1_tkoff_t.get());
_trajectory_generator.set_circle_radius_m(_param_l1_cir_radius.get());
_trajectory_generator.set_circle_speed_m_s(_param_l1_cir_speed.get());
_trajectory_generator.set_circle_transition_duration_s(_param_l1_cir_trans.get());
_trajectory_generator.set_manual_height_deadzone(_param_l1_man_dz.get());
_trajectory_generator.set_manual_max_climb_rate_m_s(_param_l1_man_vz.get());
_trajectory_generator.set_manual_min_height_m(_param_l1_man_hmin.get());
_trajectory_generator.set_manual_max_height_m(_param_l1_man_hmax.get());

GeometricController::Parameters parameters{};
parameters.mass_kg = _param_l1_mass.get();
parameters.position_gain[0] = _param_l1_kpx.get();
parameters.position_gain[1] = _param_l1_kpy.get();
parameters.position_gain[2] = _param_l1_kpz.get();
parameters.velocity_gain[0] = _param_l1_kvx.get();
parameters.velocity_gain[1] = _param_l1_kvy.get();
parameters.velocity_gain[2] = _param_l1_kvz.get();
parameters.rotation_gain[0] = _param_l1_krx.get();
parameters.rotation_gain[1] = _param_l1_kry.get();
parameters.rotation_gain[2] = _param_l1_krz.get();
parameters.angular_velocity_gain[0] = _param_l1_kox.get();
parameters.angular_velocity_gain[1] = _param_l1_koy.get();
parameters.angular_velocity_gain[2] = _param_l1_koz.get();
parameters.inertia_kg_m2[0] = _param_l1_jxx.get();
parameters.inertia_kg_m2[1] = _param_l1_jyy.get();
parameters.inertia_kg_m2[2] = _param_l1_jzz.get();
_geometric_controller.set_parameters(parameters);
}

void L1AdaptiveControl::update_subscriptions()
{
if (_vehicle_local_position_sub.update(&_vehicle_local_position)) {
_has_local_position = true;
}

if (_vehicle_attitude_sub.update(&_vehicle_attitude)) {
_has_attitude = true;
}

if (_vehicle_attitude_setpoint_sub.update(&_vehicle_attitude_setpoint)) {
_has_attitude_setpoint = true;
}

if (_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity)) {
_has_angular_velocity = true;
}

if (_manual_control_setpoint_sub.update(&_manual_control_setpoint)) {
_has_manual_control_setpoint = true;
}

_input_rc_sub.update(&_input_rc);

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
_state.quat_body_to_ned[0] = _vehicle_attitude.q[0];
_state.quat_body_to_ned[1] = _vehicle_attitude.q[1];
_state.quat_body_to_ned[2] = _vehicle_attitude.q[2];
_state.quat_body_to_ned[3] = _vehicle_attitude.q[3];

_state.attitude_valid = true;
}

if (_has_angular_velocity) {
_state.angular_velocity_body[0] = _vehicle_angular_velocity.xyz[0];
_state.angular_velocity_body[1] = _vehicle_angular_velocity.xyz[1];
_state.angular_velocity_body[2] = _vehicle_angular_velocity.xyz[2];

_state.angular_velocity_valid = true;
}

_state_valid_for_control =
_state.position_valid &&
_state.velocity_valid &&
_state.attitude_valid &&
_state.angular_velocity_valid &&
!_state.failsafe;
}

void L1AdaptiveControl::update_trajectory_input()
{
_trajectory_input.timestamp_us = _state.timestamp_us;

for (int i = 0; i < 3; i++) {
_trajectory_input.current_position_ned[i] = _state.position_ned[i];
}

_trajectory_input.current_yaw = yaw_from_quat_body_to_ned(_state.quat_body_to_ned);

_trajectory_input.state_valid_for_control = _state_valid_for_control;
_trajectory_input.armed = _state.armed && _failure_mode_selected;
_trajectory_input.failsafe = _state.failsafe;
_trajectory_input.nav_state = _state.nav_state;
_trajectory_input.initialize_in_hover = true;

update_manual_height_control_input();
_trajectory_input.manual_height_control_enabled = _failure_mode_selected || _rc_height_control_enabled.load();
_trajectory_input.manual_height_control_valid = _manual_height_control_valid;
_trajectory_input.manual_height_stick = _manual_height_stick;
}

void L1AdaptiveControl::apply_trajectory_command()
{
const uint8_t command = _trajectory_command_mode.load();

if (command == static_cast<uint8_t>(TrajectoryGenerator::CommandedMode::Circle)) {
_trajectory_generator.set_commanded_mode(TrajectoryGenerator::CommandedMode::Circle);

} else {
_trajectory_generator.set_commanded_mode(TrajectoryGenerator::CommandedMode::Hover);
}
}

void L1AdaptiveControl::run_trajectory_generator()
{
_trajectory_update_executed = _trajectory_generator.update(_trajectory_input, _trajectory_output);
}

void L1AdaptiveControl::update_manual_height_control_input()
{
	_manual_height_control_valid = false;
	_manual_height_stick = 0.f;

	if (!_failure_mode_selected && !_rc_height_control_enabled.load()) {
		return;
	}

	const hrt_abstime now_us = hrt_absolute_time();
	const hrt_abstime manual_control_timeout_us =
		static_cast<hrt_abstime>(_param_l1_man_tout.get() * 1e6f);
	const bool rc_valid = _input_rc.channel_count >= 3
			      && !_input_rc.rc_lost
			      && !_input_rc.rc_failsafe
			      && now_us - _input_rc.timestamp_last_signal <= manual_control_timeout_us;

	if (rc_valid) {
		const float rc_throttle = (_input_rc.values[2] - 1500.0f) / 500.0f;
		_manual_height_stick = math::constrain(rc_throttle, -1.0f, 1.0f);
		_manual_height_control_valid = true;
		return;
	}

	const bool joystick_valid = _has_manual_control_setpoint
				    && _manual_control_setpoint.valid
				    && PX4_ISFINITE(_manual_control_setpoint.throttle)
				    && now_us - _manual_control_setpoint.timestamp_sample <= manual_control_timeout_us;

	if (joystick_valid) {
		_manual_height_stick = math::constrain(_manual_control_setpoint.throttle, -1.0f, 1.0f);
		_manual_height_control_valid = true;
	}
}

void L1AdaptiveControl::update_failure_mode()
{
	const bool position_selected = _state.nav_state == vehicle_status_s::NAVIGATION_STATE_L1_FAILURE;
	const bool altitude_selected = _state.nav_state == vehicle_status_s::NAVIGATION_STATE_L1_FAILURE_ALT;
	const bool selected = position_selected || altitude_selected;
	const bool mode_changed = selected != _failure_mode_selected
				  || altitude_selected != _altitude_failure_mode_selected;

	if (mode_changed) {
		_failure_mode_selected = selected;
		_altitude_failure_mode_selected = altitude_selected;
		_trajectory_generator.reset();
		reset_l1_adaptive_state();
		PX4_WARN("L1 failure mode %s",
			  !selected ? "left" : (altitude_selected ? "altitude selected" : "position selected"));
	}
}

void L1AdaptiveControl::update_controller_input()
{
_controller_input.timestamp_us = _state.timestamp_us;
_controller_input.manual_tilt_enabled = false;
_controller_input.manual_desired_body_z_axis_ned[0] = 0.f;
_controller_input.manual_desired_body_z_axis_ned[1] = 0.f;
_controller_input.manual_desired_body_z_axis_ned[2] = 1.f;

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

if (_altitude_failure_mode_selected && _has_attitude_setpoint) {
	const hrt_abstime now_us = hrt_absolute_time();
	const hrt_abstime manual_control_timeout_us =
		static_cast<hrt_abstime>(_param_l1_man_tout.get() * 1e6f);
	const bool attitude_setpoint_valid =
		now_us - _vehicle_attitude_setpoint.timestamp <= manual_control_timeout_us
		&& PX4_ISFINITE(_vehicle_attitude_setpoint.q_d[0])
		&& PX4_ISFINITE(_vehicle_attitude_setpoint.q_d[1])
		&& PX4_ISFINITE(_vehicle_attitude_setpoint.q_d[2])
		&& PX4_ISFINITE(_vehicle_attitude_setpoint.q_d[3]);

	if (attitude_setpoint_valid) {
		float manual_rotation[3][3]{};
		quat_to_rotation_matrix_body_to_ned(_vehicle_attitude_setpoint.q_d, manual_rotation);
		get_matrix_column(manual_rotation, 2, _controller_input.manual_desired_body_z_axis_ned);

		if (_controller_input.manual_desired_body_z_axis_ned[2] > 0.5f) {
			_controller_input.manual_tilt_enabled = true;
		}
	}
}

// Partial Motor 1 degradation retains enough actuator authority for yaw.
_controller_input.target_yaw = _trajectory_output.yaw;
_controller_input.target_yaw_rate = _trajectory_output.yaw_rate;
_controller_input.target_yaw_accel = _trajectory_output.yaw_accel;
_controller_input.yaw_control_enabled = true;

_controller_input.state_valid_for_control = _state_valid_for_control && _trajectory_output.valid;
_controller_input.armed = _state.armed;
_controller_input.failsafe = _state.failsafe;
_controller_input.nav_state = _state.nav_state;
}

void L1AdaptiveControl::run_geometric_controller()
{
_geometric_update_executed = _geometric_controller.update(_controller_input, _geometric_output);
}

void L1AdaptiveControl::reset_l1_adaptive_state()
{
	_l1_state = L1AdaptiveState{};

	for (int i = 0; i < 4; i++) {
		_L1thrustMomentCmd[i] = 0.f;
	}
}

void L1AdaptiveControl::run_l1_adaptive_augmentation()
{
	_l1_update_executed = false;

	for (int i = 0; i < 4; i++) {
		_L1thrustMomentCmd[i] = 0.f;
		_totalThrustMomentCmd[i] = 0.f;
	}

	if (!_failure_mode_selected || !_geometric_output.valid || !_state_valid_for_control || !_state.armed
	    || _state.failsafe) {
		reset_l1_adaptive_state();
		return;
	}

	const float mass_kg = _param_l1_mass.get();
	const float inertia_kg_m2[3] = {
		_param_l1_jxx.get(),
		_param_l1_jyy.get(),
		_param_l1_jzz.get()
	};
	const float inertia_inverse[3] = {
		1.f / math::max(inertia_kg_m2[0], 1e-6f),
		1.f / math::max(inertia_kg_m2[1], 1e-6f),
		1.f / math::max(inertia_kg_m2[2], 1e-6f)
	};
	const float as_v = _param_l1_as_v.get();
	const float as_omega = _param_l1_as_omega.get();
	const float max_l1_thrust_n = mass_kg * GRAVITY_MSS * _param_l1_a_thr_frac.get();
	const float max_l1_roll_pitch_moment_nm = _param_l1_a_rp_max.get();
	const float max_l1_yaw_moment_nm = _param_l1_a_yaw_max.get();
	const bool l1_enabled = _param_l1_adapt_en.get() != 0;

	float R[3][3]{};
	quat_to_rotation_matrix_body_to_ned(_state.quat_body_to_ned, R);

	const float thrustMomentCmd[4] = {
		_geometric_output.target_thrust,
		_geometric_output.M[0],
		_geometric_output.M[1],
		_geometric_output.M[2]
	};

	if (!_l1_state.initialized) {
		copy3(_state.velocity_ned, _l1_state.v_hat_prev);
		copy3(_state.velocity_ned, _l1_state.v_prev);
		copy3(_state.angular_velocity_body, _l1_state.omega_hat_prev);
		copy3(_state.angular_velocity_body, _l1_state.omega_prev);

		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < 3; col++) {
				_l1_state.R_prev[row][col] = R[row][col];
			}
		}

		for (int i = 0; i < 4; i++) {
			_l1_state.u_b_prev[i] = thrustMomentCmd[i];
		}

		_l1_state.last_update_us = _state.timestamp_us;
		_l1_state.initialized = true;
	}

	const float dt = math::constrain((_state.timestamp_us - _l1_state.last_update_us) * 1e-6f, 0.001f, 0.02f);

	float colx_prev[3]{};
	float coly_prev[3]{};
	float colz_prev[3]{};
	get_matrix_column(_l1_state.R_prev, 0, colx_prev);
	get_matrix_column(_l1_state.R_prev, 1, coly_prev);
	get_matrix_column(_l1_state.R_prev, 2, colz_prev);

	float vpred_error_prev[3]{};
	float omegapred_error_prev[3]{};

	for (int i = 0; i < 3; i++) {
		vpred_error_prev[i] = _l1_state.v_hat_prev[i] - _l1_state.v_prev[i];
		omegapred_error_prev[i] = _l1_state.omega_hat_prev[i] - _l1_state.omega_prev[i];
	}

	const float previous_total_thrust =
		_l1_state.u_b_prev[0]
		+ _l1_state.u_ad_prev[0]
		+ _l1_state.sigma_m_hat_prev[0];

	float v_hat[3]{};

	for (int i = 0; i < 3; i++) {
		const float gravity_term = (i == 2) ? GRAVITY_MSS : 0.f;

		v_hat[i] = _l1_state.v_hat_prev[i]
				  + (gravity_term
				     - colz_prev[i] * previous_total_thrust / mass_kg
				     + colx_prev[i] * _l1_state.sigma_um_hat_prev[0] / mass_kg
				     + coly_prev[i] * _l1_state.sigma_um_hat_prev[1] / mass_kg
				     + vpred_error_prev[i] * as_v) * dt;
	}

	const float omega_prev[3] = {
		_l1_state.omega_prev[0],
		_l1_state.omega_prev[1],
		_l1_state.omega_prev[2]
	};

	const float j_omega_prev[3] = {
		inertia_kg_m2[0] * omega_prev[0],
		inertia_kg_m2[1] * omega_prev[1],
		inertia_kg_m2[2] * omega_prev[2]
	};

	float gyro_moment_prev[3]{};
	cross3(omega_prev, j_omega_prev, gyro_moment_prev);

	const float tempVec[3] = {
		_l1_state.u_b_prev[1] + _l1_state.u_ad_prev[1] + _l1_state.sigma_m_hat_prev[1],
		_l1_state.u_b_prev[2] + _l1_state.u_ad_prev[2] + _l1_state.sigma_m_hat_prev[2],
		_l1_state.u_b_prev[3] + _l1_state.u_ad_prev[3] + _l1_state.sigma_m_hat_prev[3]
	};

	float omega_hat[3]{};

	for (int i = 0; i < 3; i++) {
		omega_hat[i] = _l1_state.omega_hat_prev[i]
					  + (-inertia_inverse[i] * gyro_moment_prev[i]
					     + inertia_inverse[i] * tempVec[i]
					     + omegapred_error_prev[i] * as_omega) * dt;
	}

	float vpred_error[3]{};
	float omegapred_error[3]{};
	float PhiInvmu_v[3]{};
	float PhiInvmu_omega[3]{};

	for (int i = 0; i < 3; i++) {
		vpred_error[i] = v_hat[i] - _state.velocity_ned[i];
		omegapred_error[i] = omega_hat[i] - _state.angular_velocity_body[i];
		PhiInvmu_v[i] = phi_inverse_mu(vpred_error[i], as_v, dt);
		PhiInvmu_omega[i] = phi_inverse_mu(omegapred_error[i], as_omega, dt);
	}

	float colx[3]{};
	float coly[3]{};
	float colz[3]{};
	get_matrix_column(R, 0, colx);
	get_matrix_column(R, 1, coly);
	get_matrix_column(R, 2, colz);

	float sigma_m_hat[4]{};
	float sigma_um_hat[2]{};

	sigma_m_hat[0] = dot3(colz, PhiInvmu_v) * mass_kg;
	sigma_m_hat[1] = -inertia_kg_m2[0] * PhiInvmu_omega[0];
	sigma_m_hat[2] = -inertia_kg_m2[1] * PhiInvmu_omega[1];
	sigma_m_hat[3] = -inertia_kg_m2[2] * PhiInvmu_omega[2];

	sigma_um_hat[0] = -dot3(colx, PhiInvmu_v) * mass_kg;
	sigma_um_hat[1] = -dot3(coly, PhiInvmu_v) * mass_kg;

	const float lpf1_thrust_keep = expf(-_param_l1_q1_thr.get() * dt);
	const float lpf1_moment_keep = expf(-_param_l1_q1_mom.get() * dt);
	const float lpf2_moment_keep = expf(-_param_l1_q2_mom.get() * dt);

	float u_ad_int[4]{};
	float u_ad[4]{};

	u_ad_int[0] = lpf1_thrust_keep * _l1_state.lpf1_prev[0] + (1.f - lpf1_thrust_keep) * sigma_m_hat[0];

	for (int i = 1; i < 4; i++) {
		u_ad_int[i] = lpf1_moment_keep * _l1_state.lpf1_prev[i] + (1.f - lpf1_moment_keep) * sigma_m_hat[i];
		u_ad[i] = lpf2_moment_keep * _l1_state.lpf2_prev[i] + (1.f - lpf2_moment_keep) * u_ad_int[i];
	}

	u_ad[0] = u_ad_int[0];

	_L1thrustMomentCmd[0] = l1_enabled ? math::constrain(-u_ad[0], -max_l1_thrust_n, max_l1_thrust_n) : 0.f;
	_L1thrustMomentCmd[1] = l1_enabled ? math::constrain(-u_ad[1], -max_l1_roll_pitch_moment_nm, max_l1_roll_pitch_moment_nm) : 0.f;
	_L1thrustMomentCmd[2] = l1_enabled ? math::constrain(-u_ad[2], -max_l1_roll_pitch_moment_nm, max_l1_roll_pitch_moment_nm) : 0.f;
	_L1thrustMomentCmd[3] = l1_enabled ? math::constrain(-u_ad[3], -max_l1_yaw_moment_nm, max_l1_yaw_moment_nm) : 0.f;

	for (int i = 0; i < 4; i++) {
		_l1_state.u_b_prev[i] = thrustMomentCmd[i];
		_l1_state.u_ad_prev[i] = _L1thrustMomentCmd[i];
		_l1_state.sigma_m_hat_prev[i] = sigma_m_hat[i];
		_l1_state.lpf1_prev[i] = u_ad_int[i];
		_l1_state.lpf2_prev[i] = u_ad[i];
		_totalThrustMomentCmd[i] = thrustMomentCmd[i] + _L1thrustMomentCmd[i];
	}

	sigma_um_hat[0] = math::constrain(sigma_um_hat[0], -max_l1_thrust_n, max_l1_thrust_n);
	sigma_um_hat[1] = math::constrain(sigma_um_hat[1], -max_l1_thrust_n, max_l1_thrust_n);

	_l1_state.sigma_um_hat_prev[0] = sigma_um_hat[0];
	_l1_state.sigma_um_hat_prev[1] = sigma_um_hat[1];

	copy3(v_hat, _l1_state.v_hat_prev);
	copy3(_state.velocity_ned, _l1_state.v_prev);
	copy3(omega_hat, _l1_state.omega_hat_prev);
	copy3(_state.angular_velocity_body, _l1_state.omega_prev);

	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			_l1_state.R_prev[row][col] = R[row][col];
		}
	}

	_l1_state.last_update_us = _state.timestamp_us;
	_l1_update_executed = true;
}

void L1AdaptiveControl::publish_control_setpoints()
{
	_control_setpoint_published = false;

	if (!_geometric_output.valid || !_state_valid_for_control || !_state.armed || _state.failsafe) {
		for (int i = 0; i < 3; i++) {
			_published_thrust_body[i] = 0.f;
			_published_torque_body[i] = 0.f;
		}

		return;
	}

	const float motor_command_max = _param_l1_mot_cmdmax.get();
	const float max_motor_thrust_n =
		_param_l1_mot_k2.get() * motor_command_max * motor_command_max
		+ _param_l1_mot_k1.get() * motor_command_max;
	const float max_thrust_n = math::max(4.f * max_motor_thrust_n, 1e-3f);
	const float max_roll_moment_nm =
		math::max(_param_l1_arm_roll.get() * max_motor_thrust_n, 1e-3f);
	const float max_pitch_moment_nm =
		math::max(_param_l1_arm_pitch.get() * max_motor_thrust_n, 1e-3f);
	const float max_yaw_moment_nm = math::max(_param_l1_yaw_mmax.get(), 1e-3f);
	const float thrust_n = math::constrain(_totalThrustMomentCmd[0], 0.f, max_thrust_n);

	vehicle_thrust_setpoint_s thrust_sp{};
	thrust_sp.timestamp = hrt_absolute_time();
	thrust_sp.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;
	thrust_sp.xyz[0] = 0.f;
	thrust_sp.xyz[1] = 0.f;
	thrust_sp.xyz[2] = -math::constrain(thrust_n / max_thrust_n, 0.f, 1.f);

	vehicle_torque_setpoint_s torque_sp{};
	torque_sp.timestamp = thrust_sp.timestamp;
	torque_sp.timestamp_sample = thrust_sp.timestamp_sample;
	torque_sp.xyz[0] = math::constrain(_totalThrustMomentCmd[1] / max_roll_moment_nm, -1.f, 1.f);
	torque_sp.xyz[1] = math::constrain(_totalThrustMomentCmd[2] / max_pitch_moment_nm, -1.f, 1.f);
	torque_sp.xyz[2] = math::constrain(_totalThrustMomentCmd[3] / max_yaw_moment_nm, -1.f, 1.f);

	_vehicle_thrust_setpoint_pub.publish(thrust_sp);
	_vehicle_torque_setpoint_pub.publish(torque_sp);

	for (int i = 0; i < 3; i++) {
		_published_thrust_body[i] = thrust_sp.xyz[i];
		_published_torque_body[i] = torque_sp.xyz[i];
	}

	_control_setpoint_published = true;
	_control_setpoint_publish_count++;
}

void L1AdaptiveControl::print_debug_info()
{
PX4_INFO("L1 | armed=%d failsafe=%d nav=%u valid=%d | traj=%u valid=%d t=%.1fs",
 (int)_state.armed,
 (int)_state.failsafe,
 (unsigned)_state.nav_state,
 (int)_state_valid_for_control,
 (unsigned)_trajectory_output.mode,
 (int)_trajectory_output.valid,
 (double)_trajectory_output.elapsed_time_s);

PX4_INFO("   pos=[%.2f %.2f %.2f] vel=[%.2f %.2f %.2f] target_z=%.2f target_vz=%.2f",
 (double)_state.position_ned[0],
 (double)_state.position_ned[1],
 (double)_state.position_ned[2],
 (double)_state.velocity_ned[0],
 (double)_state.velocity_ned[1],
 (double)_state.velocity_ned[2],
 (double)_trajectory_output.position_ned[2],
 (double)_trajectory_output.velocity_ned[2]);

PX4_INFO("   rc_height=%d manual=%d valid=%d throttle=%.2f | subs lp/att/omega/status=%d/%d/%d/%d",
 (int)_rc_height_control_enabled.load(),
 (int)_has_manual_control_setpoint,
 (int)_manual_height_control_valid,
 (double)_manual_height_stick,
 (int)_has_local_position,
 (int)_has_attitude,
 (int)_has_angular_velocity,
 (int)_has_vehicle_status);

PX4_INFO("   controller: traj=%d geo=%d l1=%d publish=%d count=%u",
 (int)_trajectory_update_executed,
 (int)_geometric_update_executed,
 (int)_l1_update_executed,
 (int)_control_setpoint_published,
 (unsigned)_control_setpoint_publish_count);

PX4_INFO("   output: F=%.2fN M=[%.2f %.2f %.2f] thrust_z=%.3f torque=[%.3f %.3f %.3f]",
 (double)_totalThrustMomentCmd[0],
 (double)_totalThrustMomentCmd[1],
 (double)_totalThrustMomentCmd[2],
 (double)_totalThrustMomentCmd[3],
 (double)_published_thrust_body[2],
 (double)_published_torque_body[0],
 (double)_published_torque_body[1],
 (double)_published_torque_body[2]);
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

} else {
PX4_ERR("alloc failed");
}

delete instance;
_object.store(nullptr);
_task_id = -1;

return PX4_ERROR;
}

int L1AdaptiveControl::print_status()
{
PX4_INFO("L1 adaptive control");
PX4_INFO("  state: armed=%d failsafe=%d nav=%u valid=%d",
 (int)_state.armed,
 (int)_state.failsafe,
 (unsigned)_state.nav_state,
 (int)_state_valid_for_control);

PX4_INFO("  trajectory: mode=%u valid=%d target_z=%.3f target_vz=%.3f elapsed=%.2fs",
 (unsigned)_trajectory_output.mode,
 (int)_trajectory_output.valid,
 (double)_trajectory_output.position_ned[2],
 (double)_trajectory_output.velocity_ned[2],
 (double)_trajectory_output.elapsed_time_s);

PX4_INFO("  trajectory command: %s radius=%.2fm speed=%.2fm/s",
 trajectory_mode() == TrajectoryGenerator::CommandedMode::Circle ? "circle" : "hover",
 (double)_trajectory_generator.circle_radius_m(),
 (double)_trajectory_generator.circle_speed_m_s());

PX4_INFO("  rc height: enabled=%d received=%d valid=%d throttle=%.3f",
 (int)_rc_height_control_enabled.load(),
 (int)_has_manual_control_setpoint,
 (int)_manual_height_control_valid,
 (double)_manual_height_stick);

PX4_INFO("  pipeline: traj=%d geo=%d l1=%d publish=%d count=%u",
 (int)_trajectory_update_executed,
 (int)_geometric_update_executed,
 (int)_l1_update_executed,
 (int)_control_setpoint_published,
 (unsigned)_control_setpoint_publish_count);

PX4_INFO("  subscriptions: local_pos=%d attitude=%d angular_vel=%d manual=%d status=%d",
 (int)_has_local_position,
 (int)_has_attitude,
 (int)_has_angular_velocity,
 (int)_has_manual_control_setpoint,
 (int)_has_vehicle_status);

perf_print_counter(_loop_perf);
perf_print_counter(_loop_interval_perf);

return 0;
}

void L1AdaptiveControl::set_rc_height_control_enabled(bool enabled)
{
_rc_height_control_enabled.store(enabled);
_manual_height_control_valid = false;
_manual_height_stick = 0.f;
}

void L1AdaptiveControl::set_trajectory_mode(TrajectoryGenerator::CommandedMode mode)
{
_trajectory_command_mode.store(static_cast<uint8_t>(mode));
}

int L1AdaptiveControl::custom_command(int argc, char *argv[])
{
if (argc >= 1 && !strcmp(argv[0], "rc_control")) {
if (!is_running()) {
PX4_ERR("module not running");
return -1;
}

L1AdaptiveControl *instance = get_instance();

if (instance == nullptr) {
PX4_ERR("module instance unavailable");
return -1;
}

if (argc < 2 || !strcmp(argv[1], "status")) {
PX4_INFO("RC height control: %s", instance->rc_height_control_enabled() ? "enabled" : "disabled");
PX4_INFO("  manual: received=%d valid=%d throttle=%.3f",
 (int)instance->_has_manual_control_setpoint,
 (int)instance->_manual_height_control_valid,
 (double)instance->_manual_height_stick);
PX4_INFO("  trajectory: mode=%u target_z=%.3f target_vz=%.3f",
 (unsigned)instance->_trajectory_output.mode,
 (double)instance->_trajectory_output.position_ned[2],
 (double)instance->_trajectory_output.velocity_ned[2]);
return 0;
}

if (!strcmp(argv[1], "enable") || !strcmp(argv[1], "1") || !strcmp(argv[1], "true")) {
instance->set_rc_height_control_enabled(true);
PX4_INFO("RC height control enabled");
return 0;
}

if (!strcmp(argv[1], "disable") || !strcmp(argv[1], "0") || !strcmp(argv[1], "false")) {
instance->set_rc_height_control_enabled(false);
PX4_INFO("RC height control disabled");
return 0;
}

return print_usage("unknown rc_control argument");
}

if (argc >= 1 && !strcmp(argv[0], "trajectory")) {
if (!is_running()) {
PX4_ERR("module not running");
return -1;
}

L1AdaptiveControl *instance = get_instance();

if (instance == nullptr) {
PX4_ERR("module instance unavailable");
return -1;
}

if (argc < 2 || !strcmp(argv[1], "status")) {
PX4_INFO("Trajectory command: %s",
 instance->trajectory_mode() == TrajectoryGenerator::CommandedMode::Circle ? "circle" : "hover");
PX4_INFO("  radius: %.2fm speed: %.2fm/s",
	 (double)instance->_trajectory_generator.circle_radius_m(),
	 (double)instance->_trajectory_generator.circle_speed_m_s());
PX4_INFO("  output: mode=%u valid=%d pos=[%.3f %.3f %.3f] vel=[%.3f %.3f %.3f] yaw=%.3f",
 (unsigned)instance->_trajectory_output.mode,
 (int)instance->_trajectory_output.valid,
 (double)instance->_trajectory_output.position_ned[0],
 (double)instance->_trajectory_output.position_ned[1],
 (double)instance->_trajectory_output.position_ned[2],
 (double)instance->_trajectory_output.velocity_ned[0],
 (double)instance->_trajectory_output.velocity_ned[1],
 (double)instance->_trajectory_output.velocity_ned[2],
 (double)instance->_trajectory_output.yaw);
return 0;
}

if (!strcmp(argv[1], "hover")) {
instance->set_trajectory_mode(TrajectoryGenerator::CommandedMode::Hover);
PX4_INFO("Trajectory command set to hover");
return 0;
}

if (!strcmp(argv[1], "circle")) {
instance->set_trajectory_mode(TrajectoryGenerator::CommandedMode::Circle);
PX4_INFO("Trajectory command set to circle");
return 0;
}

return print_usage("unknown trajectory argument");
}

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
L1 adaptive control module skeleton for PX4 v1.17.0.

Current stage:
- Subscribe vehicle_local_position
- Subscribe vehicle_attitude
- Subscribe vehicle_angular_velocity
- Subscribe manual_control_setpoint
- Subscribe vehicle_status
- Convert uORB messages into internal controller state
- Generate takeoff-hover/circle trajectory
- Convert trajectory output into GeometricController input
- Run geometric controller
- Run L1 adaptive augmentation
- Publish vehicle_thrust_setpoint and vehicle_torque_setpoint for PX4 control_allocator
- Switch post-takeoff trajectory between hover and fixed-yaw circle
)DESCR_STR");

PRINT_MODULE_USAGE_NAME("l1_adaptive_control", "controller");
PRINT_MODULE_USAGE_COMMAND("start");
PRINT_MODULE_USAGE_COMMAND("status");
PRINT_MODULE_USAGE_COMMAND_DESCR("rc_control", "enable/disable/status optional RC throttle height control");
PRINT_MODULE_USAGE_COMMAND_DESCR("trajectory", "hover/circle/status trajectory command");
PRINT_MODULE_USAGE_COMMAND("stop");
PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

return 0;
}

extern "C" __EXPORT int l1_adaptive_control_main(int argc, char *argv[])
{
return L1AdaptiveControl::main(argc, argv);
}
