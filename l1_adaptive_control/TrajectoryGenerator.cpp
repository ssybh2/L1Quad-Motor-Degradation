#include "TrajectoryGenerator.hpp"

#include <math.h>
#include <mathlib/mathlib.h>

namespace
{

static constexpr float PI_F = 3.14159265358979323846f;
static constexpr float TWO_PI_F = 2.f * PI_F;

void copy3(const float in[3], float out[3])
{
out[0] = in[0];
out[1] = in[1];
out[2] = in[2];
}

}

bool TrajectoryGenerator::update(const Input &input, Output &output)
{
_last_input = input;

output.timestamp_us = input.timestamp_us;

set_zero_derivatives(output);

// Safety gate:
// If the PX4 state is not valid for control or the vehicle is disarmed, only
// mirror current position for debug and reset the trajectory start point.
// Do not generate a valid trajectory.
if (!input.state_valid_for_control || !input.armed || input.failsafe) {
reset();

output.position_ned[0] = input.current_position_ned[0];
output.position_ned[1] = input.current_position_ned[1];
output.position_ned[2] = input.current_position_ned[2];

output.yaw = input.current_yaw;
output.elapsed_time_s = 0.f;
output.mode = Mode::WaitForValidState;
output.valid = false;

_last_output = output;
return true;
}

// Initialize trajectory start point on the first valid state.
if (!_initialized) {
_start_time_us = input.timestamp_us;
_last_update_us = input.timestamp_us;

_start_position_ned[0] = input.current_position_ned[0];
_start_position_ned[1] = input.current_position_ned[1];
_start_position_ned[2] = input.current_position_ned[2];

copy3(_start_position_ned, _takeoff_target_position_ned);
_skip_takeoff = input.initialize_in_hover;

if (!_skip_takeoff) {
	// NED convention: z becomes more negative when the vehicle moves upward.
	_takeoff_target_position_ned[2] = _start_position_ned[2] - _takeoff_height_m;
}

copy3(_takeoff_target_position_ned, _hover_position_ned);
copy3(_hover_position_ned, _circle_center_position_ned);

_start_yaw = input.current_yaw;

_initialized = true;
}

const float elapsed_s = math::max((input.timestamp_us - _start_time_us) * 1e-6f, 0.f);
output.elapsed_time_s = elapsed_s;

output.yaw = _start_yaw;
output.yaw_rate = 0.f;
output.yaw_accel = 0.f;

if (!_skip_takeoff && elapsed_s < _takeoff_duration_s) {
output.mode = Mode::Takeoff;
_manual_hold_initialized = false;

const float s = math::constrain(elapsed_s / _takeoff_duration_s, 0.f, 1.f);

// Smooth cubic trajectory:
// h(s) = 3s^2 - 2s^3
// h_dot = (6s - 6s^2) / T
// h_ddot = (6 - 12s) / T^2
// h_jerk = -12 / T^3
const float h = 3.f * s * s - 2.f * s * s * s;
const float h_dot = (6.f * s - 6.f * s * s) / _takeoff_duration_s;
const float h_ddot = (6.f - 12.f * s) / (_takeoff_duration_s * _takeoff_duration_s);
const float h_jerk = -12.f / (_takeoff_duration_s * _takeoff_duration_s * _takeoff_duration_s);

for (int i = 0; i < 3; i++) {
const float delta = _takeoff_target_position_ned[i] - _start_position_ned[i];

output.position_ned[i] = _start_position_ned[i] + delta * h;
output.velocity_ned[i] = delta * h_dot;
output.acceleration_ned[i] = delta * h_ddot;
output.jerk_ned[i] = delta * h_jerk;
output.snap_ned[i] = 0.f;
}

} else {
const float commanded_speed_m_s =
	_commanded_mode == CommandedMode::Circle ? _circle_speed_m_s : 0.f;
update_circle_target(input, output, commanded_speed_m_s);
}

_last_update_us = input.timestamp_us;
output.valid = true;

_last_output = output;

return true;
}

void TrajectoryGenerator::reset()
{
_initialized = false;
_skip_takeoff = false;
_start_time_us = 0;
_last_update_us = 0;
_manual_hold_initialized = false;
reset_circle_state();
}

void TrajectoryGenerator::set_commanded_mode(CommandedMode mode)
{
if (_commanded_mode == mode) {
return;
}

// Speed zero is hover. Preserve the current target position when stopping the
// circle so the reference does not jump back to the original takeoff point.
if (mode == CommandedMode::Hover && _last_output.valid) {
sync_hover_reference_from_output(_last_output);
}

_commanded_mode = mode;
reset_circle_state();
}

void TrajectoryGenerator::set_takeoff_height_m(float height_m)
{
if (std::isfinite(height_m)) {
_takeoff_height_m = math::constrain(height_m, 0.1f, 10.f);
}
}

void TrajectoryGenerator::set_takeoff_duration_s(float duration_s)
{
if (std::isfinite(duration_s)) {
_takeoff_duration_s = math::constrain(duration_s, 0.1f, 20.f);
}
}

void TrajectoryGenerator::set_circle_radius_m(float radius_m)
{
if (!std::isfinite(radius_m)) {
return;
}

const float constrained_radius_m = math::constrain(radius_m, MIN_CIRCLE_RADIUS_M, MAX_CIRCLE_RADIUS_M);

if (fabsf(constrained_radius_m - _circle_radius_m) < 1e-4f) {
return;
}

_circle_radius_m = constrained_radius_m;

// If the radius changes during a circle, preserve the current setpoint and
// smoothly transition to the new radius around the same circle center.
if (_circle_initialized && _last_output.valid) {
copy3(_last_output.position_ned, _circle_transition_start_position_ned);
copy3(_circle_center_position_ned, _circle_start_position_ned);
_circle_start_position_ned[1] -= _circle_radius_m;
_circle_start_position_ned[2] = _circle_center_position_ned[2];
_circle_current_speed_rad_s = _circle_speed_m_s / _circle_radius_m;
_circle_transition_start_time_s = _last_output.elapsed_time_s;
_circle_orbit_start_time_s = _circle_transition_start_time_s + _circle_transition_duration_s;
}
}

void TrajectoryGenerator::set_circle_speed_m_s(float speed_m_s)
{
if (!std::isfinite(speed_m_s)) {
return;
}

const float constrained_speed_m_s = math::constrain(speed_m_s, 0.05f, 2.f);

if (fabsf(constrained_speed_m_s - _circle_speed_m_s) < 1e-4f) {
return;
}

if (_circle_initialized && _last_output.valid && _last_output.mode == Mode::Circle) {
const float old_phase_rad = _circle_current_speed_rad_s
			    * math::max(_last_output.elapsed_time_s - _circle_orbit_start_time_s, 0.f);
const float new_speed_rad_s = constrained_speed_m_s / _circle_radius_m;
_circle_orbit_start_time_s = _last_output.elapsed_time_s - old_phase_rad / new_speed_rad_s;
_circle_current_speed_rad_s = new_speed_rad_s;

} else if (_circle_initialized) {
_circle_current_speed_rad_s = constrained_speed_m_s / _circle_radius_m;
}

_circle_speed_m_s = constrained_speed_m_s;
}

void TrajectoryGenerator::set_circle_transition_duration_s(float duration_s)
{
if (std::isfinite(duration_s)) {
_circle_transition_duration_s = math::constrain(duration_s, 0.1f, 20.f);
}
}

void TrajectoryGenerator::set_manual_height_deadzone(float deadzone)
{
if (std::isfinite(deadzone)) {
_manual_height_deadzone = math::constrain(deadzone, 0.f, 0.5f);
}
}

void TrajectoryGenerator::set_manual_max_climb_rate_m_s(float climb_rate_m_s)
{
if (std::isfinite(climb_rate_m_s)) {
_manual_max_climb_rate_m_s = math::constrain(climb_rate_m_s, 0.05f, 3.f);
}
}

void TrajectoryGenerator::set_manual_min_height_m(float height_m)
{
if (std::isfinite(height_m)) {
_manual_min_height_m = math::constrain(height_m, 0.f, 10.f);
}
}

void TrajectoryGenerator::set_manual_max_height_m(float height_m)
{
if (std::isfinite(height_m)) {
_manual_max_height_m = math::constrain(height_m, 0.1f, 50.f);
}
}

float TrajectoryGenerator::circle_period_s() const
{
if (_circle_speed_m_s < 1e-5f || _circle_radius_m < 1e-5f) {
return 0.f;
}

return TWO_PI_F * _circle_radius_m / _circle_speed_m_s;
}

void TrajectoryGenerator::set_zero_derivatives(Output &output)
{
for (int i = 0; i < 3; i++) {
output.velocity_ned[i] = 0.f;
output.acceleration_ned[i] = 0.f;
output.jerk_ned[i] = 0.f;
output.snap_ned[i] = 0.f;
}

output.yaw_rate = 0.f;
output.yaw_accel = 0.f;
}

void TrajectoryGenerator::set_hold_position(Output &output, const float position_ned[3])
{
output.position_ned[0] = position_ned[0];
output.position_ned[1] = position_ned[1];
output.position_ned[2] = position_ned[2];

set_zero_derivatives(output);
}

void TrajectoryGenerator::update_manual_hold_target(const Input &input, Output &output)
{
const float target_vz_ned = update_manual_height_reference(input);

output.position_ned[0] = _manual_hold_position_ned[0];
output.position_ned[1] = _manual_hold_position_ned[1];
output.position_ned[2] = _manual_hold_position_ned[2];

set_zero_derivatives(output);
output.velocity_ned[2] = target_vz_ned;
sync_hover_reference_from_output(output);
}

float TrajectoryGenerator::update_manual_height_reference(const Input &input)
{
if (!_manual_hold_initialized) {
_manual_hold_position_ned[0] = _hover_position_ned[0];
_manual_hold_position_ned[1] = _hover_position_ned[1];
_manual_hold_position_ned[2] = _hover_position_ned[2];
_manual_hold_initialized = true;
_last_update_us = input.timestamp_us;
}

const float dt = math::constrain((input.timestamp_us - _last_update_us) * 1e-6f, 0.f, 0.1f);
float stick = input.manual_height_control_valid ? math::constrain(input.manual_height_stick, -1.f, 1.f) : 0.f;

if (fabsf(stick) < _manual_height_deadzone) {
stick = 0.f;
}

const float target_vz_ned = -stick * _manual_max_climb_rate_m_s;
_manual_hold_position_ned[2] += target_vz_ned * dt;

const float min_z_ned = _start_position_ned[2] - _manual_max_height_m;
const float max_z_ned = _skip_takeoff
			? _start_position_ned[2] + _manual_max_height_m
			: _start_position_ned[2] - _manual_min_height_m;
_manual_hold_position_ned[2] = math::constrain(_manual_hold_position_ned[2], min_z_ned, max_z_ned);

_hover_position_ned[2] = _manual_hold_position_ned[2];

return target_vz_ned;
}

void TrajectoryGenerator::update_circle_target(const Input &input, Output &output, float speed_m_s)
{
const float radius = _circle_radius_m;
float target_vz_ned = 0.f;

// A circle with zero tangential speed is the hover trajectory. Both commands
// intentionally use this function so their position reference is identical.
if (radius < 1e-5f || speed_m_s < 1e-5f) {
output.mode = Mode::Hover;

if (input.manual_height_control_enabled) {
update_manual_hold_target(input, output);

} else {
_manual_hold_initialized = false;
set_hold_position(output, _hover_position_ned);
sync_hover_reference_from_output(output);
}

return;
}

if (!_circle_initialized) {
copy3(_hover_position_ned, _circle_center_position_ned);
copy3(_hover_position_ned, _circle_transition_start_position_ned);
copy3(_circle_center_position_ned, _circle_start_position_ned);
_circle_start_position_ned[1] -= radius;
_circle_current_speed_rad_s = speed_m_s / radius;
_circle_transition_start_time_s = output.elapsed_time_s;
_circle_orbit_start_time_s = _circle_transition_start_time_s + _circle_transition_duration_s;
_circle_initialized = true;
}

if (input.manual_height_control_enabled) {
target_vz_ned = update_manual_height_reference(input);
_circle_center_position_ned[2] = _hover_position_ned[2];

} else {
_manual_hold_initialized = false;
_circle_center_position_ned[2] = _hover_position_ned[2];
}

_circle_transition_start_position_ned[2] = _circle_center_position_ned[2];
_circle_start_position_ned[2] = _circle_center_position_ned[2];

const float transition_time_s = output.elapsed_time_s - _circle_transition_start_time_s;

if (transition_time_s < _circle_transition_duration_s) {
update_circle_transition_target(output, transition_time_s, target_vz_ned);
return;
}

output.mode = Mode::Circle;

const float t = math::max(output.elapsed_time_s - _circle_orbit_start_time_s, 0.f);
const float w = _circle_current_speed_rad_s;
const float theta = w * t;
const float s = sinf(theta);
const float c = cosf(theta);

// Original real-airframe circle convention:
// x = r sin(wt), y = -r cos(wt), starting at (0, -r).
output.position_ned[0] = _circle_center_position_ned[0] + radius * s;
output.position_ned[1] = _circle_center_position_ned[1] - radius * c;
output.position_ned[2] = _circle_center_position_ned[2];

output.velocity_ned[0] = radius * w * c;
output.velocity_ned[1] = radius * w * s;
output.velocity_ned[2] = target_vz_ned;

output.acceleration_ned[0] = -radius * w * w * s;
output.acceleration_ned[1] = radius * w * w * c;
output.acceleration_ned[2] = 0.f;

output.jerk_ned[0] = -radius * w * w * w * c;
output.jerk_ned[1] = -radius * w * w * w * s;
output.jerk_ned[2] = 0.f;

output.snap_ned[0] = radius * w * w * w * w * s;
output.snap_ned[1] = -radius * w * w * w * w * c;
output.snap_ned[2] = 0.f;

if (_circle_yaw_mode == CircleYawMode::Fixed) {
output.yaw = _start_yaw;
output.yaw_rate = 0.f;
output.yaw_accel = 0.f;
}
}

void TrajectoryGenerator::update_circle_transition_target(Output &output, float transition_time_s,
		float target_vz_ned)
{
output.mode = Mode::CircleTransition;

const float duration = _circle_transition_duration_s;
const float s = math::constrain(transition_time_s / duration, 0.f, 1.f);
const float s2 = s * s;
const float s3 = s2 * s;
const float s4 = s3 * s;
const float s5 = s4 * s;
const float s6 = s5 * s;
const float s7 = s6 * s;

// Seventh-order smooth step used by the original transition trajectory.
const float h = 35.f * s4 - 84.f * s5 + 70.f * s6 - 20.f * s7;
const float h_dot = (140.f * s3 - 420.f * s4 + 420.f * s5 - 140.f * s6) / duration;
const float h_ddot = (420.f * s2 - 1680.f * s3 + 2100.f * s4 - 840.f * s5)
		     / (duration * duration);
const float h_jerk = (840.f * s - 5040.f * s2 + 8400.f * s3 - 4200.f * s4)
		     / (duration * duration * duration);
const float h_snap = (840.f - 10080.f * s + 25200.f * s2 - 16800.f * s3)
		     / (duration * duration * duration * duration);

for (int i = 0; i < 2; i++) {
const float delta = _circle_start_position_ned[i] - _circle_transition_start_position_ned[i];
output.position_ned[i] = _circle_transition_start_position_ned[i] + delta * h;
output.velocity_ned[i] = delta * h_dot;
output.acceleration_ned[i] = delta * h_ddot;
output.jerk_ned[i] = delta * h_jerk;
output.snap_ned[i] = delta * h_snap;
}

output.position_ned[2] = _circle_center_position_ned[2];
output.velocity_ned[2] = target_vz_ned;
output.acceleration_ned[2] = 0.f;
output.jerk_ned[2] = 0.f;
output.snap_ned[2] = 0.f;
}

void TrajectoryGenerator::reset_circle_state()
{
_circle_initialized = false;
_circle_current_speed_rad_s = 0.f;
_circle_transition_start_time_s = 0.f;
_circle_orbit_start_time_s = 0.f;
}

void TrajectoryGenerator::sync_hover_reference_from_output(const Output &output)
{
_hover_position_ned[0] = output.position_ned[0];
_hover_position_ned[1] = output.position_ned[1];
_hover_position_ned[2] = output.position_ned[2];
}
