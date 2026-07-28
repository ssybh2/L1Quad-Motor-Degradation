#pragma once

#include <drivers/drv_hrt.h>

#include <stdint.h>

class TrajectoryGenerator
{
public:
enum class Mode : uint8_t {
WaitForValidState = 0,
Takeoff = 1,
Hover = 2,
CircleTransition = 3,
Circle = 4
};

enum class CommandedMode : uint8_t {
Hover = 0,
Circle = 1
};

enum class CircleYawMode : uint8_t {
Fixed = 0
};

struct Input {
hrt_abstime timestamp_us{0};

float current_position_ned[3]{0.f, 0.f, 0.f};
float current_yaw{0.f};

bool state_valid_for_control{false};
bool armed{false};
bool failsafe{false};
bool initialize_in_hover{false};
uint8_t nav_state{0};

float manual_height_stick{0.f};
bool manual_height_control_enabled{false};
bool manual_height_control_valid{false};
};

struct Output {
hrt_abstime timestamp_us{0};

float position_ned[3]{0.f, 0.f, 0.f};
float velocity_ned[3]{0.f, 0.f, 0.f};
float acceleration_ned[3]{0.f, 0.f, 0.f};
float jerk_ned[3]{0.f, 0.f, 0.f};
float snap_ned[3]{0.f, 0.f, 0.f};

float yaw{0.f};
float yaw_rate{0.f};
float yaw_accel{0.f};

float elapsed_time_s{0.f};
Mode mode{Mode::WaitForValidState};

bool valid{false};
};

TrajectoryGenerator() = default;
~TrajectoryGenerator() = default;

bool update(const Input &input, Output &output);

void reset();
void set_commanded_mode(CommandedMode mode);
void set_takeoff_height_m(float height_m);
void set_takeoff_duration_s(float duration_s);
void set_circle_radius_m(float radius_m);
void set_circle_speed_m_s(float speed_m_s);
void set_circle_transition_duration_s(float duration_s);
void set_manual_height_deadzone(float deadzone);
void set_manual_max_climb_rate_m_s(float climb_rate_m_s);
void set_manual_min_height_m(float height_m);
void set_manual_max_height_m(float height_m);

CommandedMode commanded_mode() const { return _commanded_mode; }
float takeoff_height_m() const { return _takeoff_height_m; }
float takeoff_duration_s() const { return _takeoff_duration_s; }
float circle_radius_m() const { return _circle_radius_m; }
float circle_speed_m_s() const { return _circle_speed_m_s; }
float circle_transition_duration_s() const { return _circle_transition_duration_s; }
float manual_height_deadzone() const { return _manual_height_deadzone; }
float manual_max_climb_rate_m_s() const { return _manual_max_climb_rate_m_s; }
float manual_min_height_m() const { return _manual_min_height_m; }
float manual_max_height_m() const { return _manual_max_height_m; }
float circle_period_s() const;

const Input &last_input() const { return _last_input; }
const Output &last_output() const { return _last_output; }

private:
void set_zero_derivatives(Output &output);
void set_hold_position(Output &output, const float position_ned[3]);
void update_manual_hold_target(const Input &input, Output &output);
float update_manual_height_reference(const Input &input);
void update_circle_target(const Input &input, Output &output, float speed_m_s);
void update_circle_transition_target(Output &output, float transition_time_s, float target_vz_ned);
void reset_circle_state();
void sync_hover_reference_from_output(const Output &output);

bool _initialized{false};
bool _skip_takeoff{false};

hrt_abstime _start_time_us{0};

float _start_position_ned[3]{0.f, 0.f, 0.f};
float _takeoff_target_position_ned[3]{0.f, 0.f, -1.f};
float _hover_position_ned[3]{0.f, 0.f, -1.f};
float _circle_center_position_ned[3]{0.f, 0.f, -1.f};
float _circle_transition_start_position_ned[3]{0.f, 0.f, -1.f};
float _circle_start_position_ned[3]{0.f, -1.f, -1.f};
float _manual_hold_position_ned[3]{0.f, 0.f, 0.f};

float _start_yaw{0.f};
bool _manual_hold_initialized{false};
hrt_abstime _last_update_us{0};

CommandedMode _commanded_mode{CommandedMode::Hover};
CircleYawMode _circle_yaw_mode{CircleYawMode::Fixed};
bool _circle_initialized{false};
float _takeoff_height_m{1.0f};
float _takeoff_duration_s{2.0f};
float _circle_radius_m{1.0f};
float _circle_speed_m_s{0.5f};
float _circle_transition_duration_s{2.0f};
float _manual_height_deadzone{0.10f};
float _manual_max_climb_rate_m_s{0.3f};
float _manual_min_height_m{0.5f};
float _manual_max_height_m{2.0f};
float _circle_current_speed_rad_s{0.f};
float _circle_transition_start_time_s{0.f};
float _circle_orbit_start_time_s{0.f};

static constexpr float MIN_CIRCLE_RADIUS_M = 0.2f;
static constexpr float MAX_CIRCLE_RADIUS_M = 20.0f;

Input _last_input{};
Output _last_output{};
};
