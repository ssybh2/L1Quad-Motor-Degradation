#pragma once

// DSun geometric-controller core adapted to the existing PX4/L1 module interface.
// The public interface is intentionally kept compatible with L1AdaptiveControl and its tests.

#include <drivers/drv_hrt.h>

#include <stdint.h>

class GeometricController
{
public:
	struct Parameters {
		float mass_kg{0.62f};
		float position_gain[3]{14.0f, 15.0f, 15.0f};
		float velocity_gain[3]{1.5f, 0.9f, 1.1f};
		float rotation_gain[3]{0.55f, 0.35f, 0.15f};
		float angular_velocity_gain[3]{0.035f, 0.03f, 0.004f};
		float inertia_kg_m2[3]{0.002016f, 0.001827f, 0.00322f};
	};

	struct Input {
		hrt_abstime timestamp_us{0};

		float position_ned[3]{0.f, 0.f, 0.f};
		float velocity_ned[3]{0.f, 0.f, 0.f};
		float quat_body_to_ned[4]{1.f, 0.f, 0.f, 0.f};
		float angular_velocity_body[3]{0.f, 0.f, 0.f};

		float target_position_ned[3]{0.f, 0.f, 0.f};
		float target_velocity_ned[3]{0.f, 0.f, 0.f};
		float target_acceleration_ned[3]{0.f, 0.f, 0.f};
		float target_jerk_ned[3]{0.f, 0.f, 0.f};
		float target_snap_ned[3]{0.f, 0.f, 0.f};

		// PX4-facing yaw representation. The DSun controller converts these
		// scalar yaw kinematics to the 2-D heading-vector representation internally.
		float target_yaw{0.f};
		float target_yaw_rate{0.f};
		float target_yaw_accel{0.f};
		bool yaw_control_enabled{true};

		// Retained for the repository's L1 altitude-failure mode.
		bool manual_tilt_enabled{false};
		float manual_desired_body_z_axis_ned[3]{0.f, 0.f, 1.f};

		bool state_valid_for_control{false};
		bool armed{false};
		bool failsafe{false};
		uint8_t nav_state{0};
	};

	struct Output {
		hrt_abstime timestamp_us{0};

		float r_error[3]{0.f, 0.f, 0.f};
		float v_error[3]{0.f, 0.f, 0.f};
		float target_thrust{0.f};
		float M[3]{0.f, 0.f, 0.f};
		float target_force[3]{0.f, 0.f, 0.f};
		float z_axis[3]{0.f, 0.f, 1.f};
		float x_axis_desired[3]{1.f, 0.f, 0.f};
		float y_axis_desired[3]{0.f, 1.f, 0.f};
		float z_axis_desired[3]{0.f, 0.f, 1.f};
		float Rdes[3][3]{{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}};
		float eR[3]{0.f, 0.f, 0.f};

		float a_error[3]{0.f, 0.f, 0.f};
		float target_force_dot[3]{0.f, 0.f, 0.f};
		float b3_dot[3]{0.f, 0.f, 0.f};
		float target_thrust_dot{0.f};
		float j_error[3]{0.f, 0.f, 0.f};
		float target_force_ddot[3]{0.f, 0.f, 0.f};

		float b3c[3]{0.f, 0.f, 1.f};
		float b3c_dot[3]{0.f, 0.f, 0.f};
		float b3c_ddot[3]{0.f, 0.f, 0.f};
		float A2[3]{0.f, 1.f, 0.f};
		float A2_dot[3]{0.f, 0.f, 0.f};
		float A2_ddot[3]{0.f, 0.f, 0.f};
		float b2c[3]{0.f, 1.f, 0.f};
		float b2c_dot[3]{0.f, 0.f, 0.f};
		float b2c_ddot[3]{0.f, 0.f, 0.f};
		float b1c_dot[3]{0.f, 0.f, 0.f};
		float b1c_ddot[3]{0.f, 0.f, 0.f};
		float Rd_dot[3][3]{{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}};
		float Rd_ddot[3][3]{{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}};
		float Omegad[3]{0.f, 0.f, 0.f};
		float Omegad_dot[3]{0.f, 0.f, 0.f};
		float ew[3]{0.f, 0.f, 0.f};

		float momentAdd[3]{0.f, 0.f, 0.f};

		bool valid{false};
	};

	GeometricController() = default;
	~GeometricController() = default;

	bool update(const Input &input, Output &output);
	void set_parameters(const Parameters &parameters) { _parameters = parameters; }

	const Input &last_input() const { return _last_input; }
	const Output &last_output() const { return _last_output; }
	const Parameters &parameters() const { return _parameters; }

private:
	Parameters _parameters{};
	Input _last_input{};
	Output _last_output{};
};
