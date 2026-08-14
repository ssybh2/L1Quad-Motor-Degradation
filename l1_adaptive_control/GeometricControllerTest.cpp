#include "GeometricController.hpp"

#include <gtest/gtest.h>

#include <math.h>

namespace
{

static constexpr float kMassKg = 3.0f;
static constexpr float kGravityMss = 9.80665f;

GeometricController::Input make_hover_input()
{
	GeometricController::Input input{};
	input.timestamp_us = 1'000'000;
	input.position_ned[0] = 0.f;
	input.position_ned[1] = 0.f;
	input.position_ned[2] = -1.f;
	input.velocity_ned[0] = 0.f;
	input.velocity_ned[1] = 0.f;
	input.velocity_ned[2] = 0.f;
	input.quat_body_to_ned[0] = 1.f;
	input.quat_body_to_ned[1] = 0.f;
	input.quat_body_to_ned[2] = 0.f;
	input.quat_body_to_ned[3] = 0.f;
	input.angular_velocity_body[0] = 0.f;
	input.angular_velocity_body[1] = 0.f;
	input.angular_velocity_body[2] = 0.f;
	input.target_position_ned[0] = 0.f;
	input.target_position_ned[1] = 0.f;
	input.target_position_ned[2] = -1.f;
	input.target_yaw = 0.f;
	input.state_valid_for_control = true;
	input.armed = true;
	input.failsafe = false;
	return input;
}

bool finite3(const float v[3])
{
	return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

float norm3(const float v[3])
{
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

} // namespace

TEST(GeometricController, HoverProducesLevelAttitudeZeroOmegaDAndWeightThrust)
{
	GeometricController controller;
	GeometricController::Parameters parameters = controller.parameters();
	parameters.mass_kg = kMassKg;
	controller.set_parameters(parameters);
	GeometricController::Input input = make_hover_input();
	GeometricController::Output output{};

	EXPECT_TRUE(controller.update(input, output));
	ASSERT_TRUE(output.valid);

	EXPECT_NEAR(output.x_axis_desired[0], 1.f, 1e-4f);
	EXPECT_NEAR(output.x_axis_desired[1], 0.f, 1e-4f);
	EXPECT_NEAR(output.x_axis_desired[2], 0.f, 1e-4f);
	EXPECT_NEAR(output.y_axis_desired[0], 0.f, 1e-4f);
	EXPECT_NEAR(output.y_axis_desired[1], 1.f, 1e-4f);
	EXPECT_NEAR(output.y_axis_desired[2], 0.f, 1e-4f);
	EXPECT_NEAR(output.z_axis_desired[0], 0.f, 1e-4f);
	EXPECT_NEAR(output.z_axis_desired[1], 0.f, 1e-4f);
	EXPECT_NEAR(output.z_axis_desired[2], 1.f, 1e-4f);

	EXPECT_NEAR(norm3(output.Omegad), 0.f, 1e-5f);
	EXPECT_NEAR(output.target_thrust, kMassKg * kGravityMss, 1e-3f);
	EXPECT_NEAR(norm3(output.eR), 0.f, 1e-5f);
	EXPECT_NEAR(norm3(output.ew), 0.f, 1e-5f);
}

TEST(GeometricController, DynamicCircleDoesNotForceOmegaDToZeroAndStaysFinite)
{
	GeometricController controller;
	GeometricController::Input input = make_hover_input();

	const float radius = 1.f;
	const float speed = 0.5f;
	const float omega = speed / radius;

	input.position_ned[0] = radius;
	input.position_ned[1] = 0.f;
	input.position_ned[2] = -1.f;

	input.target_position_ned[0] = radius;
	input.target_position_ned[1] = 0.f;
	input.target_position_ned[2] = -1.f;

	input.target_velocity_ned[0] = 0.f;
	input.target_velocity_ned[1] = speed;
	input.target_velocity_ned[2] = 0.f;

	input.target_acceleration_ned[0] = -radius * omega * omega;
	input.target_acceleration_ned[1] = 0.f;
	input.target_acceleration_ned[2] = 0.f;

	input.target_jerk_ned[0] = 0.f;
	input.target_jerk_ned[1] = -radius * omega * omega * omega;
	input.target_jerk_ned[2] = 0.f;

	input.target_snap_ned[0] = radius * omega * omega * omega * omega;
	input.target_snap_ned[1] = 0.f;
	input.target_snap_ned[2] = 0.f;

	GeometricController::Output output{};
	EXPECT_TRUE(controller.update(input, output));
	ASSERT_TRUE(output.valid);

	EXPECT_GT(norm3(output.Omegad), 1e-4f);
	EXPECT_TRUE(finite3(output.Omegad));
	EXPECT_TRUE(finite3(output.Omegad_dot));
	EXPECT_TRUE(finite3(output.M));
	EXPECT_TRUE(finite3(output.target_force));
	EXPECT_TRUE(finite3(output.target_force_dot));
	EXPECT_TRUE(finite3(output.target_force_ddot));
	EXPECT_TRUE(isfinite(output.target_thrust));
	EXPECT_TRUE(isfinite(output.target_thrust_dot));
}
