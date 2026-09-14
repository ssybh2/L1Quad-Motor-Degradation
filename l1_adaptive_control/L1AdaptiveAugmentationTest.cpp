#include "L1AdaptiveAugmentation.hpp"

#include <gtest/gtest.h>

#include <math.h>

namespace
{

static constexpr float kMassKg = 3.0f;
static constexpr float kGravityMss = 9.80665f;

L1AdaptiveAugmentation::Parameters sitl_parameters()
{
	L1AdaptiveAugmentation::Parameters parameters{};
	parameters.mass_kg = 3.f;
	parameters.inertia_kg_m2[0] = 0.023f;
	parameters.inertia_kg_m2[1] = 0.023f;
	parameters.inertia_kg_m2[2] = 0.0459f;
	parameters.inertia_inverse[0] = 43.478f;
	parameters.inertia_inverse[1] = 43.478f;
	parameters.inertia_inverse[2] = 21.786f;
	parameters.as_v = -5.f;
	parameters.as_omega = -10.f;
	parameters.cutoff_q1_thrust = 10.f;
	parameters.cutoff_q1_moment = 10.f;
	parameters.cutoff_q2_moment = 2.f;
	parameters.l1_enable = 1;
	return parameters;
}

L1AdaptiveAugmentation::Input make_hover_input(hrt_abstime timestamp_us)
{
	L1AdaptiveAugmentation::Input input{};
	input.timestamp_us = timestamp_us;
	input.quat_body_to_ned[0] = 1.f;
	input.baseline_thrust_moment[0] = kMassKg * kGravityMss;
	input.baseline_valid = true;
	input.state_valid = true;
	input.armed = true;
	input.failsafe = false;
	return input;
}

} // namespace

TEST(L1AdaptiveAugmentation, FirstUpdateMatchesOriginalModeAdaptiveInitializationAndFixedDt)
{
	L1AdaptiveAugmentation augmentation;
	augmentation.set_parameters(sitl_parameters());

	L1AdaptiveAugmentation::Output output{};
	EXPECT_TRUE(augmentation.update(make_hover_input(1'000'000), output));
	ASSERT_TRUE(output.valid);

	EXPECT_NEAR(output.velocity_hat[0], 0.f, 1e-6f);
	EXPECT_NEAR(output.velocity_hat[1], 0.f, 1e-6f);
	EXPECT_NEAR(output.velocity_hat[2], kGravityMss * 0.0025f, 1e-6f);
	EXPECT_NEAR(output.adaptive_thrust_moment[0], -0.7218507f, 2e-4f);
	EXPECT_NEAR(output.adaptive_thrust_moment[1], 0.f, 1e-6f);
	EXPECT_NEAR(output.adaptive_thrust_moment[2], 0.f, 1e-6f);
	EXPECT_NEAR(output.adaptive_thrust_moment[3], 0.f, 1e-6f);
}

TEST(L1AdaptiveAugmentation, SourceInitUsesIdentityRprevNotMeasuredAttitude)
{
	L1AdaptiveAugmentation augmentation;
	augmentation.set_parameters(sitl_parameters());

	L1AdaptiveAugmentation::Input input = make_hover_input(1'000'000);
	const float roll = 0.7f;
	input.quat_body_to_ned[0] = cosf(roll * 0.5f);
	input.quat_body_to_ned[1] = sinf(roll * 0.5f);
	input.quat_body_to_ned[2] = 0.f;
	input.quat_body_to_ned[3] = 0.f;

	L1AdaptiveAugmentation::Output output{};
	ASSERT_TRUE(augmentation.update(input, output));
	ASSERT_TRUE(output.valid);

	EXPECT_NEAR(output.velocity_hat[0], 0.f, 1e-6f);
	EXPECT_NEAR(output.velocity_hat[1], 0.f, 1e-6f);
	EXPECT_NEAR(output.velocity_hat[2], kGravityMss * 0.0025f, 1e-6f);
}

TEST(L1AdaptiveAugmentation, SourceAlgorithmUsesFixedDtAndDoesNotApplyCustomAdaptiveLimit)
{
	L1AdaptiveAugmentation augmentation;
	augmentation.set_parameters(sitl_parameters());

	L1AdaptiveAugmentation::Output output{};
	ASSERT_TRUE(augmentation.update(make_hover_input(1'000'000), output));

	L1AdaptiveAugmentation::Input disturbed = make_hover_input(1'100'000);
	disturbed.velocity_ned[2] = 2.f;
	ASSERT_TRUE(augmentation.update(disturbed, output));
	ASSERT_TRUE(output.valid);

	EXPECT_NEAR(output.adaptive_thrust_moment[0], 58.1694f, 2e-3f);
}

TEST(L1AdaptiveAugmentation, ReentryKeepsSourceSigmaButResetsUbUadAndFilters)
{
	L1AdaptiveAugmentation augmentation;
	augmentation.set_parameters(sitl_parameters());

	L1AdaptiveAugmentation::Output output{};
	ASSERT_TRUE(augmentation.update(make_hover_input(1'000'000), output));

	L1AdaptiveAugmentation::Input invalid = make_hover_input(1'002'500);
	invalid.armed = false;
	EXPECT_FALSE(augmentation.update(invalid, output));
	EXPECT_FALSE(output.valid);

	ASSERT_TRUE(augmentation.update(make_hover_input(5'000'000), output));
	ASSERT_TRUE(output.valid);

	EXPECT_NEAR(output.velocity_hat[2], 0.00015291f, 2e-6f);
	EXPECT_NEAR(output.adaptive_thrust_moment[0], -0.00450217f, 2e-5f);
}
