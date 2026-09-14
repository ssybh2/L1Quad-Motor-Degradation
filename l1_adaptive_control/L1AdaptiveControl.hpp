#pragma once

#include "GeometricController.hpp"
#include "L1AdaptiveAugmentation.hpp"
#include "L1SourceConfig.hpp"
#include "MotorMixer.hpp"
#include "TrajectoryGenerator.hpp"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class L1AdaptiveControl :
	public ModuleBase<L1AdaptiveControl>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	L1AdaptiveControl();
	~L1AdaptiveControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	struct InternalState {
		hrt_abstime timestamp_us{0};
		float position_ned[3]{0.f, 0.f, 0.f};
		float velocity_ned[3]{0.f, 0.f, 0.f};
		float quat_body_to_ned[4]{1.f, 0.f, 0.f, 0.f};
		float angular_velocity_body[3]{0.f, 0.f, 0.f};
		bool position_valid{false};
		bool velocity_valid{false};
		bool attitude_valid{false};
		bool angular_velocity_valid{false};
		bool armed{false};
		bool failsafe{false};
		uint8_t arming_state{0};
		uint8_t nav_state{0};
	};

	void Run() override;
	void apply_parameter_values();
	void update_subscriptions();
	void update_internal_state();
	void update_failure_mode();
	void update_trajectory_input();
	void run_trajectory_generator();
	void update_controller_input();
	void run_geometric_controller();
	void run_l1_adaptive_augmentation();
	void run_motor_mixer();
	void apply_motor_degradation();
	void publish_motor_commands();
	void print_debug_info();

	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};

	vehicle_local_position_s _vehicle_local_position{};
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	vehicle_status_s _vehicle_status{};

	bool _has_local_position{false};
	bool _has_attitude{false};
	bool _has_angular_velocity{false};
	bool _has_vehicle_status{false};
	bool _failure_mode_selected{false};

	InternalState _state{};
	bool _state_valid_for_control{false};

	TrajectoryGenerator _trajectory_generator{};
	TrajectoryGenerator::Input _trajectory_input{};
	TrajectoryGenerator::Output _trajectory_output{};
	bool _trajectory_update_executed{false};

	GeometricController _geometric_controller{};
	GeometricController::Input _controller_input{};
	GeometricController::Output _geometric_output{};
	bool _geometric_update_executed{false};

	L1AdaptiveAugmentation _l1_adaptive_augmentation{};
	L1AdaptiveAugmentation::Output _l1_output{};
	bool _l1_update_executed{false};

	MotorMixer _motor_mixer{};
	MotorMixer::MotorCommand _motor_command{};
	MotorMixer::MotorCommand _degraded_motor_command{};
	bool _motor_mix_executed{false};

	float _baseline_thrust_moment[4]{0.f, 0.f, 0.f, 0.f};
	float _combined_thrust_moment[4]{0.f, 0.f, 0.f, 0.f};
	float _published_motor_control[4]{0.f, 0.f, 0.f, 0.f};
	bool _actuator_motors_published{false};
	uint32_t _actuator_publish_count{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::L1_MOT1_SCALE>) _param_l1_mot1_scale,
		(ParamFloat<px4::params::L1_CIR_RADIUS>) _param_l1_cir_radius,
		(ParamFloat<px4::params::L1_CIR_RAD_Y>) _param_l1_cir_rad_y,
		(ParamFloat<px4::params::L1_CIR_SPEED>) _param_l1_cir_speed,
		(ParamInt<px4::params::L1_TRAJ_IDX>) _param_l1_traj_idx,
		(ParamInt<px4::params::L1_LAND_FLAG>) _param_l1_land_flag,
		(ParamFloat<px4::params::L1_KPX>) _param_l1_kpx,
		(ParamFloat<px4::params::L1_KPY>) _param_l1_kpy,
		(ParamFloat<px4::params::L1_KPZ>) _param_l1_kpz,
		(ParamFloat<px4::params::L1_KVX>) _param_l1_kvx,
		(ParamFloat<px4::params::L1_KVY>) _param_l1_kvy,
		(ParamFloat<px4::params::L1_KVZ>) _param_l1_kvz,
		(ParamFloat<px4::params::L1_KRX>) _param_l1_krx,
		(ParamFloat<px4::params::L1_KRY>) _param_l1_kry,
		(ParamFloat<px4::params::L1_KRZ>) _param_l1_krz,
		(ParamFloat<px4::params::L1_KOX>) _param_l1_kox,
		(ParamFloat<px4::params::L1_KOY>) _param_l1_koy,
		(ParamFloat<px4::params::L1_KOZ>) _param_l1_koz,
		(ParamInt<px4::params::L1_ADAPT_EN>) _param_l1_adapt_en,
		(ParamFloat<px4::params::L1_AS_V>) _param_l1_as_v,
		(ParamFloat<px4::params::L1_AS_OMEGA>) _param_l1_as_omega,
		(ParamFloat<px4::params::L1_Q1_THR>) _param_l1_q1_thr,
		(ParamFloat<px4::params::L1_Q1_MOM>) _param_l1_q1_mom,
		(ParamFloat<px4::params::L1_Q2_MOM>) _param_l1_q2_mom
	)

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};
	hrt_abstime _last_print_us{0};
};
