// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 XGC-Team contributors.
#pragma once

#include <lib/indi/IndiControl.hpp>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/indi_rate_status.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

// Read-only companion of mc_rate_control. Deliberately has no control-output publisher.
class IndiShadow : public ModuleParams
{
public:
	IndiShadow(ModuleParams *parent, bool vtol) : ModuleParams(parent), _vtol(vtol) {}
	void parametersUpdated();
	void update(const vehicle_angular_velocity_s &gyro, const matrix::Vector3f &rate_sp,
		    const vehicle_torque_setpoint_s &issued, bool armed, bool landed, bool rotary_wing);

private:
	void applyPending();
	indi::IndiControl _control;
	indi::Config _pending_config{};
	int _mode{0};
	int _pending_mode{0};
	int _log_hz{50};
	int _pending_log_hz{50};
	float _max_age{0.01f};
	float _pending_max_age{0.01f};
	bool _parameters_pending{false};
	const bool _vtol;
	uint32_t _gyro_device_id{0};
	uint32_t _sample_count{0};
	uint32_t _rejected_samples{0};
	uint64_t _last_publish{0};
	control_allocator_status_s _allocator{};
	esc_status_s _esc{};
	uORB::Subscription _selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _allocator_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _esc_sub{ORB_ID(esc_status)};
	uORB::Publication<indi_rate_status_s> _status_pub{ORB_ID(indi_rate_status)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MC_INDI_MODE>) _param_mode,
		(ParamFloat<px4::params::MC_INDI_GR>) _param_gr,
		(ParamFloat<px4::params::MC_INDI_GP>) _param_gp,
		(ParamFloat<px4::params::MC_INDI_GY>) _param_gy,
		(ParamFloat<px4::params::MC_INDI_KR>) _param_kr,
		(ParamFloat<px4::params::MC_INDI_KP>) _param_kp,
		(ParamFloat<px4::params::MC_INDI_KY>) _param_ky,
		(ParamFloat<px4::params::MC_INDI_TAU>) _param_tau,
		(ParamFloat<px4::params::MC_INDI_FTAU>) _param_ftau,
		(ParamFloat<px4::params::MC_INDI_DU>) _param_du,
		(ParamFloat<px4::params::MC_INDI_MAX_AGE>) _param_max_age,
		(ParamInt<px4::params::MC_INDI_LOG>) _param_log_hz
	)
};
