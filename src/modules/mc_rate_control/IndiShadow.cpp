// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 XGC-Team contributors.
#include "IndiShadow.hpp"
#include <drivers/drv_hrt.h>
#include <algorithm>
#include <cmath>

void IndiShadow::parametersUpdated()
{
	_pending_config.effectiveness = {_param_gr.get(), _param_gp.get(), _param_gy.get()};
	_pending_config.rate_gain = {_param_kr.get(), _param_kp.get(), _param_ky.get()};
	_pending_config.actuator_tau = _param_tau.get();
	_pending_config.filter_tau = _param_ftau.get();
	_pending_config.increment_limit = _param_du.get();
	_pending_config.output_limit = 1.f;
	_pending_mode = _param_mode.get();
	_pending_log_hz = _param_log_hz.get();
	_pending_max_age = _param_max_age.get();
	_parameters_pending = true;

}

void IndiShadow::applyPending()
{
	// These are one configuration transaction; never replace filter/model halves in flight.
	_mode = (_pending_mode == 1) ? 1 : 0;
	_log_hz = std::max(1, std::min(1000, _pending_log_hz));
	_max_age = _pending_max_age;

	if (!std::isfinite(_max_age) || _max_age < 0.000125f || _max_age > 0.1f) {
		_mode = 0;
	}

	_control.configure(_pending_config);
	_parameters_pending = false;
	_last_publish = 0;
}

void IndiShadow::update(const vehicle_angular_velocity_s &gyro, const matrix::Vector3f &rate_sp,
		       const vehicle_torque_setpoint_s &issued, bool armed, bool landed, bool rotary_wing)
{
	if (_parameters_pending && !armed) { applyPending(); }

	if (_mode == 0) { return; }

	const uint64_t now = hrt_absolute_time();
	++_sample_count;
	indi_rate_status_s status{};
	status.timestamp = now;
	status.timestamp_sample = gyro.timestamp_sample;
	status.command_timestamp = issued.timestamp;
	status.mode = static_cast<uint8_t>(_mode);
	// Permanent qualification blockers. A numerically valid candidate is NOT flight ready.
	status.flags = indi_rate_status_s::FLAG_MODEL_UNQUALIFIED | indi_rate_status_s::FLAG_FILTER_UNQUALIFIED;
	status.ready_for_takeover = false;

	if (_parameters_pending) { status.flags |= indi_rate_status_s::FLAG_PARAMETERS_PENDING; }

	sensor_selection_s selection{};

	if (_selection_sub.update(&selection) && selection.gyro_device_id != _gyro_device_id) {
		_gyro_device_id = selection.gyro_device_id;
		_control.reset();
		status.flags |= indi_rate_status_s::FLAG_SENSOR_CHANGED;
	}

	if (_gyro_device_id == 0) { status.flags |= indi_rate_status_s::FLAG_GYRO_ID_UNKNOWN; }

	status.gyro_device_id = _gyro_device_id;
	_allocator_sub.update(&_allocator);
	_esc_sub.update(&_esc);
	status.allocator_timestamp = _allocator.timestamp;
	status.allocator_age = NAN;
	status.esc_oldest_age = NAN;

	if (_allocator.timestamp > 0 && now >= _allocator.timestamp) {
		status.allocator_age = static_cast<float>(now - _allocator.timestamp) * 1.e-6f;

		if (!_allocator.torque_setpoint_achieved) {
			status.flags |= indi_rate_status_s::FLAG_ALLOCATOR_SATURATED;
		}
	}

	if (!std::isfinite(status.allocator_age) || status.allocator_age > 0.02f) {
		status.flags |= indi_rate_status_s::FLAG_ALLOCATOR_STALE;
	}

	// Diagnostic age of report timestamps ONLY. Drivers may repeat the previous RPM.
	// No claim of per-motor new-sample availability or measured actuator torque is made.
	if (_esc.esc_count > 0 && _esc.esc_count <= 8) {
		float oldest = 0.f;
		bool valid = true;

		for (unsigned i = 0; i < _esc.esc_count; ++i) {
			const uint64_t t = _esc.esc[i].timestamp;

			if (t == 0 || t > now) { valid = false; break; }

			oldest = std::max(oldest, static_cast<float>(now - t) * 1.e-6f);
		}

		if (valid) { status.esc_oldest_age = oldest; }
	}

	status.sample_age = (gyro.timestamp_sample > 0 && now >= gyro.timestamp_sample)
			    ? static_cast<float>(now - gyro.timestamp_sample) * 1.e-6f : NAN;
	const bool stale = !std::isfinite(status.sample_age) || status.sample_age > _max_age;

	if (stale) { status.flags |= indi_rate_status_s::FLAG_INPUT_STALE; }

	if (landed || !armed) { status.flags |= indi_rate_status_s::FLAG_LANDED; }

	if (_vtol || !rotary_wing) { status.flags |= indi_rate_status_s::FLAG_UNSUPPORTED_AIRFRAME; }

	indi::Sample sample{};
	sample.timestamp_sample = gyro.timestamp_sample;

	for (unsigned i = 0; i < 3; ++i) {
		sample.rate[i] = gyro.xyz[i];
		sample.rate_setpoint[i] = rate_sp(i);
		sample.angular_acceleration[i] = gyro.xyz_derivative[i];
		sample.issued_command[i] = issued.xyz[i];
		status.rate[i] = gyro.xyz[i];
		status.rate_setpoint[i] = rate_sp(i);
		status.issued_command[i] = issued.xyz[i];
	}

	indi::Result result{};

	if (stale || landed || !armed || _vtol || !rotary_wing) {
		_control.reset();
		++_rejected_samples;

	} else {
		result = _control.update(sample);

		if (!result.numerically_valid && result.code != indi::Code::Priming) { ++_rejected_samples; }
	}

	status.kernel_code = static_cast<uint8_t>(result.code);
	status.numerically_valid = result.numerically_valid;
	status.warmed_up = result.warmed_up;
	status.limited_axes = result.limited_axes;
	status.dt = result.dt;
	status.sample_count = _sample_count;
	status.rejected_samples = _rejected_samples;

	for (unsigned i = 0; i < 3; ++i) {
		status.alpha_filtered[i] = result.acceleration_filtered[i];
		status.alpha_setpoint[i] = result.acceleration_setpoint[i];
		status.input_model[i] = result.input_model[i];
		status.input_filtered[i] = result.input_filtered[i];
		status.candidate[i] = result.candidate[i];
		status.clipping_residual[i] = result.allocation_residual[i];
	}

	// Computation runs at the gyro callback rate; diagnostic publication is separately limited.
	const uint64_t interval = 1000000u / static_cast<unsigned>(_log_hz);

	if (_last_publish == 0 || now - _last_publish >= interval) {
		_status_pub.publish(status);
		_last_publish = now;
	}
}
