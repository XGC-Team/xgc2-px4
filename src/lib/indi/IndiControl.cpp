// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 XGC-Team contributors.
#include "IndiControl.hpp"
#include <algorithm>
#include <cmath>

namespace indi
{
namespace
{
bool finite(const Vec3 &v)
{
	return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
float clip(float value, float limit)
{
	return std::max(-limit, std::min(limit, value));
}
}

bool IndiControl::validConfig(const Config &c)
{
	if (!finite(c.effectiveness) || !finite(c.rate_gain)) { return false; }

	for (unsigned i = 0; i < 3; ++i) {
		if (c.effectiveness[i] < 1.e-4f || c.effectiveness[i] > 1.e6f
		    || c.rate_gain[i] <= 0.f || c.rate_gain[i] > 1000.f) { return false; }
	}

	return std::isfinite(c.actuator_tau) && c.actuator_tau >= 0.001f && c.actuator_tau <= 1.f
	       && std::isfinite(c.filter_tau) && c.filter_tau >= 0.001f && c.filter_tau <= 1.f
	       && std::isfinite(c.output_limit) && c.output_limit > 0.f && c.output_limit <= 1.f
	       && std::isfinite(c.increment_limit) && c.increment_limit > 0.f
	       && c.increment_limit <= c.output_limit;
}

bool IndiControl::configure(const Config &config)
{
	reset();
	_configured = validConfig(config);

	if (_configured) { _config = config; }

	return _configured;
}

void IndiControl::reset()
{
	_last_sample = 0;
	_elapsed = 0.f;
	_held_command = {};
	_input_model = {};
	_input_stage1 = {};
	_input_stage2 = {};
	_alpha_stage1 = {};
	_alpha_stage2 = {};
}

Result IndiControl::update(const Sample &s)
{
	Result r{};

	if (!_configured) { return r; }

	if (s.timestamp_sample == 0 || !finite(s.rate) || !finite(s.rate_setpoint)
	    || !finite(s.angular_acceleration) || !finite(s.issued_command)) {
		reset();
		r.code = Code::InvalidInput;
		return r;
	}

	// This is an estimate driven by the LAST issued command, not a measurement
	// of the motor state. The current command must not affect the current observation.
	if (_last_sample == 0) {
		_last_sample = s.timestamp_sample;
		_held_command = s.issued_command;
		_input_model = s.issued_command;
		_input_stage1 = _input_stage2 = _input_model;
		_alpha_stage1 = _alpha_stage2 = s.angular_acceleration;
		r.code = Code::Priming;
		return r;
	}

	if (s.timestamp_sample <= _last_sample) {
		reset();
		r.code = Code::InvalidTime;
		return r;
	}

	const uint64_t dt_us = s.timestamp_sample - _last_sample;

	// Check BEFORE subtract-to-float/clamping. A missing sample is not a normal 20 ms step.
	if (dt_us < 125 || dt_us > 20000) {
		reset();
		r.code = Code::InvalidTime;
		return r;
	}

	r.dt = static_cast<float>(dt_us) * 1.e-6f;
	const float actuator_b = -std::expm1(-r.dt / _config.actuator_tau);
	const float filter_b = -std::expm1(-r.dt / _config.filter_tau);

	for (unsigned i = 0; i < 3; ++i) {
		_input_model[i] += actuator_b * (_held_command[i] - _input_model[i]);
		_input_stage1[i] += filter_b * (_input_model[i] - _input_stage1[i]);
		_input_stage2[i] += filter_b * (_input_stage1[i] - _input_stage2[i]);
		_alpha_stage1[i] += filter_b * (s.angular_acceleration[i] - _alpha_stage1[i]);
		_alpha_stage2[i] += filter_b * (_alpha_stage1[i] - _alpha_stage2[i]);
		r.acceleration_setpoint[i] = _config.rate_gain[i] * (s.rate_setpoint[i] - s.rate[i]);
		const float increment = (r.acceleration_setpoint[i] - _alpha_stage2[i]) / _config.effectiveness[i];

		if (!std::isfinite(increment) || !std::isfinite(_input_stage2[i])) {
			reset();
			r = Result{};
			r.code = Code::NumericFault;
			return r;
		}

		const float limited_increment = clip(increment, _config.increment_limit);
		const float unconstrained = _input_stage2[i] + limited_increment;
		r.candidate[i] = clip(unconstrained, _config.output_limit);

		if (increment != limited_increment || unconstrained != r.candidate[i]) {
			r.limited_axes |= static_cast<uint8_t>(1u << i);
		}

		r.allocation_residual[i] = r.acceleration_setpoint[i] - _alpha_stage2[i]
					 - _config.effectiveness[i] * (r.candidate[i] - _input_stage2[i]);
	}

	if (!finite(r.allocation_residual)) {
		reset();
		r = Result{};
		r.code = Code::NumericFault;
		return r;
	}

	r.input_model = _input_model;
	r.input_filtered = _input_stage2;
	r.acceleration_filtered = _alpha_stage2;
	_last_sample = s.timestamp_sample;
	_held_command = s.issued_command;
	_elapsed = std::min(20.f, _elapsed + r.dt);
	r.code = Code::Valid;
	r.numerically_valid = true;
	r.warmed_up = _elapsed >= 8.f * std::max(_config.actuator_tau, _config.filter_tau);
	return r;
}
} // namespace indi
