// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 XGC-Team contributors.
#pragma once

#include <array>
#include <cstdint>

namespace indi
{
using Vec3 = std::array<float, 3>;

// Diagonal, normalized-torque specialization. Not an RPM controller.
// All vectors are in PX4 body FRD. The host owns uORB and all control authority.
struct Config {
	Vec3 effectiveness{}; // (rad/s^2) / normalized torque; zero means uncalibrated
	Vec3 rate_gain{};     // 1/s
	float actuator_tau{0.f}; // s, identified end-to-end first-order approximation
	float filter_tau{0.f};   // s, EACH of two identical discrete low-pass stages
	float increment_limit{0.1f}; // normalized torque relative to filtered baseline
	float output_limit{1.f};
};

struct Sample {
	uint64_t timestamp_sample{0}; // us, monotonic IMU sample clock
	Vec3 rate{};                 // rad/s
	Vec3 rate_setpoint{};        // rad/s; no acceleration feed-forward inferred
	Vec3 angular_acceleration{}; // rad/s^2; upstream processing is host responsibility
	Vec3 issued_command{};       // normalized torque issued AFTER this observation
};

enum class Code : uint8_t {
	Unconfigured = 0, Priming = 1, Valid = 2, InvalidInput = 3,
	InvalidTime = 4, NumericFault = 5
};

struct Result {
	Code code{Code::Unconfigured};
	float dt{0.f};
	Vec3 acceleration_setpoint{};
	Vec3 acceleration_filtered{};
	Vec3 input_model{};
	Vec3 input_filtered{};
	Vec3 candidate{};
	Vec3 allocation_residual{}; // rad/s^2; ONLY the local candidate clipping residual
	uint8_t limited_axes{0};
	bool numerically_valid{false};
	bool warmed_up{false}; // elapsed-time heuristic, NOT observer qualification
};

class IndiControl
{
public:
	// Atomic validation: a bad configuration disables the kernel, never retains it silently.
	bool configure(const Config &config);
	void reset();
	Result update(const Sample &sample);
	bool configured() const { return _configured; }
	static bool validConfig(const Config &config);

private:
	Config _config{};
	bool _configured{false};
	uint64_t _last_sample{0};
	float _elapsed{0.f};
	Vec3 _held_command{};
	Vec3 _input_model{};
	Vec3 _input_stage1{};
	Vec3 _input_stage2{};
	Vec3 _alpha_stage1{};
	Vec3 _alpha_stage2{};
};
} // namespace indi
