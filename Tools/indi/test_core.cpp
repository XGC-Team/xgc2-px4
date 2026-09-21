// SPDX-License-Identifier: BSD-3-Clause
#include "IndiControl.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>

#define CHECK(x) do { if (!(x)) { std::cerr << __func__ << ':' << __LINE__ << ": " #x "\n"; std::exit(1); } } while (0)
using namespace indi;
Config config()
{
	Config c{}; c.effectiveness = {2.f, 4.f, 8.f}; c.rate_gain = {3.f, 5.f, 7.f};
	c.actuator_tau = 0.03f; c.filter_tau = 0.02f; c.increment_limit = 1.f; return c;
}
Sample sample(uint64_t t) { Sample s{}; s.timestamp_sample = t; return s; }
bool near(float a, float b, float e = 1.e-5f) { return std::fabs(a - b) <= e; }
void unconfigured()
{
	IndiControl c; CHECK(c.update(sample(1000)).code == Code::Unconfigured);
}
void bad_config()
{
	IndiControl c; Config p = config(); CHECK(c.configure(p));
	p.effectiveness[1] = 0.f; CHECK(!c.configure(p)); CHECK(!c.configured());
	p = config(); p.actuator_tau = 0.f; CHECK(!c.configure(p));
	p = config(); p.effectiveness[0] = -1.f; CHECK(!c.configure(p));
	p = config(); p.rate_gain[2] = std::numeric_limits<float>::infinity(); CHECK(!c.configure(p));
	p = config(); p.filter_tau = std::numeric_limits<float>::quiet_NaN(); CHECK(!c.configure(p));
	p = config(); p.increment_limit = 1.1f; CHECK(!c.configure(p));
}
void incremental_identity()
{
	IndiControl c; CHECK(c.configure(config()));
	Sample s = sample(1000); s.rate_setpoint = {0.1f, 0.1f, 0.1f};
	CHECK(c.update(s).code == Code::Priming); s.timestamp_sample += 1000;
	Result r = c.update(s); CHECK(r.numerically_valid);
	CHECK(near(r.candidate[0], .15f)); CHECK(near(r.candidate[1], .125f)); CHECK(near(r.candidate[2], .0875f));
	for (float residual : r.allocation_residual) { CHECK(near(residual, 0.f)); }
}
void constant_disturbance()
{
	IndiControl c; CHECK(c.configure(config()));
	Sample s = sample(1000); s.issued_command = {.2f, .2f, .2f};
	s.angular_acceleration = {.5f, .9f, 1.7f}; // G u + 0.1 on each axis
	c.update(s); s.timestamp_sample += 1000; Result r = c.update(s);
	CHECK(near(r.candidate[0], -.05f)); CHECK(near(r.candidate[1], -.025f)); CHECK(near(r.candidate[2], -.0125f));
}
void no_extra_dt_factor()
{
	IndiControl a, b; a.configure(config()); b.configure(config());
	Sample s = sample(1000); s.rate_setpoint = {.1f, .1f, .1f}; a.update(s); b.update(s);
	s.timestamp_sample = 2000; const auto ra = a.update(s);
	s.timestamp_sample = 5000; const auto rb = b.update(s);
	for (unsigned i = 0; i < 3; ++i) { CHECK(near(ra.candidate[i], rb.candidate[i])); }
}
void current_command_is_not_past_input()
{
	IndiControl c; c.configure(config()); Sample s = sample(1000); c.update(s);
	s.timestamp_sample = 2000; s.issued_command = {1.f, 1.f, 1.f};
	CHECK(near(c.update(s).input_model[0], 0.f));
	s.timestamp_sample = 3000; CHECK(near(c.update(s).input_model[0], -std::expm1(-.001f/.03f)));
}
void timestamp_guards()
{
	IndiControl c; c.configure(config()); Sample s = sample(1000); c.update(s);
	CHECK(c.update(s).code == Code::InvalidTime); CHECK(c.update(s).code == Code::Priming);
	s.timestamp_sample = 900; CHECK(c.update(s).code == Code::InvalidTime);
	s.timestamp_sample = 1000; CHECK(c.update(s).code == Code::Priming);
	s.timestamp_sample = 21001; CHECK(c.update(s).code == Code::InvalidTime);
	s.timestamp_sample = 22000; c.update(s); s.timestamp_sample += 124;
	CHECK(c.update(s).code == Code::InvalidTime);
}
void invalid_input()
{
	IndiControl c; c.configure(config()); auto s = sample(1000);
	s.rate[0] = std::numeric_limits<float>::quiet_NaN(); CHECK(c.update(s).code == Code::InvalidInput);
	s = sample(1000); s.issued_command[2] = std::numeric_limits<float>::infinity();
	CHECK(c.update(s).code == Code::InvalidInput); CHECK(c.update(sample(0)).code == Code::InvalidInput);
}
void overflow_guard()
{
	IndiControl c; c.configure(config()); auto s = sample(1000); c.update(s);
	s.timestamp_sample += 1000; s.rate_setpoint[0] = std::numeric_limits<float>::max();
	const auto r = c.update(s); CHECK(!r.numerically_valid); CHECK(r.code == Code::NumericFault);
}
void limiting_is_not_windup()
{
	IndiControl c; Config p = config(); p.increment_limit = .05f; c.configure(p);
	auto s = sample(1000); s.rate_setpoint = {10.f, 10.f, 10.f}; c.update(s);
	for (int k = 0; k < 1000; ++k) {
		s.timestamp_sample += 1000; auto r = c.update(s); CHECK(r.numerically_valid);
		CHECK(r.limited_axes == 7); CHECK(near(r.candidate[0], .05f));
		CHECK(near(r.input_model[0], 0.f)); // the candidate is never fed back as achieved input
		CHECK(r.allocation_residual[0] > 29.f);
	}
}
void absolute_limit()
{
	IndiControl c; c.configure(config()); auto s = sample(1000);
	s.issued_command = {.99f, .99f, .99f}; s.rate_setpoint = {1.f, 1.f, 1.f}; c.update(s);
	s.timestamp_sample += 1000; auto r = c.update(s);
	for (float v : r.candidate) { CHECK(v <= 1.f); }
	CHECK(r.limited_axes != 0);
}
void matched_filter_algebra()
{
	IndiControl c; const Config p = config(); c.configure(p); auto s = sample(1000); c.update(s);
	Vec3 actual{}, held{};
	for (int k = 1; k < 5000; ++k) {
		const uint64_t us = (k % 3 == 0) ? 1500 : 1000; const float dt = us * 1.e-6f;
		const float b = -std::expm1(-dt/p.actuator_tau); s.timestamp_sample += us;
		for (unsigned i = 0; i < 3; ++i) {
			actual[i] += b*(held[i]-actual[i]); s.angular_acceleration[i] = p.effectiveness[i]*actual[i];
			s.issued_command[i] = .2f*std::sin(.02f*k + static_cast<float>(i));
		}
		auto r = c.update(s); CHECK(r.numerically_valid);
		for (unsigned i = 0; i < 3; ++i) { CHECK(near(r.acceleration_filtered[i], p.effectiveness[i]*r.input_filtered[i], 2.e-5f)); }
		held = s.issued_command;
	}
}
void reset_and_warmup()
{
	IndiControl c; c.configure(config()); auto s = sample(1000); c.update(s); Result r{};
	for (int k = 0; k < 300; ++k) { s.timestamp_sample += 1000; r = c.update(s); }
	CHECK(r.warmed_up); c.reset(); s.timestamp_sample += 1000; CHECK(c.update(s).code == Code::Priming);
}
void randomized_finite()
{
	IndiControl c; c.configure(config()); std::mt19937 rng(20260922); std::uniform_real_distribution<float> d(-1.f, 1.f);
	auto s = sample(1000); c.update(s);
	for (int k = 0; k < 20000; ++k) {
		s.timestamp_sample += 125 + rng()%5000;
		for (unsigned i = 0; i < 3; ++i) { s.rate[i]=d(rng); s.rate_setpoint[i]=d(rng); s.angular_acceleration[i]=20.f*d(rng); s.issued_command[i]=d(rng); }
		auto r = c.update(s); CHECK(r.numerically_valid);
		for (float v : r.candidate) { CHECK(std::isfinite(v) && std::fabs(v)<=1.f); }
	}
}
int main()
{
	using Test = void(*)();
	const Test tests[] = {unconfigured, bad_config, incremental_identity, constant_disturbance,
		no_extra_dt_factor, current_command_is_not_past_input, timestamp_guards, invalid_input,
		overflow_guard, limiting_is_not_windup, absolute_limit, matched_filter_algebra,
		reset_and_warmup, randomized_finite};
	for (Test t : tests) { t(); }
	std::cout << "PASS: 14 C++ test groups; deterministic seed 20260922\n";
	return 0;
}
