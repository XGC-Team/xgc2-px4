// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 XGC-Team contributors.

/**
 * INDI research mode
 *
 * Shadow only; never replaces PID output. Changes are latched while disarmed.
 * This parameter does not enable a flight-qualified INDI controller.
 *
 * @value 0 Disabled
 * @value 1 Shadow computation and diagnostics
 * @min 0
 * @max 1
 * @group INDI Research
 */
PARAM_DEFINE_INT32(MC_INDI_MODE, 0);

/**
 * INDI roll effectiveness
 *
 * Angular acceleration per normalized torque, NOT per N m or RPM.
 * Zero means uncalibrated and disables numerical candidates.
 * Identify on the pinned airframe and output chain. Latched while disarmed.
 *
 * @min 0
 * @max 1000000
 * @decimal 3
 * @group INDI Research
 */
PARAM_DEFINE_FLOAT(MC_INDI_GR, 0.0f);

/**
 * INDI pitch effectiveness
 *
 * Angular acceleration per normalized torque, NOT per N m or RPM.
 * Zero means uncalibrated and disables numerical candidates.
 * Identify on the pinned airframe and output chain. Latched while disarmed.
 *
 * @min 0
 * @max 1000000
 * @decimal 3
 * @group INDI Research
 */
PARAM_DEFINE_FLOAT(MC_INDI_GP, 0.0f);

/**
 * INDI yaw effectiveness
 *
 * Angular acceleration per normalized torque, NOT per N m or RPM.
 * Zero means uncalibrated and disables numerical candidates.
 * Identify on the pinned airframe and output chain. Latched while disarmed.
 *
 * @min 0
 * @max 1000000
 * @decimal 3
 * @group INDI Research
 */
PARAM_DEFINE_FLOAT(MC_INDI_GY, 0.0f);

/**
 * INDI roll rate error gain
 *
 * Units 1/s. Only used by shadow controller. Latched while disarmed.
 * Default is an illustrative diagnostic setting, not flight tuning.
 *
 * @min 0.001
 * @max 1000
 * @decimal 3
 * @group INDI Research
 */
PARAM_DEFINE_FLOAT(MC_INDI_KR, 5.0f);

/**
 * INDI pitch rate error gain
 *
 * Units 1/s. Only used by shadow controller. Latched while disarmed.
 * Default is an illustrative diagnostic setting, not flight tuning.
 *
 * @min 0.001
 * @max 1000
 * @decimal 3
 * @group INDI Research
 */
PARAM_DEFINE_FLOAT(MC_INDI_KP, 5.0f);

/**
 * INDI yaw rate error gain
 *
 * Units 1/s. Only used by shadow controller. Latched while disarmed.
 * Default is an illustrative diagnostic setting, not flight tuning.
 *
 * @min 0.001
 * @max 1000
 * @decimal 3
 * @group INDI Research
 */
PARAM_DEFINE_FLOAT(MC_INDI_KY, 5.0f);

/**
 * INDI actuator model time constant
 *
 * Zero means unidentified; model is driven by commanded, not measured, torque.
 * Latched while disarmed.
 *
 * @unit s
 * @min 0
 * @max 1
 * @decimal 6
 * @group INDI Research
 */
PARAM_DEFINE_FLOAT(MC_INDI_TAU, 0.0f);

/**
 * INDI filter stage time constant
 *
 * Two identical discrete first-order stages; NOT a Butterworth cutoff.
 * Latched while disarmed.
 *
 * @unit s
 * @min 0.001
 * @max 1
 * @decimal 6
 * @group INDI Research
 */
PARAM_DEFINE_FLOAT(MC_INDI_FTAU, 0.02f);

/**
 * INDI candidate increment limit
 *
 * Normalized torque relative to the filtered baseline, not a per-second slew limit.
 * Latched while disarmed.
 *
 * @min 0.0001
 * @max 1
 * @decimal 6
 * @group INDI Research
 */
PARAM_DEFINE_FLOAT(MC_INDI_DU, 0.1f);

/**
 * INDI maximum diagnostic sample age
 *
 * Samples older than this are rejected by the shadow controller.
 * Latched while disarmed.
 *
 * @unit s
 * @min 0.000125
 * @max 0.1
 * @decimal 6
 * @group INDI Research
 */
PARAM_DEFINE_FLOAT(MC_INDI_MAX_AGE, 0.01f);

/**
 * INDI diagnostic publication limit
 *
 * Computation is gyro-triggered independently of this limit.
 * High rates require logger and CPU budget verification. Latched while disarmed.
 *
 * @unit Hz
 * @min 1
 * @max 1000
 * @group INDI Research
 */
PARAM_DEFINE_INT32(MC_INDI_LOG, 50);
