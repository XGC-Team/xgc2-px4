# Research-only shadow diagnostic. Never an actuator or controller input.
uint64 timestamp
uint64 timestamp_sample
uint64 command_timestamp     # actual PID torque publication clock, not actuator observation time
uint64 allocator_timestamp
uint32 gyro_device_id
uint32 sample_count
uint32 rejected_samples
uint32 flags
uint32 FLAG_MODEL_UNQUALIFIED = 1
uint32 FLAG_FILTER_UNQUALIFIED = 2
uint32 FLAG_ALLOCATOR_STALE = 4
uint32 FLAG_PARAMETERS_PENDING = 8
uint32 FLAG_SENSOR_CHANGED = 16
uint32 FLAG_INPUT_STALE = 32
uint32 FLAG_LANDED = 64
uint32 FLAG_UNSUPPORTED_AIRFRAME = 128
uint32 FLAG_ALLOCATOR_SATURATED = 256
uint32 FLAG_GYRO_ID_UNKNOWN = 512
uint8 mode                  # 0 disabled; 1 shadow only. No active mode exists.
uint8 kernel_code           # indi::Code
uint8 limited_axes          # local candidate limiter, NOT allocator saturation
bool numerically_valid
bool warmed_up
bool ready_for_takeover     # Always false in this implementation
float32 dt                  # s
float32 sample_age          # s, publication time minus IMU sample time
float32 allocator_age       # s; NaN if no allocator status received
float32 esc_oldest_age      # s; NaN if ESC reports unavailable/invalid
float32[3] rate             # rad/s, FRD
float32[3] rate_setpoint    # rad/s, FRD
float32[3] alpha_filtered   # rad/s^2, includes PX4 upstream filtering
float32[3] alpha_setpoint   # rad/s^2
float32[3] input_model      # normalized torque, model estimate ONLY
float32[3] input_filtered   # normalized torque
float32[3] candidate        # normalized torque; NEVER published as a setpoint
float32[3] issued_command   # normalized torque, actual PID publication values
float32[3] clipping_residual # rad/s^2, local candidate limiter residual ONLY
