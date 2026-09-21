# PX4 数据、调度和接口契约

所有源码结论针对 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，不是滚动 `main`。源码可确认接口与处理顺序，不能证明某块硬件的实际频率、噪声或电调反馈质量。

## 1. 当前已接入的链路

```text
IMU driver / FIFO
 -> VehicleAngularVelocity（PX4传感器选择、滤波、校正）
 -> vehicle_angular_velocity
 -> mc_rate_control::Run（原来的gyro回调）
 -> 原PID + 原偏航低通 + 原电池缩放
 -> 先发布 vehicle_thrust_setpoint
 -> 再发布 vehicle_torque_setpoint
 -> 控制分配、逻辑Motor映射、输出驱动、安全限制（均原样保留）

同一个 mc_rate_control::Run，在上述发布之后：
 -> IndiShadow::update（只读传入角速度、设定值和本次最终PID力矩）
 -> IndiControl::update（影子模型与候选增量）
 -> indi_rate_status（诊断；不成为控制输入）
```

影子代码没有新增任务、定时轮询线程或 ROS 往返链。没有第二个力矩发布者。关闭模式立即返回；开启后确实增加运算、消息读取和可选诊断开销，必须测量目标板上的影响。

## 2. 消息表：存在与合格分开

| 量 | 真实来源/字段 | 本阶段用途 | 必须保留的限制 |
|---|---|---|---|
| 机体角速度 | `vehicle_angular_velocity.xyz` | 内核 `rate`，FRD、rad/s | 来自PX4处理链，不是每个原始IMU样本 |
| 角加速度 | `vehicle_angular_velocity.xyz_derivative` | 内核输入，再做成对的新增滤波 | 原有上游滤波不能视为零延迟；不额外重复差分 |
| 样本时刻 | `timestamp_sample` | 内核实际dt、单调性和年龄检查 | 不使用发布时刻替代采样时刻；年龄不含全部群延迟 |
| 角速度目标 | 宿主 `_rates_setpoint` | 已经处理Acro输入和消息中的非有限值 | `vehicle_rates_setpoint`没有独立角加速度前馈；不差分阶跃 |
| 最终PID命令 | 宿主发布的 `vehicle_torque_setpoint.xyz/timestamp` | 模型驱动量、诊断 | 是请求，不是分配结果，更不是实测力矩 |
| 传感器选择 | `sensor_selection.gyro_device_id` | 变化时重置影子状态 | 与角速度消息非原子配对；生产版需要同消息epoch/device_id |
| 分配状态 | `control_allocator_status.timestamp/torque_setpoint_achieved` | 年龄和饱和告警 | 限频发布，不用于逐周期重建执行器基准 |
| 电调报告 | `esc_status.esc_count/esc[i].timestamp` | 仅报告最老时间戳年龄 | 不是“本周期每个电机都有新RPM”的证明；当前不把RPM用于控制 |
| 新诊断 | `indi_rate_status` | 候选、基准、残差、时戳、拒绝计数、阻断原因 | 默认50 Hz发布；计算依然跟随gyro。`ready_for_takeover=false` |

当前 `sample_count` 是**影子适配器实际收到的回调数**，不是IMU硬件样本计数，不能用它证明没有丢过uORB更新。`rejected_samples`是已收到但被拒绝的样本数。未收到的更新需要额外的生产者序号、uORB generation或FIFO计数才能辨识。

原生 `sensor_gyro_fifo` 含批量样本、缩放、样本间隔与设备标识。真正需要逐样本处理时必须消费批中全部有效样本，并处理批丢失、传感器切换和标定；不能将包频率当成原始采样率，也不能直接把传感器坐标的原始数值当作FRD数据。当前影子实现没有绕开原有校正链。[S1]

## 3. 已核实的处理次序和频率边界

`VehicleAngularVelocity::Run` 的FIFO路径先将原始数值复制到工作数组，再依次调用 `FilterAngularVelocity` 和 `FilterAngularAcceleration`；前者原地修改数组。动态RPM/FFT陷波、普通陷波、角速度低通会进入后续差分的上游，差分后还有角加速度低通。`CalibrateAndPublish` 最后执行校正/旋转、偏置处理和发布限频。[S1]

因此，在 `xyz_derivative` 和模型基准上再各加相同滤波器，只能保证**新增部分**相同，不能保证总传递函数相同。当前代码永久保留 `FLAG_FILTER_UNQUALIFIED`。切换IMU、热校正变化、动态陷波更新都需要进入最终滤波epoch和时间对齐方案。

`mc_rate_control` 由 `SubscriptionCallbackWorkItem` 消费角速度；原PID将dt约束在0.125–20 ms。新增内核在自己的状态更新前先检查真实时间戳：重复、倒退、过短和大于20 ms的间隔被拒绝/重新初始化，不把丢样伪装为一个正常的限幅dt。[S2]

分配器在力矩设定值更新时运行，读取对应可用的推力值；因此保留“先推力、后力矩”的发布次序。分配、辅助控制、动态设定值处理、变化率限制和最终裁剪均可能改变实际的电机请求。[S3]

`control_allocator_status` 发布间隔至少5 ms，即理论上最多约200 Hz，实际可能更低；它是为较慢的状态/抗积分饱和处理设计的。当前内核即使以1 kHz被调用，也不会因此获得1 kHz分配反馈。[S3]

本分支不改 `IMU_GYRO_RATEMAX`、`IMU_INTEG_RATE`、任何陀螺滤波参数或输出协议。500/1000 Hz只是候选研究设置，不是已经在目标飞控获得的数据。需要区分原始采样率、FIFO包率、控制消息发布率、控制器运行率、分配/输出率、逐电机新测量率及闭环带宽。

## 4. 为什么不从旧饱和残差反算当前力矩

旧 `control_allocator_status.unallocated_torque` 没有与当前宿主请求进行严格的逐样本关联。用“最新请求减旧残差”可能把两个不同周期拼在一起。当前代码不做这种重建。

即使拿到了对应的 `getAllocatedControl()`，其语义仍是分配模型下的控制量。该版本代码实际计算：

```cpp
(_effectiveness * (_actuator_sp - _actuator_trim)).emult(_control_allocation_scale)
```

遗漏trim或归一化scale，就会改变接口量纲。[S4] 它还不是电机对机体产生的真实力矩；输出驱动、起转、停机、限幅与电机动态在后面。分配结果可以成为执行器模型输入，但不能被包装成物理传感器读数。

## 5. 桨序与输出映射

必须区别：论文旋翼编号、PX4逻辑Motor编号、物理输出引脚、电调遥测数组位置。

本阶段在归一化力矩层运行，故不维护自己的旋翼混控矩阵或物理引脚表；所有候选仍只作为诊断。实际PID输出继续使用原控制分配器，因而没有另一个“X形四轴Motor1”硬编码来源。

未来需要物理效能时复用PX4几何配置与逻辑顺序。该版本旋翼静态力/矩列的实现为：

```cpp
thrust = ct * axis;
moment = ct * position.cross(axis) - ct * km * axis;
```

`axis`先归一化，旋向通过`km`等配置体现；不能把论文矩阵的偏航正负号直接移植。[S5] 另外，PX4几何系数不能未经标定就当作牛顿和牛米系数。物理推力曲线、转子惯量和机体惯量需要独立标定。

后续RPM使用应按 `EscReport.actuator_function` 对应逻辑Motor，验证唯一性、缺失项和目标驱动是否正确填写；不按`esc[i]`默认对应`control[i]`。机械转速转换为 `omega=2*pi*RPM/60`，电气RPM必须先按电机极对数转换。反转/可逆电机另行建立模型，不在常规多旋翼初版支持范围内。[S6]

## 6. 如何补齐传感器与执行器数据

**方案A：RPM驱动的物理力矩重建。** 从驱动获得逐电机新样本标识、有效性和采样时刻，校验功能编号，再在统一时间轴重建转速/转速变化和力矩。`esc_status`字段存在不代表已满足要求；必要时新增驱动级轻量主题，不能把重复的旧RPM当作新样本。遥测与闭环转速命令是两个不同能力。

**方案B：经辨识的执行器观察器。** 消费与命令严格关联的最终分配/输出信息，按电机分别使用推力映射、电压和时间常数进行预测；有稀疏RPM时再进行带创新限幅的校正。输入历史按**实际命令发布时间**保存，重放到IMU样本时刻。不允许用未来命令更新过去的执行器状态。观察器误差应以独立数据给出界或可信工作域，而不是宣称仅凭IMU能唯一分离扰动与电机模型误差。

当前库只是方案B之前的影子代理：用前一条PID归一化请求作一阶预测，尚未处理分配/驱动全部变化和实际发布时间差。因此 `FLAG_MODEL_UNQUALIFIED` 永久置位，并同时记录 `command_timestamp` 与 `timestamp_sample` 供后续量化。该代理不是完整的已标定状态观察器。

**角加速度生产路径。** 优先测量现有链路的总响应；如果无法与执行器基准匹配，再在既有传感器模块中增加经过同一设备选择/校正的专用路径。FIFO工作数组在滤波前后不同，专用路径必须在被原地修改前取数据，不应复制另一套未经维护的传感器选择器。新增主题应含设备ID、采样序号、滤波epoch和样本时刻。是否传批量样本或降采样后的控制样本，依据实际板上时序确定。

## 7. ROS 2 与日志

INDI内环不依赖DDS。ROS 2继续负责上层轨迹/设定值及试验管理，沿用原有模式和失效保护。本版本默认 `dds_topics.yaml` 的 `vehicle_angular_velocity` 发布项还是注释状态；版本较新不意味着全部高频信号默认通过ROS 2可用。[S7]

新 `indi_rate_status` 当前为内部uORB主题，未添加到DDS映射，也不会凭空出现在`px4_msgs`。需要外部诊断时，应同步消息定义、添加明确的低频发布限制、冻结配套消息版本，并测试双向通信。不得为了遥测方便把内环绕到伴飞计算机。

`listener indi_rate_status` 和 `uorb top` 可检查已发布的诊断；它们不是WCET或无丢样证明。新增主题不会自动进入默认ULog主题列表。应明确加入日志配置，保留标准诊断主题，并验证写入率与掉块。使用自定义`logger_topics.txt`前先检查本版本替换/合并行为，不要用只有INDI一行的文件覆盖标准飞行日志。

## 源码索引

所有链接固定到基线提交，而非移动中的 `main`。

- [S1] [角速度、角加速度和 FIFO 处理](https://github.com/XGC-Team/xgc2-px4/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp)
- [S2] [内环宿主与原有发布顺序](https://github.com/XGC-Team/xgc2-px4/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_rate_control/MulticopterRateControl.cpp)
- [S3] [控制分配、限制和状态发布节拍](https://github.com/XGC-Team/xgc2-px4/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/control_allocator/ControlAllocator.cpp)
- [S4] [分配矩阵、trim 与归一化 scale](https://github.com/XGC-Team/xgc2-px4/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/lib/control_allocation/control_allocation/ControlAllocation.hpp)
- [S5] [旋翼几何效能列](https://github.com/XGC-Team/xgc2-px4/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp)
- [S6] [电调逐项字段](https://github.com/XGC-Team/xgc2-px4/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/EscReport.msg)
- [S6b] [电调报告集合](https://github.com/XGC-Team/xgc2-px4/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/EscStatus.msg)
- [S7] [DDS 映射](https://github.com/XGC-Team/xgc2-px4/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.yaml)
- [S8] [参数元数据的扫描和生成](https://github.com/XGC-Team/xgc2-px4/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/lib/parameters/CMakeLists.txt)

