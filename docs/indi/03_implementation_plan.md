# 技术路线、源码边界与接管计划

## 1. 阶段定义

| 阶段 | 内容 | 本次状态 |
|---|---|---|
| P0 | 固定提交、数学推导、反例检查、真实uORB数据接入、只读影子候选 | 已写入代码；主机内核/数学/结构检查已执行 |
| P1 | 全速关联的分配反馈、总滤波同步、可验证的执行器状态估计、完整效能矩阵接口 | 设计在本文；尚未实现/验收 |
| P2 | 单一控制权下的命令替换、饱和闭环、参数与模式状态机、SITL/HITL异常注入 | 待P1接口合格后实现 |
| P3 | 目标板构建、拆桨测试、受控场地低风险逐级飞行和PID对照 | 需要真实硬件、配置和日志；本次未执行 |

影子阶段是落实数据来源和阻断条件的第一步，不是把正式控制器称作完成。没有目标板和动力模型之前不建立“可刷写/可飞行”的默认配置。

## 2. 本次实际源码修改

| 文件 | 实际变化 |
|---|---|
| `src/lib/indi/IndiControl.hpp/.cpp` | 固定尺寸纯C++数据结构，对角归一化效能增量律、一阶输入代理、成对两级滤波、状态复位、有限值/时间检查、候选限幅 |
| `src/lib/indi/CMakeLists.txt` | 独立`IndiControl`库目标；数学层无uORB、ROS或硬件驱动依赖 |
| `src/modules/mc_rate_control/IndiShadow.hpp/.cpp` | `ModuleParams`宿主子对象；参数暂存/未解锁应用；真实uORB读取；仅诊断发布 |
| `src/modules/mc_rate_control/indi_params.c` | 12个有说明/范围的新参数；未辨识模型默认零；不改变原PID参数意义 |
| `MulticopterRateControl.hpp/.cpp` | 只增加对象与构造、参数通知、PID发布后的只读影子调用 |
| `src/modules/mc_rate_control/CMakeLists.txt` | 将库和适配器纳入原模块构建 |
| `msg/IndiRateStatus.msg`、`msg/CMakeLists.txt` | 新的内部诊断类型与构建注册 |
| `Tools/indi/` | C++测试、数学反例、结构契约与并行执行脚本 |

原PID、姿态环、位置环、控制分配器、输出函数映射、驱动、Commander、DDS配置和默认日志配置未被替换。数学核心使用`std::array<float,3>`作为可离线验证的数据边界，PX4适配器与原`matrix::Vector3f`显式拷贝。当前是对角特例，没有再实现一套通用矩阵库。

P1升级完整矩阵时应在参数/模型更新阶段使用PX4已有`lib/matrix`进行有限性、秩、尺度与条件性检查，并生成固定尺寸算子；不要在高频循环无条件调用通用求逆、动态分配内存或无迭代上限的优化器。正则化逆不是“恢复失去的控制权”，条件数/秩失效仍应阻断接管。

## 3. P1-A：增加真正关联的全速分配反馈

目标文件为`ControlAllocator.hpp/.cpp`及新增内部消息。保持原`control_allocator_status`的限频职责，不将全速实时闭环绑定在该状态主题上。

建议新增的 **草案主题**（当前不存在于固件）：

```text
indi_allocator_feedback
  timestamp                   # 分配完成的本地时刻
  timestamp_sample            # 源力矩命令携带的IMU样本时刻，不是电机采样时刻
  command_timestamp           # 源力矩命令的发布时间
  sequence                    # 每次真正分配递增
  configuration_epoch         # 几何/trim/scale/故障掩码变化时递增
  matrix_index
  requested_torque[3]
  allocated_torque[3]          # getAllocatedControl()的归一化结果
  allocated_thrust[3]
  outputs_enabled
  valid
  failed_motor_mask
```

发布位置必须在本次`allocate()`、辅助控制、`updateSetpoint()`、`applySlewRateLimit()`、`clipActuatorSetpoint()`完成之后，且与本次原请求关联。没有`do_update`时不得冒充新分配结果。多分配矩阵应分别标识，常规多旋翼初版只支持已审核的单矩阵配置。

复用`getAllocatedControl()`，保留trim和scale，不在INDI模块重建另一份归一化混控表。NaN停机语义只能在模型内部按明确的“已停止电机”状态转换，不能修改传给输出驱动的NaN，也不能把未知电机测量统一清零。配置epoch改变要失效旧的执行器/滤波状态。

发布的结果仍是**分配模型下的命令**。下一步的状态估计负责电机动态、输出映射、实际发布时间与测量反馈，不能直接把这个主题叫作“实测力矩”。

## 4. P1-B：统一测量与执行器时间轴

需要新增固定长度、固定容量的输入历史缓冲：保存源样本时刻、实际发布时刻、受限输入和配置epoch。以IMU样本时刻为融合目标，逐段按真实输入保持区间传播；禁止读取未来数据、禁止无界追赶历史。超过容量或失去序号连续性时明确复位或降级，不能继续使用过期基准。

RPM路径应逐电机校验`actuator_function`、新样本标识、方向、极对数换算、时间戳和有效性；重复旧RPM不触发微分更新。模型路径应消费受限的实际输出命令，包含推力非线性、电压、起转/停转状态与单电机时间常数，并将未验证的模型误差作为资格阻断项。

测量滤波与模型基准滤波必须对应**总链路**，不只对应新增的一个低通参数。可选实现是复用完整已知滤波结构，或在既有传感器模块增加专用的、带同一校正来源和epoch的INDI数据路径。必须验证噪声放大、群延迟、滤波状态热更新和IMU切换。不能通过关闭已有安全相关传感器处理来获得一个漂亮的理论模型。

## 5. P2：究竟在哪里替换原命令

当前影子调用在原力矩已经发布之后，不能接管。正式接管需要把“计算候选”和“唯一发布”分开，在原`mc_rate_control::Run`的**最终力矩消息发布前**选择一次输出。

采用本文的“最终归一化力矩接口”定义时，候选的单位就是原有偏航低通/电池缩放之后的接口单位。因此选择位置应位于原电池缩放分支之后、推力/力矩发布之前。候选不能再经过第二次电池缩放或偏航低通；执行器模型必须已包含决定保留的后级动态。原推力目标仍按原PX4路径生成和缩放。下面是设计示意，**不是已启用代码**：

```cpp
// pid_final is the existing final normalized PID torque.
// indi_result requires qualified, time-aligned feedback; absent in P0.
final_torque = authority.select(pid_final, indi_result, flight_state);
// Exactly one publication path; keep thrust-before-torque ordering.
publish_thrust(existing_thrust, timestamp_sample);
publish_torque(final_torque, timestamp_sample);
```

`authority.select`不得只看数值是否有限。至少检查：明确支持的机型、armed/rates-enabled状态、测量年龄和连续性、配置epoch、执行器估计资格、总滤波资格、效能秩/尺度、输入限制、计算时间和预设切换状态。

若改用“物理力矩/单电机转速”路线，则需另建经验证的物理到归一化映射，或把动态分配作为原`control_allocator`中的可选方法；不能把RPM填进`actuator_motors.control[]`，不能新增直接PWM写入者，也不能INDI混控后又调用原混控进行第二次分配。

## 6. 饱和与状态机

(T1)后的逐轴裁剪不是受限多旋翼控制分配。P1/P2应向控制器反馈同周期已实现控制量或采用一致的受约束分配，保存倾斜/总推力/偏航的明确优先级。保持同一几何和逻辑Motor配置来源，处理变化率限制、低油门Airmode及失效电机。不能使用未实现的候选更新“已生效输入”。

建议状态：`OFF -> SHADOW -> PRIMING -> READY -> ACTIVE`，另有`DEGRADED/FAULT`。本次只实现OFF与SHADOW，内核Priming只表示数值初始化。启动/起转期间需要可解释的状态播种，不能从地面约束下的角加速度辨识空中效能。

首次P2试验采用未解锁配置控制器、禁止任意空中参数改写。只有在台架与仿真验证了状态衔接后，才允许受监督的接管或回退。PID备用路径的积分器和偏航滤波状态需要明确的跟踪/冻结/重置策略；INDI当前输出与PID输出不一致时应验证有界过渡，不能把开关切回PID当作自动安全。

共享IMU失效时，PID可能同样不可用；应按原飞行状态与故障框架处理，而不是在数学库中直接解锁/上锁或关闭电机。原Commander和输出安全优先级继续有效。

## 7. 参数契约

当前12个参数全部有范围和用途说明。`MC_INDI_GR/GP/GY`单位是角加速度/归一化力矩；默认零未标定。`TAU`是输入代理时间常数，`FTAU`是每一级低通时间常数，`DU`是相对滤波基准的候选增量上限，并非每秒变化率。`LOG`只限制诊断发布；`MAX_AGE`只用于当前影子样本年龄门限。

新参数通过`ModuleParams/DEFINE_PARAMETERS`接入。参数更新先暂存；当前周期刚读取的armed状态决定能否整组应用，避免使用上一个周期的解锁状态产生竞态。模式、模型、滤波、限幅和日志配置不在飞行中半更新。无效模式关闭影子路径，无效数学模型不给有效候选。

未来的在线辨识参数应与持久标定参数分离，写入Flash不得发生在高频环或每个辨识周期。原PID的自动调参、响应设置和增益不被悄悄重新解释为INDI参数；P2必须显式处理原自动调参工具的兼容性。

## 8. 硬件验收前需要的资料

飞控准确板型与硬件修订、IMU/传感器配置、电机和桨、电调固件/协议、是否支持可靠RPM、原参数文件、原生PID ULog、机架几何和载荷/惯量信息。还需要机载计算平台及ROS 2配套版本，但其网络延迟不应成为板内角速度内环的依赖。

没有这些信息可以完成库、接口设计、主机测试与源码结构检查，但不能给出已验证的飞行增益、采样率、控制效能或动力模型。本阶段保留这些阻断项，而不是为它们填入看似合理的通用数字。
