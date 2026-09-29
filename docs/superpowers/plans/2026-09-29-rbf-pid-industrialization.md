# RBF-PID Industrialization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 1 ms 固定控制周期、压力单位 bar、上游已完成压力滤波的前提下，补齐 RBF-PID 的安全边界、Ksys 数据链路、泵反馈接口和统一负流量策略，使其可以先以受控灰度方式用于注塑机保压/背压压力段。

**Architecture:** 保留 `RBF_PID_Handle` 作为控制器状态所有者，保留压力控制器作为策略、段配置和输出边界。复用现有 `HYD_PARAM_KSYS_BAR_PER_RPM` IEC 参数读写链路，并在压力控制器内统一转换为 `bar/(L/min)`。泵电机是单泵多缸系统级共享执行器，因此通过独立的共享泵反馈接口写入现有 `HYD_PumpFeedback` 快照；轴反馈接口只负责位置、速度、流量、压力和时间戳。泵反馈先用于实际执行量估计、冻结自适应和诊断，不直接扩大 RBF 网络输入维度。

**Tech Stack:** C99、CMake/CTest、现有 IEC C adapter、`HYD_MotionControlFB`、`HYD_PressureController`、`RBF_PID`。

---

## 决策边界

1. 嵌入式硬件保证压力闭环每次调用间隔为 1 ms。RBF-PID 和压力控制器直接使用 `RBF_PID_FIXED_SAMPLING_PERIOD = 0.001 s`；本阶段不根据外部时间戳判断周期是否准确，也不增加周期故障分支。运行时间、稳态计数和控制历史按每次调用累计一个 1 ms 步长。
2. `pressure_controller.c` 直接使用已经滤波的 `input->measuredPressure`，不再做实际压力一阶低通；普通 PID 的独立变化率滤波暂时保留。
3. Ksys IEC 单位固定为 `bar/rpm`，RBF 内部 `K` 固定为 `bar/(L/min)`：
   `Kflow = Ksys * flowToPumpSpeedGain`。
4. 负流量统一在压力控制器最终输出阶段判断，RBF、普通 PID 和 limiter 使用同一超压阈值和低压安全条件。
5. 非有限反馈、同向饱和、保护限幅或 Jacobian 故障期间，冻结全部在线自适应。泵扭矩本阶段只通过系统级接口采集、校验和诊断，不作为自适应冻结条件；扭矩阈值和滞回将在后续标定后单独引入。
6. 普通 PI/PID 保留为固定参数回退；本阶段不引入 Smith 预估器、在线 Ksys 辨识或更高维 RBF 网络。

## 文件责任

| 文件 | 责任 |
|---|---|
| `src/rbf_pid.c`, `include/rbf_pid.h` | 固定 1 ms、输入/网络校验、RBF/PID 计算、冻结与回退状态 |
| `src/pressure_controller.c`, `include/pressure_controller.h` | 压力直通、Ksys 转换、段状态生命周期、统一负流量、最终执行量接口 |
| `src/motion_control.c`, `include/motion_control.h` | IEC/轴反馈传入、最终 limiter 流量回传、RBF 状态初始化边界 |
| `src/motion_interface.c`, `include/motion_interface.h` | IEC Ksys/共享泵反馈写入接口 |
| `pousHydMotion.xml` | 独立泵反馈 POU 的 IEC 源布局，必须与 C 头文件和生成等价文件同步 |
| `include/common_types.h` | 泵反馈类型、单位和现有 Ksys 参数契约 |
| `tests/plcdemo/POUS.h/.c` | PLC demo 结构和初始化同步 |
| `tests/test_rbf_pid*.c`, `tests/test_pressure_controller.c` | 核心控制、故障、边界和 HIL 回归 |
| `tests/test_parameter_access.c`, `tests/test_parameter_iec.c` | Ksys IEC 读写回归 |
| `tests/test_interface_layout_consistency.py` | IEC 结构布局回归 |
| `rbf_pid_formulas.md`, `docs/RBF-PID算法评审报告.md` | 当前公式、单位和工业边界文档 |

## 输出下限统一设计

压力控制链路中的下限必须分成“策略下限”和“硬件能力下限”，不能让每一层独立解释 `outputMin`：

| 名称 | 位置 | 含义 |
|---|---|---|
| `requestedOutputMin` | `HYD_PressureControllerInput.outputMin` | 工艺/调用方允许的最小流量请求；当前压力段可配置为 `-5 L/min`，它不是最终硬件下限 |
| `reliefPolicyMin` | 压力控制器状态机 | 负流量未授权时为 `0`；快速卸压状态为 `requestedOutputMin` |
| `pumpCapabilityMin` | 泵转速能力 | `-pumpSpeedLimit * HYD_PUMP_NEGATIVE_SPEED_RATIO / flowToPumpSpeedGain` |
| `effectiveOutputMin` | 压力控制器输出 | `max(reliefPolicyMin, pumpCapabilityMin)`，统一传给 RBF、压力输出、总 limiter 和泵转换器 |

目标压力下降时使用带滞回的卸压状态：

```text
pressureError = targetPressure - measuredPressure
targetDrop = previousTargetPressure - targetPressure

enter relief when:
    pressure closed loop
    measuredPressure > RELIEF_MIN_FEEDBACK_PRESSURE
    and (pressureError <= -RELIEF_ENTER_ERROR
         or targetDrop >= RELIEF_TARGET_DROP)

exit relief when:
    measuredPressure <= RELIEF_MIN_FEEDBACK_PRESSURE
    or pressureError >= -RELIEF_EXIT_ERROR
```

建议初始标定值：`RELIEF_MIN_FEEDBACK_PRESSURE = 5 bar`、`RELIEF_ENTER_ERROR = 2 bar`、`RELIEF_EXIT_ERROR = 0.5 bar`、`RELIEF_TARGET_DROP = 2 bar`。进入后才允许负流量；退出后 `effectiveOutputMin = 0`，避免继续抽油。进入/退出均在固定 1 ms 步长下执行，不依赖外部时间戳。

`outputMin` 的传递关系必须是：

```text
pressure controller resolves effectiveOutputMin
 -> RBF_PID_Handle.output_min_flow
 -> HYD_PressureControllerOutput.effectiveOutputMin
 -> HYD_OutputLimiterInput.minimumFlow
 -> HYD_PumpConverterInput.minimumFlow
 -> final hardware clamp by pump reverse-speed capability
```

任何层都不能再次使用 `MIN_OUTPUT=-25`、固定 `-5` 或独立的负流量布尔值覆盖已解析的 `effectiveOutputMin`。泵转换器仍保留最终硬件安全钳位，但不得把硬件能力误当成工艺策略。

最终负向能力公式为：

```text
hardwareReverseMin = -pumpSpeedLimit * HYD_PUMP_NEGATIVE_SPEED_RATIO
                       / flowToPumpSpeedGain
effectiveOutputMin = max(reliefPolicyMin, hardwareReverseMin)
```

非卸压状态下 `reliefPolicyMin = 0`；卸压状态下 `reliefPolicyMin = requestedOutputMin`。因此工艺请求只能放宽到硬件允许的反转范围，不能绕过泵速安全比例。

---

## Task 1: 恢复接口测试基线

**Files:**
- Modify: `tests/rbf_pid_test.c`
- Modify: `tests/test_pressure_controller.c`
- Modify: `tests/test_rbf_pid_contract.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: 删除旧 API 测试引用**

移除已删除的 `pressure_accel_ff_enabled`、`gain_compensation_enabled`、`network_seed`、`RBF_PID_SetSeed()`、`RBF_PID_SetKsysBarPerRpm()`、`RBF_PID_TrackOutput()` 等引用，测试只使用当前头文件公开 API。

- [ ] **Step 2: 增加本阶段行为测试**

在 RBF 测试中增加：

```c
test_invalid_feedback_does_not_command_max_flow();
test_same_direction_saturation_freezes_all_adaptation();
test_fixed_1ms_discrete_update_contract();
test_negative_jacobian_exposes_fallback_fault();
```

在压力控制器测试中增加：

```c
test_rbf_uses_unfiltered_measured_pressure();
test_ksys_bar_per_rpm_reaches_rbf_flow_domain();
test_negative_flow_threshold_is_identical_for_pid_and_rbf();
```

- [ ] **Step 3: 运行基线**

```bash
cmake --build --preset unixgcc --target HydroMotionLib rbf_pid_test test_pressure_controller
ctest --test-dir out/build/unixgcc -R '^(test_pressure_controller|test_rbf_pid|test_rbf_pid_contract)$' --output-on-failure
```

预期：目标可以编译，新增行为测试先失败，且失败原因对应待实现行为。

---

## Task 2: 固定 1 ms 和 RBF 输入安全

**Files:**
- Modify: `include/rbf_pid.h`
- Modify: `src/rbf_pid.c`
- Test: `tests/rbf_pid_test.c`

- [ ] **Step 1: 增加故障位和状态**

增加：

```c
#define RBF_PID_FAULT_INPUT_INVALID    0x0002u
#define RBF_PID_FAULT_NETWORK_INVALID  0x0004u
```

在句柄中增加 `adaptation_frozen`、`input_valid`，并将 `sampling_period` 固定初始化为 1 ms。正常控制路径不读取外部 `dt` 来改变该值。

- [ ] **Step 2: 统一有限值校验**

在 `RBF_PID_Update()` 开始处校验设定值、反馈、历史输出和网络宽度。无效反馈不得转换为 0 bar；应设置故障、保持上一拍安全输出、不推进历史、不更新网络和增益。网络宽度固定下限不小于 0.2，权值/中心/宽度必须有限。外部时间戳不参与本阶段的周期判定。

- [ ] **Step 3: 固定离散公式**

保留当前 1 ms 离散参数语义：

```text
du = KP*(e[k]-e[k-1])
   + clamp(KI*e[k], -I_STEP_LIMIT, I_STEP_LIMIT)
   - KD*filtered_delta2_pressure[k]
   + feedforward_delta
u[k] = clamp(u[k-1] + du, output_min, output_max)
```

不在本阶段把 `KI/KD` 改成连续时间单位；所有增量参数按固定 1 ms 离散系数解释。同步更新 `include/rbf_pid.h` 和 `rbf_pid_formulas.md` 的输入与单位说明。

- [ ] **Step 4: 定向验证**

```bash
cmake --build --preset unixgcc --target rbf_pid_test
ctest --test-dir out/build/unixgcc -R '^test_rbf_pid$' --output-on-failure
```

验收：NaN/Inf 不再变成大正误差；固定 1 ms 离散公式和历史推进保持一致；网络污染不产生 NaN/Inf 输出。

---

## Task 3: 统一饱和、保护和自适应冻结

**Files:**
- Modify: `include/rbf_pid.h`
- Modify: `src/rbf_pid.c`
- Modify: `include/pressure_controller.h`
- Modify: `src/pressure_controller.c`
- Test: `tests/rbf_pid_test.c`
- Test: `tests/test_pressure_controller.c`

- [ ] **Step 1: 统一冻结原因**

为输入无效、同向饱和、保护限幅、低激励和 Jacobian 无效保留可观测冻结原因，并在压力输出中增加 `adaptiveFrozen` / `fallbackActive`。

- [ ] **Step 2: 让网络和增益共享冻结条件**

将同向饱和、外部 limiter 生效、输入/网络故障条件同时传给网络更新和 `rbf_pid_step_adaptive_gains()`。冻结 `w/c/b` 与 `KP/KI/KD`；恢复必须经过连续有效样本窗口。

- [ ] **Step 3: 增加最终执行量回传**

在 `HYD_PressureControllerInput` 增加：

```c
HYD_REAL appliedOutputFlow;
HYD_BOOL appliedOutputValid;
HYD_BOOL limiterActive;
```

新增 `HYD_PressureController_ReportAppliedOutput()`，调用时序固定为：

```text
RBF/PID 计算请求流量
 -> HYD_OutputLimiter_ExecuteWithProtection
 -> HYD_PumpConverter_Execute/ApplySlewLimit
 -> HYD_PressureController_ReportAppliedOutput(applied_flow, limiter_flags)
 -> 下一拍 RBF 使用 applied_flow 更新 u_prev/饱和状态
```

删除当前在压力控制分支中无条件把 `rbfPid.Output/u_prev` 写成请求流量的路径；首拍或无回传时使用本拍请求流量作为临时历史，但立即冻结自适应，直到收到有效实际流量。段切换首拍仍由跟踪流量种子初始化，不能使用未验证的旧请求值。

- [ ] **Step 4: 完成 Jacobian 回退**

Jacobian 连续异常时冻结自适应、保留最近有限输出、让外层报告固定参数回退；仅连续有效 Jacobian 样本达到恢复窗口才清故障。

- [ ] **Step 5: 验证**

验收：饱和期间所有自适应参数不变；最终流量小于请求流量时下一拍冻结；Jacobian 故障可观测且不继续漂移。

---

## Task 4: 移除外层压力滤波并统一负流量

**Files:**
- Modify: `src/pressure_controller.c`
- Modify: `include/pressure_controller.h`
- Modify: `include/common_types.h`
- Test: `tests/test_pressure_controller.c`
- Modify: `src/output_limiter.c`
- Modify: `include/output_limiter.h`
- Modify: `src/pump_converter.c`
- Modify: `include/pump_converter.h`
- Modify: `src/motion_control.c`
- Test: `tests/test_negative_flow.c`

- [ ] **Step 1: 压力反馈直通**

改为：

```c
filteredPressure = input->measuredPressure;
```

`pressureFilterAlpha` 不再参与实际压力反馈；兼容保留输出字段，但其值等于输入压力。独立变化率滤波仅用于普通 PID 测量微分，并不改变 RBF 的 `P_actual`。

- [ ] **Step 2: 单一负流量函数**

集中定义：

```c
#define HYD_PRESSURE_NEGATIVE_FLOW_THRESHOLD_BAR 2.0

static HYD_BOOL HYD_ShouldSuppressNegativeFlow(HYD_REAL error)
{
    return error > -HYD_PRESSURE_NEGATIVE_FLOW_THRESHOLD_BAR;
}
```

所有 P/PI/PID/RBF 分支在最终输出阶段使用该函数：误差大于 -2 bar 时禁止负流量；误差小于等于 -2 bar 且压力闭环未进入低压安全区时才允许负流量。删除 RBF 的 5 bar 和普通 PID 的独立判断。

该函数同时接入 `HYD_OutputLimiter` 的 `allowNegativeFlow` 输入；低压安全条件（目标压力和实际压力均低于 5 bar）优先覆盖超压误差判断。更新现有 `tests/test_negative_flow.c`，不新增第二套负流量规则。

- [ ] **Step 3: 实现卸压状态机和 canonical lower bound**

在 `HYD_PressureControllerState` 增加 `previousTargetPressure` 和 `reliefActive`；在 `HYD_PressureControllerOutput` 增加 `effectiveOutputMin` 和 `reliefActive`。每个固定 1 ms 周期先根据目标压力下降量和压力误差更新状态，再解析 `effectiveOutputMin`。RBF、普通 PID 和前馈输出都只使用这个下限。

在 `HYD_OutputLimiterInput` 与 `HYD_PumpConverterInput` 增加 `minimumFlow`。`minimumFlow` 有限时作为策略下限，实际执行下限取它与泵反转能力下限的最大值；`allowNegativeFlow` 只保留兼容字段，不再独立决定负流量范围。删除压力控制器中 RBF 的 `fabs(error)<5` 和普通 PID 的 `error>-2` 两套独立钳位。

- [ ] **Step 4: 删除重复下限覆盖**

`HYD_EnsureRbfPidInitialized()` 不再每拍写入 `MIN_OUTPUT=-25`；RBF 句柄每拍使用压力控制器解析出的 `effectiveOutputMin`。运动层把 `pressureOutput.effectiveOutputMin` 传入 limiter 和泵转换器，泵转换器只负责按实际泵速限制做最终安全裁剪。

- [ ] **Step 5: 边界测试**

覆盖 `-2`、`-2-epsilon`、`0`、正误差和低压安全区，并验证 RBF 与普通 PID 结果一致。

另覆盖：目标压力反向跳变立即进入卸压、接近目标退出卸压、退出后 `effectiveOutputMin=0`、RBF/limiter/pump converter 三层下限一致，以及配置下限 `-5` 大于硬件反转能力时取硬件能力下限。

---

## Task 5: 完善 Ksys IEC 到 RBF 链路

**Files:**
- Modify: `src/pressure_controller.c`
- Modify: `include/pressure_controller.h`
- Modify: `src/motion_control.c`
- Modify: `src/motion_interface.c`
- Modify: `tests/test_parameter_access.c`
- Modify: `tests/test_parameter_iec.c`
- Modify: `tests/test_pressure_controller.c`

- [ ] **Step 1: 复用现有 IEC 写入接口**

保留 `HYD_PARAM_KSYS_BAR_PER_RPM`、`HYD_MotionControlFB_WriteParameter()`、`HYD_WRITEPARAMETER` 和 `HYD_READPARAMETER`，不创建第二套 Ksys 存储。补充有限值和非负校验，0 表示禁用。

- [ ] **Step 2: 单一 Kflow 解析**

在压力控制器内实现唯一来源：

```text
if input.systemGainBarPerRpm > 0:
    Kflow = input.systemGainBarPerRpm * input.flowToPumpSpeedGain
else if segment.systemGain > 0:
    Kflow = segment.systemGain
else:
    Kflow = 0
```

只把 `Kflow` 传给 `RBF_PID_SetGainCompensation()`，配置指纹比较也使用同一结果。不得把 `bar/rpm` 直接写入 RBF 的 `K`。

具体调用约束：

1. `HYD_ResolveRbfKflow(input, segment)` 是唯一解析函数；
2. `HYD_ApplyRbfPidConfig()` 使用解析后的 `Kflow` 调用 setter；
3. `HYD_RbfPidConfigMatches()` 比较同一个 `Kflow`，不能继续比较 `segment->systemGain`；
4. Ksys 或流量到转速增益无效时返回 0，仅关闭补偿，不改变固定 PID 输出。

- [ ] **Step 3: 运行 Ksys 验收**

验证 `1.5 * 20 = 30 bar/(L/min)`，Ksys 为 0 时关闭补偿，负数/NaN/Inf 拒绝写入；IEC 写入后下一拍 RBF 的 `K` 正确更新。

## Task 6: 一阶惯性模型下的快速卸压验证

**Files:**
- Modify: `tests/test_rbf_pid_first_order_gain1p5.c`
- Modify: `tests/test_rbf_pid_pressure_benchmark.c`
- Test: `tests/test_pressure_controller.c`
- Test: `tests/test_negative_flow.c`

- [ ] **Step 1: 建立目标反向跳变场景**

使用现有一阶模型 `K=1.5 bar/rpm`、`tau=1.0 s`、`Ts=1 ms`，先将压力稳定在 100 bar，再把目标一次性降到 20 bar。压力控制器输入允许 `requestedOutputMin=-5 L/min`，泵反转能力由 `HYD_PUMP_NEGATIVE_SPEED_RATIO` 和泵速上限决定。

- [ ] **Step 2: 验证卸压进入条件**

在目标反向跳变后的首个控制周期内，必须满足：

```text
reliefActive == true
effectiveOutputMin < 0
outputFlow < 0
```

实际反转流量不得超过泵反转硬件能力，且输出经过 limiter 和 pump converter 后保持一致。

- [ ] **Step 3: 验证接近目标退出条件**

当 `targetPressure - measuredPressure >= -0.5 bar` 或压力低于 5 bar 安全门限时，必须满足：

```text
reliefActive == false
effectiveOutputMin == 0
outputFlow >= 0
```

不得出现负流量在目标附近持续保持或 0 附近来回抖动。

- [ ] **Step 4: 验证性能门槛**

记录 100 bar -> 20 bar 目标跳变的：

- 达到 `20 +/- 0.5 bar` 的时间；
- 最大负流量和最大反转转速；
- 目标附近负流量持续时间；
- 重新升压到 80 bar 的建立时间，与未抽空基线比较；
- 压力是否出现低于安全下限或重新建压明显变慢。

验收要求：反向跳变比禁止负流量基线显著缩短卸压时间；目标附近无持续反转；再次升压时间不因抽油明显恶化；压力无 NaN、输出无越过泵硬限幅。

---

## Task 7: 增加系统级共享油泵反馈接口

**Files:**
- Modify: `include/common_types.h`
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Modify: `include/motion_interface.h`
- Modify: `src/motion_interface.c`
- Modify: `pousHydMotion.xml`
- Modify: `tests/plcdemo/POUS.h`
- Modify: `tests/plcdemo/POUS.c`
- Test: `tests/test_interface_layout_consistency.py`
- Test: `tests/test_parameter_iec.c`

- [ ] **Step 1: 复用现有系统级泵反馈结构**

复用 `include/common_types.h` 中已有的 `HYD_PumpFeedback`、`validFlags` 和时间戳字段，不另造轴级泵反馈结构。运行时保存一份按液压系统/泵资源归属的共享快照；单泵系统只有一个快照，多泵扩展时再增加 `pumpId` 或资源组标识。不得把泵反馈复制到每个轴的 PLC 输入结构中。

- [ ] **Step 2: 新增独立 HYD_SETPUMPFEEDBACK**

新增独立系统级 IEC FB（不扩展 `HYD_SETAXISFEEDBACK`），字段为：

```text
PUMP_SPEED_RPM           REAL
PUMP_ANGLE_DEG           REAL
PUMP_TORQUE_PCT_TN       REAL
TIMESTAMP                REAL
ENABLE                   BOOL
```

`__mcl_cmd_SetPumpFeedback()` 由 PLC 每周期最多写入一次共享快照，校验三项数据的有限性，设置对应 `HYD_PUMP_FEEDBACK_VALID_*` 标志。反馈时间按系统 1 ms 周期累计，不作为控制周期准确性或数据新鲜度的判定条件。轴接口 `__mcl_cmd_SetAxisFeedback()` 保持原字段和原语义，不再承载泵转速、角度或扭矩。

同步修改 `pousHydMotion.xml`、`include/motion_interface.h`、`tests/plcdemo/POUS.h`、`tests/plcdemo/POUS.c` 和独立 FB body/声明的字段顺序、默认值与调用点。`tests/test_interface_layout_consistency.py` 必须同时验证 XML、公开 C 头文件和 PLC 生成等价头文件。

- [ ] **Step 3: 传入压力控制器**

在 `HYD_PressureControllerInput` 增加当前共享泵反馈快照。每个轴控制器在执行时只读同一份系统快照，不写泵反馈。第一阶段实际 rpm 有效时，使用：

```text
actualAppliedFlow = actualSpeedRpm / flowToPumpSpeedGain
```

作为实际执行流量估计、饱和检测和学习冻结；扭矩和角度只进入诊断/遥测，不直接扩展 RBF 输入维度，也不在本阶段触发冻结。无效泵反馈时退回 limiter 回传流量；泵反馈有效标志、固定 1 ms 累计规则、单写者规则和有限值规则必须在接口测试中覆盖。

- [ ] **Step 4: 同步 PLC demo 和布局测试**

更新独立 `HYD_SETPUMPFEEDBACK` 的 `POUS.h/.c` 字段、初始化和调用，运行接口布局检查及 IEC 回归；确认同一个泵反馈快照不会因轴数量增加而重复写入。

---

## Task 8: 修复 RBF 状态生命周期

**Files:**
- Modify: `src/pressure_controller.c`
- Modify: `src/motion_control.c`
- Modify: `include/pressure_controller.h`
- Test: `tests/test_pressure_controller.c`
- Test: `tests/test_rbf_pid_hil.c`

- [ ] **Step 1: 区分完整初始化和段切换**

完整初始化只在 FB 初始化、RESET、泵配置或压力归一化标尺改变时发生。压力段切换使用 `RBF_PID_SoftReset()`，以当前输出、压力和前馈重新种子，不清空已经验证的 `w/c/b` 和增益。

- [ ] **Step 2: 归一化标尺变化禁止盲目复用网络**

当压力或流量归一化标尺变化时，执行完整网络重置；只有标尺不变时保留网络。记录重置原因，便于确认跨段学习是否实际发生。

- [ ] **Step 3: 验证**

测试段切换首拍无扰、相同泵配置下网络保留、标尺变化时网络重置、HIL 长时间运行无故障。

---

## Task 9: 全量验证和文档收口

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `tests/rbf_pid_test.c`
- Modify: `tests/test_pressure_controller.c`
- Modify: `tests/test_rbf_pid_hil.c`
- Modify: `tests/test_rbf_pid_pressure_benchmark.c`
- Modify: `tests/test_rbf_pid_first_order_gain1p5.c`
- Modify: `tests/test_negative_flow.c`
- Modify: `src/output_limiter.c`
- Modify: `include/output_limiter.h`
- Modify: `src/pump_converter.c`
- Modify: `include/pump_converter.h`
- Modify: `docs/RBF-PID算法评审报告.md`
- Modify: `rbf_pid_formulas.md`

- [ ] **Step 1: 编译门禁**

```bash
cmake --build --preset unixgcc -j2
```

必须不再有旧字段/API 编译错误。

- [ ] **Step 2: 核心测试门**

```bash
ctest --test-dir out/build/unixgcc -R '^(test_pressure_controller|test_rbf_pid|test_rbf_pid_contract|test_rbf_pid_pressure_benchmark|test_rbf_pid_hil|test_parameter_access|test_parameter_iec)$' --output-on-failure
```

必须覆盖固定 1 ms 离散计算、异常输入、Ksys 单位转换、泵反馈传递、统一负流量、饱和冻结、段切换和长期参数边界。

- [ ] **Step 3: 静态和完整回归**

```bash
git diff --check
ctest --test-dir out/build/unixgcc --output-on-failure
```

另在目标 MCU 记录 1 ms WCET，确认六节点 RBF 和压力链路在周期预算内。

- [ ] **Step 4: 更新工业结论**

文档明确本阶段只批准保压/背压压力段灰度使用；普通 PI/PID 是默认回退。量产放行仍需真实泵阀死区、泄漏、油温、负载扰动、传感器故障和保护限幅数据。

## 敏捷执行顺序

1. **接口基线:** Task 1。
2. **RBF 核心安全:** Task 2 + Task 3。
3. **外层策略和 Ksys:** Task 4 + Task 5。
4. **卸压验证:** Task 6。
5. **泵反馈和生命周期:** Task 7 + Task 8。
6. **验证收口:** Task 9。

每个批次必须先通过目标测试和 `git diff --check`，再进入下一批次。若控制性能回归，保持普通 PI/PID 回退可用，暂停开放 RBF 在线学习并缩小本批次范围。

## 计划自检

- 已覆盖 RBF 核心、无外层压力滤波、Ksys IEC 写入与单位转换、泵转速/角度/扭矩写入、统一负流量判断。
- 已确认 Ksys 通用 IEC 接口当前存在，本计划补齐运行时消费链路，不创建重复存储。
- 每个任务均列出文件、实现契约、测试命令或验收标准。
- 本文件只定义执行计划，不修改控制实现。
