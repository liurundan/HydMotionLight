# RBF-PID Industrialization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 RBF-PID 改造成固定 1 ms、单位明确、带边界保护和 Ksys 前馈的工业压力控制模块，并通过 IEC 参数链路把 `Ksys [bar/rpm]` 传入压力环。

**Architecture:** 保留现有增量 PID 和 6 节点 RBF 结构，但把 RBF 限定为局部过程增益辨识和有界增益调度器。基础控制始终由确定性的增量 PI/PID、输出跟踪和硬件限幅提供；Ksys 用于初始化/约束局部过程增益，并提供不重复累加的静态流量前馈。快速建压和卸压阶段不使用稳态软流量上限，保压接近目标后才启用正向软限幅。

**Tech Stack:** C99、CMake、CTest、现有 IEC 参数访问接口、`PressureModel` 一阶仿真。

---

## Task 7: 回归、静态检查和文档

**Files:**

- Modify: `docs/RBF-PID算法评审报告.md`
- Test: all pressure/RBF tests

- [ ] **Step 1: 更新控制契约文档**

文档必须明确：

- `Ksys` 的 IEC 单位是 `bar/rpm`；
- RBF 内部 `Kflow` 是 `bar/(L/min)`；
- RBF 周期固定 1 ms；
- `Ki/Kd` 是离散每扫描量；
- BOOST/HOLD/RELIEF 的限幅差异；
- RBF 只做有界自适应，基础 PID 和硬件限幅始终有效；
- 旧 `segment.systemGain` 的兼容语义。

- [ ] **Step 2: 运行定向验证**

```bash
cmake --build --preset unixgcc
ctest --test-dir out/build/unixgcc -R 'test_pressure_controller|test_rbf_pid|test_rbf_pid_contract|test_rbf_pid_pressure_benchmark|test_pressure_model|test_parameter_access|test_parameter_iec' --output-on-failure
```

- [ ] **Step 3: 运行完整回归和差异检查**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
git diff --check
```

任何与 RBF/Ksys 路径相关的失败必须修复后才能宣称完成；无关历史失败必须单独记录，不得通过放宽断言处理。

---

## 实施顺序和停止条件

建议按以下敏捷切片合入，每个切片都能编译并运行定向测试：

1. 参数号、FB 参数、IEC 读写、压力输入字段；
2. Ksys -> Kflow 唯一换算和单位测试；
3. 固定 1 ms 离散 PID 和 RBF Jacobian 输入语义；
4. 有界自适应、饱和/泄压冻结和有限值回退；
5. Ksys 前馈、分阶段正向软限幅、快速负向卸压；
6. 无扰跟踪保留网络状态；
7. 一阶 benchmark、完整回归和文档。

在以下条件全部满足前停止，不进入更复杂的在线辨识：

- IEC 能写入并读回 Ksys；
- `1.5 bar/rpm * 20 rpm/(L/min) = 30 bar/(L/min)` 有自动化测试；
- Ksys 前馈不会每扫描重复累加；
- BOOST 和 RELIEF 不被稳态 Qcap 饿死；
- 输出饱和、NaN/Inf、低激励时自适应冻结且输出仍有限；
- 策略切换保持无扰且不丢网络参数；
- 一阶模型和现有压力控制回归通过。

---

## 应用场景定位

### RBF-PID 更有优势的场景

- **塑化背压/储料背压：** 压力增益随材料黏度、熔体温度、螺杆位置和泄漏变化，过程非线性明显，且动作时间足够长，RBF 有时间建立局部模型。推荐优先试用 RBF-PI，保留固定 Ksys 基线。
- **射胶保压和 V/P 切换后的压力段：** 进入压力控制后，熔体压缩性、止逆环泄漏、模腔充填状态会改变局部增益。RBF 可在限定范围内修正 Kp/Ki，改善不同材料/模具的重复性。
- **压力受限的射胶过程：** 仅当压力环作为速度环的限幅/保护环时使用 RBF；主射胶速度环仍由现有速度控制器负责，避免把 RBF 直接放入高速位置/速度主回路。

### 不建议优先使用的场景

- 开合模、顶针和低压护模：安全边界优先，RBF 自适应收益小。
- 纯高速射胶速度主控：时间短、饱和多、激励不可控，RBF 学习容易被执行器限制污染。
- 传感器失效、泵配置无效或长期输出饱和：必须回退固定 PI/硬件保护。

RBF-PID 的竞争优势不是“任何工况都比 PI 快”，而是能在压力增益随工艺状态变化的非线性场合，用有界在线修正减少重复人工整定，同时保留 PI 的可解释性、限幅和故障回退。


## Task 6: 完善一阶模型 benchmark 和工程指标测试

**Files:**

- Modify: `tests/test_rbf_pid_pressure_benchmark.c`

- [ ] **Step 1: 明确 benchmark 单位和工况**

首个确定性基准使用：

```text
Ksys = 1.5 bar/rpm
flowToPumpSpeedGain = 20 rpm/(L/min)
Kflow = 30 bar/(L/min)
T = 1.0 s
Ts = 1 ms
无噪声、无纯滞后
目标：50、80、100 bar
```

仿真模型仍使用现有 `PressureModel` 一阶方程，不在 benchmark 中伪造实际反馈或修改控制周期。

- [ ] **Step 2: 增加完整指标统计**

对每个上升阶跃统计：

```text
10% -> 90% 上升时间；
相对 Pset 的最大超调百分比；
稳态最后 1 s 峰峰值；
最大正向流量和最大负向流量；
输出是否有限、是否触发硬件边界。
```

不得把下降阶跃上一平台的压力当作新平台的向上超调。

- [ ] **Step 3: 增加最小压力扰动覆盖**

同一测试目标增加：

```text
Ksys 无效：回退旧控制；
Ksys=1.5：验证 Kflow=30；
输出正向饱和：学习冻结；
过压进入 RELIEF：负流量快速通过；
压力反馈小扰动：RBF-PI 不产生持续放大纹波。
```

第一阶段通过标准为：无 NaN/Inf、无越界、闭环可回到目标、稳态纹波和超调在 benchmark 目标内。`<800 ms` 只有在给定泵/阀/体积参数能提供足够动态能力时才判定；代码不能仅凭 RBF 算法承诺所有机型都达到该时间。

---

## Task 5: 实现 Ksys 前馈、分阶段限幅和快速卸压

**Files:**

- Modify: `src/rbf_pid.c:386-518`
- Modify: `src/pressure_controller.c:330-403, 550-614`
- Test: `tests/rbf_pid_test.c`
- Test: `tests/test_pressure_controller.c`

- [ ] **Step 1: 使用无重复累加的 Ksys 前馈**

当 Ksys 有效时，用当前压力设定值计算流量基线：

```text
Qff(k) = targetFlow + targetPressure(k) / Kflow
ΔQff   = Qff(k) - Qff(k-1)
```

输出只叠加 `ΔQff`，不能每个扫描重复叠加 `Qff`。`targetFlow` 仍作为旧配方的保压偏置；Ksys 无效时完全保留旧输出行为。

- [ ] **Step 2: 接入压力控制器输入**

在 `HYD_PressureControllerInput` 末尾增加：

```c
HYD_REAL systemGainBarPerRpm;
```

`src/motion_control.c` 从 `fb->_params.ksysBarPerRpm` 传入；同时给 `HYD_ApplyRbfPidConfig()` 增加一个 `systemGainBarPerRpm` 形参，使 Ksys 不依赖隐藏全局状态。调用：

```c
RBF_PID_SetKsysBarPerRpm(&state->rbfPid,
                         (float)systemGainBarPerRpm,
                         (float)flowToPumpSpeedGain);
```

同步/跟踪路径也必须把同一个 `systemGainBarPerRpm` 传入，不能在 `HYD_SynchronizeRbfPidState()` 中清除 Ksys 配置。

RBF 路径使用已经完成传感器滤波的 `input->measuredPressure`，将该策略的二次滤波系数固定为 `1.0` 或直接旁路 `HYD_ResolveFilterAlpha()`；经典 P/PI/PID 路径的既有滤波行为不改。

旧 `segment.systemGain [bar/(L/min)]` 只保留给非新 Ksys 的兼容路径，不能覆盖有效的 IEC Ksys。

- [ ] **Step 3: 分阶段应用正向软限幅**

正向输出只在 HOLD/接近目标阶段可使用：

```text
Qcap = 1.05 * Pset / Kflow
```

规则：

- BOOST：不使用 Qcap，只使用 `outputMax` 和泵转换器硬限幅；
- HOLD：使用 `min(Qcap, outputMax)`，减少超调；
- RELIEF：不使用 Qcap，允许负流量快速卸压；
- Ksys 无效：不启用 Qcap。

该规则只改变 RBF 内部软限幅，不改变最终泵速保护。

- [ ] **Step 4: 修复负流量死区语义**

把 RBF 分支的 `fabs(error) < 5` 改成只针对欠压/过压方向的统一判断：

```text
只有误差表示“未超过卸压死区”时才禁止负流量；
一旦进入 RELIEF，负流量直接通过到 outputMin。
```

不得因为 Ksys 软限幅削弱负向命令。

- [ ] **Step 5: 实现无扰跟踪而不丢网络**

`HYD_SynchronizeRbfPidState()` 改为调用 `RBF_PID_TrackOutput()`：

```text
更新 u_prev、Output、e_prev1/e_prev2、y_prev1/y_prev2；
清空 du 和前馈历史；
保留 c、b、w、base gains、confidence；
```

只有显式完整 Reset 才重建 RBF 网络。增加测试断言切换前后网络权重保持一致、输出变化在一个扫描允许范围内。

---

## Task 1: 锁定现有行为和失败基线

**Files:**

- Test: `tests/rbf_pid_test.c`
- Test: `tests/test_pressure_controller.c`
- Test: `tests/test_rbf_pid_pressure_benchmark.c`

- [ ] **Step 1: 记录当前基线命令和已知失败**

```bash
cmake --build --preset unixgcc
ctest --test-dir out/build/unixgcc -R 'test_pressure_controller|test_rbf_pid|test_rbf_pid_pressure_benchmark' --output-on-failure
```

记录当前 `test_rbf_pid` 中前馈开关断言失败，不通过修改断言来掩盖问题。

- [ ] **Step 2: 为每个新行为先添加失败测试**

至少先添加以下测试入口：

```c
static void test_ksys_bar_per_rpm_converts_to_flow_domain(void);
static void test_ksys_feedforward_is_applied_as_delta_without_reaccumulation(void);
static void test_ksys_soft_cap_is_disabled_during_boost_and_relief(void);
static void test_relief_keeps_full_negative_command_path(void);
static void test_rbf_learning_freezes_on_same_direction_saturation(void);
static void test_tracking_preserves_rbf_network_state(void);
```

- [ ] **Step 3: 运行单测确认新增断言失败**

```bash
cmake --build --preset unixgcc --target rbf_pid_test test_pressure_controller
./out/build/unixgcc/rbf_pid_test
./out/build/unixgcc/test_pressure_controller
```

Expected: 新测试因 API/行为尚未实现而失败，现有无关测试不应被删除或放宽。

---

## Task 2: 增加 IEC Ksys 数据链路

**Files:**

- Modify: `include/common_types.h:559-626`
- Modify: `src/motion_control.c:3560-3590, 4120-4220`
- Test: `tests/test_parameter_access.c`
- Test: `tests/test_parameter_iec.c`

- [ ] **Step 1: 追加稳定参数号和字段**

在 `HYD_PARAM_COUNT` 前追加：

```c
HYD_PARAM_KSYS_BAR_PER_RPM,
```

把它放在已有枚举末尾、`HYD_PARAM_COUNT` 之前。将字段追加到 `HYD_MotionFBParams` 末尾：

```c
HYD_REAL ksysBarPerRpm; /* bar/rpm; 0 disables Ksys feedforward */
```

不要插入旧枚举中间，也不要重命名现有的 `systemGain`，因为它仍是旧配方的 `bar/(L/min)` 兼容字段。

- [ ] **Step 2: 初始化和读写映射**

在 FB 初始化中将 `ksysBarPerRpm = 0.0`。在 `HYD_MotionControlFB_ReadParameter()` 和 `HYD_MotionControlFB_WriteParameter()` 增加：

```c
case HYD_PARAM_KSYS_BAR_PER_RPM:
    *value = fb->_params.ksysBarPerRpm;
    break;
```

写入时接受 `0` 作为禁用；非有限值和负值返回 `false`，正值写入后才返回成功。使用现有错误返回约定，不新增错误码。

- [ ] **Step 3: 验证 IEC 通用链路**

测试 `HYD_WRITEPARAMETER.PARAMETERNUMBER = HYD_PARAM_KSYS_BAR_PER_RPM` 的上升沿行为，并通过 `HYD_READPARAMETER` 读回同一值。覆盖：

```text
1.5  -> DONE=true，读回 1.5
0.0  -> DONE=true，表示禁用
-1.0 -> ERROR=true，原值不变
NaN  -> ERROR=true，原值不变
```

---

## Task 3: 建立 RBF-PID 的单位和固定周期契约

**Files:**

- Modify: `include/rbf_pid.h:61-154, 220-238`
- Modify: `src/rbf_pid.c:1-16, 549-576, 623-684`
- Test: `tests/rbf_pid_test.c`

- [ ] **Step 1: 增加明确的物理状态字段和 API**

追加以下字段，避免复用含义不清的兼容槽：

```c
float system_gain_bar_per_rpm;
float process_gain_bar_per_flow;
float feedforward_flow;
float feedforward_flow_prev;
float base_KP;
float base_KI;
float base_KD;
float adaptation_confidence;
bool ksys_valid;
```

声明：

```c
void RBF_PID_SetKsysBarPerRpm(RBF_PID_Handle *pid,
                              float ksys_bar_per_rpm,
                              float flow_to_pump_speed_gain);
void RBF_PID_TrackOutput(RBF_PID_Handle *pid,
                         float output_flow,
                         float setpoint,
                         float feedback);
```

`RBF_PID_SetGainCompensation()` 暂时保留给旧流量域调用者，但新压力控制路径不得把 `bar/rpm` 传给它。

- [ ] **Step 2: 实现唯一单位换算入口**

在 `RBF_PID_SetKsysBarPerRpm()` 中只接受有限且大于零的两个输入：

```text
process_gain_bar_per_flow =
    ksys_bar_per_rpm * flow_to_pump_speed_gain;
```

若输入无效，清除 `ksys_valid`、将 `process_gain_bar_per_flow` 置零，控制器回退到无 Ksys 旧行为。不得用指令泵速代替实际反馈，因为第一阶段 Ksys 是离线固定标定值。

- [ ] **Step 3: 固定 RBF 周期**

定义单一常量：

```c
#define RBF_PID_FIXED_SAMPLING_PERIOD 0.001f
```

`RBF_PID_Init()` 将任何传入周期归一到该固定值，并保留原调用签名以减少 ABI 变化。`RBF_PID_Update()` 使用离散每扫描量的 `KI/KD`，不读取外部动态 `dt`。

- [ ] **Step 4: 添加单位契约测试**

输入 `Ksys=1.5`、`flowToPumpSpeedGain=20`，断言：

```c
assert(fabsf(pid.process_gain_bar_per_flow - 30.0f) < 1e-6f);
assert(pid.ksys_valid);
```

输入 `0`、负数、无穷和 NaN，断言控制器安全回退且不会产生 NaN 流量。

---

## Task 4: 修正 RBF 局部模型和有界自适应

**Files:**

- Modify: `src/rbf_pid.c:267-384, 400-452`
- Modify: `include/rbf_pid.h`
- Test: `tests/rbf_pid_test.c`

- [ ] **Step 1: 修改 3 维输入语义**

在 `rbf_pid_step_rbf_nn()` 中使用：

```c
x[0] = pid->u_prev / flow_scale;
x[1] = pid->y_prev1 / pressure_scale;
x[2] = (pid->y_prev1 - pid->y_prev2) / pressure_rate_scale;
```

保留当前 `RBF_HNUM=6` 和权重/中心/宽度限幅。`x[0]` 改为绝对流量后，Jacobian 的物理解释是 `dP/dQ`，不再把 `dP/dΔQ` 与静态 Ksys 混为一谈。

- [ ] **Step 2: 以 Kflow 作为 Jacobian 先验**

当 Ksys 有效时，使用 Kflow 作为安全基准；RBF 网络只提供有限修正：

```text
J_network = dP/dQ from RBF
J_effective = clamp(
    (1 - confidence) * Kflow + confidence * J_network,
    0.5 * Kflow,
    2.0 * Kflow)
```

当 Ksys 无效时，使用有限的网络 Jacobian；网络 Jacobian 非有限或接近零时使用保守固定值并冻结学习。`adaptation_confidence` 由最近窗口的预测残差和输入激励更新，范围固定为 `[0,1]`，不新增 IEC 参数。

- [ ] **Step 3: 限制参数自适应范围和速度**

在初始化和每次配置应用时保存 `base_KP/base_KI/base_KD`。在线更新满足：

```text
Kp ∈ [min_KP, max_KP]
Ki ∈ [min_KI, max_KI]
Kd ∈ [min_KD, max_KD]
|ΔKp| ≤ 1% of configured range per scan
|ΔKi| ≤ 1% of configured range per scan
```

第一阶段默认冻结 `Kd` 在线学习；PID 模式仍可使用配方给出的固定 `Kd`。RBF-PI 继续强制 `Kd=0`。这样保留自适应性，同时避免微分参数在噪声下漂移。

- [ ] **Step 4: 统一学习冻结条件**

以下任一条件成立时冻结 RBF 权重、中心、宽度和 PID 参数：

```text
输出同方向饱和；
输出或压力不是有限值；
RELIEF/超压卸压阶段；
输入激励不足；
模型置信度低于内部阈值。
```

冻结不等于停止输出；控制器继续按基础增量 PID 和硬件限幅运行。

- [ ] **Step 5: 保留 RBF 网络的有限回退**

每次学习和 Jacobian 计算后都执行有限值和边界检查。若检查失败：

```text
恢复最近一次有限网络快照；
process_gain_bar_per_flow 回退到 Kflow；
KP/KI/KD 回退到 base_KP/base_KI/base_KD；
冻结学习，继续输出有限的 PID 指令。
```

---


## 设计边界

### 已锁定的约束

- RBF-PID 控制周期固定为 `Ts = 0.001 s`，由嵌入式系统保证；RBF-PID 不根据调用时间戳改变离散控制律。
- 压力输入已经在传感器数据处理层完成滤波。RBF-PID 不再叠加默认低通；已有经典控制器滤波配置保持兼容。
- 卸压需要快速响应。RBF-PID 不对负流量增加 Ksys 静态软限幅；负流量仅受压力控制器输出下限、泵转换器负转速比例和最终硬件保护限制。
- IEC 写入的 Ksys 单位固定为 `bar/rpm`。控制器内部另存换算后的 `Kflow [bar/(L/min)]`，禁止混用。
- 第一阶段不增加 Smith 预估器、在线 Ksys 标定、复杂多模型状态机、动态分段参数和新的神经网络层。
- Ksys 第一阶段按“每个 FB/共享泵的一项静态参数”处理；不增加每个压力段的 Ksys 覆盖字段。后续若实机证明不同段确实需要不同 Ksys，再单独扩展。

### 推荐的控制结构

```text
IEC Ksys[bar/rpm]
        |
        v
Kflow = Ksys * flowToPumpSpeedGain  [bar/(L/min)]
        |
        +--> Qff = targetPressure / Kflow       (仅 Ksys 有效时)
        +--> RBF Jacobian 物理量程基准
        +--> 接近目标阶段的正向软限幅

压力误差 --> 有界增量 PI/PID --> ΔQ --> Q(k)=Q(k-1)+ΔQ+ΔQff
                 ^
                 |
          RBF 局部过程模型
          只在可辨识、未饱和、非泄压阶段更新
```

RBF 网络输入仍保持 3 维以避免结构性 ABI 扩展，但语义调整为：

```text
x[0] = Q(k-1) / Q_scale              // 绝对流量，便于估计 dP/dQ
x[1] = P(k-1) / P_scale              // 工作压力
x[2] = dP(k-1) / dP_scale            // 压力变化趋势
```

网络输出继续预测当前压力的归一化值，Jacobian 改为物理意义明确的 `dP/dQ [bar/(L/min)]`。RBF 不直接生成不受约束的流量指令。

固定 1 ms 下，PID 参数明确采用“离散每扫描量”的语义，控制律为：

```text
de  = e(k) - e(k-1)
dde = e(k) - 2*e(k-1) + e(k-2)
du  = Kp*de + Ki_discrete*e(k) + Kd_discrete*dde
Q(k)=clamp(Q(k-1) + du + (Qff(k)-Qff(k-1)), Qmin, Qmax)
```

不在该模块中乘除可变 `dt`。现有 `sampling_period` 仅保留为固定周期配置和诊断字段，必须等于 `0.001`。

---

## 文件责任划分

### 修改文件

- `include/common_types.h`
  - 追加 `HYD_PARAM_KSYS_BAR_PER_RPM`，放在枚举末尾以保持旧参数号不变。
  - 在 `HYD_MotionFBParams` 末尾追加 `ksysBarPerRpm`，单位明确为 `bar/rpm`，`0` 表示无 Ksys 前馈。
- `include/rbf_pid.h`
  - 追加 Ksys/Kflow、前馈历史、基础增益和学习可信度字段。
  - 声明 `RBF_PID_SetKsysBarPerRpm()`、`RBF_PID_TrackOutput()`。
  - 把旧 `RBF_PID_SetGainCompensation()` 标记为流量域兼容接口，不再把其参数称为 Ksys。
- `src/rbf_pid.c`
  - 重写 RBF 输入、物理 Jacobian、Ksys 换算、前馈、分阶段限幅、学习冻结和固定 1 ms 离散 PID。
  - 保留静态分配、无动态内存、现有 `RBF_HNUM=6` 和结构体多实例能力。
- `include/pressure_controller.h`
  - 在 `HYD_PressureControllerInput` 末尾追加 `systemGainBarPerRpm`。
- `src/pressure_controller.c`
  - 把 `systemGainBarPerRpm` 传给 RBF。
  - RBF 分支不再丢弃 `feedforwardFlow`。
  - 切换时调用 `RBF_PID_TrackOutput()`，只复位误差/输出历史，不重建已学习的 RBF 网络。
  - 只在 BOOST/HOLD 的正向接近目标阶段启用 Kflow 软限幅；RELIEF 阶段不使用该软限幅。
- `src/motion_control.c`
  - 初始化、读取、写入 `ksysBarPerRpm`。
  - 组装压力输入时传递 FB 参数中的 Ksys。
- `src/motion_interface.c`
  - 复用现有通用读写链路；仅补充参数号验证测试，不新建 IEC FB。
- `tests/rbf_pid_test.c`
  - 增加固定周期、Ksys 换算、前馈增量、饱和冻结、快速卸压和异常回退测试。
- `tests/test_pressure_controller.c`
  - 增加压力控制器到 RBF 的 Ksys 传递、RBF 前馈叠加和切换保留网络状态测试。
- `tests/test_parameter_access.c`、`tests/test_parameter_iec.c`
  - 增加 IEC `ReadParameter/WriteParameter` 对 Ksys 的读写、零值禁用和非法值拒绝测试。
- `tests/test_rbf_pid_pressure_benchmark.c`
  - 增加 `Ksys=1.5 bar/rpm`、`Kflow=30 bar/(L/min)` 的一阶模型指标统计，补充 10%->90% 上升时间。

### 不修改文件

- `src/pump_converter.c` 和 `src/output_limiter.c` 的硬件负转速比例。它们继续作为最终安全边界。
- `PressureModel` 物理模型。第一阶段只扩展 benchmark 和指标，不改变模型方程。
- 非压力闭环的速度、位置、开合模、顶针控制路径。

---
