# 压力闭环控制算法仿真 · 低压纹波抑制设计文档

- 日期：2026-08-02
- 背景：立式注塑机伺服泵控液压系统（伺服电机→齿轮泵→电磁换向阀→油缸活塞→压力）；数字压力传感器位于油泵出口。
- 目标：① 在现有代码基础上构建压力闭环控制算法的测试/仿真代码（含控制对象仿真模型、噪声与齿轮泵流量脉动干扰，输入=电机转速 rpm，输出=实际压力反馈 bar，系统增益 K=4.5，滞后 τ=100ms）；② 对比 RBF-PID / RBF-PI 与常规 PI 的仿真结果并给出数据分析；③ 验证“用电机实时位置补偿低压纹波”的思路是否可行。
- 关键决策（已与用户确认，全部采用推荐项）：
  1. 仿真被控对象**复用现有 `src/sim/PressureModel.c`** 的一阶分支（`model_type=FIRST_ORDER`）。
  2. 低压纹波采用**前馈抵消**：基于 `PressureModelState.pump_phase_rev`（电机/泵实时位置，单位转、已 wrap 到 [0,1)）预测齿落相位，在转速指令上叠加反向项。
  3. 交付形式为**新增 C 仿真 harness**（headless，可 CMake 编译、可命令行运行）。
  4. 对比报告**全部关键指标**（上升/调节时间、超调、稳态误差、压力纹波 RMS、抗扰/鲁棒性）。

## 1. 现有可复用资产

| 资产 | 文件 | 用途 |
|---|---|---|
| 被控对象模型 | `src/sim/PressureModel.c` (`PressureModel_Step`) | 一阶/物理模型；内部已维护 `pump_phase_rev`（泵位置，Z=13 个齿）；物理分支含流量脉动、齿落、传感器/电机/过程噪声 |
| 压力控制器 | `src/pressure_controller.c` (`HYD_PressureController_Execute`) | 支持 `HYD_PRESSURE_CONTROLLER_PI` / `RBF_PI` / `RBF_PID`；封装 `RBF_PID` 自适应模块 |
| 泵转换 | `src/pump_converter.c` (`HYD_PumpConverter_Execute`) | 流量[L/min] → 转速[rpm]（`flowToPumpSpeedGain` = rpm 每 L/min） |
| 闭环骨架 | `tests/test_pressure_model.c::run_closed_loop_pressure_case` | 控制器→泵转换→模型→反馈 的闭环步进模式，直接借鉴 |
| 类型定义 | `include/common_types.h`, `include/pressure_model.h`, `include/rbf_pid.h` | `HYD_MotionSegment`、`HYD_PressureControllerInput/Output`、`PressureModelParams/State/Output` |

注意：`PressureModel_Step` 一阶分支本身**不含**流量脉动与噪声（那是物理分支的特性）。为满足需求“叠加噪声和齿轮泵引起流量脉动干扰”，采用 **一阶被控对象（精确承载 K=4.5 / τ=0.1s）+ 在 harness 中注入与现有模型同形的扰动** 的组合（见 §2）。

## 2. 被控对象与扰动注入

### 2.1 一阶标称模型（承载增益与滞后）
- `model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER`
- `first_order_k_bar_per_rpm = 4.5`  ← 用户给定系统增益 K
- `first_order_tau_s = 0.1`          ← 用户给定滞后 100ms
- `first_order_delay_s = 0.0`（滞后已由 τ 体现，避免重复计入）
- `motor_tau_s = 0.0`（关闭电机转速低通，使脉动能抵达压力端）
- `min_rpm / max_rpm` 保持默认（-100 / 2000）

标称关系：`P(k) ≈ (K·rpm(k)·dt + τ·P(k-1)) / (τ+dt)`，增益为 4.5 bar/rpm，时间常数 0.1s。

### 2.2 齿轮泵流量脉动（干扰）
齿轮泵每转 Z=13 个齿产生一次流量脉动。利用模型内部 `pump_phase_rev`（单位：转，wrap 到 [0,1)）作为电机实时位置：
- 脉动相位 `φ = 2π · Z · pump_phase_rev`，`Z = 13`
- 转速指令上叠加流量脉动干扰：`rpm_cmd = rpm_ctrl + A_ripple · sin(φ)`
- 由此压力纹波频率 `f_ripple = Z · rpm / 60`，**周期 T = 60/(Z·rpm)** —— 与“低速时纹波周期与转速相关、且随转速降低而变长”的实测一致。
- `A_ripple`（rpm 幅值）为可调扰动强度，典型取使稳态压力纹波约 1~3% 设定值。

### 2.3 噪声（干扰）
- 传感器噪声：反馈给控制器的测量值叠加高斯噪声 `N(0, σ_sensor)`（bar）。
- 电机噪声（可选）：模型 `enable_motor_noise` 已提供，harness 可开启以模拟转速抖动。
- 注：一阶分支 `measured_pressure_bar == real_pressure_bar` 且无噪声，故传感器噪声在 harness 内对反馈量叠加（与 `enable_sensor_noise` 语义一致）。

## 3. 控制器配置（三策略对比）

统一通过 `HYD_MotionSegment` 配置，复用 `HYD_PressureController_Execute`：

| 字段 | PI | RBF_PI | RBF_PID |
|---|---|---|---|
| `pressureController` | `HYD_PRESSURE_CONTROLLER_PI` | `HYD_PRESSURE_CONTROLLER_RBF_PI` | `HYD_PRESSURE_CONTROLLER_RBF_PID` |
| `pressureKp` / `Ki` / `Kd` | 固定整定（如 Kp=0.05, Ki=0.005） | 由 RBF 自适应边界给出 | 由 RBF 自适应边界给出 |
| `pressureRbfConfig.min/maxKp/Ki/Kd` | — | 保守边界（如 Kp 0.04–0.06, Ki 0.0008–0.0016, Kd=0） | 含 Kd 边界（0.015–0.035） |
| `systemGain` | 0（不启用增益补偿） | 4.5（用户给定增益，供 RBF 输出补偿） | 4.5 |
| `pressureFilterAlpha` | 0.2（轻滤波） | 1.0（RBF 内部已处理） | 1.0 |
| `pressureCeiling` | `3·target` | `3·target` | `3·target` |
| `disablePressureAccelFeedforward` | — | 1.0 | 1.0（避免与我们的前馈补偿叠加） |

`flowToPumpSpeedGain = 20`（rpm 每 L/min），`pumpSpeedLimit = 1800` rpm（与现有测试一致，便于横比）。

## 4. 电机位置前馈补偿（核心验证点）

在闭环中引入开关 `use_feedforward` 与前馈增益 `K_ff`：

```
rpm_ctrl   = flow_to_rpm(output.outputFlow)              # 控制器输出
φ          = 2π · 13 · plant_state.pump_phase_rev        # 用“上一拍”泵位置预测齿落
rpm_ff     = use_feedforward ? K_ff · sin(φ) : 0.0
rpm_cmd    = rpm_ctrl + A_ripple·sin(φ) + rpm_ff          # 干扰 + 前馈抵消
PressureModel_Step(..., rpm_cmd, ...)                     # 模型推进
measured   = clamp(real_pressure + sensor_noise, 0, range)# 反馈给控制器
```

- 干扰项 `A_ripple·sin(φ)` 与补偿项 `K_ff·sin(φ)` 同相位，故**理想抵消时 `K_ff ≈ −A_ripple`**。
- 实现中 `K_ff` 做扫描（如 `−1.5·A_ripple … +0.5·A_ripple`），绘制“压力纹波 RMS 随 K_ff 变化”曲线，给出最优 `K_ff` 与可达的纹波抑制比。
- 若 `K_ff` 取最优附近能显著降低纹波 RMS，则证明“获取电机实时位置进行补偿”在原理与数据上均成立。

## 5. 指标（全部关键指标）

对每条闭环记录时间序列（dt=1ms，仿真时长 8s，取后 2s 为稳态窗）：

1. **上升时间**：压力首次越过 0.9·target 的时间（自启动起）。
2. **调节时间**：最后一次越出 ±2% 带宽的时刻（之后保持在内）。
3. **超调量**：`(max(P) − target)/target · 100%`。
4. **稳态误差**：稳态窗内 `mean(P) − target`。
5. **压力纹波 RMS**：稳态窗内压力标准差（`std`）；同时报告峰峰值 p2p。
6. **抗扰/鲁棒性**：
   - 增益失配：被控对象实际 `K=5.4`（与标称 4.5 失配），对比三控制器的误差/纹波退化。
   - 阶跃扰动：稳态后注入一次流量阶跃（模拟负载突变），测恢复时间与被扰峰值偏差。

## 6. 文件布局与构建

- 新增：`tests/sim_pressure_control.c`（headless 仿真 harness，含上述闭环、三控制器、前馈扫描、指标、CSV 输出、结果打印）。
- 新增（可选）：`tests/sim_output/*.csv`（每次运行输出，供绘图）。
- 构建：在 `CMakeLists.txt` 增加
  `add_executable(sim_pressure_control tests/sim_pressure_control.c)` +
  `target_link_libraries(sim_pressure_control PRIVATE HydroMotionLib ${HYD_THREAD_LIB})` +
  `add_test(NAME sim_pressure_control COMMAND sim_pressure_control)`。
- 运行：`cmake --build --preset mingw && ctest --test-dir out/build/mingw -R sim_pressure_control`。
- 可视化伴侣：`docs/pressure_control_companion.html`（已有概念版，运行时回填 §④ 对比表与真实曲线）。

## 7. 第二步数据分析计划

运行后得到三组数据，分析并回答：

- **RBF-PID / RBF-PI 是否优于常规 PI？** 从指标表逐项对比，重点看：调节时间、超调、稳态误差、纹波 RMS、抗扰/鲁棒性。预期 RBF 类在增益失配/扰动下退化更小（自适应增益），但需注意 RBF 对学习率/边界更敏感。
- **低压纹波与转速的关系**：从数据验证 `T = 60/(13·rpm)`，并展示不同转速下纹波周期。
- **前馈补偿有效性**：`K_ff` 扫描曲线显示纹波 RMS 在 `K_ff≈−A_ripple` 处最小，给出抑制百分比；结论“用电机实时位置补偿低压纹波可行”。
- 所有结论均**以数据驱动**，并标注数据时效性与仿真假设（一阶+扰动注入近似、离散步长 1ms）。

## 8. 开放参数（实现时可微调）

- `A_ripple`（脉动幅值 rpm）、`σ_sensor`（传感器噪声 bar）、`flowToPumpSpeedGain`、`pumpSpeedLimit`、控制器整定与 RBF 边界、仿真时长与稳态窗、目标压力（默认 100 bar）。
- 若 RBF 在 K=4.5 下 `systemGain` 语义需调整，以现有测试（`5.4·flow_to_speed_gain`）为参照校准。
