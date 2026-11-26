# 速度段限压改为压力环取小（pQ override）— 设计文档

日期：2026-11-26
状态：已确认（2026-11-26，§13）
关联：`2026-05-26-pressure-limit-soft-position-limit-design.md`（被替换的比例限速）、`2026-11-23-pressure-loop-2dof-ripple-comp-design.md`（保压 PI 整定公式）

---

## 1. 目标与验收口径

位置/速度段（MoveAbsolute、MoveVelocity、MoveContinuousAbsolute、Stop 减速）在实际压力达到 PRESSURELIMIT 时转入压力环，压低流量，把压力稳定在限压值；负载下降后自动、连续地回到速度控制。

验收 A（用户 2026-11-26 选定）：

- 稳态：限压生效后压力稳定在 Plim ± 2 bar；
- 瞬态：越限峰值 ≤ 1.1 × Plim，靠上机整定参数 40/41 达到；
- 不做压力预判制动。硬堵转（型腔瞬间充满）峰值受伺服延迟和斜率限制，可能超过 10%，由工艺层降低末段射速解决（§9）。

## 2. 证据

数据：`docs/data_deg/vofa_waveform_20261125_115208_697.csv`，1 ms 采样。

**单位**：泵转速一列 170 对应 1700 rpm，反转卸压限幅 −13.6 对应 −136 rpm（= 0.08 × 1700）。所以记录单位是 **10 rpm/计数**，不是 0.1 rpm。下文按 ×10 换算。

| 射出段 | 限压 | 越限时刻 | 段末压力 | 段末泵转速 | 螺杆速度 |
|---|---|---|---|---|---|
| 8367–8876 ms | 100 bar | 8615 ms | 124.3 bar | 217 rpm | 7 mm/s |
| 8879–9323 ms | 140 bar | 8930 ms | 179.7 bar | 154 rpm | 4.6 mm/s |

- 用现有公式 `scale = 1 − 3·(P − Plim)/Plim` 反算段末转速：
  - 第一段：866 × 0.271 ≈ 235 rpm，实测 217 rpm；
  - 第二段：1046 × 0.151 ≈ 158 rpm，实测 154 rpm。
- 结论：现有限幅器是纯比例控制，稳态误差约为 Plim·(1 − 保压所需流量/请求流量)/3，与实测的 40 bar 一致。越限时的瞬态只有约 10 bar，加大 Kp 只能缩小稳态误差，消不掉。
- 换段时限幅状态被清零（`HYD_ResetCriteriaForSegment` → `HYD_OutputLimiter_ResetState`）。8879 ms 泵指令在 3 ms 内从 217 rpm 跳到 868 rpm。
- **伺服实测**：
  - 指令 1000 → 126 rpm 阶跃时，实际转速约 6 ms 后才开始变化，下降斜率约 35 000 rpm/s；
  - 升速斜率约 20 000–30 000 rpm/s；
  - 回路总延迟（采样 + 伺服）按 5–8 ms 估计。

## 3. 决策

### 3.1 行业做法

调研来自搜索摘要，没有逐页核对原文。

- **Rexroth SYDFE**：p/Q 控制器输出取小，"只有对应当前工作点的控制器起作用"。
- **Rexroth MLC Hydraulics**：力限制激活后运动命令继续执行，程序等"到位"或"到力限"中的任意一个。
- **Moog 651/D941 阀**：流量控制上叠加压力限制。
- **Beckhoff TF5810**：取代型压力调节（ablösende Druckregelung）。
- **专利 US8871128**：速度环需求经低选器限制在压力环需求以下。
- **抗积分饱和**：Lipták 的外部复位反馈，即未被选中的控制器跟踪当前实际输出。
- **PLCopen Part 6 MC_LimitLoad**：伴随运动块工作。触限时运动块保持 Busy、不出 Done，由应用层处理。
- **注塑工艺（KEBA、海天/弘讯、Demag）**：位置作为转保压主判据，最大注射时间作为后备；超时则转保压或报"射出超时/射出监控失败"。

### 3.2 选定方案

| 方案 | 结论 |
|---|---|
| 保留比例缩放，加大 Kp | 否。稳态误差是结构性的 |
| **速度、压力同时计算，取最小值** | **是**。行业共识，只改一处，覆盖所有速度类块 |
| 超压后状态机切到 PressureHandle 段 | 否。要设进出阈值，切换有冲击，运动块会被 CommandAborted |
| 新增 Hyd-LimitLoad 功能块 | 否。现有块已有 PRESSURELIMIT 输入；新增块会带来同轴多块所有权仲裁问题 |

### 3.3 不复用 `HYD_PressureController_Execute`

1. **跟踪方式不对，这是决定性原因**。现有跟踪为 I = Q − Kp·e（无扰切换），取小结构需要 I = Q（外部复位反馈）。
   仿真对比（Plim = 140 bar）：
   - 现有跟踪：62 bar 就开始抢控制；
   - 外部复位反馈：139.3 bar 才首次接管。
2. **RBF 会每拍重新初始化**：跟踪请求会触发 RBF 自适应复位（`needsAdaptiveReset = … || trackingRequested`）。
3. **模型部分在限压时都派不上用场**：
   - 参考轨迹和 8 拍参考延迟：参考是常数，延迟只会让接管晚 8 ms；
   - 模型前馈 ṙ/Kq：ṙ = 0，恒为零；
   - 泄漏前馈：被积分吸收；
   - 纹波 LMS：系数是保压段的 FB 公共状态，在速度段学习会污染保压段；
   - 反转卸压：速度段不允许泵反转。
4. **整定目标不同**：保压要平稳，取 fc = 5 Hz；限压要快速压低流量，取 fc = 10 Hz（§5）。
5. **内存**：再开一份控制器状态，每轴多 736 B（§6）。

## 4. 设计

### 4.1 结构（`src/output_limiter.c`）

```
Q_v   = requestedFlow × derate × softLimitScale        // 速度链路，原样保留
e     = Plim − P                                        // P 不滤波
Q_p   = Kp·e + I
Q_cmd = clamp(min(Q_v, Q_p), minFlow, Qmax)             // 速度段 minFlow = 0
selected = (Q_p < Q_v)
if selected:  I ← clamp(I + Ki·e·dt, 0, Qmax)           // 积分只在压力环被选中时累加
else:         I ← Q_cmd                                 // 外部复位反馈：跟踪本拍实际输出
pumpSpeed = Q_cmd × G，然后照旧按泵转速上限裁剪
```

- **对应 2DOF 结构**：本 PI 就是 2DOF 中的反馈 PI，Kp、Ki 的单位和整定公式与参数 11/14 相同。前馈的角色由速度链路 Q_v 承担：通过外部复位反馈，积分始终等于实际执行的流量。参考固定为 Plim，所以没有参考轨迹，也没有模型前馈。
- **接管点**：未选中时 Q_p = Q_cmd + Kp·e。只有当 Q_v 单拍的增量超过 Kp·e 时才会提前接管，所以压力越接近限值，越早约束流量的增长，到 Plim 时完全接管。
- **退出限压**：负载下降后 e 变大，Q_p 连续上升，超过 Q_v 后回到速度控制，不会跳变。原来的回差、慢恢复、快恢复三套逻辑全部删除。
- **dt**：由调用方通过输入传入 `deltaTime`（motion_control 中已有该变量），钳到 [0.001, 0.1] s，与现行做法一致。
- **压力段**：`effectiveMaxPressure = 0`，本逻辑不生效，保压路径一行不动。

### 4.2 跨段连续性

- 积分 I 用 `fb->_lastCommandedFlow`（上一拍实际执行的流量）作为未选中时的跟踪源，不依赖限幅器内部保存的旧值。所以 `ResetState` 在段开始、实时更新、直接接管时清零无害：第一拍 Q_p = Q_last + Kp·e，从上一段实际流量继续。
- 直接接管时，限压 PI 被选中的情况下，`HYD_CaptureDirectContinuityState` 保留实际执行流量和泵转速（包括 0），不用规划流量代替；否则接管后跟踪源会退回限压前的流量（T3c）。
- 例（8879 ms 数据）：换段时 Plim 由 100 改为 140，P = 124，实际流量 217 rpm。新逻辑首拍 Q_p = 217 + 20·16 ≈ 537 rpm，而不是 868 rpm（T3）。注意 P = 124 是旧比例限幅器的稳态。新逻辑下第一段会稳在 P ≈ 100，换段后 Q_p = Q_last + Kp·40 已接近或高于 Q_v，泵几乎按 Q_v 全速升压，直到 P 接近 140 才由压力环重新接管（T3b）。这是外部复位反馈"有压力余量就不限流"的设计行为，瞬态相当于在新 Plim 处硬堵转，处理见 §9 上机注意第 1 条。

### 4.3 运动块语义与状态位

- 限压期间运动块保持 Busy，不出 Done，不报 CommandAborted/Error，与 MC_LimitLoad 一致。
- `STATE.limitFlags` 的 `HYD_LIMIT_FLAG_PRESSURE` = selected，通过已有的 `HYD_ReadStatus.PRESSURELIMITACTIVE` 输出。不新增诊断码，不升级故障。
- 堵转判定、转保压、超时报警由工艺层负责（§9）。

### 4.4 参数与接口

| 编号 | 名称 | 单位 | 0 的含义 | 范围 |
|---|---|---|---|---|
| 40（新） | HYD_PARAM_PRESSURE_LIMIT_KP | L/min/bar | 用 §14B 默认值 1.0 | 有限值，≥ 0 |
| 41（新） | HYD_PARAM_PRESSURE_LIMIT_KI | L/min/(bar·s) | 用 §14B 默认值 16 | 有限值，≥ 0 |

- 枚举追加在 `HYD_PARAM_PUMP_RIPPLE_ORDER`（39）之后，`HYD_PARAM_COUNT` 加 2；`HYD_MotionFBParams` 新增 `pressureLimitKp`、`pressureLimitKi`，初值为 0。
- `HYD_OutputLimiterInput` 新增 `pressureLimitKp`、`pressureLimitKi`、`previousCommandFlow`（= `fb->_lastCommandedFlow`，§4.2 的跟踪源）、`deltaTime`。
- `HYD_OutputLimiterState`：
  - 删除 `pressureScale`、`pressureScaleLastTime`；
  - 新增 `pressureLimitIntegral`；
  - `pressureLimitActive` 改为表示"压力环被选中"。
- **§14B 常数**：
  - 新增 `HYD_THRESH_PRESSURE_LIMIT_PI_KP 1.0f`、`HYD_THRESH_PRESSURE_LIMIT_PI_KI 16.0f`；
  - 删除 `HYD_THRESH_PRESSURE_LIMIT_KP / _MIN_SCALE / _HYSTERESIS_RATIO / _SCALE_RELEASE_RATE / _SCALE_FAST_RELEASE_RATE`（已确认只有 output_limiter.c 使用）；
  - `_DEBOUNCE_S`、`_FAULT_ESCALATION_S` 原注释写明"为源码兼容保留"，不动。
- **软限位**：`softLimitActive` 的判定改为"软限位系数 < 1 且软限位使 Q_v 成为最终输出"。逻辑等价，只是比较对象从 pressureScale 换成 Q_p。

### 4.5 转保压时的流量初值修正（`motion_control.c`）

- **问题**：`HYD_PrimeSegmentControllers` 在速度/位置段转压力段续接时，先取 `_plannerState.lastTargetFlow`，这是限幅前的流量。直接接管路径（`HYD_CaptureDirectContinuityState` / `Restore`）同样携带这个值。
- **后果**：堵转转保压时，保压 PI 会以约 1040 rpm 而不是约 150 rpm 作为初值。
- **改法**：该分支改为优先取 `_lastCommandedFlow`（实际执行的流量），为 0 时再退回 `lastTargetFlow`。用测试固定（§8 T5）。

### 4.6 兼容性

- PRESSURELIMIT = 0 且 `fb.PRESSURE_LIMIT` = 0 时，输出与现在逐位相同。例外：§4.5 的转保压初值改为优先取实际执行流量。软限位减速带、DERATE 等使实际流量低于规划流量时，即使不限压，转保压初值也会与现在不同（取实际流量，属于修正）。
- 压力段、RBF、2DOF、纹波补偿不改。
- IEC 功能块接口不变，只增加参数号。
- 现有依赖比例缩放数值的测试要改写（§8）。

## 5. 整定与默认值

### 5.1 公式（与 2DOF §8.2 同形）

```
G   = flowToPumpSpeedGain = 20 rpm/(L/min)（50 cc/rev）
Kp  = 2π·fc_lim / (Kq_max·G)        [L/min/bar]
Ki  = Kp · 2π·fc_lim / 4            [L/min/(bar·s)]，即 Ti = 4/(2π·fc_lim) ≈ 64 ms
fc_lim = 10 Hz
```

**Kq 取工作压力区间的最高值，与参数 37 的规则相反**：
- 前馈用 Kq 时，高估会导致超调，所以参数 37 取 0.9 × 最低值；
- 反馈增益 Kp ∝ 1/Kq，高估 Kq 只会让回路变慢，低估才可能失稳。

### 5.2 默认值

**选定**：Kp = 1.0、Ki = 16，按油阀全关 Kq = 3.2 bar/s/rpm 设计。

- **为什么用油阀全关的 Kq 设计**：同一泵上任何轴打开阀口，都只会增加受压容积，所以 Kq 都不高于油阀全关时的值（按 hor1121：Kq 2.9–3.2，140–160 bar 时升到约 4.5）。
- 因此默认值对本泵所有轴都稳定，不辨识也能直接用。

线性回路校核（PI + 积分对象 + 纯延迟 Td）：

| 对象 Kq | 增益 | 穿越频率 | 相位裕度（Td = 5 / 8 ms） |
|---|---|---|---|
| 3.2（油阀全关） | 默认 1.0/16 | 10.3 Hz | 58° / 47° |
| 4.5（油阀全关，160 bar） | 默认 1.0/16 | 14.3 Hz | 54° / 39° |
| 1.45（射出估计） | 默认 1.0/16 | 5.1 Hz | 55° / 49° |
| 1.45（射出估计） | 射出整定 2.2/35 | 10.3 Hz | 58° / 47° |
| 2.9（射出估计偏低一半） | 射出整定 2.2/35 | 20 Hz | 47° / **25°** |

- **射出轴整定值**：Kp ≈ 2.2、Ki ≈ 35，基于 Kq_inj,max ≈ 1.45 的粗估。
- 这个粗估只来自 9385–9440 ms 的一个卸压窗口：dP/dt = −182 bar/s，n = −126 rpm，P = 170 bar，算得 Kq = 1.15–1.45，取决于是否计入泄漏项。
- 表中最后一行说明，粗估偏低时相位裕度只剩 25°。所以**必须按 §10 辨识后再写 40/41**；辨识前用默认值。

## 6. 嵌入式资源

测量方法：主机 gcc、float 构建（`HYD_REAL_PRECISION_FLOAT 1`）下的 `sizeof`。ARM 上对齐可能有 ±4 B 差异。

| 项 | 现状 | 本方案 | 若复用压力控制器 |
|---|---|---|---|
| `HYD_MotionControlFB` / 轴 | 3432 B | +8 B（参数 +8；限幅状态 24 → 20 B，−4 被对齐吃掉；原型实测 3440 B） | +736 B |
| 20 轴 FB 池（`.motionAlgoSection`） | 68.6 KB | +160 B | +14.7 KB（+21%） |
| 栈：`HYD_OutputLimiterInput` | — | +12 B | — |
| 代码 | 比例缩放 + 回差 + 双速恢复约 110 行 | PI 约 30 行，代码量净减少 | — |
| 每轴每拍运算 | — | 约 6 次 float 乘加 + 2 次比较；用 Ki 形式，没有除法 | RBF 前向计算 |

**编码约束**：
- 新代码全部用 `HYD_REAL` 和带 `f` 后缀的常量，避免 float 构建下隐式提升为 double（Cortex-M 上 double 是软件模拟）；
- 不加滤波器、延迟线、查表，不用 malloc；
- 输入非有限值（NaN/Inf）时沿用现有 `IsFinite` 检查，输出 0。

## 7. 仿真验证（临时脚本，不入库）

**模型**：
- 射出缸：dP/dt = Kq·(n − P/ksys − kA·v)；熔体阻力按数据粗略拟合；
- 叠加 1.2 bar 的 12 阶纹波和 0.2 bar 噪声；
- 伺服采用实测特性：5 ms 纯延迟，降速 35 000 rpm/s，升速 30 000 rpm/s；
- 工况：Plim = 140 bar，请求转速 1040 rpm。

**模型校核**：同一模型跑现有比例限幅器，压力稳定在 183 bar，实测 180 bar，稳态行为能复现。

| 对象 Kq | 现有限幅器 峰值/稳态 | 默认 1.0/16 峰值/稳态 | 射出整定 2.2/35 峰值/稳态 |
|---|---|---|---|
| 1.2，渐进堵转 | 183.5 / 183.3 | 155.3 / 139.5 | **147.8 / 139.6** |
| 1.45，渐进堵转 | 183.4 / 183.3 | 155.1 / 139.4 | **147.7 / 139.4** |
| 3.2，渐进堵转 | 183.3 / 183.0 | 154.7 / 139.7 | 147.5 / 139.8 |
| 1.2，硬堵转 | 184.0 / 173.1 | 171.7 / 139.6 | 158.0 / 139.3 |
| 3.2，硬堵转 | 188.7 / 180.9 | 183.8 / 139.6 | 173.0 / 139.7 |

- **稳态**：所有情况都在 140 ± 0.6 bar，压力标准差 0.6–0.95 bar。
- **渐进堵转**（与实测相似）：用射出整定值时峰值 ≤ 1.06 Plim，满足验收 A；默认值约 1.11 Plim，略超。
- **硬堵转**：伺服 5–6 ms 延迟加斜率限制，泵从 1040 rpm 降到约 150 rpm 需要约 30 ms，这期间压力按约 Kq·Δn 上升。PI 消不掉这部分超调，由工艺层降低末段射速解决（§9）。
- **外部复位反馈的接管点**：139.3 bar；采用现有跟踪方式时为 62 bar（§3.3）。

## 8. 测试（ctest）

- **T1 `test_output_limiter`**：替换原有比例缩放和回差的测试，新增：
  - P 远低于 Plim：Q = Q_v，未选中；
  - Q 永远不大于 Q_v，也不小于 0；
  - 未选中时积分等于上一拍实际流量；
  - 闭环小对象（积分对象，Kq = 3，1 ms）堵转后稳态 |P − Plim| < 0.5 bar；
  - 负载下降后 Q 连续回到 Q_v，任意一拍增量不超过 Kp·Δe + Ki·e·dt + Q_v 自身的增量；
  - 40/41 为 0 时使用 §14B 默认值；
  - `effectiveMaxPressure = 0` 时与不限压逐位相同；
  - 软限位和降额与取小组合时，标志位正确。
- **T2 `test_parameter_access`**：40/41 的读写往返；负值、NaN 拒绝写入；`HYD_PARAM_COUNT` 边界。
- **T3 换段连续性**：射出段限压中换下一段（Plim 改变）时，首拍流量 ≤ Q_last + Kp·(Plim − P)，复现 8879 ms 场景。
  - 活动段实时更新（`ApplyLiveUpdate` 的 `isSegmentActive` 分支）只改 `maxPressure`，不复位限幅状态，积分连续，不需要单独测试（读代码确认）。
  - Stop 减速仍在同一段内，限幅状态不复位；段完成后重新激活与直接接管走同一"复位 + 跟踪 `_lastCommandedFlow`"机制，由本测试覆盖。
  - T3b：P = 旧 Plim（100 bar）、实际流量约 1 L/min 时上调 Plim 到 140，首拍泵转速约 820 rpm（接近 Q_v），固定"有余量不限流"的语义。
  - T3c：接管前限压已把实际流量压到 0。P = 150（高于新 Plim 140）时接管瞬间和首拍泵转速都为 0；P = 124 时首拍约 320 rpm（0 + Kp·16），不回到规划流量 868 rpm。
- **T4 运动块语义**：MoveVelocity、MoveContinuousAbsolute 在持续限压 1 s 内保持 Busy，不出 Done/CommandAborted/Error，`PRESSURELIMITACTIVE` = TRUE。
- **T5 转保压初值**：受限状态下直接接管为 PressureHandle，保压 PI 的初始输出等于实际执行流量（约 7.7 L/min），而不是限幅前的流量。
- **回归**：
  - 现有依赖 `PRESSURE_LIMIT` 的集成测试（`test_motion_interface_unit`、`test_movevelocity_stop_reverse_restart`、`test_movecontinuousabsolute_integration`）全部重跑，只调整断言具体数值的部分；
  - 压力段和 2DOF 测试必须数值不变。

## 9. PLC 侧

- **`iec/ST_HydAxisRef.st`**：新增 `rPressureLimitKp : REAL := 0; rPressureLimitKi : REAL := 0;`。
- **`iec/FB_HydAxis.st`**：
  - 第 0 步：值大于 0 时才写入参数 40/41，与参数 37/38 的做法相同；
  - 新增输出 `bPressureLimited := fbHYD_ReadStatus.VALID AND fbHYD_ReadStatus.PRESSURELIMITACTIVE AND (iStep = 100 OR iStep = 105)`，只在运行步有效，Stop 减速步不输出。
- **工艺层（不在本库，上机前必须具备）**：
  1. 每段射出设压力上限（即 PRESSURELIMIT）；
  2. **最大射出时间后备**：由工艺层参数提供，超时自动转保压（用户已确认，§13），是上机测试的前提；
  3. 可选：`bPressureLimited` 且螺杆速度 < 阈值持续 T，判为堵转，报警或转保压；
  4. 末段射速降低，用于压住硬堵转时的瞬态超调。
- **原因**：改动后，以前靠超出约 40 bar 把螺杆推到转保压位置的模具，现在可能会停在位置前面。
- **上机注意（2026-11-26 终审补充）**：
  1. **段间上调 Plim**：上一段处于限压（P ≈ 旧 Plim）时把下一段 Plim 调高，新段几乎按 Q_v 全速升压，到新 Plim 附近才由压力环接管，瞬态相当于硬堵转（§7：约 1.2–1.3 Plim）。需要上调时同时降低该段射速。
  2. **库诊断保持关闭**：受限压的轴，`flowTolerance`、`velocityTolerance`、`timeoutLimit` 保持 0（默认值，`FB_HydAxis` 不写）。这些诊断按限压前的参考值判断，完全堵转时会误报 FLOW_DEVIATION / VELOCITY_DEVIATION / TIMEOUT。
  3. **`bPressureLimited` 去抖**：它是逐拍的"压力环被选中"位，没有回差，工艺层用于堵转判定时自行去抖（§11）。
  4. **完整下载**：`FB_HydAxis` 新增输出，`ST_HydAxisRef` 新增字段，内存布局改变，需完整下载，不能在线修改；注意保持型数据。

## 10. 射出轴辨识与上机整定步骤

### 10.1 记录准备

- **采样**：1 ms VOFA 记录，记录列：
  - 实际压力（原始值，或 PLC 已有的 MA3）；
  - 伺服实际转速；
  - 泵转速指令；
  - 射出位置。
- **单位**：伺服实际转速和泵转速指令均为 10 rpm/计数（已确认，§13）。单位错了，Kq 会差 100 倍。

### 10.2 Kq 辨识：保压阶跃试验（无需工装）

**方法 A（闭环，首选）**：

1. 正常射一模，进入保压后螺杆基本不动（速度 < 0.5 mm/s）。
2. 保压分 3 段，压力斜坡时间取最小，每段 1 s：60 → 100 → 140 bar；上限按模具允许，再降回 60 bar。
3. 取样窗口：升压段和降压段中，螺杆速度 < 0.5 mm/s、|n| > 100 rpm、压力处于 30%–70% 过渡区的样本。

**方法 B（开环，用于交叉验证）**：

- 进入保压、螺杆静止后，泵转速给恒定值 150、250、350 rpm 各一次，压力到 Pmax（按模具允许，例如 150 bar）时由工艺层立即切回保压或停泵。
- 转速不低于 100 rpm：hor1121 记录中 10–25 rpm 的开环数据拟合 R² 只有 0.3–0.4，无法用于辨识。
- 恒速段 dP/dt 近似恒定，Kq = (dP/dt) / (n − P/ksys)，直接按 20 bar 分段计算。
- **静止过滤**：`--max-pos-rate` 默认 0.5 mm/s。位置尺分辨率 0.01 mm 时，单个计数跳变经平滑后约 0.9 mm/s，相当于只取位置完全不动的样本；脚本提示样本不足时改为 1.0。

两种方法的后续处理相同：
4. 拟合 dP/dt = Kq·(n − P/ksys)；ksys 可先用 5.2，射出阀口打开时泄漏项占比 < 5%。
5. 按 20 bar 分段给出 Kq，**取最高值**作为 Kq_max。
6. 再重复一模，两次 Kq_max 相差 > 15% 时取较大值。

**脚本**：扩展 `scripts/identify_pressure_plant.py`，仍只用标准库：
- 新增 `--pressure-col --speed-col --speed-scale --pos-col --max-pos-rate`，用于指定列名、换算转速单位、只取螺杆静止的样本；
- 输出新增"参数 40/41 建议值"，按 §5.1 公式、fc = 10 Hz、Kq_max 计算。

### 10.3 计算与写入

Kp = 62.8 / (Kq_max × 20)，Ki = Kp × 15.7，写入 `rPressureLimitKp` / `rPressureLimitKi`。

### 10.4 闭环验证：射出堵转试验（实现之后）

1. 先确认工艺层的最大射出时间后备已经生效。
2. 转保压位置设得比满射位置更深，使螺杆必然在到位前堵转。
3. 射出段 PRESSURELIMIT 依次设 80 → 120 → 140 bar，各射 3 模。
4. 判据：
   - 稳态 |P − Plim| ≤ 2 bar；
   - 峰值 ≤ 1.1 Plim；
   - 不出现持续振荡（30–60 ms 周期、幅值增长）；
   - `bPressureLimited` = TRUE，运动块保持 Busy。
5. 调整规则：
   - 峰值 > 1.1 Plim 且无振荡：Kp、Ki 同乘 1.25，即 fc 提高到约 12.5 Hz，最多提高一次；
   - 出现振荡，或稳态纹波 > ±2 bar：fc 降到 7 Hz（Kp、Ki 同乘 0.7）；
   - 稳态收敛慢（> 300 ms）：检查参数 41 是否已经写入。

## 11. 风险

| 风险 | 对策 |
|---|---|
| 限压真正生效后，螺杆停在转保压位置之前 | 工艺层最大射出时间后备（§9），作为上机前提 |
| 射出 Kq 粗估不准 | 辨识前用默认值；默认值对本泵所有轴都稳定（§5.2） |
| 硬堵转峰值 > 1.1 Plim | 工艺层降低末段射速；预判制动不在本次范围 |
| 压力环选中位在边界附近抖动 | 不影响流量连续性；工艺层用到该位时自行去抖 |
| fc = 10 Hz 时 12 阶纹波（约 30 Hz）被放大约 1.3 倍 | 稳态纹波超过 ±2 bar 时把 fc 降到 7 Hz |
| 限压把实际流量压到 0 的瞬间转保压，保压 PI 初值退回规划流量（§4.5 的 0 回退） | 与改动前相同；后续单独处理，需同时修改转保压初值与压力段默认初值 |

## 12. 不在本次范围

- 压力预判制动；
- 压力段、RBF、2DOF、纹波补偿的任何改动；
- 保压卸压速度（反转限幅已由 b3c78ff 调整）；
- 速度规划器在限压期间的状态冻结：规划器继续按原曲线走，Q_v 只参与取小，退出限压由 Q_p 连续爬升保证无扰。

## 13. 用户确认（2026-11-26）

1. VOFA 记录中泵转速为 10 rpm/计数（CSV 为方便缩放显示），辨识脚本 `--speed-scale 10`。
2. 现场可做射出保压的压力阶跃试验，闭环、开环均可（§10.2 方法 A/B）。
3. 最大射出时间由工艺层参数提供；射出时间超过最大值时工艺层自动转保压。§10.4 的前提已具备。
