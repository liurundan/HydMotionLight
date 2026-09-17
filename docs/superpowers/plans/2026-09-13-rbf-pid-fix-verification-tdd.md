# RBF-PID/PI 压力控制算法修复 TDD + 仿真验证计划

- 日期：2026-09-13
- 依据：评估报告 v3（`docs/压力控制算法实用性评估报告-2026-09-13.md`）
- 分支：`feature/debug-rbf-pid`（修复在此分支或新开 `feature/rbf-pid-fix-verified`）
- **硬约束**：每条修复建议必须由仿真实验数据验证，否则不予通过；上机验证通过后才合并 master

---

## 0. 核心原则：四层验证漏斗

```
单元 TDD（RED-GREEN） → 闭环仿真定量对比（VERIFY） → 5算法4工况矩阵（BENCHMARK） → 分阶段上机（STAGING）
   ↓ 不通过则修代码              ↓ 不通过则改方案            ↓ 不通过则回到设计          ↓ 不通过则回退
```

**任何一层未通过，不得进入下一层。** 第 1-2 层在本计划内完成；第 3 层产出对比报告供你决策；第 4 层由你上机执行。

---

## 1. 仿真地面真相（Ground Truth）定义

### 1.1 仿真植物模型

采用当前分支 `src/sim/PressureModel.c` 的 `PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED` 物理模型，**不使用** `hydro_sim`（其压力物理过简，`test_rbf_pid_hil.c` 注释已明示"the sim physics does not model a pressure-regulated pump"）。

模型已含的非线性（无需重写）：
- 含气油液有效体积模量 βe(P)（`PressureModel_EffectiveBulkModulusPa`，rbf/sim/PressureModel.c:207-221）
- 泵内泄 ηv(P,n)（:599-603）
- 13/26/39 阶流量纹波（:586-591）
- 溢流阀带滞回（:615-627）
- 伺服二阶 + 加速度限幅（:564-571）
- 传感器延迟 + 量化 + 噪声（:674-687）

### 1.2 上机辨识参数加载

从 `feature/pressure-ripple-suppression` 分支取 `docs/ripple-analysis/identified_params.json`，合入本分支，作为仿真植物的默认标定参数：

| 参数 | 值 | 物理意义 |
|---|---|---|
| tau | 0.0437 s | 伺服一阶时间常数 |
| V_eff | 4.332e-4 m³ | 油腔有效体积 |
| leak | 1.202e-12 m³/Pa/s | 系统泄漏系数 |
| beta_min | 3.454e8 Pa | 含气油液最低刚度 |
| p_trans | 2.077e6 Pa | 刚度塌缩过渡压力 |
| 拟合 RMS | 7.75% | 与上机 CSV 的相对误差 |

**判据**：仿真植物用此参数复现上机 CSV 的开环响应，RMS ≤ 10% 方可作为地面真相（7.75% 已达标，复现验证见 Task 0.2）。

### 1.3 闭环验证台架构（新建）

新建 `tests/test_rbf_pid_closed_loop.c`，闭环结构：

```
┌─────────────┐  flow Q   ┌──────────────────┐  rpm    ┌──────────────┐
│ RBF/PI 控制器 │─────────►│ Q→rpm 换算(20rpm/ │────────►│ PressureModel │
│ (rbf_pid.c / │◄─────────│ L/min) pump_conv  │         │  (物理植物)   │
│  pressure_    │  P_meas  └──────────────────┘         └──────┬───────┘
│  controller.c)│◄──────────────────────────────────────────────┘
└─────────────┘                       压力 bar (含纹波+噪声)
```

**关键**：控制器输出 flow[L/min] → `flowToPumpSpeedGain`(20 rpm/(L/min)) → rpm → `PressureModel_Step(rpm)` → `measured_pressure_bar` → 控制器反馈。1ms 一拍，10s 场景 = 10000 步。

### 1.4 指标定义与采集

每场景采集以下指标（在 t∈[ settle_start, end ] 窗口内）：

| 指标 | 符号 | 定义 | 采集方式 |
|---|---|---|---|
| 超调量 | Mp | (P_max − P_set)/P_set × 100% | 升压段峰值 |
| 上升时间 | tr | P 首次达 90%·P_set 的时间 | 线性搜索 |
| 稳态误差 | ess | mean(P_set − P_meas) in 稳态窗 | 末 2s 均值 |
| 稳态纹波 | σ_ss | std(P_meas) in 稳态窗 | 末 2s 标准差 |
| 建稳时间 | ts | P 进入 ±2%·P_set 后不再退出 | 扫描 |
| 增益漂移 | ΔKP/KI/KD | (末值−初值)/初值 | 末拍−首拍 |
| Jacobian 符号 | sign(J) | 稳态窗内 J 的符号统计 | 末 2s sign 计数 |

---

## 2. 阶段一：仿真基础设施与基线（Task 0.x）

### Task 0.1 — 合入辨识参数与仿真模型基线
- **动作**：从 `feature/pressure-ripple-suppression` cherry-pick `identified_params.json` + PressureModel 的标定参数加载代码（若本分支 InitParams 默认值未含辨识值）
- **TDD**：`test_pressure_model_calibrated.c` 验证加载辨识参数后，开环阶跃响应形状与上机 CSV 一致
- **交付**：`docs/verification/baseline_sim_params.md`（参数表 + 复现曲线数据）

### Task 0.2 — 开环复现验证
- **实验**：用辨识参数，注入上机 CSV 同款输入（set_v 10/20/30/40 rpm 阶跃），比较输出压力
- **判据**：RMS ≤ 10%（辨识报告 7.75%）
- **不通过**：调标定参数或查模型差异，不进入后续

### Task 0.3 — 闭环验证台搭建
- **动作**：新建 `tests/test_rbf_pid_closed_loop.c`，实现 §1.3 闭环，提供场景注入接口（目标压力曲线、负载流量、纹波开关）
- **TDD**：纯比例控制器（KP=1.5, KI=0, 无前馈）闭环跑 100 bar 阶跃，应能稳定（ess < 20 bar）
- **交付**：可复用的 `HYD_ClosedLoopScenario` 结构 + 指标采集函数

### Task 0.4 — 基线指标采集（当前未修复代码）
- **动作**：用 Task 0.3 验证台，跑 §3 的 4 个标准场景，采集当前 RBF-PID/PI/经典 PI 的基线指标
- **交付**：`docs/verification/baseline_metrics.json`（4 场景 × 3 控制器 × 7 指标）
- **用途**：后续每个修复的 VERIFY 阶段与此基线对比

---

## 3. 标准验证场景（4 个）

| # | 场景 | 目标 | 持续 | 注入扰动 | 验证重点 |
|---|---|---|---|---|---|
| S1 | 升压超调 | 0→150 bar 阶跃 | 5s | 13齿纹波(150bar段~32rpm) | Mp、tr、ts |
| S2 | 保压纹波 | 100 bar 保持 | 10s | 13齿纹波(~26rpm, 5.6Hz) | σ_ss、ess、增益漂移 |
| S3 | 低压背压 | 15 bar 保持 | 10s | 13齿纹波(~10rpm, 2.2Hz) + 含气刚度塌缩 | σ_ss、ess、自适应是否生效 |
| S4 | 工况切换 | 150→30 bar 下调 | 5s | 切换瞬间无纹波 | 切换冲击、Jacobian 符号、u_prev 残留 |

---

## 4. 阶段二：机制验证（证明 v3 纠错正确，非修复）

**目的**：在改代码前，先用仿真数据证明报告 v3 的几处机制性纠错是对的，避免基于错误理解去"修复"。

### Task 1.1 — 验证 f_velfb 累积性（非自消除）
- **实验**：S1 场景，RBF-PID 模式，记录每拍 `u_prev` 中的 f_velfb 累积分量（需在验证台加探针）
- **假设**（v3）：Σf_velfb = −0.15·(P(k)−P₀)，稳态时该累积和**保留为常值偏置**（非归零）
- **判据**：
  - 升压段末 Σf_velfb ≈ −0.15·(P_set−P₀)（数值吻合 ±15%）
  - 稳态末 2s Σf_velfb **不归零**（|末值| > 0.5·|升压末值|）
- **不通过**：v3"比例反馈"定性有误，需回到 4.2 重新推导
- **交付**：`docs/verification/mechanism_f_velfb.md`（数据表 + 曲线）

### Task 1.2 — 验证稳态冻结对纹波的效果
- **实验**：S2 场景，对比"冻结开"(当前) vs "冻结关"(强制 is_steady=false) 两种配置
- **假设**（v3 4.1）：冻结开 → σ_ss 较小；冻结关 → 持续自适应把纹波当激励 → σ_ss 增大
- **判据**：冻结关的 σ_ss > 冻结开的 σ_ss × 1.3（纹波恶化 ≥30%）
- **不通过**：说明冻结策略未必有效，4.1 改判需重新评估
- **交付**：`docs/verification/mechanism_steady_freeze.md`

### Task 1.3 — 验证 Jacobian 符号先验缺失的危害
- **实验**：S1 场景，构造 RBF 网络初始权重使 J 辨识为负（注入扰动或改初值），观察 KP 漂移方向
- **假设**（v3 4.4）：J<0 时 KP 朝错误方向漂移（应增却减或反之）
- **判据**：J<0 拍占比 >5% 时，KP 漂移方向与正确方向相反
- **不通过**：Jacobian 钳位[−5,50]可能已足够，4.4 优先级降低
- **交付**：`docs/verification/mechanism_jacobian_sign.md`

**阶段二总结**：三份机制验证报告汇总，确认 v3 纠错方向正确后，方可进入阶段三修复。若任一不通过，回到报告 v3 重新定性。

---

## 5. 阶段三：修复 TDD（每项 RED-GREEN-VERIFY）

每项遵循：**RED**（写失败测试证明缺陷）→ **GREEN**（修复使测试过）→ **VERIFY**（闭环仿真指标达标）→ **MERGE 准入**。

### Task 2.1 — 冻结死区相对化 + 时间确认 + 去 P_set>5 门限（建议#2 / 4.1）

**RED**：`test_steady_deadzone_low_pressure.c`
- S3 场景（15 bar 背压），当前代码：断言 `rbfPid.steady_count` 在低压段始终为 0（因 P_set≤5? 否，15>5，但测 5bar 模保段）/ 或断言自适应在低压段从未生效（|ΔKP|≈0）

**GREEN**（rbf_pid.c 修改）：
```c
// rbf_pid.c:264 改
#define STEADY_DEAD_ZONE     10.0f   → 删除绝对值，改函数计算
// 新增：rbf_pid_resolve_steady_threshold(P_set) = max(0.5f, 0.04f * fabsf(P_set))
// rbf_pid.c:316 改
int is_steady = (fabsf(error) < threshold) && (fabsf(de) < threshold) &&
                 (pid->steady_confirm_count >= N_CONFIRM);  // N_CONFIRM=5
// rbf_pid.c:546 删除 fabsf(P_set) > 5.0f 硬门限，改 threshold 判据统一
```

**VERIFY**（闭环仿真判据）：
| 指标 | 基线(当前) | 修复后判据 |
|---|---|---|
| S3 σ_ss | 基线值 X | ≤ X × 1.1（不恶化） |
| S3 ess | 基线值 Y | ≤ Y × 0.7（改善 ≥30%） |
| S3 自适应生效拍数 | 0 | > 100 拍（低压段确实生效） |
| S2 σ_ss | 基线值 Z | ≤ Z × 1.1（保压段不恶化） |

**不通过**：低压段自适应生效但纹波恶化 → 调 threshold 系数或加纹波陷波；ess 未改善 → 查 KI 窗口

### Task 2.2 — Jacobian 符号先验监督（建议#4 / 4.4）

**RED**：`test_jacobian_sign_guard.c`
- 构造 J 辨识为负的场景，断言当前代码 KP 朝错误方向漂移（用 Task 1.3 的实验数据驱动）

**GREEN**（rbf_pid.c 修改）：
```c
// rbf_pid.c:400 step_adaptive_gains 入口加
if (pid->Jacobian < 0.0f) {
    // 符号先验违反：∂P/∂u 物理上为正，J<0 是网络误辨识
    // 冻结本拍增益整定，保持上拍输出
    return;  // 或仅冻结 KP/KI/KD 更新，仍更新网络
}
```

**VERIFY**：
| 指标 | 判据 |
|---|---|
| S1 J<0 拍 KP 漂移方向 | 与基线相反 → 修复后方向正确 |
| S1 Mp | ≤ 基线 × 1.1（不恶化） |
| S1 ess | ≤ 基线 × 1.1 |
| S2 增益漂移 ΔKP | |末−初| ≤ 基线 × 0.5（漂移幅度减半） |

### Task 2.3 — D 通道独立滤波恢复（建议#3 / 4.5）

**RED**：`test_d_channel_filter.c`
- S2 场景，注入 5.6Hz 纹波，断言当前 D 项（无滤波）放大纹波：`std(KD*d_term) > std(error) × 2`

**GREEN**（rbf_pid.c 修改）：
```c
// rbf_pid.c:472-474 恢复滤波（去注释）
float flt_alpha = 0.2f;  // D 通道专用，与测量滤波解耦
d_term = flt_alpha * raw_d_term + (1.0f - flt_alpha) * pid->prev_d_term;
// 库内测量滤波 pressure_controller.c:524 改直通 alpha=1.0（默认），保留字段兼容老配置
```

**VERIFY**：
| 指标 | 判据 |
|---|---|
| S2 D 项纹波放大 | std(KD*d_term) ≤ 基线 × 0.4 |
| S2 σ_ss | ≤ 基线 × 0.85（纹波改善 ≥15%） |
| S1 Mp | ≤ 基线 × 1.05（D 滞后不恶化超调） |
| 相位裕度（开环） | ≥ 基线 + 5° |

### Task 2.4 — RBF 输入 du_prev 命名澄清（建议#6 / 4.7，v3 已撤回缺陷定性）

**说明**：v2 曾判 du_prev 为缺陷并计划合入 u_prev 修改；**v3 依据用户指正 + 刘金琨教材撤回该定性**——du_prev 是增量 PID 的合法辨识输入（∂y/∂(Δu) 与 ∂y/∂u 同号，方向辨识有效，甚至更贴合增量语义）。本任务降级为纯命名澄清，不改控制行为。

**RED**：`test_rbf_input_naming.c`
- 断言 `f_dd_press_prev` 字段名与实际用途（du 归一化尺度）语义不符，文档/注释易误导阅读者

**GREEN**（rbf_pid.h / rbf_pid.c 修改，纯命名）：
```c
// rbf_pid.h:151 注释更新
float du_scale;  // 原 f_dd_press_prev，实际用作 Δu 归一化尺度
// rbf_pid.c 全局替换 f_dd_press_prev → du_scale（功能不变）
// rbf_pid.c:161-163 rbf_pid_effective_du_scale 注释更新
```
**注意**：不 cherry-pick 15ff9f8 的 x[0]=u_prev 改动；du_prev 输入保持不变。

**VERIFY**：纯命名重构，4 场景全部 byte-identical no-regression（输出与基线逐拍一致）

### Task 2.5 — f_velfb 工程加固（建议#1 / 4.2，按比例增益理解）

**RED**：`test_f_velfb_hysteresis.c`
- 断言当前门限无滞回：在 8%·P_set 边界附近拍间抖动（f_velfb 在 0/非0 间跳变）
- 断言跨段未复位 u_prev：S4 切换后 Σf_velfb 残留上段偏置

**GREEN**（rbf_pid.c 修改）：
```c
// rbf_pid.c:485 门限加滞回
static float near_target_hyst = 0.0f;  // 状态字段
float threshold_enter = P_set * 0.08f;
float threshold_exit  = P_set * 0.04f;  // 滞回 4%
if (pressure_error > threshold_enter) near_target_hyst = 1.0f;
else if (pressure_error < threshold_exit) near_target_hyst = 0.0f;
if (pid->pressure_accel_ff_enabled && near_target_hyst > 0.5f) { ... }

// 段切换时复位 u_prev 的 f_velfb 累积分（pressure_controller.c 段切换处）
```

**VERIFY**：
| 指标 | 判据 |
|---|---|
| S4 切换后 Σf_velfb | |末值| < 0.2 L/min（残留清除） |
| 门限边界抖动次数 | ≤ 基线 × 0.1 |
| S1 Mp | ≤ 基线 × 1.05（不恶化超调抑制） |

### Task 2.6 — 清理死代码与语义瑕疵（建议#10 / 4.12-4.16）

**RED**：`test_feedback_flow_semantics.c`
- 断言 RBF 分支 `feedbackFlow = rawOutputFlow − feedforwardFlow` 语义错误（应 = rawOutputFlow，因 RBF 不含 ff）

**GREEN**：
- `pressure_controller.c:589` 改 `feedbackFlow = rawOutputFlow`（不减 ff）
- 删 `&& 1` 死条件（rbf_pid.c:486）
- 清理注释代码（:472-473, :499-501）
- `interf_term` 钳位 ±0.08 改与 fMaxFlow 量纲适配：`±0.05 * fMaxFlow`

**VERIFY**：纯语义修复，不要求指标改善，但 4 场景全部 no-regression（|指标差| < 5%）

---

## 6. 阶段四：5 算法 × 4 工况对比矩阵（BENCHMARK）

修复全部 VERIFY 通过后，跑完整对比矩阵，产出决策报告。

|  | S1 升压 | S2 保压 | S3 背压 | S4 切换 |
|---|---|---|---|---|
| 经典 PI（修后） | | | | |
| 经典 PID（修后） | | | | |
| RBF-PI（修后） | | | | |
| RBF-PID（修后） | | | | |
| RBF-PID（修前基线） | | | | |

每格填 7 指标。**决策规则**：
- RBF-PID 修后在某场景综合优于经典 PI → 该场景推荐 RBF-PID
- 否则该场景推荐经典 PI，RBF-PID 仅作实验策略

**交付**：`docs/verification/algorithm_comparison_matrix.md` + CSV 数据 + 图表

---

## 7. 阶段五：上机 staging 准入

仿真全通过后，分阶段上机：

| 阶段 | 内容 | 准入条件 | 退场条件 |
|---|---|---|---|
| 上机-1 | 经典 PI 修后（库内滤波直通+D滤波） | S1-S4 仿真 no-regression | 上机曲线 ess/Mp 与仿真偏差 < 30% |
| 上机-2 | RBF-PID 修后（冻结相对化+Jacobian监督+u_prev+D滤波） | 上机-1 通过 + 仿真矩阵支持 | 上机 σ_ss 与仿真偏差 < 40% |
| 上机-3 | f_velfb 工程加固 | 上机-2 通过 | 上机 Mp 抑制效果与仿真一致 |

**全部上机通过后，合并 master。**

---

## 8. 任务依赖与执行顺序

```
Task 0.1 → 0.2 → 0.3 → 0.4   (基础设施，必须串行)
    ↓
Task 1.1, 1.2, 1.3           (机制验证，可并行)
    ↓ (全过才继续)
Task 2.2 (Jacobian) → 2.1 (冻结) → 2.3 (D滤波) → 2.5 (f_velfb) → 2.6 (清理) → 2.4 (命名澄清，纯重构无依赖)
    ↓ (每项 VERIFY 过才下一项)
阶段四矩阵 → 阶段五上机
```

**并行可能**：Task 1.x 三项互相独立，可派 3 个 subagent 并行；Task 2.x 有依赖（2.2→2.1→2.3→2.5 串行），Task 2.4 纯命名重构可随时插入或与 2.6 合并。

---

## 9. 失败处理协议

任一 VERIFY 不通过：
1. **不强行通过**——记录失败指标，分析根因
2. 若指标轻微偏离（< 15%）：调参数后重跑，最多 3 轮
3. 若指标严重偏离（≥ 15%）：该修复建议**作废**，回报告 v3 重新设计，或该缺陷降级为"已知限制"
4. 所有失败记录入 `docs/verification/failures.md`，供你决策是否上机

---

## 10. 交付物清单

| 阶段 | 交付物 | 格式 |
|---|---|---|
| 0 | baseline_sim_params.md, baseline_metrics.json | md + json |
| 1 | mechanism_f_velfb.md, mechanism_steady_freeze.md, mechanism_jacobian_sign.md | md（含数据表） |
| 2 | 每项 TDD 测试 + 修复 commit + verify_metrics_*.json | c + json |
| 3 | algorithm_comparison_matrix.md + CSV | md + csv |
| 4 | staging_report.md | md |

---

## 11. 关键风险与对策

| 风险 | 对策 |
|---|---|
| 仿真植物与上机偏差大 | Task 0.2 开环复现把关，RMS>10% 则不继续 |
| 机制验证推翻 v3 纠错 | 阶段二为硬门槛，任一不通过回到报告重写 |
| 修复间相互抵消（如 D 滤波改善纹波但冻结已解决） | 矩阵对比含"修前基线"列，隔离单因素 |
| 仿真通过但上机仍失败 | 上机 staging 分阶段，每阶段退场条件含仿真-上机偏差阈值 |

---

*本计划严格执行"无仿真数据不通过"原则。每个修复建议的真伪由仿真实验数据裁决，不由理论推导或 AI 分析独断。*
