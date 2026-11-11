# 减速语义统一、短行程可达性与限压恢复 — 设计文档

- 日期：2026-11-10
- 分支：feature/rbf-pid-industrialization
- 状态：已实施（实现计划 docs/superpowers/plans/2026-11-10-decel-semantics-and-reachability.md）

## 1. 背景与根因

实验数据：`docs/data_deg/vofa_waveform_20261110_082428_443.csv`（1ms 控制/仿真周期，开合模使用
`HYD_MoveContinuousAbsolute`（带终点速度的位置控制）多段运动）。

单位链（见 `iec/FB_HydAxis.st`）：
- `rVel = uiSpd × 0.1% × rMaxSpeed`，本实验 rMaxSpeed = 1800 mm/s（uiSpd=10 ⇒ 18 mm/s）
- 泵速[0.1rpm] = 线速度[mm/s] × 0.96（17.28 ⇔ 18 mm/s）
- `uiAcc/uiDec` 单位 ms，受 `HYD_MIN_MOTION_TIME_MS = 100ms` 下限限幅（88 → 100）
- 仿真轴为运动学模型：跟随规划速度（≈3ms滞后），不消耗泵速

### 现象与根因对照

| # | 现象（实测） | 根因 |
|---|---|---|
| 1 | 开模段1泵速爬坡 473~500ms（期望100ms） | `output_limiter.c` 压力限幅：上段实际压力 3.0bar > 新段限压 2.0bar → scale→0；泄压后按 `HYD_THRESH_PRESSURE_LIMIT_SCALE_RELEASE_RATE=2.0/s` 恢复（500ms 满量程）。泵速 = scale(t)×规划流量（前100ms二次曲线+后段线性34.56u/s，逐点吻合）。规划器本身 100ms 到位（轴速数据证明） |
| 2a | 开模段3→4入口 50→30% 用 67ms | 段入口超速钳制（`motion_planner.c:596-608`）以 a_cfg = maxVel/t_dec = 540mm/s² 限斜率 |
| 2b | 开模段4→5入口 30→10% 用 200ms | 同上，a_cfg = 180mm/s²（低速段 → 减速反而更慢） |
| 3 | 合模段1→2、段3→4入口 100→90% 用 11ms | 同上，a_cfg = 1620mm/s²（高速段 → 极陡） |
| 4 | 合模段4内 90→50% 用 44ms（65mm 行程本应足够100ms减速） | 制动触发距离 (v²−vt²)/(2·a_cfg) = 5.67mm 过晚（实测 5.2~6.2mm 吻合）+ 包络 sqrt(vt²+2·a_cfg·s) 跟随斜率 = −a_cfg = −1620mm/s²（实测 −1629 吻合） |

统一公式验证：**实际减速时间 = t_dec × Δv / V段速**（67 / 200 / 11 / 44ms 四点全部命中）。

根源：`HYD_ResolveConfiguredBrakingAcceleration()` = maxVelocity/t_dec（“本段速度减到0”语义）与
`HYD_AdvanceStatefulVelocity` 的 Δv/t_dec（“当前到目标”语义）并存冲突；前者用于制动距离、
安全包络、入口钳制等全部硬约束，段切换时永远是绑定约束。

## 2. 决策

- 加/减速时间语义：**A — Δv/t（任意速度过渡固定耗时 t_acc/t_dec）**
- 实施范围：方案1（语义统一）+ 方案2（短行程可达性）+ 方案3（限压快速恢复）
- 方案4（仿真改为消耗 PUMP_SPEED 闭环）**不在本次范围**，已记录待后续

## 3. 设计

### D1 制动语义统一（方案1）

原则：**规划层全部 Δv/t_dec；安全包络保持 vmax/t_dec 不动（纯兜底层）**。

| 位置 | 现状 | 改为 |
|---|---|---|
| 段入口超速钳制 `motion_planner.c:596-608` | decel = a_cfg | `HYD_ResolveBrakingAcceleration(seg, prev, maxVel)`（Δv/t_dec）→ 入口过渡恒为 t_dec |
| 制动触发距离 `motion_planner.c:399-408` | (v²−vt²)/(2·a_cfg) | **(prev+vt)/2 × t_dec + (prev−vt)×dt**（恒时间语义的精确制动距离） |
| 制动斜坡（`HYD_AdvanceStatefulVelocity`） | 已是 Δv/t_dec | 不动 |
| 安全包络 `motion_planner.c:446` 及 `HYD_ComputeDelayAwareBrakingVelocityMagnitude` | sqrt(vt²+2·a_cfg·s) | **不动** |
| SPEED_RAMP 减速路径（`ResolveConfiguredBrakingAcceleration` 其余调用处） | a_cfg | Δv 形式（decelStartVel→0 即 decelStartVel/t_dec） |
| `ResolveContinuousAbsoluteCrossingVelocity` 可达加速度（motion_control.c:1004） | maxVel/t_acc | (requested−projected)/t_acc |
| blend 相关：`ResolveSegmentBrakingAccelerationForBlend`、`ResolveDirectBlendSwitchTolerance`、`ShouldCutoverDirectBlend` | a_cfg 假设 | 计划阶段逐一审计并对齐语义 |

安全性质（选择“包络不动”的理由）：正常规划 a_planned = Δv/t_dec ≤ vmax/t_dec = a_env（因 Δv ≤ vmax），
且 Δv 触发距离 (prev+vt)/2·T 恒大于包络咬合距离 (prev²−vt²)/(2·a_env) → 包络在正常轨迹上永不介入。

**禁止事项**：包络不得改用随当前 v 收缩的 Δv 加速度——v 降 → a 降 → 包络降 → 追砍指令，正反馈病态。

### D2 短行程 + 高目标速度在线可达性（方案2）

语义A下可达性为线性闭式解（每周期计算、无新增规划器状态）：

```
s_needed(v0→vp→vt) = (v0+vp)/2·T_acc + (vp+vt)/2·T_dec
⇒ vp = (2·s_rem − T_acc·v0 − T_dec·vt) / (T_acc + T_dec)
巡航目标 = min(segment.maxVelocity, vp)，下限不低于 vt 的可达处理
```

- vp < vt（终速不可达）→ 沿用现有 `ADAPTENDVELTOAVOIDOVERSHOOT` 的 sqrt 自适应降终速
- 纯减速都不足（s_rem < (v0+vt)/2·T_dec）→ 按需求减速度 (v0²−vt²)/(2·s_rem) 执行，上限 = 包络减速度；
  同时由 **motion_control 侧**复算并记录 WARNING 级诊断（不自动停机——反应归流程层，符合库边界）
- `HYD_MotionPlannerOutput` 结构不新增字段

### D3 限压 scale 分级恢复（方案3，`output_limiter.c`）

- `actual ≤ releasePressure`（回差带以下）：新 §14B 常量
  `HYD_THRESH_PRESSURE_LIMIT_SCALE_FAST_RELEASE_RATE`（默认 10.0/s，100ms 满量程）
- 回差带内：维持现有 2.0/s 防振荡
- 常量置于 `include/hyd_config.h` §14B（遵守仓库约束7）

### D4 验证（TDD）

1. 单测先行：
   - 入口过渡 = t_dec（高速 100→90%、低速 30→10% 两向）
   - 段内 END 制动：触发点 = (v+vt)/2·T，时长 = t_dec
   - vp 线性公式与钳制；纯减速不足 → 需求减速度 + WARNING 诊断
   - scale 分级恢复（带下快、带内慢）
   - 既有“包络是硬上限”“blend 制动在规划斜坡内”等测试保持通过
2. 集成：复现本次 CSV 开合模段序列，断言各过渡时长 ≈ t_dec ± 2ms
3. 全量 `ctest --test-dir out/build/unixgcc`

## 4. 工艺侧配套建议（文档性，不改库）

- 切段时限压值勿阶跃至低于上一段实际压力（否则必然触发限幅-恢复过程）
- 需要“到达段边界时已处于下段速度”时，END_SPEED 应设为**下一段**速度（段内前瞻），
  而非本段速度（入口钳制模式）；语义统一后两种写法的过渡时长均正确，但轨迹不同

## 5. 验收标准

- 复现场景中四类过渡（2a/2b/3/4）时长均 = 100ms（限幅后 t_dec）± 2ms
- 开模段1（压力段后首段）泵速建立 ≤ ~200ms（100ms 规划斜坡 + ≤100ms 快速恢复）
- 全部既有测试通过；新增测试覆盖 D1~D3 行为
- 嵌入式生产构建 `./scripts/deploy_embedded_prod.sh` 不受影响（纯 C99、无新依赖）
