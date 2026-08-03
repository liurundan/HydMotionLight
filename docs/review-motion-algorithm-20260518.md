# hdy-motion-light 运动控制算法代码评审报告

**评审日期**: 2026-05-18  
**评审范围**: motion_control.c / motion_planner.c / pressure_controller.c / pump_converter.c / velocity_controller.c / ramp_controller.c / segment_completion.c / vp_transfer.c / motion_interface.c  
**评审维度**: 代码质量 · 算法完成度 · 注塑机核心动作适配度 · 查缺补漏  

---

## 一、总体评价

| 维度 | 评分(1-10) | 说明 |
|------|-----------|------|
| 架构设计 | **8.5** | PLCopen FB模式清晰，模块职责划分合理，IEC适配层与核心引擎解耦良好 |
| 代码质量 | **7.5** | 防御性编程到位（NULL检查/参数校验），但部分路径存在浮点精度隐患和magic number |
| 位置控制完成度 | **8.0** | 双规划器(位置梯形/时间斜坡)实现完整，方向推断逻辑完备 |
| 速度控制完成度 | **6.0** | 缺少连续修改目标速度能力，无S曲线，INVELOCITY信号语义不完整 |
| 压力控制完成度 | **8.0** | P/PI/PID/RBF-PID四策略齐备，抗积分饱和+无扰跟踪实现正确 |
| 注塑机适配度 | **6.5** | V/P切换仅提供信号不执行切换，缺低压护模集成模式，缺电动接口 |
| 测试覆盖 | **7.0** | 单元测试框架在位，3个关键测试用例仍失败 |

**综合评分: 7.2 / 10** — 基础框架扎实，核心算法可用，但在注塑工艺的关键交互场景上存在明显能力缺口。

---

## 二、逐模块深度评审

### 2.1 motion_planner.c — 运动规划器

#### ✅ 优点
1. **双规划器设计合理**: `HYD_PLANNER_POSITION_BASED`(在线梯形)和`HYD_PLANNER_TIME_BASED`(时间斜坡+制动保护)覆盖了注塑机主要场景
2. **方向反转处理** (L391-443): 当检测到方向翻转时，先减速到零再反转，避免液压冲击——这是注塑机安全运行的关键
3. **制动加速度独立参数** (`maxDeceleration`): 允许减速与加速使用不同参数，符合液压系统不对称特性
4. **位置制动保护** (`HYD_ComputePositionBasedVelocityMagnitude`): `sqrt(2*a*s)` 制动包络在位置模式下提供了安全减速

#### ⚠️ 问题与缺陷

| # | 严重度 | 问题 | 位置 | 影响 |
|---|--------|------|------|------|
| P1-1 | **高** | 无S曲线/Jerk限制 | 全文件 | 梯形曲线加速度突变导致液压冲击，影响注塑重复精度和模具寿命 |
| P1-2 | **高** | 在线梯形规划器在接近目标时用`maxMagnitude - brakingAccel * deltaTime`递减，若deltaTime异常(>周期)可能一步跳到负值 | L162-164 | 虽有Max(0.0,...)保护，但大deltaTime下速度包络不够平滑 |
| P1-3 | **中** | `HYD_CompareTolerance`使用16*EPSILON的容差系数，在位置很大时(>1000mm)容差可能超过物理有意义范围 | L14-20 | 可能导致大行程段判断失误 |
| P1-4 | **中** | 速度斜坡模式的`decelElapsed/decelStartVel`是从外部传入的，但规划器内部不维护这个状态，依赖调用方正确传入 | L334-341 | 如果调用方忘记设置`_isDecelerating`，减速段将不执行 |
| P1-5 | **低** | `HYD_PlanTrapezoid` / `HYD_EvalTrapezoid` 是离线梯形规划，当前主流程未使用（仅测试中用），但作为公共API暴露 | L191-252 | API冗余，增加维护负担 |

#### 📝 详细分析: S曲线缺失

当前速度规划是纯梯形（加速度阶跃），对于液压伺服泵控系统:
- 泵速度突变 → 液压管路压力脉动 → 机构振动
- 注塑周期重复性下降（模内压力波动可达±5%）
- 竞品系统(如KEBA/Europress)均支持7段S曲线

**建议**: 新增 `HYD_PLANNER_S_CURVE` 规划器类型，实现7段式速度曲线（加加速度约束），不破坏现有接口。

---

### 2.2 motion_control.c — 控制器主流程

#### ✅ 优点
1. **命令合法性矩阵** (L61-90): 位掩码实现的状态-命令映射，O(1)查询，非常高效
2. **两阶段所有权模型**: executionId递增+命令种类追踪，支持命令抢占和状态恢复
3. **诊断分层设计**: 压力/流量/速度/位置/超时五通道独立判断，启动抑制+切换抑制+故障升级
4. **V/P切换评估** (vp_transfer.c): 支持位置/压力/时间/速度降四种切换判据
5. **Hold/Resume语义**: 暂停时冻结elapsed时间，恢复时重新整定控制器，避免积分器饱和

#### ⚠️ 问题与缺陷

| # | 严重度 | 问题 | 位置 | 影响 |
|---|--------|------|------|------|
| MC-1 | **高** | Stop命令绕过规划器独立减速，不更新`_plannerState` | L1381-1428 | stop期间规划器状态与实际输出脱节，stop→resume后可能产生速度跳变 |
| MC-2 | **高** | SPEED_RAMP段完成时先进入`_isDecelerating`，但减速逻辑在planner中用`decelElapsed`计算，若 deltaTime=0 则不减速 | L972-978, L1437 | 已知Bug: Done信号后速度未归零 |
| MC-3 | **高** | 减速率硬编码 `derateRatio = 0.5` | L1372 | 不同动作（合模vs射胶）需要不同的降额系数 |
| MC-4 | **中** | 诊断切换抑制窗口 `_switchSuppressEndTime` 使用压力准则的抑制时间，而非独立配置 | L516-517 | 压力通道未配置时，切换抑制窗口可能为0 |
| MC-5 | **中** | `buildPositionSegment`中 `maxFlow = velocity * velocityToFlowGain` 未考虑减速段流量需求 | motion_interface.c L91 | 减速时速度高→流量需求可能超过maxFlow设置 |
| MC-6 | **中** | 模拟反馈闭环在 `Publish()` 中执行 `position += velocity * deltaTime`，用欧拉积分 | motion_interface.c L460 | 位置累积误差随时间增长，不适合长时间仿真 |
| MC-7 | **低** | `HYD_RequestCommandQueue`中Abort命令可以覆盖任意pending命令，但START等非Abort命令不能覆盖已排队的非NONE命令 | L243-258 | 设计选择，但可能导致紧急Hold被已排队的Next阻塞 |

#### 📝 详细分析: Stop命令减速逻辑缺陷

```c
// motion_control.c L1381-1428
if (fb->_isStopping) {
    HYD_REAL stopElapsed = fb->AXIS_REF.timestamp - fb->_stopStartTime;
    HYD_REAL stopMag = fabs(fb->_stopStartVel);
    HYD_REAL stopDeceleration = ...;
    HYD_REAL decelMag = stopMag - stopDeceleration * stopElapsed;
    
    // 直接覆盖规划器输出
    plannerOutput.targetVelocity = decelMag * stopSign;
    plannerOutput.targetFlow = HYD_ClampReal(decelMag * segment->velocityToFlowGain, ...);
    pumpOutput.commandFlow = plannerOutput.targetFlow;
    pumpOutput.pumpSpeed = ...;
}
```

**问题**:
1. `_plannerState.lastTargetVelocity` 未更新 → 如果在stop期间有Resume，规划器将使用过时的速度值
2. 没有调用 `HYD_ApplyVelocityRateLimit` → 减速斜率完全依赖线性公式，没有与加速度限制协调
3. 停止完成后 `_stopStartVel = 0.0f` 但未重置 `_plannerState`
4. 停止判定条件 `decelMag < 0.001f && fabs(fb->AXIS_REF.velocity) < 0.01f` 使用硬编码阈值

**修复建议**: 将stop减速逻辑统一到规划器内部，通过增加 `HYD_PLANNER_STOP_DECEL` 模式或让 `_isStopping` 状态传播到 `MotionPlannerInput` 中。

---

### 2.3 pressure_controller.c — 压力控制器

#### ✅ 优点
1. **四策略支持**: P → PI → PID → RBF-PID，渐进复杂度
2. **抗积分饱和** (L542-556): 条件积分 — 仅在输出未饱和或误差方向有利于退出饱和时才更新积分项，实现正确
3. **无扰跟踪** (L529-540): 策略切换或Hold→Resume时，通过tracking term补偿输出跳变
4. **增益调度** (L514-519): kpHigh在误差大时自动提升增益，对注塑保压阶段的快速响应很实用
5. **RBF-PID状态同步** (L316-357): 切换到RBF-PID时，从当前输出反推积分项，避免输出跳变
6. **测量滤波 + 微分滤波**: 双一阶滤波器，alpha可配，适合液压压力信号的噪声特性

#### ⚠️ 问题与缺陷

| # | 严重度 | 问题 | 位置 | 影响 |
|---|--------|------|------|------|
| PC-1 | **中** | RBF-PID内部使用`float`，而外部核心引擎使用`HYD_REAL`(可能为double) | 多处 | float→double转换损失精度，特别是在积分和Jacobian计算中 |
| PC-2 | **中** | 压力控制器的dt从`input->timestamp - state->previousTimestamp`推导，未做合理性校验 | L249-253 | 如果PLC周期抖动大(>2ms)，dt异常可能导致积分项跳变 |
| PC-3 | **低** | `HYD_LEGACY_PRESSURE_FLOW_KP = 1.5` 硬编码默认增益 | L5 | 不同液压系统差异大，默认值仅适合参考 |
| PC-4 | **低** | deadband在误差进入死区时error=0.0，离开时立即非零 → 可能产生控制输出的阶跃 | L446-448 | 对保压阶段精度要求高的场景可能有影响 |

#### 📝 关键发现: 无连续修改目标压力能力

当前目标压力 `segment->targetPressure` 在段启动后固定不变。RampController 可以平滑压力斜坡，但**目标值本身不可在线修改**。

这对注塑保压阶段是严重限制:
- 多级保压: 需要在运行中切换目标压力(如 80MPa → 60MPa → 40MPa)
- 当前实现: 必须用多段Recipe，每段一个压力目标
- 问题: 段间切换会导致短暂的零输出(PrimeSegmentControllers重置)，产生压力波动

---

### 2.4 pump_converter.c — 泵速转换

#### 评审结论: 功能完整，无重大问题
- 流量→泵速线性转换 + 泵速限幅
- `isfinite()`检查防止NaN传播
- 方向字段已预留但当前取绝对值处理

⚠️ **一个注意点**: 当前假设线性增益 `flowToPumpSpeedGain`，对于变排量泵在低速区存在非线性。高精度场景可能需要增益曲线表。

---

### 2.5 velocity_controller.c — 速度校正器

#### 评审结论: P-only校正，功能最简

| # | 严重度 | 问题 |
|---|--------|------|
| VC-1 | **高** | 仅P控制，无积分项 → 对持续速度偏差（如负载变化）无法消除稳态误差 |
| VC-2 | **中** | 误差计算 `fabs(target) - fabs(actual)` 丢失了方向信息，无法区分正向超速和反向偏差 |
| VC-3 | **低** | 饱和检测 `correction != input->kp * error` 用浮点等值比较，可能误判 |

**建议**: 升级为PI控制器，增加积分项消除稳态偏差。这对射胶速度精度至关重要（射胶速度重复精度要求±0.5%）。

---

### 2.6 vp_transfer.c — V/P切换评估

#### ✅ 优点: 四种切换判据完备（位置/压力/时间/速度降）

#### ⚠️ 缺陷

| # | 严重度 | 问题 |
|---|--------|------|
| VP-1 | **高** | 仅输出`ready`信号，不执行实际模式切换——切换逻辑留给外部工艺层 |
| VP-2 | **中** | 多个判据同时满足时，按代码顺序返回第一个匹配的reason，无优先级配置 |
| VP-3 | **中** | 仅限 `HYD_SEGMENT_TYPE_INJECTION + HYD_MODE_SPEED_RAMP` 组合，座台射移等场景不支持 |
| VP-4 | **低** | 速度降判据 `velocityReference - actualVelocity >= threshold` 是单向的，不检测速度突升 |

**建议**: 将V/P切换做成可配置的多判据融合器，支持优先级和融合逻辑（OR/AND），并集成到控制器内部执行模式切换。

---

### 2.7 motion_interface.c — IEC适配层

#### ✅ 优点
1. 两阶段所有权模型: _PENDING → _EXEC_ID 确保命令仲裁正确
2. BufferMode(ABORT/BUFFER)实现了PLCopen标准子集
3. MoveAbsolute/MoveVelocity/PressureHandle三大命令覆盖主要场景

#### ⚠️ 问题

| # | 严重度 | 问题 |
|---|--------|------|
| IF-1 | **高** | `CONTINUOUSUPDATE`和`JERK`引脚已定义但被`validateUnsupportedMotionOptions`拒绝 |
| IF-2 | **高** | MoveVelocity的`INVELOCITY`信号语义不完整——仅标志"到达目标速度"，未实现到达后持续速度校正 |
| IF-3 | **中** | Stop命令的`DECELERATION`参数直接透传到core，未做合理性校验（如<0或>1e6） |
| IF-4 | **中** | `dfCycleTime = 0.001f` 全局周期硬编码为1ms，不同PLC扫描周期需外部覆盖 |
| IF-5 | **低** | 模拟模式反馈在Publish中更新，但位移积分仅用欧拉法 |

---

## 三、注塑机核心动作适配度分析

### 3.1 合模 (Clamping / 开合模)

| 需求 | 当前状态 | 差距 |
|------|---------|------|
| 快速合模+慢速低压接近 | Recipe多段可实现 ✅ | 每段速度固定，无法在段内动态降速 |
| **低压护模** | ⚠️ 依赖外部逻辑 | **无集成低压护模模式**: 需要位置控制+压力上限+自动切换到压力保持 |
| 合模到位检测 | 位置+速度双判据 ✅ | stableWindow防抖 ✅ |
| 开模 | 位置模式 ✅ | 无开模减速曲线优化 |
| 合模力保持 | 压力闭环 ✅ | 压力不可在线修改 ⚠️ |

**关键缺口: 低压护模**
当前没有"位置控制+压力保护"的集成模式。需要:
1. 位置模式运行中实时监测压力
2. 当压力超过阈值时自动从位置控制切换到压力保持
3. 输出低压报警信号

### 3.2 射胶 (Injection / 射胶)

| 需求 | 当前状态 | 差距 |
|------|---------|------|
| 多级射胶速度 | Recipe多段 ⚠️ | 段间切换有0输出过渡，压力波动 |
| **V/P切换** | 评估信号 ✅，执行切换 ❌ | 需外部工艺层实现，不一致性风险 |
| 射胶速度在线调整 | ❌ 不支持 | **CONTINUOUSUPDATE被拒绝** |
| 保压多级压力 | Recipe多段 ⚠️ | 同上，段间过渡有压力波动 |
| **保压压力在线调整** | ❌ 不支持 | 无连续修改目标压力能力 |
| 螺杆位置控制 | MoveAbsolute ✅ | 无减速比/螺杆直径转换 |
| 射座前进/后退 | 位置模式 ✅ | 简单场景足够 |

**关键缺口: 连续修改速度/压力**
射胶是注塑机最核心的动作，要求速度和压力能够在运行中连续调整:
- 操作员在线调参
- 自适应射胶(根据模腔压力反馈调整速度)
- 多级保压平滑过渡

### 3.3 储料 (Plasticizing / 储料)

| 需求 | 当前状态 | 差距 |
|------|---------|------|
| 背压控制 | 压力闭环 ✅ | 基础功能完备 |
| 螺杆转速控制 | 速度斜坡 ⚠️ | 需要旋转运动支持(方向=RETRACT模拟后退) |
| 计量位置控制 | 位置模式 ✅ | 需注意螺杆方向映射 |

### 3.4 顶针 (Ejection / 顶针)

| 需求 | 当前状态 | 差距 |
|------|---------|------|
| 多次顶出 | Recipe多段 ✅ | 基本满足 |
| 顶针力保持 | 压力闭环 ✅ | 满足 |

### 3.5 座台 (Nozzle / 座台)

| 需求 | 当前状态 | 差距 |
|------|---------|------|
| 座台前进/后退 | 位置模式 ✅ | 简单场景足够 |
| 座台贴紧力保持 | 压力闭环 ✅ | 满足 |

---

## 四、专项分析: 速度/压力连续修改需求

### 4.1 速度连续修改 (MoveVelocity ContinuousUpdate)

**当前状态**: `CONTINUOUSUPDATE`引脚存在于`HYD_MOVEVELOCITY`，但被`validateUnsupportedMotionOptions`拒绝。

**需要实现的核心逻辑**:

```
当CONTINUOUSUPDATE=true且EXECUTE持续为true时:
  每个PLC扫描周期:
    1. 读取新的VELOCITY/ACCELERATION/DECELERATION输入
    2. 如果VELOCITY变化:
       a. 用新VELOCITY更新_activeSegment.maxVelocity
       b. 规划器根据当前速度和新目标计算斜坡
       c. 保持方向不变
    3. 输出更新后的INVELOCITY/DONE/BUSY
```

**影响范围**:
- `motion_control.c`: `_activeSegment` 需要支持运行时参数更新（或引入"overlay"参数层）
- `motion_planner.c`: 规划器需要接受"目标速度变更"事件，而不是仅依赖elapsedTime斜坡
- `motion_interface.c`: MoveVelocity需要在线读取输入参数并更新到core

**实现优先级: P0** — 射胶速度在线调整是注塑机的基本操作需求。

### 4.2 压力连续修改 (PressureHandle ContinuousUpdate)

**当前状态**: `HYD_PRESSUREHANDLE`无`CONTINUOUSUPDATE`引脚。

**需要实现的核心逻辑**:

```
新增PressureHandle的CONTINUOUSUPDATE行为:
  当EXECUTE持续为true时:
    每个PLC扫描周期:
      1. 读取新的PRESSURE/PRESSURERAMPRATE输入
      2. 如果PRESSURE变化:
         a. 更新_activeSegment.targetPressure
         b. RampController根据新的rampRate向新目标过渡
         c. 压力控制器无缝跟踪
      3. 更新INPRESSURE信号
```

**影响范围**:
- `motion_interface.c`: PressureHandle需要新增CONTINUOUSUPDATE引脚和在线更新逻辑
- `motion_control.c`: `_activeSegment`参数运行时更新机制
- `pressure_controller.c`: 已有无扰跟踪支持，影响最小
- `ramp_controller.c`: 已支持连续执行，新目标自动斜坡

**实现优先级: P0** — 多级保压是注塑标准工艺。

---

## 五、专项分析: 电动功能需求

### 5.1 当前架构评估

当前架构完全围绕"伺服泵控液压"设计:
- 输出: 泵转速(rpm) → 流量(L/min)
- 反馈: 位置(mm) / 速度(mm/s) / 压力(MPa) / 流量(L/min)
- 控制变量: 流量（液压能量载体）

电动伺服需要完全不同的控制链:
- 输出: 电机转矩(Nm)或转速(rpm)
- 反馈: 电机电流(A) / 编码器位置 / 转矩(Nm)
- 控制变量: 转矩（电磁能量载体）

### 5.2 需要新增的能力

| 功能模块 | 当前 | 需要 | 优先级 |
|---------|------|------|--------|
| `HYD_MODE_ELECTRIC_TORQUE` | ❌ | 电动转矩控制模式 | P1 |
| `HYD_MODE_ELECTRIC_SPEED` | ❌ | 电动转速控制模式 | P1 |
| `HYD_ElectricController` | ❌ | 电动控制器（电流环+速度环+位置环三环） | P1 |
| `HYD_MotorRef` 反馈结构 | ❌ | 电机反馈(编码器位置/转速/电流/转矩) | P1 |
| 泵速→转矩输出转换 | ❌ | `ElectricConverter`模块 | P1 |
| 油电复合协调 | ❌ | 液压+电动双通道输出 | P2 |
| IEC: `HYD_ELECTRICCONTROL` FB | ❌ | 电动控制专用IEC功能块 | P1 |

### 5.3 架构建议

```
                    ┌─────────────────────────────────────────┐
                    │           HYD_MotionControlFB           │
                    │  ┌─────────┐  ┌──────────────────────┐ │
  AXIS_REF ────────►│  │Planner   │  │ Mode Router          │ │
  MotorRef ────────►│  │(shared) │  │ ┌────────┐ ┌───────┐ │ │
                    │  └─────────┘  │ │Pressure│ │Electric│ │ │
                    │               │ │Ctrl    │ │Ctrl   │ │ │
                    │               │ └───┬────┘ └───┬───┘ │ │
                    │               │     │          │     │ │
                    │               │ ┌───▼──────────▼───┐ │ │
                    │               │ │  Output Merger   │ │ │
                    │               │ │ (pump+motor cmd)│ │ │
                    │               │ └───────┬──────────┘ │ │
                    │               └─────────┼────────────┘ │
                    │                         │              │
                    │         ┌───────────────▼────────────┐  │
                    │         │PUMP_SPEED / MOTOR_TORQUE   │  │
                    │         └───────────────────────────┘  │
                    └─────────────────────────────────────────┘
```

**关键设计原则**:
1. `HYD_MotionSegment` 新增 `HYD_MODE_ELECTRIC_*` 模式枚举
2. 电动控制器独立于压力控制器，通过Mode Router互斥选择
3. 新增 `HYD_MotorRef` 结构（编码器位置/转速/电流/转矩反馈）
4. 输出层支持双通道: `PUMP_SPEED` + `MOTOR_TORQUE`
5. IEC层新增 `HYD_ELECTRICCONTROL` 功能块

### 5.4 电动IEC接口设计建议

```c
// FUNCTION_BLOCK HYD_ElectricControl
typedef struct {
    // Inputs
    __DECLARE_VAR(BOOL,EN)
    __DECLARE_VAR(BOOL,ENO)
    __DECLARE_VAR(SINT,AXISID)
    __DECLARE_VAR(BOOL,EXECUTE)
    __DECLARE_VAR(REAL,TORQUE)          // 目标转矩 Nm
    __DECLARE_VAR(REAL,SPEED)           // 目标转速 rpm
    __DECLARE_VAR(REAL,CURRENT_LIMIT)   // 电流限制 A
    __DECLARE_VAR(INT,CONTROL_MODE)     // 0=转矩 1=转速 2=位置
    __DECLARE_VAR(INT,BUFFERMODE)
    // Outputs
    __DECLARE_VAR(BOOL,INTORQUE)        // 到达目标转矩
    __DECLARE_VAR(BOOL,INSPEED)         // 到达目标转速
    __DECLARE_VAR(BOOL,BUSY)
    __DECLARE_VAR(BOOL,ACTIVE)
    __DECLARE_VAR(BOOL,COMMANDABORTED)
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    // Motor feedback
    __DECLARE_VAR(REAL,ACT_POSITION)    // 编码器位置
    __DECLARE_VAR(REAL,ACT_SPEED)       // 实际转速
    __DECLARE_VAR(REAL,ACT_CURRENT)     // 实际电流
    __DECLARE_VAR(REAL,ACT_TORQUE)      // 实际转矩
} HYD_ELECTRICCONTROL;
```

---

## 六、下一步算法开发计划

### Phase 1: 紧急修复 (1-2周)

| # | 任务 | 优先级 | 工作量 | 关联问题 |
|---|------|--------|--------|---------|
| 1.1 | 修复Stop命令减速逻辑——统一到规划器内部 | P0 | 3d | MC-1, MC-2 |
| 1.2 | 修复Done信号后速度未归零Bug | P0 | 2d | MC-2 |
| 1.3 | 修复3个失败测试用例 | P0 | 2d | — |
| 1.4 | Stop命令的DECELERATION参数校验 | P1 | 0.5d | IF-3 |

### Phase 2: 速度/压力连续修改 (2-3周)

| # | 任务 | 优先级 | 工作量 | 关联问题 |
|---|------|--------|--------|---------|
| 2.1 | 实现_activeSegment运行时参数更新机制(overlay层) | P0 | 3d | — |
| 2.2 | 实现MoveVelocity CONTINUOUSUPDATE | P0 | 3d | IF-1 |
| 2.3 | 实现PressureHandle CONTINUOUSUPDATE | P0 | 2d | — |
| 2.4 | 连续修改场景的单元测试 | P0 | 2d | — |
| 2.5 | INVELOCITY信号语义完善 | P1 | 1d | IF-2 |

### Phase 3: 注塑工艺增强 (3-4周)

| # | 任务 | 优先级 | 工作量 | 关联问题 |
|---|------|--------|--------|---------|
| 3.1 | 低压护模集成模式(HYD_MODE_POSITION_PRESSURE_GUARD) | P0 | 5d | — |
| 3.2 | V/P切换执行器集成(不只是评估信号) | P0 | 3d | VP-1 |
| 3.3 | 段间平滑过渡(零输出问题修复) | P0 | 3d | — |
| 3.4 | 速度校正器升级为PI | P1 | 2d | VC-1 |
| 3.5 | derateRatio可配置化 | P1 | 1d | MC-3 |

### Phase 4: S曲线规划器 (2-3周)

| # | 任务 | 优先级 | 工作量 | 关联问题 |
|---|------|--------|--------|---------|
| 4.1 | 新增HYD_PLANNER_S_CURVE规划器 | P1 | 5d | P1-1 |
| 4.2 | 7段式S曲线速度规划实现 | P1 | 3d | P1-1 |
| 4.3 | JERK参数支持 | P1 | 2d | IF-1 |
| 4.4 | S曲线规划器测试 | P1 | 2d | — |

### Phase 5: 电动功能 (4-6周)

| # | 任务 | 优先级 | 工作量 | 关联问题 |
|---|------|--------|--------|---------|
| 5.1 | HYD_MotorRef反馈结构设计 | P1 | 1d | — |
| 5.2 | HYD_MODE_ELECTRIC_TORQUE/SPEED模式定义 | P1 | 1d | — |
| 5.3 | 电动控制器核心实现(电流环+速度环) | P1 | 5d | — |
| 5.4 | ElectricConverter模块(转矩→电流命令) | P1 | 3d | — |
| 5.5 | Output Merger(双通道输出) | P1 | 3d | — |
| 5.6 | HYD_ELECTRICCONTROL IEC功能块 | P1 | 3d | — |
| 5.7 | 电动模式集成测试 | P1 | 3d | — |
| 5.8 | 油电复合协调逻辑 | P2 | 5d | — |

### Phase 6: 精度与健壮性提升 (2-3周)

| # | 任务 | 优先级 | 工作量 | 关联问题 |
|---|------|--------|--------|---------|
| 6.1 | 浮点精度审计(float→double统一) | P2 | 2d | PC-1 |
| 6.2 | 压力控制器dt合理性校验 | P2 | 1d | PC-2 |
| 6.3 | 泵速转换非线性增益曲线 | P2 | 2d | — |
| 6.4 | 仿真积分精度提升(辛普森/龙格库塔) | P2 | 1d | MC-6 |
| 6.5 | V/P切换多判据融合器 | P2 | 3d | VP-2 |

---

## 七、总结

### 已完成的良好工作
1. PLCopen FB架构成熟，状态机覆盖完整(9状态+8命令)
2. 位置控制双规划器设计合理，方向反转保护正确
3. 压力控制器四策略+抗饱和+无扰跟踪实现完备
4. 诊断系统五通道+启动/切换抑制+故障升级+快照留存
5. IEC适配层两阶段所有权模型健壮

### 关键缺口（查缺补漏）
1. **⚠️ 无速度连续修改能力** — 射胶速度在线调整是注塑机标配功能
2. **⚠️ 无压力连续修改能力** — 多级保压平滑过渡是工艺刚需
3. **⚠️ 无低压护模集成模式** — 合模安全的核心功能
4. **⚠️ V/P切换仅评估不执行** — 射胶最关键的模式切换
5. **⚠️ 段间过渡零输出** — 多段Recipe切换时压力/速度波动
6. **⚠️ 无电动功能** — 油电复合是注塑机发展趋势
7. **⚠️ 无S曲线** — 影响重复精度和模具寿命
8. **⚠️ Stop命令减速逻辑脱节** — 安全停止的关键路径

### 一句话总结
**基础框架扎实、核心算法可用，但在注塑工艺的关键交互场景(连续修改、模式切换、安全护模)上存在明显能力缺口，需要按Phase 1→2→3优先级推进。电动功能是重要但非紧急的扩展方向。**
