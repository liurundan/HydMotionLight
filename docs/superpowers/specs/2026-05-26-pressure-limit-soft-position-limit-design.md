# 压力限制与软限位保护设计规格

日期: 2026-05-26
状态: 待确认

---

## 1. 背景与动机

当前运动控制库在位置模式和速度模式下缺乏通用的最大压力保护机制。planner 只规划速度/流量，不监控压力反馈。当负载突变（合模撞模、射胶遇高粘度熔体）时，系统压力飙升只能依赖外部安全阀——这是最后防线，不应作为常规保护。

同时，`cylinderConfig.strokeMm` 参数已存在但运行时未生效，油缸超行程无报警、无停机保护。

本设计为上机测试前的安全底线功能。

---

## 2. 设计目标

1. 在所有控制模式下提供通用最大压力保护（比例限速 + 报警 + 故障升级）
2. 启用 `cylinderConfig.strokeMm` 作为运行时软限位，接近极限时平滑减速并报警
3. 启动前校验 `targetPosition` 不超出行程范围
4. 不破坏现有 `pressureCeiling`（低压模保护）的功能

---

## 3. 压力限制

### 3.1 参数定义

**FB 级全局参数（新增）：**

```c
// HYD_MotionControlFB 中新增
HYD_REAL PRESSURE_LIMIT;  // MPa, 全局最大压力限制, 0 表示不启用
```

**段级参数（新增）：**

```c
// HYD_MotionSegment 中新增
HYD_REAL maxPressure;     // MPa, 本段最大压力限制, 0 表示使用全局限制
```

**生效规则：**

```
effectiveMaxPressure = 取两者中的较小非零值:
  - segment->maxPressure (如果 > 0)
  - fb->PRESSURE_LIMIT   (如果 > 0)

如果两者都为 0，则不启用压力限制。
```

### 3.2 适用范围

| 控制模式 | 是否生效 | 说明 |
|----------|----------|------|
| HYD_MODE_POSITION | 是 | 主要保护场景 |
| HYD_MODE_SPEED_RAMP | 是 | 主要保护场景 |
| HYD_MODE_PRESSURE_CLOSED_LOOP | 是 | 作为最终兜底（防止 targetPressure 误设） |

### 3.3 限制算法（比例限速）

当 `AXIS_REF.pressure > effectiveMaxPressure` 时，按比例缩减输出流量：

```c
overRatio = (actualPressure - effectiveMaxPressure) / effectiveMaxPressure;
scale = clamp(1.0 - Kp_pressure_limit * overRatio, minScale, 1.0);
commandFlow *= scale;
pumpSpeed *= scale;
```

参数：
- `Kp_pressure_limit`：限制增益，默认 5.0（超压10%时输出降至50%）
- `minScale`：最小缩放比，默认 0.0（允许完全停泵）

### 3.4 保护层级

| 层级 | 触发条件 | 动作 | 诊断码 |
|------|----------|------|--------|
| L1 比例限速 | pressure > effectiveMaxPressure | 按比例降低 commandFlow/pumpSpeed | 无（静默限制） |
| L2 报警 | pressure > effectiveMaxPressure 且持续 > debounceTime | WARNING + DERATE | HYD_DIAG_CODE_OVER_PRESSURE |
| L3 故障 | pressure > effectiveMaxPressure 且持续 > faultEscalationTime | FAULT + 停机 | HYD_DIAG_CODE_OVER_PRESSURE (FAULT severity) |

时间参数（hyd_config.h §14B）：
- `debounceTime`：100ms（避免瞬态误报）
- `faultEscalationTime`：500ms（持续超压必须停机）

### 3.5 与现有机制的关系

- `pressureCeiling`：位置窗口内的精细保护，保留不变
- `maxPressure`：全段全时段的安全包络
- 两者独立评估，任一触发即生效
- 优先级：`pressureCeiling` FAULT > `maxPressure` FAULT > `maxPressure` WARNING

### 3.6 实现位置

在 `HYD_ExecuteActiveSegmentControl` 返回后、`HYD_OutputLimiter_Execute` 调用时生效。

扩展 `HYD_OutputLimiterInput`：

```c
// 新增字段
HYD_REAL actualPressure;         // 当前压力反馈
HYD_REAL effectiveMaxPressure;   // 生效的最大压力限制, 0 表示不启用
```

在 `HYD_OutputLimiter_Execute` 内部，在现有 DERATE/STOP 逻辑之后、pumpSpeedLimit 裁剪之前，插入压力比例限速逻辑。

---

## 4. 软限位保护

### 4.1 参数定义

**扩展 `HYD_CylinderConfig`：**

```c
typedef struct {
    HYD_REAL areaExtendMm2;       // 无杆侧有效面积 [mm²]
    HYD_REAL areaRetractMm2;      // 有杆侧有效面积 [mm²]
    HYD_REAL strokeMm;            // 最大行程 [mm]（已有）
    HYD_REAL softLimitBandMm;     // 减速带宽度 [mm], 0 表示不启用软限位
    HYD_REAL softLimitRetractMm;  // 负向软限位位置 [mm], 默认 0（油缸完全缩回）
} HYD_CylinderConfig;
```

**坐标约定：**
- `position = 0` 对应油缸完全缩回
- `position = strokeMm` 对应油缸完全伸出
- `softLimitRetractMm` 支持 > 0（偏移量），表示负向不允许缩回到该位置以下

**生效条件：**
- `strokeMm > 0` 且 `softLimitBandMm > 0` 时启用软限位
- 正向极限 = `strokeMm`
- 负向极限 = `softLimitRetractMm`

### 4.2 运行时保护算法

```c
if (cylinderConfig.strokeMm > 0 && cylinderConfig.softLimitBandMm > 0) {
    band = cylinderConfig.softLimitBandMm;

    // 正向软限位（接近 strokeMm）
    if (direction == EXTEND) {
        remaining = cylinderConfig.strokeMm - position;
        if (remaining < band) {
            scale = clamp(remaining / band, 0.0, 1.0);
            commandFlow *= scale;
            pumpSpeed *= scale;
        }
    }

    // 负向软限位（接近 softLimitRetractMm）
    if (direction == RETRACT) {
        remaining = position - cylinderConfig.softLimitRetractMm;
        if (remaining < band) {
            scale = clamp(remaining / band, 0.0, 1.0);
            commandFlow *= scale;
            pumpSpeed *= scale;
        }
    }
}
```

### 4.3 保护层级

| 层级 | 触发条件 | 动作 | 诊断码 |
|------|----------|------|--------|
| L1 减速 | position 进入 softLimitBand | 比例降低输出 | 无 |
| L2 报警 | position 到达极限（remaining <= 0） | WARNING | HYD_DIAG_CODE_SOFT_LIMIT_REACHED |
| L3 故障 | position 超出极限且持续 > debounce | FAULT + 停机 | HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED |

关键约束：软限位只限制**向极限方向**的运动，不阻止**远离极限方向**的运动（允许退回）。

### 4.4 启动前校验

在 `HYD_RecipeValidator_ValidateSegment` 中增加：

```c
if (cylinderConfig.strokeMm > 0) {
    if (segment->targetPosition > cylinderConfig.strokeMm) {
        *code = HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED;
        return false;  // 拒绝启动
    }
    if (segment->targetPosition < cylinderConfig.softLimitRetractMm) {
        *code = HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED;
        return false;
    }
}
```

### 4.5 实现位置

与压力限制共享同一执行点。两个限制独立计算 scale，取最小值：

```c
finalScale = min(pressureLimitScale, softLimitScale);
commandFlow *= finalScale;
pumpSpeed *= finalScale;
```

---

## 5. 新增诊断码

```c
HYD_DIAG_CODE_SOFT_LIMIT_REACHED,   // WARNING: 到达软限位
HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED,  // FAULT: 超出软限位
```

新增诊断标志位：

```c
HYD_DIAG_FLAG_SOFT_LIMIT = 1U << 11,
```

---

## 6. 新增 hyd_config.h 阈值

```c
// §14B 新增

/* 压力限制比例增益：超压 overRatio 时的输出缩减系数。
 * scale = 1.0 - Kp * overRatio, 超压10%时 scale = 1.0 - 5.0*0.1 = 0.5
 * 单位：无量纲。 */
#define HYD_THRESH_PRESSURE_LIMIT_KP             5.0f

/* 压力限制最小缩放比：允许完全停泵。
 * 单位：无量纲 [0, 1)。 */
#define HYD_THRESH_PRESSURE_LIMIT_MIN_SCALE      0.0f

/* 压力限制 WARNING 触发前的 debounce 时长。
 * 单位：秒。 */
#define HYD_THRESH_PRESSURE_LIMIT_DEBOUNCE_S     0.10f

/* 压力限制 WARNING -> FAULT 升级时长。
 * 单位：秒。 */
#define HYD_THRESH_PRESSURE_LIMIT_FAULT_ESCALATION_S  0.50f
```

---

## 7. 执行流变更

变更后的每周期执行流：

```
AXIS_REF (feedback) -> Execute()
  -> ramp_controller
  -> motion_planner / pressure_controller
  -> pump_converter
  -> output_limiter (扩展：含压力限制 + 软限位 + DERATE/STOP + pumpSpeedLimit)
  -> segment_completion
  -> diagnostics
  -> outputs
```

压力限制和软限位的 scale 计算集成到 `output_limiter` 内部，作为 `OutputLimiterInput` 的附加约束。执行顺序：

1. 计算 pressureLimitScale（压力比例限速）
2. 计算 softLimitScale（位置软限位）
3. finalScale = min(pressureLimitScale, softLimitScale)
4. 应用 finalScale 到 commandFlow / pumpSpeed
5. 现有 protectionAction DERATE/STOP 逻辑（叠加）
6. pumpSpeedLimit 硬裁剪

---

## 8. 不变项

- `pressureCeiling` 机制保持不变（位置窗口内的低压模保护）
- `output_limiter` 现有的 protectionAction DERATE/STOP 逻辑保持不变
- `velocityController` 保持不变
- 泵速始终非负，方向由 `plannedDirection` 信号
- `maxPressure = 0` 且 `PRESSURE_LIMIT = 0` 时行为与当前完全一致（向后兼容）
- `softLimitBandMm = 0` 或 `strokeMm = 0` 时软限位不生效（向后兼容）

---

## 9. 测试要点

1. 压力限制：位置模式下模拟负载突增，验证泵速按比例降低
2. 压力限制：速度模式下模拟堵转，验证 WARNING -> FAULT 升级
3. 压力限制：压力模式下设置 targetPressure > maxPressure，验证兜底生效
4. 压力限制：maxPressure = 0 且 PRESSURE_LIMIT = 0 时不干预（向后兼容）
5. 压力限制：段级 maxPressure < 全局 PRESSURE_LIMIT，验证取较小值
6. 软限位：位置模式 targetPosition > strokeMm，验证启动前拒绝
7. 软限位：运行中接近 strokeMm，验证平滑减速
8. 软限位：超出极限后反向运动不受限
9. 软限位：softLimitRetractMm > 0 时负向限位正确
10. 两者叠加：同时触发压力限制和软限位，取最小 scale
