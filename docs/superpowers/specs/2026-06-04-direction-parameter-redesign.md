# Direction 参数规则重设计

**日期：** 2026-06-04  
**状态：** 已确认  
**基于：** Beckhoff TF5810 MC_Direction 枚举规范

---

## 1. 变更概要

将运动控制库的 Direction 参数从当前 PLCopen 三值映射（`>0=EXTEND, <0=RETRACT, 0=AUTO`）升级为 Beckhoff TF5810 四值枚举（`0=Shortest_Way, 1=Positive, 2=Negative, 3=Current`），并在 `HYD_MOVEABSOLUTE`、`HYD_MOVEVELOCITY`、`HYD_MOVEPROFILE` 三个功能块中实现新规则。

---

## 2. 新旧 Direction 枚举对比

### 2.1 Beckhoff 原始定义

```pascal
TYPE MC_Direction BkPlcMc:
(
    MC_Positive_Direction_BkPlcMc := 1,
    MC_Shortest_Way_BkPlcMc,          -- = 0
    MC_Negative_Direction_BkPlcMc,    -- = 2
    MC_Current_Direction_BkPlcMc      -- = 3
);
END_TYPE
```

### 2.2 新枚举定义（本项目）

```c
typedef enum {
    HYD_DIRECTION_SHORTEST_WAY   = 0,  /* 自动最短路径 */
    HYD_DIRECTION_POSITIVE       = 1,  /* 强制正向 (EXTEND) */
    HYD_DIRECTION_NEGATIVE       = 2,  /* 强制负向 (RETRACT) */
    HYD_DIRECTION_CURRENT        = 3,  /* 保持当前方向 */
    HYD_DIRECTION_HOLD           = 4   /* 无运动（保压专用） */
} HYD_MotionDirection;
```

### 2.3 向后兼容别名

```c
#define HYD_DIRECTION_AUTO    HYD_DIRECTION_SHORTEST_WAY
#define HYD_DIRECTION_EXTEND  HYD_DIRECTION_POSITIVE
#define HYD_DIRECTION_RETRACT HYD_DIRECTION_NEGATIVE
```

### 2.4 关键差异

| 项目 | 旧值 | 新值 | 说明 |
|------|------|------|------|
| HYD_DIRECTION_HOLD | 3 | **4** | 与 HYD_DIRECTION_CURRENT(3) 冲突，重编号 |
| PLC 接口映射 | SINT>0→EXTEND,SINT<0→RETRACT | **SINT值直接枚举** | 0/1/2/3 一一对应 |
| 方向推断 | 只有 AUTO→位置推断 | **新增 CURRENT→继承记忆** | 需新增 `_lastActiveDirection` |

**影响面分析：** HYD_DIRECTION_HOLD 仅在源代码中通过符号名使用（`HYD_DIRECTION_HOLD`），无硬编码数值 3 的引用，重新编号不影响任何现有代码路径。

---

## 3. 各功能块新规则

### 3.1 `HYD_MOVEPROFILE`（Recipe 模式）

| 方向参数 | MOTION.DIRECTION (USINT 0-3) |
|----------|---------------------------|
| 映射方式 | `buildSegmentFromMotion()` 解析时通过 `mapPlcOpenDirectionToSegmentDirection()` 转换 |
| 运行时行为 | 由 `HYD_Segment_ResolveDirection()` 统一处理，无需 FB 级特殊逻辑 |

MoveProfile 运行时方向由段描述符的 `direction` 字段携带至 planner，`ResolveDirection` 会统一处理四种值。FB 命令接口无需修改。

### 3.2 `HYD_MOVEABSOLUTE`（Direct 模式位置控制）

| 方向参数 | 行为 |
|----------|------|
| `SHORTEST_WAY` (0) | Velocity 恒取绝对值（符号无效）；方向由目标位置与当前位置的差值自动判定 |
| `POSITIVE` (1) | 强制正向运动；**校验目标位置 ≥ 当前位置 - tolerance**，不满足则 `ERROR=true` |
| `NEGATIVE` (2) | 强制负向运动；**校验目标位置 ≤ 当前位置 + tolerance**，不满足则 `ERROR=true` |
| `CURRENT` (3) | 继承 `_lastActiveDirection`；静止轴默认正向；Velocity 取绝对值 |

**报警触发条件：**
- POSITIVE 且 `targetPos < currentPos - positionTolerance` → `ERROR=true, ERRORID=HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT`
- NEGATIVE 且 `targetPos > currentPos + positionTolerance` → 同上

**Velocity 符号处理：** 除 SHORTEST_WAY 外的所有方向参数下，Velocity 符号被忽略（取绝对值），方向优先级高于 Velocity 符号。

### 3.3 `HYD_MOVEVELOCITY`（Direct 模式速度控制）

| 方向参数 | 行为 |
|----------|------|
| `SHORTEST_WAY` (0) | 根据 Velocity 正负区分方向（`>0→POSITIVE, <0→NEGATIVE, ==0→lastActiveDirection`） |
| `POSITIVE` (1) | 强制正向运动，Velocity 符号被忽略（取绝对值） |
| `NEGATIVE` (2) | 强制负向运动，Velocity 符号被忽略（取绝对值） |
| `CURRENT` (3) | 继承 `_lastActiveDirection`；静止轴默认正向；Velocity 取绝对值 |

**Direction 优先级规则：** `POSITIVE/NEGATIVE/CURRENT → Direction 覆盖 Velocity 符号；SHORTEST_WAY → Velocity 符号决定方向`。

---

## 4. 架构修改清单

### 4.1 枚举层 (`include/common_types.h`)

- 新增 `HYD_DIRECTION_SHORTEST_WAY=0`、`HYD_DIRECTION_POSITIVE=1`、`HYD_DIRECTION_NEGATIVE=2`、`HYD_DIRECTION_CURRENT=3`
- `HYD_DIRECTION_HOLD` 从 3 移到 4
- 保留 `#define` 向后兼容别名

### 4.2 FB 状态层 (`include/motion_control.h`)

- 在 `HYD_MotionControlFB` INTERNAL 区新增：`HYD_MotionDirection _lastActiveDirection;`
- 在 `HYD_MotionControlFB_Init()` 中初始化为 `HYD_DIRECTION_POSITIVE`

### 4.3 方向解析层 (`include/segment_limits.h` + `src/segment_limits.c`)

- `HYD_Segment_ResolveDirection()` 签名新增 `HYD_MotionDirection lastActiveDirection` 参数
- 新增 `HYD_DIRECTION_CURRENT` 分支：返回 `lastActiveDirection`，若为 HOLD 或无记录则返回 `HYD_DIRECTION_POSITIVE`
- 所有调用点同步更新参数

### 4.4 Planner 层 (`include/motion_planner.h` + `src/motion_planner.c`)

- `HYD_MotionPlannerInput` 新增 `HYD_MotionDirection lastActiveDirection` 字段
- `HYD_MotionPlanner_Execute()` 调用 `ResolveDirection` 时传入此字段

### 4.5 PLC 接口层 (`src/motion_interface.c`)

- `mapPlcOpenDirection()` 重写为 switch-case 枚举映射（0→SHORTEST_WAY, 1→POSITIVE, 2→NEGATIVE, 3→CURRENT）
- `__mcl_cmd_MoveAbsolute()` execRising 分支新增方向-位置一致性校验
- `__mcl_cmd_MoveVelocity()` execRising 分支新增 Direction 优先逻辑

### 4.6 编译适配层 (`src/motion_control.c`)

- `HYD_BeginSegment()` 末尾新增 `_lastActiveDirection` 更新逻辑
- 所有 `HYD_Segment_ResolveDirection()` 调用点传入新增参数

### 4.7 其他调用点

| 文件 | 函数 | 修改内容 |
|------|------|----------|
| `src/segment_completion.c` | `HYD_SegmentCompletion_IsPositionReached` | 传入 `lastActiveDirection` |
| `src/motion_control.c` | `HYD_AreBlendDirectionsCompatible` | 传入 `lastActiveDirection` |
| `src/motion_control.c` | `HYD_AdjustPumpGainByDirection` | 传入 `lastActiveDirection` |

---

## 5. 方向决策流程（统一模型）

```
PLC DIRECTION (SINT: 0=Shortest_Way, 1=Positive, 2=Negative, 3=Current)
    │
    ▼
mapPlcOpenDirection() → HYD_MotionDirection 枚举
    │
    ├── MoveAbsolute: 提前校验方向-位置一致性（POSITIVE/NEGATIVE时）
    ├── MoveVelocity: 提前解析方向覆盖Velocity符号（所有非SHORTEST_WAY模式）
    └── MoveProfile:  直接写入 segment.direction
    │
    ▼
segment.direction = 枚举值 (0-4)
    │
    ▼
HYD_BeginSegment() → 更新 _lastActiveDirection（跳过HOLD）
    │
    ▼
HYD_Segment_ResolveDirection(segment, axisRef, lastActiveDirection)
    │
    ├── POSITIVE/NEGATIVE/HOLD → 直接返回
    ├── SHORTEST_WAY → 位置差推断 (delta>tol→POSITIVE, delta<-tol→NEGATIVE)
    └── CURRENT → lastActiveDirection; 若为HOLD→POSITIVE
    │
    ▼
HYD_MotionPlanner_Execute() → 根据方向计算符号与速度
```

---

## 6. 关键代码示例

### 6.1 `mapPlcOpenDirection` 重写

```c
static HYD_MotionDirection mapPlcOpenDirection(IEC_SINT direction) {
    switch ((int)direction) {
        case 1:  return HYD_DIRECTION_POSITIVE;
        case 2:  return HYD_DIRECTION_NEGATIVE;
        case 3:  return HYD_DIRECTION_CURRENT;
        default: return HYD_DIRECTION_SHORTEST_WAY;
    }
}
```

### 6.2 `HYD_Segment_ResolveDirection` 新增 CURRENT 分支

```c
HYD_MotionDirection HYD_Segment_ResolveDirection(
        const HYD_MotionSegment* segment,
        const HYD_AxisRef* axisRef,
        HYD_MotionDirection lastActiveDirection) {
    
    if (segment == NULL || axisRef == NULL) {
        return HYD_DIRECTION_HOLD;
    }
    
    /* 显式方向声明优先 */
    if (segment->direction == HYD_DIRECTION_POSITIVE ||
        segment->direction == HYD_DIRECTION_NEGATIVE ||
        segment->direction == HYD_DIRECTION_HOLD) {
        return segment->direction;
    }
    
    /* CURRENT: 继承上一次运动方向 */
    if (segment->direction == HYD_DIRECTION_CURRENT) {
        if (lastActiveDirection == HYD_DIRECTION_POSITIVE ||
            lastActiveDirection == HYD_DIRECTION_NEGATIVE) {
            return lastActiveDirection;
        }
        return HYD_DIRECTION_POSITIVE;  /* 静止轴默认正向 */
    }
    
    /* SHORTEST_WAY: 位置差推断 */
    HYD_REAL positionTolerance = HYD_Segment_GetPositionTolerance(segment);
    HYD_REAL delta = segment->targetPosition - axisRef->position;
    if (delta > positionTolerance)  return HYD_DIRECTION_POSITIVE;
    if (delta < -positionTolerance) return HYD_DIRECTION_NEGATIVE;
    return HYD_DIRECTION_HOLD;
}
```

### 6.3 MoveAbsolute 方向校验与 Velocity 处理

```c
HYD_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
HYD_REAL velocity = __GET_VAR(data__->VELOCITY);
HYD_REAL targetPos = __GET_VAR(data__->POSITION);
HYD_REAL currentPos = fb->AXIS_REF.position;

if (dir == HYD_DIRECTION_POSITIVE &&
    targetPos < currentPos - fb->_params.positionTolerance) {
    __SET_VAR(data__->, ERROR,, true);
    __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT);
    __SET_VAR(data__->, EXECUTE0,, execute);
    return;
}
if (dir == HYD_DIRECTION_NEGATIVE &&
    targetPos > currentPos + fb->_params.positionTolerance) {
    __SET_VAR(data__->, ERROR,, true);
    __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT);
    __SET_VAR(data__->, EXECUTE0,, execute);
    return;
}

/* 非 SHORTEST_WAY 模式：Direction 优先级高于 Velocity 符号 */
if (dir != HYD_DIRECTION_SHORTEST_WAY) {
    velocity = fabs(velocity);
}

HYD_MotionSegment segment = buildPositionSegment(targetPos, velocity,
    __GET_VAR(data__->ACCELERATION),
    __GET_VAR(data__->DECELERATION),
    dir, fb);
```

### 6.4 MoveVelocity 方向解析逻辑

```c
HYD_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
HYD_REAL velocity = __GET_VAR(data__->VELOCITY);

if (dir == HYD_DIRECTION_SHORTEST_WAY) {
    /* 根据 Velocity 正负区分方向 */
    if (velocity > 0.0f) {
        dir = HYD_DIRECTION_POSITIVE;
    } else if (velocity < 0.0f) {
        dir = HYD_DIRECTION_NEGATIVE;
    } else {
        dir = (fb->_lastActiveDirection == HYD_DIRECTION_NEGATIVE)
              ? HYD_DIRECTION_NEGATIVE : HYD_DIRECTION_POSITIVE;
    }
}

/* Direction 优先级高于 Velocity 符号 —— 统一取绝对值 */
velocity = fabs(velocity);

HYD_MotionSegment segment = buildVelocitySegment(velocity,
    __GET_VAR(data__->ACCELERATION),
    __GET_VAR(data__->DECELERATION),
    dir, fb);
```

### 6.5 `_lastActiveDirection` 维护（`HYD_BeginSegment` 末尾）

```c
/* 更新轴方向记忆（跳过 HOLD，压力模式不改变运动方向） */
if (fb->_activeSegment.direction != HYD_DIRECTION_HOLD) {
    HYD_MotionDirection resolved = HYD_Segment_ResolveDirection(
        &fb->_activeSegment, &fb->AXIS_REF, fb->_lastActiveDirection);
    if (resolved == HYD_DIRECTION_POSITIVE || resolved == HYD_DIRECTION_NEGATIVE) {
        fb->_lastActiveDirection = resolved;
    }
}
```

---

## 7. 报警触发条件汇总

| 触发条件 | 严重度 | 诊断码 | 触发位置 |
|----------|--------|--------|----------|
| MoveAbsolute + POSITIVE + 目标在当前位置后方 | ERROR | `HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT` | `motion_interface.c:__mcl_cmd_MoveAbsolute` |
| MoveAbsolute + NEGATIVE + 目标在当前位置前方 | ERROR | 同上 | 同上 |
| MoveVelocity + SHORTEST_WAY + Velocity==0 | - | 无 | 退化为 lastActiveDirection |
| 任意 FB + DIRECTION 值超出 0-3 | - | 无 | 默认视为 SHORTEST_WAY |

---

## 8. 兼容性保证

| 项目 | 策略 |
|------|------|
| 现有枚举符号名 (`HYD_DIRECTION_EXTEND` 等) | `#define` 别名，零修改 |
| `HYD_DIRECTION_HOLD` (3→4) | 全代码库仅通过符号名引用，零修改 |
| `HYD_AXISMOTION.DIRECTION` 读写 | 值的语义变化由 `mapPlcOpenDirectionToSegmentDirection` 内部处理 |
| 测试代码（使用 `HYD_DIRECTION_EXTEND` 等） | 通过别名自动适配 |
| 单泵方向冲突检测 (`GetPumpRequest`) | 方向值变化对 CONFLICT 逻辑透明（仍比较 POSITIVE vs NEGATIVE） |

---

## 9. 自审清单

- [x] 无 TODO/TBD 占位符
- [x] 枚举重定义与别名无冲突
- [x] `_lastActiveDirection` 初始化路径明确
- [x] 所有 `ResolveDirection` 调用点已列举
- [x] MoveAbsolute 方向校验在 `execRising` 早期执行（先于 segment 构建）
- [x] CURRENT 方向在静止轴退化为 POSITIVE（与 Beckhoff 规范一致）
- [x] 单泵方向冲突检测逻辑不受影响
