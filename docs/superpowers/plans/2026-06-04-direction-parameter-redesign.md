# Direction 参数规则重设计 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将运动控制库 Direction 参数从 PLCopen 三值映射升级为 Beckhoff TF5810 四值枚举，实现 MoveAbsolute/MoveVelocity/MoveProfile 新方向规则。

**Architecture:** 枚举层 → 接口映射层 → 方向解析层 → 执行层。修改从底层枚举定义向上传播到 PLC 接口，核心方向决策在 `HYD_Segment_ResolveDirection` 统一处理，FB 级添加 `_lastActiveDirection` 状态字段支持 CURRENT 方向。

**Tech Stack:** C99, IEC61131-3 matiec 类型系统, CMake + GCC

---

## File Structure

| 文件 | 角色 | 操作 |
|------|------|------|
| `include/common_types.h` | Direction 枚举重定义 + 兼容别名 | Modify |
| `include/motion_control.h` | FB 结构体新增 `_lastActiveDirection` | Modify |
| `include/motion_planner.h` | `HYD_MotionPlannerInput` 新增字段 | Modify |
| `include/segment_limits.h` | `ResolveDirection` 签名新增参数 | Modify |
| `src/segment_limits.c` | `ResolveDirection` 实现 CURRENT 分支 | Modify |
| `src/motion_interface.c` | `mapPlcOpenDirection` 重写 + MoveAbsolute/MoveVelocity 新逻辑 | Modify |
| `src/motion_control.c` | `Init` + `BeginSegment` + 5 个 ResolveDirection 调用点 | Modify |
| `src/motion_planner.c` | `Execute` 传入 lastActiveDirection | Modify |
| `src/segment_completion.c` | `IsPositionReached` 传入 lastActiveDirection | Modify |
| `tests/test_motion_interface_unit.c` | 新增方向映射测试 | Modify |
| `tests/test_motion_planner.c` | 更新方向测试 | Modify |
| `tests/test_direct_mode.c` | 新增 MoveAbsolute 方向校验测试 | Modify |

---

### Task 1: 枚举重定义与向后兼容别名

**Files:**
- Modify: `include/common_types.h:57-65`

- [ ] **Step 1: 修改 `HYD_MotionDirection` 枚举**

```c
// 在 include/common_types.h 中定位第 57-65 行的 HYD_MotionDirection typedef，
// 将原有 4 值枚举替换为 5 值枚举，新增向后兼容别名。

// 原代码（第 57-65 行）：
// typedef enum {
//     HYD_DIRECTION_AUTO,
//     HYD_DIRECTION_EXTEND,
//     HYD_DIRECTION_RETRACT,
//     HYD_DIRECTION_HOLD
// } HYD_MotionDirection;

// 替换为：
typedef enum {
    HYD_DIRECTION_SHORTEST_WAY   = 0,  /* 自动最短路径 */
    HYD_DIRECTION_POSITIVE       = 1,  /* 强制正向 (EXTEND) */
    HYD_DIRECTION_NEGATIVE       = 2,  /* 强制负向 (RETRACT) */
    HYD_DIRECTION_CURRENT        = 3,  /* 保持当前方向 */
    HYD_DIRECTION_HOLD           = 4   /* 无运动（保压专用） */
} HYD_MotionDirection;

/* 向后兼容别名 —— 所有使用 HYD_DIRECTION_EXTEND 等符号的代码无需修改 */
#define HYD_DIRECTION_AUTO    HYD_DIRECTION_SHORTEST_WAY
#define HYD_DIRECTION_EXTEND  HYD_DIRECTION_POSITIVE
#define HYD_DIRECTION_RETRACT HYD_DIRECTION_NEGATIVE
```

- [ ] **Step 2: 验证编译通过**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
```

Expected: 编译成功。所有通过 `#define` 别名引用的旧符号（`HYD_DIRECTION_EXTEND`、`HYD_DIRECTION_RETRACT`、`HYD_DIRECTION_AUTO`）自动映射到新值。

- [ ] **Step 3: 运行全量测试确认无回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: All tests pass。

- [ ] **Step 4: Commit**

```bash
git add include/common_types.h
git commit -m "feat: redefine HYD_MotionDirection enum for Beckhoff TF5810 compatibility

- Add HYD_DIRECTION_SHORTEST_WAY=0, HYD_DIRECTION_POSITIVE=1,
  HYD_DIRECTION_NEGATIVE=2, HYD_DIRECTION_CURRENT=3
- Move HYD_DIRECTION_HOLD to value 4 (was 3, conflicting with CURRENT)
- Preserve backward compatibility via #define aliases for
  HYD_DIRECTION_AUTO, HYD_DIRECTION_EXTEND, HYD_DIRECTION_RETRACT"
```

---

### Task 2: FB 新增 `_lastActiveDirection` 状态字段

**Files:**
- Modify: `include/motion_control.h` (INTERNAL 区)
- Modify: `src/motion_control.c` (Init 函数)

- [ ] **Step 1: 在 `HYD_MotionControlFB` INTERNAL 区新增字段**

在 `include/motion_control.h` 中定位 `HYD_MotionControlFB` 结构体的 INTERNAL 区（约第 258-348 行），在 `_lastFeedbackTimestamp` 附近添加：

```c
    /* --- Direction memory for HYD_DIRECTION_CURRENT --- */
    HYD_MotionDirection _lastActiveDirection;  /* 上一次非HOLD的实际运动方向，静止轴初始化POSITIVE */
```

- [ ] **Step 2: 在 `HYD_MotionControlFB_Init` 中初始化为 `HYD_DIRECTION_POSITIVE`**

定位 `src/motion_control.c` 中的 `HYD_MotionControlFB_Init` 函数。该函数使用 `memset(fb, 0, ...)` 将整个 FB 归零。由于 `HYD_DIRECTION_POSITIVE = 1`（非零值），memset 归零后 `_lastActiveDirection` 会是 0（即 `HYD_DIRECTION_SHORTEST_WAY`），需要显式初始化。

在 `memset(fb, 0, sizeof(HYD_MotionControlFB))` 之后添加：

```c
    fb->_lastActiveDirection = HYD_DIRECTION_POSITIVE;  /* 静止轴默认正向 */
```

- [ ] **Step 3: 验证编译通过**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
```

Expected: 编译成功。

- [ ] **Step 4: Commit**

```bash
git add include/motion_control.h src/motion_control.c
git commit -m "feat: add _lastActiveDirection field for CURRENT direction support"
```

---

### Task 3: `HYD_Segment_ResolveDirection` 签名扩展 + CURRENT 分支

**Files:**
- Modify: `include/segment_limits.h:41-42`
- Modify: `src/segment_limits.c:58-84`

- [ ] **Step 1: 更新头文件声明**

修改 `include/segment_limits.h` 中 `HYD_Segment_ResolveDirection` 的声明（第 41-42 行）：

```c
// 原代码：
HYD_MotionDirection HYD_Segment_ResolveDirection(const HYD_MotionSegment* segment,
                                                   const HYD_AxisRef* axisRef);

// 替换为：
HYD_MotionDirection HYD_Segment_ResolveDirection(const HYD_MotionSegment* segment,
                                                   const HYD_AxisRef* axisRef,
                                                   HYD_MotionDirection lastActiveDirection);
```

- [ ] **Step 2: 更新实现 —— 新增 CURRENT 分支**

修改 `src/segment_limits.c` 中 `HYD_Segment_ResolveDirection` 函数实现（第 58-84 行）：

```c
HYD_MotionDirection HYD_Segment_ResolveDirection(const HYD_MotionSegment* segment,
                                                   const HYD_AxisRef* axisRef,
                                                   HYD_MotionDirection lastActiveDirection) {
    HYD_REAL delta;
    HYD_REAL positionTolerance;

    if (segment == NULL || axisRef == NULL) {
        return HYD_DIRECTION_HOLD;
    }

    /* 显式方向声明优先 */
    if (segment->direction == HYD_DIRECTION_POSITIVE ||
        segment->direction == HYD_DIRECTION_NEGATIVE ||
        segment->direction == HYD_DIRECTION_HOLD) {
        return segment->direction;
    }

    /* CURRENT: 继承上一次运动方向，静止轴默认正向 */
    if (segment->direction == HYD_DIRECTION_CURRENT) {
        if (lastActiveDirection == HYD_DIRECTION_POSITIVE ||
            lastActiveDirection == HYD_DIRECTION_NEGATIVE) {
            return lastActiveDirection;
        }
        return HYD_DIRECTION_POSITIVE;  /* 静止轴默认正向 */
    }

    /* SHORTEST_WAY: 位置差推断 */
    positionTolerance = HYD_Segment_GetPositionTolerance(segment);
    delta = segment->targetPosition - axisRef->position;
    if (delta > positionTolerance) {
        return HYD_DIRECTION_POSITIVE;
    }
    if (delta < -positionTolerance) {
        return HYD_DIRECTION_NEGATIVE;
    }
    return HYD_DIRECTION_HOLD;
}
```

关键改动：（1）新增 `lastActiveDirection` 参数；（2）`EXTEND` → `POSITIVE`、`RETRACT` → `NEGATIVE` 的显式条件检查（因别名存在，两种写法等价）；（3）新增 `HYD_DIRECTION_CURRENT` 分支。

- [ ] **Step 3: Commit**

```bash
git add include/segment_limits.h src/segment_limits.c
git commit -m "feat: add CURRENT direction branch to HYD_Segment_ResolveDirection"
```

---

### Task 4: 更新 `HYD_MotionPlannerInput` 结构体 + planner 调用点

**Files:**
- Modify: `include/motion_planner.h:21-31`
- Modify: `src/motion_planner.c:462-465`
- Modify: `src/motion_control.c:1333-1350` (plannerInput 构造处)

- [ ] **Step 1: 在 `HYD_MotionPlannerInput` 中新增 `lastActiveDirection` 字段**

修改 `include/motion_planner.h`，在第 30 行（`blend` 字段之后）添加：

```c
typedef struct {
    const HYD_AxisRef* axisRef;
    const HYD_MotionSegment* segment;
    HYD_REAL elapsedTime;
    HYD_REAL deltaTime;
    HYD_REAL rampedPressure;
    HYD_REAL decelElapsed;
    HYD_REAL decelStartVel;
    HYD_MotionPlannerState* state;
    const HYD_MotionBlendContext* blend;
    HYD_MotionDirection lastActiveDirection;  /* 用于 CURRENT 方向解析 */
} HYD_MotionPlannerInput;
```

- [ ] **Step 2: 更新 `HYD_MotionPlanner_Execute` 中的 ResolveDirection 调用**

修改 `src/motion_planner.c` 第 465 行：

```c
// 原代码：
direction = HYD_Segment_ResolveDirection(input->segment, input->axisRef);

// 替换为：
direction = HYD_Segment_ResolveDirection(input->segment, input->axisRef,
                                         input->lastActiveDirection);
```

同时更新第 479 行附近对 `HYD_DIRECTION_AUTO` 的引用 —— 因 `#define` 别名存在，这里可以保留，但为一致性改为 `HYD_DIRECTION_SHORTEST_WAY`：

```c
// 原代码（约第 479 行）：
if (direction == HYD_DIRECTION_HOLD &&
    input->segment->mode == HYD_MODE_POSITION &&
    input->segment->planner == HYD_PLANNER_POSITION_BASED &&
    input->segment->direction == HYD_DIRECTION_AUTO &&
    previousDirectionSign != 0.0) {

// 替换为（别名已兼容，直接改符号名更清晰）：
if (direction == HYD_DIRECTION_HOLD &&
    input->segment->mode == HYD_MODE_POSITION &&
    input->segment->planner == HYD_PLANNER_POSITION_BASED &&
    input->segment->direction == HYD_DIRECTION_SHORTEST_WAY &&
    previousDirectionSign != 0.0) {
```

- [ ] **Step 3: 更新 `motion_control.c` 中 `plannerInput` 构造处**

在 `src/motion_control.c` 的 `HYD_ExecuteActiveSegmentControl` 函数中（约第 1333-1350 行），`memset(&plannerInput, ...)` 后将 `blend` 字段赋值行之后添加：

```c
        plannerInput.blend = fb->_directBlendContext.active
            ? &fb->_directBlendContext
            : NULL;
        plannerInput.lastActiveDirection = fb->_lastActiveDirection;  /* 新增 */
        HYD_MotionPlanner_Execute(&plannerInput, plannerOutput);
```

- [ ] **Step 4: 验证编译通过**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
```

Expected: 编译成功（此时部分调用点仍有编译错误——签名变更但调用点尚未更新。后续 Task 修复）。

- [ ] **Step 5: Commit**

```bash
git add include/motion_planner.h src/motion_planner.c src/motion_control.c
git commit -m "feat: add lastActiveDirection to MotionPlannerInput for CURRENT support"
```

---

### Task 5: 更新 `motion_control.c` 和 `segment_completion.c` 中所有 `ResolveDirection` 调用点

**Files:**
- Modify: `src/motion_control.c`（3 处调用点）
- Modify: `src/segment_completion.c`（1 处调用点）

- [ ] **Step 1: 更新 `HYD_AreBlendDirectionsCompatible`**

在 `src/motion_control.c` 第 290-291 行：

```c
// 原代码：
    activeDirection = HYD_Segment_ResolveDirection(activeSegment, &fb->AXIS_REF);
    pendingDirection = HYD_Segment_ResolveDirection(pendingSegment, &fb->AXIS_REF);

// 替换为：
    activeDirection = HYD_Segment_ResolveDirection(activeSegment, &fb->AXIS_REF,
                                                    fb->_lastActiveDirection);
    pendingDirection = HYD_Segment_ResolveDirection(pendingSegment, &fb->AXIS_REF,
                                                     fb->_lastActiveDirection);
```

同时更新第 293-294 行引用的 `HYD_DIRECTION_EXTEND` / `HYD_DIRECTION_RETRACT`（别名已兼容，可保留不修改）。

- [ ] **Step 2: 更新 `HYD_ShouldCutoverDirectBlend`**

在 `src/motion_control.c` 第 387 行：

```c
// 原代码：
    direction = HYD_Segment_ResolveDirection(segment, &fb->AXIS_REF);

// 替换为：
    direction = HYD_Segment_ResolveDirection(segment, &fb->AXIS_REF,
                                              fb->_lastActiveDirection);
```

- [ ] **Step 3: 更新 `HYD_SegmentCompletion_IsPositionReached`**

在 `src/segment_completion.c` 第 27 行：

```c
// 原代码：
    direction = HYD_Segment_ResolveDirection(segment, axisRef);

// 替换为：
    // 注意：该函数签名目前没有 lastActiveDirection 参数，
    // 需要从调用链传入。先改为传入 HYD_DIRECTION_POSITIVE 作为临时占位。
    // 后续 Task 修复调用链。
    direction = HYD_Segment_ResolveDirection(segment, axisRef,
                                              HYD_DIRECTION_POSITIVE);
```

同时更新第 28-32 行的 `HYD_DIRECTION_EXTEND` / `HYD_DIRECTION_RETRACT`：

```c
// 原代码：
    if (direction == HYD_DIRECTION_EXTEND) {
        return axisRef->position >= segment->targetPosition - positionTolerance;
    }
    if (direction == HYD_DIRECTION_RETRACT) {
        return axisRef->position <= segment->targetPosition + positionTolerance;
    }

// 替换为（符号名更新，语义不变）：
    if (direction == HYD_DIRECTION_POSITIVE) {
        return axisRef->position >= segment->targetPosition - positionTolerance;
    }
    if (direction == HYD_DIRECTION_NEGATIVE) {
        return axisRef->position <= segment->targetPosition + positionTolerance;
    }
```

- [ ] **Step 4: 验证编译通过 + 运行测试**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: 编译成功，全量测试通过。

- [ ] **Step 5: Commit**

```bash
git add src/motion_control.c src/segment_completion.c
git commit -m "fix: update all ResolveDirection call sites for new signature"
```

---

### Task 6: 重写 `mapPlcOpenDirection` PLC 方向映射

**Files:**
- Modify: `src/motion_interface.c:67-76`

- [ ] **Step 1: 替换 `mapPlcOpenDirection` 函数**

在 `src/motion_interface.c` 第 67-76 行：

```c
// 原代码：
static HYD_MotionDirection mapPlcOpenDirection(IEC_SINT direction)
{
    if (direction > 0) {
        return HYD_DIRECTION_EXTEND;
    } else if (direction < 0) {
        return HYD_DIRECTION_RETRACT;
    }
    return HYD_DIRECTION_AUTO;
}

// 替换为：
/* 新PLC方向映射 — Beckhoff TF5810 MC_Direction 兼容
 * DIRECTION SINT 值: 0=Shortest_Way, 1=Positive, 2=Negative, 3=Current
 * 超出范围的值默认视为 Shortest_Way */
static HYD_MotionDirection mapPlcOpenDirection(IEC_SINT direction) {
    switch ((int)direction) {
        case 1:  return HYD_DIRECTION_POSITIVE;
        case 2:  return HYD_DIRECTION_NEGATIVE;
        case 3:  return HYD_DIRECTION_CURRENT;
        default: return HYD_DIRECTION_SHORTEST_WAY;
    }
}
```

- [ ] **Step 2: 验证编译通过**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
```

Expected: 编译成功。

- [ ] **Step 3: Commit**

```bash
git add src/motion_interface.c
git commit -m "feat: rewrite mapPlcOpenDirection for Beckhoff TF5810 4-value enum"
```

---

### Task 7: `HYD_MOVEABSOLUTE` 方向校验 + Velocity 符号处理

**Files:**
- Modify: `src/motion_interface.c:1059-1099`

- [ ] **Step 1: 在 `__mcl_cmd_MoveAbsolute` 的 `execRising` 分支中新增方向逻辑**

在 `src/motion_interface.c` 中，定位 `execRising` 分支（约第 1059 行起）。将原代码段：

```c
        fb->USE_RECIPE = false;

        HYD_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        HYD_MotionSegment segment = buildPositionSegment(
            __GET_VAR(data__->POSITION),
            __GET_VAR(data__->VELOCITY),
            __GET_VAR(data__->ACCELERATION),
            __GET_VAR(data__->DECELERATION),
            dir,
            fb);
```

替换为：

```c
        fb->USE_RECIPE = false;

        HYD_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        HYD_REAL velocity = __GET_VAR(data__->VELOCITY);
        HYD_REAL targetPos = __GET_VAR(data__->POSITION);
        HYD_REAL currentPos = fb->AXIS_REF.position;

        /* Positive_Direction: 强制正向，校验目标位置匹配 */
        if (dir == HYD_DIRECTION_POSITIVE) {
            if (targetPos < currentPos - fb->_params.positionTolerance) {
                __SET_VAR(data__->, ERROR,, true);
                __SET_VAR(data__->, ERRORID,,
                    (IEC_WORD)HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT);
                __SET_VAR(data__->, EXECUTE0,, execute);
                return;
            }
            velocity = (IEC_REAL)fabs((double)velocity);
        }
        /* Negative_Direction: 强制负向，校验目标位置匹配 */
        else if (dir == HYD_DIRECTION_NEGATIVE) {
            if (targetPos > currentPos + fb->_params.positionTolerance) {
                __SET_VAR(data__->, ERROR,, true);
                __SET_VAR(data__->, ERRORID,,
                    (IEC_WORD)HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT);
                __SET_VAR(data__->, EXECUTE0,, execute);
                return;
            }
            velocity = (IEC_REAL)fabs((double)velocity);
        }
        /* Shortest_Way 和 Current_Direction: 方向由运行时解析，Velocity 恒取正值 */
        else {
            velocity = (IEC_REAL)fabs((double)velocity);
        }

        HYD_MotionSegment segment = buildPositionSegment(
            targetPos,
            velocity,
            __GET_VAR(data__->ACCELERATION),
            __GET_VAR(data__->DECELERATION),
            dir,
            fb);
```

- [ ] **Step 2: 验证编译通过 + 运行测试**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: 编译成功，全量测试通过。

- [ ] **Step 3: Commit**

```bash
git add src/motion_interface.c
git commit -m "feat: add MoveAbsolute direction-position validation and velocity sign override"
```

---

### Task 8: `HYD_MOVEVELOCITY` Direction 优先级逻辑

**Files:**
- Modify: `src/motion_interface.c:1216-1239`

- [ ] **Step 1: 在 `__mcl_cmd_MoveVelocity` 的 `execRising` 分支中新增方向逻辑**

在 `src/motion_interface.c` 中，定位 `execRising` 分支（约第 1216 行起）。将原代码段：

```c
        fb->USE_RECIPE = false;

        HYD_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        HYD_MotionSegment segment = buildVelocitySegment(
            targetVelocity,
            __GET_VAR(data__->ACCELERATION),
            __GET_VAR(data__->DECELERATION),
            dir,
            fb);
```

替换为：

```c
        fb->USE_RECIPE = false;

        HYD_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        HYD_REAL velocity = targetVelocity;

        /* Shortest_Way: 根据 Velocity 正负区分方向 */
        if (dir == HYD_DIRECTION_SHORTEST_WAY) {
            if (velocity > 0.0f) {
                dir = HYD_DIRECTION_POSITIVE;
            } else if (velocity < 0.0f) {
                dir = HYD_DIRECTION_NEGATIVE;
            } else {
                /* Velocity == 0: 利用 lastActiveDirection 或默认正向 */
                dir = (fb->_lastActiveDirection == HYD_DIRECTION_NEGATIVE)
                      ? HYD_DIRECTION_NEGATIVE : HYD_DIRECTION_POSITIVE;
            }
        }

        /* 所有模式下，Velocity 取绝对值（Direction 优先级高于符号） */
        velocity = (IEC_REAL)fabs((double)velocity);

        HYD_MotionSegment segment = buildVelocitySegment(
            velocity,
            __GET_VAR(data__->ACCELERATION),
            __GET_VAR(data__->DECELERATION),
            dir,
            fb);
```

- [ ] **Step 2: 验证编译通过 + 运行测试**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: 编译成功，全量测试通过。

- [ ] **Step 3: Commit**

```bash
git add src/motion_interface.c
git commit -m "feat: add MoveVelocity Direction priority over Velocity sign"
```

---

### Task 9: `HYD_BeginSegment` 中维护 `_lastActiveDirection`

**Files:**
- Modify: `src/motion_control.c:830-838`

- [ ] **Step 1: 在 `HYD_BeginSegment` 末尾添加方向记忆更新**

在 `src/motion_control.c` 中定位 `HYD_BeginSegment` 函数尾部（约第 830-838 行）。在 `HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_STARTING)` 之后、`fb->_executionId++` 之前插入：

```c
    /* 更新轴方向记忆（用于 CURRENT 方向解析） */
    if (fb->_activeSegment.direction != HYD_DIRECTION_HOLD) {
        HYD_MotionDirection resolved = HYD_Segment_ResolveDirection(
            &fb->_activeSegment, &fb->AXIS_REF, fb->_lastActiveDirection);
        if (resolved == HYD_DIRECTION_POSITIVE || resolved == HYD_DIRECTION_NEGATIVE) {
            fb->_lastActiveDirection = resolved;
        }
    }

    fb->_executionId++;
```

- [ ] **Step 2: 验证编译通过**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
```

Expected: 编译成功。

- [ ] **Step 3: Commit**

```bash
git add src/motion_control.c
git commit -m "feat: maintain _lastActiveDirection in HYD_BeginSegment"
```

---

### Task 10: 编写方向映射单元测试

**Files:**
- Modify: `tests/test_motion_interface_unit.c`

- [ ] **Step 1: 添加方向映射测试用例**

在 `tests/test_motion_interface_unit.c` 文件末尾（`main` 函数之前）添加以下测试函数：

```c
/* Test: mapPlcOpenDirection mapping (0→SHORTEST_WAY, 1→POSITIVE, 2→NEGATIVE, 3→CURRENT)
 * Tested indirectly through MOTION.DIRECTION → segment.direction in LoadProfile. */

static void test_direction_mapping_shortest_way(void) {
    HYD_CREATEMOTION cm;
    HYD_LOADPROFILE lp;
    HYD_MotionControlFB* fb;
    HYD_AXISMOTION motion;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&lp, 0, sizeof(lp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(lp.EN) = true;
    IEC_VAL(lp.EXECUTE) = true;
    lp.EXECUTE0.value = false;
    IEC_VAL(lp.AXISID) = IEC_VAL(cm.AXISID);

    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = 0;  /* HYD_Shortest_Way */
    motion.SETPOSITION = 100.0f;
    motion.SETVELOCITY = 10.0f;
    motion.ACCELERATION = 50.0f;
    __SET_VAR(lp., MOTION,, motion);

    __mcl_cmd_LoadProfile(&lp);

    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "FB must be allocated");
    ASSERT_TRUE(fb->RECIPE_SIZE == 1U, "Recipe must have 1 segment");
    ASSERT_TRUE(fb->RECIPE[0].direction == HYD_DIRECTION_SHORTEST_WAY,
                "DIRECTION=0 must map to SHORTEST_WAY");
    printf("  PASS: DIRECTION=0 → HYD_DIRECTION_SHORTEST_WAY (=%d)\n",
           (int)HYD_DIRECTION_SHORTEST_WAY);
}

static void test_direction_mapping_positive(void) {
    HYD_CREATEMOTION cm;
    HYD_LOADPROFILE lp;
    HYD_MotionControlFB* fb;
    HYD_AXISMOTION motion;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&lp, 0, sizeof(lp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(lp.EN) = true;
    IEC_VAL(lp.EXECUTE) = true;
    lp.EXECUTE0.value = false;
    IEC_VAL(lp.AXISID) = IEC_VAL(cm.AXISID);

    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = 1;  /* HYD_Positive_Direction */
    motion.SETPOSITION = 100.0f;
    motion.SETVELOCITY = 10.0f;
    motion.ACCELERATION = 50.0f;
    __SET_VAR(lp., MOTION,, motion);

    __mcl_cmd_LoadProfile(&lp);

    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "FB must be allocated");
    ASSERT_TRUE(fb->RECIPE_SIZE == 1U, "Recipe must have 1 segment");
    ASSERT_TRUE(fb->RECIPE[0].direction == HYD_DIRECTION_POSITIVE,
                "DIRECTION=1 must map to POSITIVE");
    printf("  PASS: DIRECTION=1 → HYD_DIRECTION_POSITIVE (=%d)\n",
           (int)HYD_DIRECTION_POSITIVE);
}

static void test_direction_mapping_negative(void) {
    HYD_CREATEMOTION cm;
    HYD_LOADPROFILE lp;
    HYD_MotionControlFB* fb;
    HYD_AXISMOTION motion;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&lp, 0, sizeof(lp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(lp.EN) = true;
    IEC_VAL(lp.EXECUTE) = true;
    lp.EXECUTE0.value = false;
    IEC_VAL(lp.AXISID) = IEC_VAL(cm.AXISID);

    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = 2;  /* HYD_Negative_Direction */
    motion.SETPOSITION = -100.0f;  /* Negative position for retract */
    motion.SETVELOCITY = 10.0f;
    motion.ACCELERATION = 50.0f;
    __SET_VAR(lp., MOTION,, motion);

    __mcl_cmd_LoadProfile(&lp);

    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "FB must be allocated");
    ASSERT_TRUE(fb->RECIPE_SIZE == 1U, "Recipe must have 1 segment");
    ASSERT_TRUE(fb->RECIPE[0].direction == HYD_DIRECTION_NEGATIVE,
                "DIRECTION=2 must map to NEGATIVE");
    printf("  PASS: DIRECTION=2 → HYD_DIRECTION_NEGATIVE (=%d)\n",
           (int)HYD_DIRECTION_NEGATIVE);
}

static void test_direction_mapping_current(void) {
    HYD_CREATEMOTION cm;
    HYD_LOADPROFILE lp;
    HYD_MotionControlFB* fb;
    HYD_AXISMOTION motion;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&lp, 0, sizeof(lp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(lp.EN) = true;
    IEC_VAL(lp.EXECUTE) = true;
    lp.EXECUTE0.value = false;
    IEC_VAL(lp.AXISID) = IEC_VAL(cm.AXISID);

    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = 3;  /* HYD_Current_Direction */
    motion.SETPOSITION = 100.0f;
    motion.SETVELOCITY = 10.0f;
    motion.ACCELERATION = 50.0f;
    __SET_VAR(lp., MOTION,, motion);

    __mcl_cmd_LoadProfile(&lp);

    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "FB must be allocated");
    ASSERT_TRUE(fb->RECIPE_SIZE == 1U, "Recipe must have 1 segment");
    ASSERT_TRUE(fb->RECIPE[0].direction == HYD_DIRECTION_CURRENT,
                "DIRECTION=3 must map to CURRENT");
    printf("  PASS: DIRECTION=3 → HYD_DIRECTION_CURRENT (=%d)\n",
           (int)HYD_DIRECTION_CURRENT);
}
```

- [ ] **Step 2: 在 test 文件的 `main` 函数中注册新测试**

定位 `tests/test_motion_interface_unit.c` 的 `main` 函数（通常在文件末尾），在 `RUN_TEST` 列表中追加：

```c
    RUN_TEST(test_direction_mapping_shortest_way);
    RUN_TEST(test_direction_mapping_positive);
    RUN_TEST(test_direction_mapping_negative);
    RUN_TEST(test_direction_mapping_current);
```

- [ ] **Step 3: 编译并运行测试**

```bash
cmake --build --preset unixgcc --target test_motion_interface_unit 2>&1 | tail -5
./out/build/unixgcc/test_motion_interface_unit
```

Expected: 4 个新方向映射测试全部 PASS。

- [ ] **Step 4: Commit**

```bash
git add tests/test_motion_interface_unit.c
git commit -m "test: add direction mapping unit tests (0→SW, 1→POS, 2→NEG, 3→CUR)"
```

---

### Task 11: 编写 `ResolveDirection` CURRENT 分支单元测试

**Files:**
- Modify: `tests/test_motion_planner.c`

- [ ] **Step 1: 添加 CURRENT 方向解析测试**

在 `tests/test_motion_planner.c` 文件末尾（`main` 函数之前）添加：

```c
/* Test: ResolveDirection with CURRENT inherits lastActiveDirection (POSITIVE) */
static void test_resolve_direction_current_inherits_positive(void) {
    HYD_MotionSegment segment;
    HYD_AxisRef axisRef;
    HYD_MotionDirection result;

    memset(&segment, 0, sizeof(segment));
    segment.direction = HYD_DIRECTION_CURRENT;
    segment.targetPosition = 0.0f;       /* 不相关：CURRENT 忽略位置 */
    segment.positionTolerance = 0.1f;

    axisRef.position = 50.0f;            /* 当前位置，不用于 CURRENT 解析 */

    result = HYD_Segment_ResolveDirection(&segment, &axisRef,
                                          HYD_DIRECTION_POSITIVE);
    ASSERT_TRUE(result == HYD_DIRECTION_POSITIVE,
                "CURRENT with lastActive=POSITIVE must return POSITIVE");
    printf("  PASS: CURRENT + lastActive=POSITIVE → POSITIVE (=%d)\n",
           (int)result);
}

/* Test: ResolveDirection with CURRENT inherits lastActiveDirection (NEGATIVE) */
static void test_resolve_direction_current_inherits_negative(void) {
    HYD_MotionSegment segment;
    HYD_AxisRef axisRef;
    HYD_MotionDirection result;

    memset(&segment, 0, sizeof(segment));
    segment.direction = HYD_DIRECTION_CURRENT;
    segment.positionTolerance = 0.1f;

    axisRef.position = 0.0f;

    result = HYD_Segment_ResolveDirection(&segment, &axisRef,
                                          HYD_DIRECTION_NEGATIVE);
    ASSERT_TRUE(result == HYD_DIRECTION_NEGATIVE,
                "CURRENT with lastActive=NEGATIVE must return NEGATIVE");
    printf("  PASS: CURRENT + lastActive=NEGATIVE → NEGATIVE (=%d)\n",
           (int)result);
}

/* Test: ResolveDirection with CURRENT defaults to POSITIVE when stationary */
static void test_resolve_direction_current_defaults_positive(void) {
    HYD_MotionSegment segment;
    HYD_AxisRef axisRef;
    HYD_MotionDirection result;

    memset(&segment, 0, sizeof(segment));
    segment.direction = HYD_DIRECTION_CURRENT;
    segment.positionTolerance = 0.1f;

    axisRef.position = 0.0f;

    /* HOLD 和 SHORTEST_WAY 不是运动方向，退化为 POSITIVE */
    result = HYD_Segment_ResolveDirection(&segment, &axisRef,
                                          HYD_DIRECTION_HOLD);
    ASSERT_TRUE(result == HYD_DIRECTION_POSITIVE,
                "CURRENT with lastActive=HOLD must default to POSITIVE");
    printf("  PASS: CURRENT + lastActive=HOLD → POSITIVE (=%d)\n",
           (int)result);

    result = HYD_Segment_ResolveDirection(&segment, &axisRef,
                                          HYD_DIRECTION_SHORTEST_WAY);
    ASSERT_TRUE(result == HYD_DIRECTION_POSITIVE,
                "CURRENT with lastActive=SHORTEST_WAY must default to POSITIVE");
    printf("  PASS: CURRENT + lastActive=SHORTEST_WAY → POSITIVE (=%d)\n",
           (int)result);
}

/* Test: ResolveDirection SHORTEST_WAY still works (backward compatibility) */
static void test_resolve_direction_shortest_way_extends(void) {
    HYD_MotionSegment segment;
    HYD_AxisRef axisRef;
    HYD_MotionDirection result;

    memset(&segment, 0, sizeof(segment));
    segment.direction = HYD_DIRECTION_SHORTEST_WAY;
    segment.targetPosition = 100.0f;
    segment.positionTolerance = 0.1f;

    axisRef.position = 0.0f;

    result = HYD_Segment_ResolveDirection(&segment, &axisRef,
                                          HYD_DIRECTION_POSITIVE);
    ASSERT_TRUE(result == HYD_DIRECTION_POSITIVE,
                "SHORTEST_WAY with targetPos > currentPos must return POSITIVE");
    printf("  PASS: SHORTEST_WAY + targetPos>curr → POSITIVE (=%d)\n",
           (int)result);
}

/* Test: ResolveDirection SHORTEST_WAY retracts */
static void test_resolve_direction_shortest_way_retracts(void) {
    HYD_MotionSegment segment;
    HYD_AxisRef axisRef;
    HYD_MotionDirection result;

    memset(&segment, 0, sizeof(segment));
    segment.direction = HYD_DIRECTION_SHORTEST_WAY;
    segment.targetPosition = -100.0f;
    segment.positionTolerance = 0.1f;

    axisRef.position = 0.0f;

    result = HYD_Segment_ResolveDirection(&segment, &axisRef,
                                          HYD_DIRECTION_POSITIVE);
    ASSERT_TRUE(result == HYD_DIRECTION_NEGATIVE,
                "SHORTEST_WAY with targetPos < currentPos must return NEGATIVE");
    printf("  PASS: SHORTEST_WAY + targetPos<curr → NEGATIVE (=%d)\n",
           (int)result);
}
```

- [ ] **Step 2: 在 `main` 函数中注册新测试**

```c
    RUN_TEST(test_resolve_direction_current_inherits_positive);
    RUN_TEST(test_resolve_direction_current_inherits_negative);
    RUN_TEST(test_resolve_direction_current_defaults_positive);
    RUN_TEST(test_resolve_direction_shortest_way_extends);
    RUN_TEST(test_resolve_direction_shortest_way_retracts);
```

- [ ] **Step 3: 编译并运行测试**

```bash
cmake --build --preset unixgcc --target test_motion_planner 2>&1 | tail -5
./out/build/unixgcc/test_motion_planner
```

Expected: 5 个新测试全部 PASS。

- [ ] **Step 4: Commit**

```bash
git add tests/test_motion_planner.c
git commit -m "test: add ResolveDirection CURRENT and SHORTEST_WAY unit tests"
```

---

### Task 12: 全量回归测试 + 集成场景验证

**Files:**
- No new files created

- [ ] **Step 1: 构建全部目标**

```bash
cmake --build --preset unixgcc 2>&1 | tail -10
```

Expected: 构建成功。

- [ ] **Step 2: 运行全量 CTest**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: All tests pass（所有 18+ 个注册测试通过）。

- [ ] **Step 3: 手动运行 `main` 集成示例（验证模拟场景）**

```bash
./out/build/unixgcc/main
```

Expected: 多段 recipe 模拟正常输出 pump-speed commands 和段切换日志，无崩溃。

- [ ] **Step 4: 验证 `test_direct_mode` 和 `test_scenario_matrix` 通过**

```bash
ctest --test-dir out/build/unixgcc -R 'test_direct_mode|test_scenario_matrix' --output-on-failure
```

Expected: 两个集成测试通过。

- [ ] **Step 5: Commit（如有额外修复）**

```bash
git add -u
git commit -m "chore: final regression pass after direction redesign"
```

---

### Task 13: 修复 spec 文档中的非存在函数引用

**Files:**
- Modify: `docs/superpowers/specs/2026-06-04-direction-parameter-redesign.md`

- [ ] **Step 1: 删除不存在的 `HYD_AdjustPumpGainByDirection` 条目**

在 spec 文档第 4.7 节"其他调用点"表格中，删除此行：

```
| `src/motion_control.c` | `HYD_AdjustPumpGainByDirection` | 传入 `lastActiveDirection` |
```

（该函数在源代码中不存在，仅 spec 编写时的笔误。）

- [ ] **Step 2: Commit**

```bash
git add docs/superpowers/specs/2026-06-04-direction-parameter-redesign.md
git commit -m "docs: fix spec - remove non-existent function reference"
```

---

## Plan Self-Review

**1. Spec coverage check:**
- ✅ 枚举重定义 → Task 1
- ✅ 向后兼容别名 → Task 1
- ✅ `_lastActiveDirection` 字段 → Task 2
- ✅ `ResolveDirection` CURRENT 分支 → Task 3
- ✅ `MotionPlannerInput` 扩展 → Task 4
- ✅ 所有 5 个 ResolveDirection 调用点更新 → Task 5
- ✅ `mapPlcOpenDirection` 重写 → Task 6
- ✅ MoveAbsolute 方向校验 + Velocity 处理 → Task 7
- ✅ MoveVelocity Direction 优先 + Velocity 处理 → Task 8
- ✅ `BeginSegment` 中维护 `_lastActiveDirection` → Task 9
- ✅ MoveProfile（无需额外修改）→ 由 Task 1 别名自动覆盖
- ✅ 报警条件 `HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT` → Task 7
- ✅ 方向映射测试 → Task 10
- ✅ CURRENT 解析测试 → Task 11
- ✅ 全量回归 → Task 12

**2. Placeholder scan:** No TODOs, TBDs, "implement later" patterns.

**3. Type consistency:**
- `HYD_MotionDirection _lastActiveDirection` declared in Task 2 (motion_control.h)
- Used in Task 4 (planner input), Task 5 (ResolveDirection calls), Task 9 (BeginSegment)
- `HYD_Segment_ResolveDirection(segment, axisRef, lastActiveDirection)` signature in Task 3, used consistently in Tasks 4, 5, 9
- `mapPlcOpenDirection(IEC_SINT) → HYD_MotionDirection` in Task 6, used in Tasks 7, 8
