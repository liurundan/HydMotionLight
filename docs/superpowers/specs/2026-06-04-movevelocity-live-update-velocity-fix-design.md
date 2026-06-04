# MoveVelocity 连续更新 VELOCITY 参数修复设计规格

日期: 2026-06-04
状态: 待实现

---

## 1. 背景与动机

### 1.1 问题描述

MC_MoveVelocity 速度控制功能块开启 CONTINUOUSUPDATE=1 后，目标速度设为 5 启动运动，运行时改变目标速度为 0 或 -5 时出现 `HYD_DIAG_CODE_COMMAND_NOT_ALLOWED` 参数不匹配报警。

### 1.2 根因

`__mcl_cmd_MoveVelocity` 中有两条 VELOCITY 处理路径，它们不一致：

| 路径 | 位置 | VELOCITY 处理 | 方向处理 |
|------|------|--------------|----------|
| **execRising（启动）** | `motion_interface.c:1266-1283` | `fabs(velocity)` 取绝对值 | SHORTEST_WAY → 从符号推导 |
| **CONTINUOUSUPDATE（持续更新）** | `motion_interface.c:440` | 直接传原始值 | `mapPlcOpenDirection()` → SHORTEST_WAY 不解推导 |

持续更新路径将原始 VELOCITY（可能为负或零）传入 `request.maxVelocity`，触发 validator 拒绝：

| VELOCITY | 失败校验行 | 条件 |
|----------|-----------|------|
| **-5** | `recipe_validator.c:183` | `maxVelocity < 0.0` |
| **0** | `recipe_validator.c:298-302` | `planner==TIME_BASED && maxVelocity <= 0.0` |

### 1.3 工艺适配性影响

注塑机工艺中以下场景依赖运行时速度调零和方向翻转：

| 工艺动作 | 场景 | 需要的连续更新行为 | 修复前 |
|----------|------|-------------------|--------|
| 模具调试 | 操纵杆手动推拉合模/开模 | VELOCITY 变号 = 方向翻转 | ❌ 报警 |
| 抽芯调试 | 手动进退控制 | 同上 | ❌ 报警 |
| 射胶→保压切换 | VELOCITY 衰减到 0 | maxVelocity=0，减速到零 | ❌ 报警 |

### 1.4 安全性前置结论

planner (`motion_planner.c:208-212`) 对 `maxVelocity <= 0.0` 已安全处理（直接返回 0.0 速度），不存在修改后引入非预期运动的风险。

---

## 2. 设计目标

1. `applyMoveVelocityLiveUpdate` 中 VELOCITY 处理与 execRising 路径一致：SHORTEST_WAY 从符号推导方向，`maxVelocity = fabs(velocity)`
2. validator 接受 TIME_BASED 模式下 `maxVelocity == 0.0`（减速到零场景）
3. 不改变现有 execRising 路径和 POSITION/PRESSURE 模式的任何行为

---

## 3. 修改方案

### 3.1 修改点 1：`src/motion_interface.c` — `applyMoveVelocityLiveUpdate`

**文件**：`src/motion_interface.c:422-445`

**变更**：在构造 `HYD_LiveUpdateRequest` 前增加 VELOCITY 符号→方向推导 + 绝对值归一化，匹配 execRising 路径逻辑。

```c
static HYD_BOOL applyMoveVelocityLiveUpdate(HYD_MotionControlFB* fb,
                                            IEC_WORD execId,
                                            HYD_MOVEVELOCITY* data__)
{
    HYD_LiveUpdateRequest request;

    if (fb == NULL || data__ == NULL || !__GET_VAR(data__->CONTINUOUSUPDATE)) {
        return true;
    }

    HYD_REAL rawVelocity = __GET_VAR(data__->VELOCITY);
    HYD_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));

    /* SHORTEST_WAY: derive direction from Velocity sign — match execRising path */
    if (dir == HYD_DIRECTION_SHORTEST_WAY) {
        if (rawVelocity > 0.0f) {
            dir = HYD_DIRECTION_POSITIVE;
        } else if (rawVelocity < 0.0f) {
            dir = HYD_DIRECTION_NEGATIVE;
        } else {
            dir = (fb->_lastActiveDirection == HYD_DIRECTION_NEGATIVE)
                  ? HYD_DIRECTION_NEGATIVE : HYD_DIRECTION_POSITIVE;
        }
    }

    memset(&request, 0, sizeof(request));
    request.flags = HYD_LIVE_UPDATE_MAX_VELOCITY |
                    HYD_LIVE_UPDATE_ACCELERATION |
                    HYD_LIVE_UPDATE_DECELERATION |
                    HYD_LIVE_UPDATE_CONTINUOUS_UPDATE |
                    HYD_LIVE_UPDATE_DIRECTION;
    request.ownerKind = HYD_DIRECT_CMD_MOVE_VELOCITY;
    request.ownerExecutionId = (uint16_t)execId;
    request.maxVelocity = (IEC_REAL)fabs((double)rawVelocity);
    request.maxAcceleration = __GET_VAR(data__->ACCELERATION);
    request.maxDeceleration = __GET_VAR(data__->DECELERATION);
    request.direction = dir;
    return HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
}
```

### 3.2 修改点 2：`src/recipe_validator.c` — TIME_BASED maxVelocity 校验放松

**文件**：`src/recipe_validator.c:298-302`

**变更**：`maxVelocity <= 0.0` → `maxVelocity < 0.0`

```c
    if ((segment->planner == HYD_PLANNER_TIME_BASED) &&
        (segment->mode != HYD_MODE_PRESSURE_CLOSED_LOOP) &&
        (segment->maxVelocity < 0.0)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }
```

**安全性**：`maxVelocity < 0.0` 的拒绝已由同文件第 183 行全局覆盖，此修改仅将 `maxVelocity == 0.0` 从拒绝改为接受。planner 对零值安全处理（返回 0.0 速度）。

---

## 4. 端到端场景矩阵

| 场景 | VELOCITY 变化 | DIRECTION 引脚 | 预期 | 修改后结果 |
|------|-------------|---------------|------|-----------|
| 正向→反向 | 5 → -5 | SHORTEST_WAY | dir=NEGATIVE, maxVelocity=5.0 | ✅ |
| 正向→减速到零 | 5 → 0 | SHORTEST_WAY | dir=POSITIVE, maxVelocity=0.0, planner 输出 0 | ✅ |
| 正向降速 | 5 → 2 | SHORTEST_WAY | dir=POSITIVE, maxVelocity=2.0 | ✅ |
| 正向加速 | 5 → 10 | POSITIVE | dir=POSITIVE, maxVelocity=10.0 | ✅ |
| 反向→正向 | -3 → 3 | SHORTEST_WAY | dir=POSITIVE, maxVelocity=3.0 | ✅ |
| 速度不变 | 5 → 5 | SHORTEST_WAY | dir=POSITIVE, maxVelocity=5.0（无方向变更） | ✅ |

---

## 5. 回归影响分析

| 受影响的测试/模块 | 影响 | 风险 |
|-------------------|------|------|
| `test_recipe_validator` | maxVelocity=0.0 的 SPEED_RAMP 段通过校验（之前拒绝） | 低 — planner 安全处理 |
| `test_motion_interface_unit` | 已有 MoveVelocity 连续更新测试需确认无回归 | 低 — 只新增行为 |
| `test_motion_interface_done_signals` | 集成测试 DONE 信号 | 低 |
| `test_scenario_matrix` | 全场景回归 | 低 |

---

## 6. 涉及文件

| 文件 | 变更类型 | 说明 |
|------|----------|------|
| `src/motion_interface.c` | 修改 | `applyMoveVelocityLiveUpdate` 增加 VELOCITY 归一化 + 方向推导 |
| `src/recipe_validator.c` | 修改 | TIME_BASED maxVelocity 校验 `<=` → `<` |

---

## 7. 向后兼容性

| 维度 | 评估 |
|------|------|
| API | 无变化 |
| ABI | 无变化 |
| IEC 接口 | `HYD_MOVEVELOCITY` 引脚不变 |
| 行为 | SHORTEST_WAY + 正 VELOCITY 行为不变；新增支持零/负 VELOCITY |
| validator | 接受 maxVelocity=0 的 TIME_BASED 段，planner 安全返回 0 |

---

## 8. 自检清单

- [x] SHORTEST_WAY 方向推导逻辑与 execRising 路径一致
- [x] `maxVelocity = fabs(velocity)` 确保传入幅值
- [x] validator 修改不引入负值漏洞（全局 183 行已覆盖）
- [x] planner 对 maxVelocity=0 安全处理后继
- [x] POSITION 和 PRESSURE_CLOSED_LOOP 模式不受影响
- [x] 仅 2 个修改点，改动量最小
