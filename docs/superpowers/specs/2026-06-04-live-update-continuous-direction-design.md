# LiveUpdate CONTINUOUS_UPDATE 与 DIRECTION 在线更新设计规格

日期: 2026-06-04
状态: 待确认

---

## 1. 背景与动机

当前 `HYD_LiveUpdateRequest` 支持六种在线可更新参数（targetPosition、maxVelocity、acceleration、deceleration、targetPressure、pressureRampRate）。IEC 层已有 `CONTINUOUSUPDATE` 和 `DIRECTION` 引脚，但其逻辑完全在适配层处理，核心引擎无感知：

- `CONTINUOUSUPDATE`：IEC 适配层用它 gating 是否调用 `apply*LiveUpdate()`，核心引擎不知道当前处于连续更新流还是单次更新。
- `DIRECTION`：仅在 EXECUTE 上升沿被消费，运行中无法在线翻转。

### 注塑机真实需求

**方向在线翻转（DIRECTION）**：

| 工艺动作 | 运动模式 | 场景 |
|----------|----------|------|
| 模具调试 | MOVE_VELOCITY (SPEED_RAMP) | 操作员通过操纵杆手动推拉合模/开模轴，方向随摇杆实时变化 |
| 抽芯调试 | MOVE_VELOCITY (SPEED_RAMP) | 手动控制抽芯进退 |
| 射胶位置修正 | MOVE_ABSOLUTE (POSITION) | HMI 操作员修改目标位置后反向运动 |

**连续更新感知（CONTINUOUS_UPDATE）**：

- 核心引擎知道"正在被连续更新"后，可以抑制循环刷屏的诊断消息（每个扫描周期一次 Warning 会淹没日志）
- 为未来逐周期诊断优化留接口

---

## 2. 设计目标

1. `HYD_LiveUpdateFlags` 新增 `HYD_LIVE_UPDATE_CONTINUOUS_UPDATE` 和 `HYD_LIVE_UPDATE_DIRECTION`
2. `HYD_LiveUpdateRequest` 新增 `direction` 字段
3. `HYD_MotionControlFB_ApplyLiveUpdate` 中实现方向翻转逻辑：planner 重启、velocityToFlowGain 重算、位置-方向一致性校验
4. IEC 适配层三个 `apply*LiveUpdate` 函数构造完整请求（含 CONTINUOUS_UPDATE 和 DIRECTION）
5. `CONTINUOUS_UPDATE` 模式下静默抑制非授权诊断，避免日志刷屏

---

## 3. 数据结构变更

### 3.1 `HYD_LiveUpdateFlags`（`include/motion_control.h`）

```c
typedef enum {
    HYD_LIVE_UPDATE_TARGET_POSITION    = 1U << 0,
    HYD_LIVE_UPDATE_MAX_VELOCITY       = 1U << 1,
    HYD_LIVE_UPDATE_ACCELERATION       = 1U << 2,
    HYD_LIVE_UPDATE_DECELERATION       = 1U << 3,
    HYD_LIVE_UPDATE_TARGET_PRESSURE    = 1U << 4,
    HYD_LIVE_UPDATE_PRESSURE_RAMP_RATE = 1U << 5,
    HYD_LIVE_UPDATE_CONTINUOUS_UPDATE  = 1U << 6,  /* 连续更新模式标记 */
    HYD_LIVE_UPDATE_DIRECTION          = 1U << 7,  /* 方向在线更新 */
} HYD_LiveUpdateFlags;
```

### 3.2 `HYD_LiveUpdateRequest`（`include/motion_control.h`）

```c
typedef struct {
    HYD_UINT16 flags;
    HYD_DirectCommandKind ownerKind;
    uint16_t ownerExecutionId;
    HYD_REAL targetPosition;
    HYD_REAL maxVelocity;
    HYD_REAL maxAcceleration;
    HYD_REAL maxDeceleration;
    HYD_REAL targetPressure;
    HYD_REAL pressureRampRate;
    HYD_MotionDirection direction;      /* NEW: 在线方向更新 */
} HYD_LiveUpdateRequest;
```

**向后兼容性**：`memset(&request, 0, sizeof(request))` 确保 `direction = 0 = HYD_DIRECTION_SHORTEST_WAY`，`flags` 归零表示不设置任何位。现有调用方（`motion_interface.c` 中三个 `apply*LiveUpdate`）均使用 `memset` 初始化，无需修改即可兼容。

---

## 4. 核心处理逻辑

### 4.1 `HYD_ApplyLiveUpdateOverrides` — 参数映射

位置：`src/motion_control.c`，`HYD_MotionControlFB_ApplyLiveUpdate` 之前。

在现有六个标志处理后、`return true` 之前追加 DIRECTION 处理：

```
规则：
- SPEED_RAMP 模式：直接更新 direction
- POSITION 模式：直接更新 direction（一致性校验在 ApplyLiveUpdate 层做）
- PRESSURE_CLOSED_LOOP 模式：拒绝，return false
```

CONTINUOUS_UPDATE 是元标记，不在映射函数中处理。

### 4.2 `HYD_MotionControlFB_ApplyLiveUpdate` — 决策执行

#### Case 1：段正在运行（`isSegmentActive`）

当前行为：覆盖 `_activeSegment` 参数，同步 `DIRECT_SEGMENT`，刷新 blend context。

**新增逻辑**（检测到 `HYD_LIVE_UPDATE_DIRECTION` 位时）：

1. **方向变更检测**：比较 `request->direction` 与 `fb->_activeSegment.direction`
2. **POSITION 模式一致性校验**（方向变更时）：
   - `HYD_DIRECTION_POSITIVE`：要求 `targetPosition >= currentPosition - positionTolerance`，不满足则返回 false
   - `HYD_DIRECTION_NEGATIVE`：要求 `targetPosition <= currentPosition + positionTolerance`，不满足则返回 false
   - 其他方向值（SHORTEST_WAY、CURRENT）由运行时方向解析处理
3. **velocityToFlowGain 重算**（方向变更且 cylinder 配置有效时）：根据新方向选择 EXTEND 或 RETRACT 面积
4. **Planner 状态重置**：方向变更时调用 `HYD_SafetyStateManager_ResetRuntimeActuation` 重置 planner state，然后 `HYD_PrimeSegmentControllers` 重新初始化
5. **无需重启**：不改变 FB_STATE，不清除 `SEGMENT_COMPLETED`，保持当前段的生命周期

伪代码：

```
如果方向变更 (request->direction != fb->_activeSegment.direction):
    如果是 POSITION 模式:
        执行位置-方向一致性校验
    重算 velocityToFlowGain (如果 cylinder 配置有效)
    重置 planner state
    重新 prime controllers
更新 _activeSegment 参数（含 direction）
同步 DIRECT_SEGMENT
刷新 blend context (如果存在)
```

#### Case 2：段已完成（`isSegmentCompleted`）

当前行为：从 `DIRECT_SEGMENT` 重建段，重新启动执行。

DIRECTION 变更自然透过 `HYD_ApplyLiveUpdateOverrides` 传递到 `updated` 段，已有流程覆盖所有需要（重建 → 校验 → prime → 启动）。无需额外处理。

#### Case 3：未授权

**变更**：当 `CONTINUOUS_UPDATE` 位为真时，静默返回 false，不报告诊断。

```
理由：连续更新流中，每个扫描周期都触发诊断报告会淹没 HMI 日志。PLC 工艺层已通过 FB 级 ERROR 输出处理错误。
```

### 4.3 方向变更对 planner 的影响矩阵

| 模式 | 方向变更行为 | 附加操作 |
|------|-------------|----------|
| SPEED_RAMP | 速度符号翻转，重 prime planner | 重算 velocityToFlowGain |
| POSITION | 重 prime planner（刹车曲线基于新方向） | 重算 velocityToFlowGain + 位置-方向一致性校验 |
| PRESSURE_CLOSED_LOOP | 拒绝更新 | 返回 false |

---

## 5. IEC 适配层变更

### 5.1 `validateUnsupportedMotionOptions` 简化

位置：`src/motion_interface.c`

**变更前**：
```c
static HYD_BOOL validateUnsupportedMotionOptions(IEC_BOOL continuousUpdate,
                                                 IEC_REAL jerk,
                                                 IEC_WORD* errorId)
{
    (void)continuousUpdate;
    ...
}
```

**变更后**：移除 `continuousUpdate` 参数（CONTINUOUS_UPDATE 现已完全支持）：

```c
static HYD_BOOL validateUnsupportedMotionOptions(IEC_REAL jerk,
                                                 IEC_WORD* errorId)
{
    if (fabs((double)jerk) <= 1e-6) {
        return true;
    }
    ...
}
```

调用处对应更新（MoveAbsolute、MoveVelocity）。

### 5.2 `applyMoveAbsoluteLiveUpdate`

新增 CONTINUOUS_UPDATE 和 DIRECTION 到 flags：

```c
request.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                HYD_LIVE_UPDATE_MAX_VELOCITY |
                HYD_LIVE_UPDATE_ACCELERATION |
                HYD_LIVE_UPDATE_DECELERATION |
                HYD_LIVE_UPDATE_CONTINUOUS_UPDATE |
                HYD_LIVE_UPDATE_DIRECTION;
request.direction = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
```

### 5.3 `applyMoveVelocityLiveUpdate`

新增 CONTINUOUS_UPDATE 和 DIRECTION 到 flags：

```c
request.flags = HYD_LIVE_UPDATE_MAX_VELOCITY |
                HYD_LIVE_UPDATE_ACCELERATION |
                HYD_LIVE_UPDATE_DECELERATION |
                HYD_LIVE_UPDATE_CONTINUOUS_UPDATE |
                HYD_LIVE_UPDATE_DIRECTION;
request.direction = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
```

**方向语义**：MOVE_VELOCITY 中 `mapPlcOpenDirection` 已处理 SHORTEST_WAY（从 Velocity 正负推断），在线更新时用相同的映射保持一致性。

### 5.4 `applyPressureHandleLiveUpdate`

仅新增 CONTINUOUS_UPDATE（PRESSURE_CLOSED_LOOP 无方向概念）：

```c
request.flags = HYD_LIVE_UPDATE_TARGET_PRESSURE |
                HYD_LIVE_UPDATE_PRESSURE_RAMP_RATE |
                HYD_LIVE_UPDATE_CONTINUOUS_UPDATE;
```

---

## 6. 诊断与安全边界

### 6.1 位置-方向一致性校验

| 请求方向 | 条件 | 拒绝条件 |
|----------|------|----------|
| POSITIVE | 目标必须在当前位置的前方 | `targetPosition < currentPosition - positionTolerance` |
| NEGATIVE | 目标必须在当前位置的后方 | `targetPosition > currentPosition + positionTolerance` |
| SHORTEST_WAY | 运行时解析，无限制 | — |
| CURRENT | 保持当前方向，无限制 | — |
| HOLD | 保压专用，POSITION 模式无意义 | — |

此校验仅在方向变更时执行（`request->direction != _activeSegment.direction`），不增加常态开销。

### 6.2 CONTINUOUS_UPDATE 诊断抑制

| 场景 | 行为 |
|------|------|
| Case 3 + CONTINUOUS_UPDATE | 静默 `return false`，不写诊断 |
| Case 3 + 非 CONTINUOUS_UPDATE | 保持现有行为：报告 `COMMAND_NOT_ALLOWED` Warning |

### 6.3 安全红线（不做）

- **不做运动中紧急反向**：方向翻转需要 planner 重置，不是真正的"无缝翻转"
- **不做 SHORTEST_WAY 在线重解析**：SHORTEST_WAY/CURRENT 的运行时方向解析逻辑不变，方向变更使用 IEC 层传入的明确枚举值
- **不做 PRESSURE_CLOSED_LOOP 的方向更新**：保压段无运动方向概念，拒绝

---

## 7. 测试策略

### 7.1 单元测试（`tests/test_motion_interface_unit.c`）

| 测试用例 | 覆盖点 |
|----------|--------|
| `test_live_update_continuous_update_flag_set` | CONTINUOUS_UPDATE 位正确传递到 Request |
| `test_live_update_direction_flag_set` | DIRECTION 位正确传递到 Request |
| `test_movevelocity_live_update_direction_flip` | SPEED_RAMP 模式方向翻转：velocityToFlowGain 重算，planner 状态重置 |
| `test_moveabsolute_live_update_direction_flip_forward` | POSITION 模式正向翻转，一致性校验通过，重启运动 |
| `test_moveabsolute_live_update_direction_flip_rejected` | POSITION 模式 dir=POSITIVE 但 target 在后方，拒绝更新 |
| `test_moveabsolute_live_update_direction_nochange` | 方向不变时不触发重启（优化路径） |
| `test_pressurehandle_live_update_direction_rejected` | PRESSURE_CLOSED_LOOP 请求 DIRECTION 更新被拒绝 |
| `test_live_update_continuous_suppress_diagnostic` | Case 3 + CONTINUOUS_UPDATE 静默返回，无诊断写入 |

### 7.2 集成测试（`tests/test_motion_interface_done_signals.c`）

| 测试用例 | 覆盖点 |
|----------|--------|
| `test_moveabsolute_continuousupdate_direction_cycle` | MOVE_ABSOLUTE 完整方向翻转周期：起步→到达→翻转→再到达 |
| `test_movevelocity_continuousupdate_joystick_simulation` | 模拟操纵杆：速度+方向随周期变化 |

### 7.3 回归验证

```bash
cmake --build --preset unixgcc
ctest --test-dir out/build/unixgcc --output-on-failure
```

所有现存测试必须通过，不引入 regression。

---

## 8. 涉及文件清单

| 文件 | 变更类型 | 说明 |
|------|----------|------|
| `include/motion_control.h` | 修改 | LiveUpdateFlags 枚举新增两位；LiveUpdateRequest 新增 direction 字段 |
| `src/motion_control.c` | 修改 | `HYD_ApplyLiveUpdateOverrides` 新增 DIRECTION 映射；`HYD_MotionControlFB_ApplyLiveUpdate` Case 1/3 逻辑扩展 |
| `src/motion_interface.c` | 修改 | `validateUnsupportedMotionOptions` 移除 continuousUpdate 参数；三个 `apply*LiveUpdate` 新增 flags 和 direction |
| `tests/test_motion_interface_unit.c` | 修改 | 新增 8 个单元测试 |
| `tests/test_motion_interface_done_signals.c` | 修改 | 新增 2 个集成测试 |

---

## 9. 向后兼容性

| 维度 | 评估 |
|------|------|
| ABI | `HYD_LiveUpdateRequest` 尾部新增一个 enum 字段（`int` 大小），结构体增大。所有 `memset` 初始化的调用方自动兼容 |
| API | `HYD_LiveUpdateFlags` 新增枚举值，不改变已有值，位掩码逻辑兼容 |
| 行为 | 不传 DIRECTION 位的请求行为不变；不传 CONTINUOUS_UPDATE 位的请求行为不变 |
| IEC 接口 | `HYD_MOVEABSOLUTE`、`HYD_MOVEVELOCITY`、`HYD_PRESSUREHANDLE` 的 DIN/DOUT 引脚不变，仅内部处理逻辑增强 |

---

## 10. 自检清单

- [ ] 所有新增枚举位使用 `1U << N` 格式（位掩码正确）
- [ ] `memset` 归零的 `direction` 等于 `HYD_DIRECTION_SHORTEST_WAY(0)`，不误触发方向变更
- [ ] POSITION 模式方向变更时位置-方向一致性校验在所有合法路径通过
- [ ] SPEED_RAMP 模式方向变更时 velocityToFlowGain 选择正确的油缸面积
- [ ] PRESSURE_CLOSED_LOOP 拒绝 DIRECTION 更新
- [ ] Case 3 + CONTINUOUS_UPDATE 不产生诊断输出
- [ ] Case 3 + 非 CONTINUOUS_UPDATE 保持现有诊断行为
- [ ] `validateUnsupportedMotionOptions` 调用处全部更新
- [ ] 全量测试通过（`ctest --test-dir out/build/unixgcc`）
