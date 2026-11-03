# 位置/速度运动段压力限幅与工艺堵转处理设计

## 1. 目标

针对 `HYD_MOVECONTINUOUSABSOLUTE` 和 `HYD_MOVEVELOCITY`，当运动期间实际压力超过段参数 `PRESSURELIMIT` 时：

- 液压库继续执行运动控制周期，不把功能块直接置为 `ERROR` 或 `FAULT`。
- 输出限幅器降低实际泵流量，避免控制器继续按原始速度指令增流。
- 工艺层 PLC 根据“已限压但没有位置/速度进展”判断油缸堵转，并决定停止、撤退、重试或报警。
- 不新增绝对安全压力参数，不新增 IEC 功能块引脚，不改变命令仲裁和完成信号语义。

本方案只覆盖位置控制和速度控制运动段的工艺限压行为。压力闭环段、软限位、传感器故障和其他安全诊断保持现有职责。

## 2. 当前代码事实

1. `MoveVelocity` 将 `PRESSURELIMIT` 写入 `segment.maxPressure`，`MoveContinuousAbsolute` 的 approach/sustain 段复用同一限制。
   - `src/motion_interface.c:185-237`
   - `src/motion_control.c:1060-1081`
2. 运行时对位置/速度段将段压力和轴级 `PRESSURE_LIMIT` 取较小值，传给输出限幅器；压力闭环段继续由压力控制器管理。
   - `src/motion_control.c:3019-3037`
3. `HYD_OutputLimiter_ExecuteWithProtection()` 在压力超限时按比例缩减流量，并设置 `pressureLimitActive`。
   - `src/output_limiter.c:138-164`
   - `src/output_limiter.c:356-383`
4. 当前实现会在持续超压后返回 `HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT`，再由 `motion_control.c` 转为 `STOP`。
   - `src/output_limiter.c:225-264`
   - `src/motion_control.c:3096-3107`
5. `HYD_READSTATUS.PRESSURELIMITACTIVE` 已经可以向 PLC 暴露限压状态，无需新增公共接口。
   - `src/motion_interface.c:2926-2935`

## 3. 设计原则

### 3.1 段压力参数是工艺限幅，不是堵转诊断

段压力参数描述“本段允许的压力工作包络”。达到该值时，运动指令仍然有效，但实际泵流量可以低于请求值。是否属于堵转、模具保护、负载过大或正常工艺状态，由工艺层根据当前阶段判断。

### 3.2 保持运动会话，不保持原始正流量

库侧保持 `BUSY/ACTIVE`，但不承诺保持原始 `requestedFlow`。油缸堵转时，如果仍保持固定正流量，压力会继续上升；因此 `commandFlow` 必须由限幅器实时缩减，必要时降为零。这样才能避免库主动继续增压。

### 3.3 不用库内固定时间判断工艺堵转

不同工艺阶段允许的压力受限时间不同。库只提供 `PRESSURELIMITACTIVE`、实际运动输出和反馈，PLC 使用工艺参数决定堵转计时和后续动作。

### 3.4 不改变完成信号语义

压力限幅期间不能把 `POSITIONREACHED`、`INVELOCITY` 或 `INENDVELOCITY` 强制置为成功。位置控制和速度控制仍按真实位置/速度条件完成。

## 4. 运行时行为

```text
NORMAL
  pressure <= limit
  commandFlow = requestedFlow
  pressureLimitActive = FALSE

PRESSURE_LIMITING
  pressure > limit
  commandFlow = requestedFlow * pressureScale
  pressureLimitActive = TRUE
  BUSY/ACTIVE 保持
  ERROR/FAULT 不置位

RECOVERY
  pressure < limit
  pressureScale 按限速逐步恢复
  pressure <= limit - hysteresis 且输出恢复完成
  pressureLimitActive = FALSE
```

压力限幅只作用于位置控制和速度控制产生的运动流量。若压力已达到限制且继续给正流量会造成压力上升，限幅器应将正向输出继续压低，而不是依靠固定 `minScale` 长时间憋压。

## 5. 文件级改动方案

### 5.1 `src/motion_interface.c`

保持现有 `PRESSURELIMIT` 参数映射，不增加新输入：

- `MoveVelocity` 继续构造 `HYD_MODE_SPEED_RAMP` 段。
- `MoveContinuousAbsolute` 的 approach 和 sustain 段继续携带同一个压力限制。
- 保持 `PRESSURELIMIT <= 0` 使用轴级默认值的现有兼容行为。
- 补充注释：该参数触发的是“运行限幅状态”，不直接代表功能块故障。

本文件不负责判断堵转，也不直接修改 `ERROR`、`BUSY` 或 `ACTIVE` 的最终状态。

### 5.2 `src/output_limiter.c`

保留现有比例限流算法的基本结构，做三项收敛：

1. 超过段压力参数时继续计算 `pressureScale` 和 `pressureLimitActive`。
2. 对位置/速度运动段不再调用压力超限的 WARNING→FAULT 升级路径；`diagnosticCode` 保持 `HYD_DIAG_CODE_NONE`。
3. 使用 2% 释放回差：限压状态只在压力低于限制值的 98% 后恢复；同时以每秒 2.0 的 scale 速率逐步恢复流量。在回差带内保持约 0.1% 的轻微限幅，避免恢复满流量时再次冲过限制点。
4. 压力上升时立即按比例降低流量；压力仍高于限制时不恢复输出。低于限制后缓慢恢复，低于释放阈值且 scale 回到 1.0 后才清除 `PRESSURELIMITACTIVE`。

软限位的现有诊断升级逻辑不改，避免把压力需求扩展到其他保护功能。

### 5.3 `src/motion_control.c`

在 `HYD_MotionControlFB_RunRunningState()` 中：

- 继续使用 `limiterOutput.commandFlow` 作为最终执行流量。
- 继续设置 `HYD_LIMIT_FLAG_PRESSURE` 和 `HYD_LIMIT_FLAG_DERATE`。
- 删除“压力限幅故障直接升级为 `STOP`”的工艺路径。
- 保持位置、速度、超时、传感器和内部错误的既有诊断路径。

这样两个运动块通过同一条核心路径获得一致行为，避免在 `MoveVelocity` 和 `MoveContinuousAbsolute` 中重复实现压力逻辑。

## 6. 工艺 PLC 堵转判定

工艺层读取：

- `HYD_READSTATUS.PRESSURELIMITACTIVE`
- 轴实际位置和速度反馈
- 当前运动块的目标速度或运动请求状态

建议逻辑：

```text
motionDemand := MoveVelocity.BUSY AND ABS(targetVelocity) > velocityMin;
noProgress := ABS(actualVelocity) < velocityMin
              AND ABS(actualPosition - previousPosition) < positionMinDelta;

blockedTimer(IN := pressureLimitActive AND motionDemand AND noProgress,
             PT := processBlockedTime);

IF blockedTimer.Q THEN
    processAlarm := TRUE;
    // 按工艺选择 Stop、Abort、Retract 或重试
END_IF;
```

位置控制使用相同的 `motionDemand` 和 `noProgress` 条件；不能仅凭 `POSITIONREACHED` 判断成功。`processBlockedTime`、速度阈值和位置阈值由工艺段配置，不写入液压库固定常量。

## 7. 验收标准

### 7.1 输出限幅

- 实际压力未超过限制时，输出与现有逻辑一致。
- 实际压力超过限制时，`PRESSURELIMITACTIVE = TRUE`，输出流量下降。
- 持续超压时，位置/速度功能块保持 `BUSY/ACTIVE`，不出现 `ERROR`、`COMMANDABORTED` 或库级 `FAULT`。
- 压力低于“限制值减回差”后，限幅状态解除。
- 压力在限制值上下小幅抖动时，限压状态不反复切换，输出不瞬间恢复满流量。
- 压力降到释放回差以下后，输出按配置恢复速率单调恢复，完全恢复后才清除限压状态。

### 7.2 位置控制

- `MoveContinuousAbsolute` approach 和 sustain 两个阶段均使用相同限压行为。
- 限压期间未到达目标位置时，不产生 `POSITIONREACHED`。
- 到达目标位置但未达到终端速度时，仍保持现有 `POSITIONREACHED=TRUE`、`INENDVELOCITY=FALSE` 语义。

### 7.3 速度控制

- `MoveVelocity` 限压期间保持 `BUSY=TRUE`、`ACTIVE=TRUE`。
- 实际速度长期低于阈值时，由 PLC 堵转计时器报警。
- PLC 执行 Stop/Abort 后，仍沿用现有停止和命令所有权流程。

## 8. 测试调整

现有测试把“持续压力限制后必须 Fault”作为旧契约，需要改为：

- `tests/test_output_limiter.c`
  - 保留比例降流测试。
  - 将 Fault 升级测试改为“持续限压不产生诊断故障，但 `pressureLimitActive` 保持有效”。
- `tests/test_movevelocity_stop_reverse_restart.c`
  - 改为验证持续限压时 `ERROR=FALSE`、`BUSY=TRUE`、`ACTIVE=TRUE`。
- `tests/test_movecontinuousabsolute_integration.c`
  - 改为验证持续限压时命令不报错，并保持原有位置/终端速度信号语义。
- 增加一个工艺层模拟测试，验证“限压 + 无位置/速度进展”能触发堵转状态；该测试不应调用库级 Fault。

## 9. ADR

### Decision

将段压力限制定义为“非故障的运动输出限幅状态”，堵转判断交给工艺 PLC。

### Drivers

- 注塑工艺不同阶段对压力受限的容忍时间不同。
- `MoveVelocity` 和 `MoveContinuousAbsolute` 已经共享输出限幅路径。
- `PRESSURELIMITACTIVE` 已存在，无需扩展 IEC 接口。
- 需要保持代码小改动、行为清晰、便于快速上机验证。

### Alternatives considered

- 继续沿用持续超压后库内 Fault：会把工艺堵转和液压库安全故障混在一起。
- 在两个 IEC 功能块中分别实现堵转：会产生重复逻辑和不一致行为。
- 新增绝对安全压力字段：本方案范围明确排除，暂不引入。

### Consequences

- PLC 必须实现限压状态下的无进展计时和工艺动作。
- 库不再因段压力限幅自动停止运动，系统集成测试必须覆盖 PLC 堵转处理。
- `HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT` 保留兼容定义，但本方案的段压力限幅路径不再产生该故障。

### Follow-ups

- 在目标设备上标定压力回差、压力采样滤波和工艺堵转时间。
- 完成限压状态下的 Stop、Abort、Retract 流程验证。
