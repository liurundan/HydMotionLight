# HydMotionLight IEC 接口使用文档 V1.0

## 1. 概述

HydMotionLight 是面向伺服泵控注塑机液压系统的运动控制库，提供 PLCopen 风格的 IEC 61131-3 功能块接口。PLC 程序通过调用这些功能块，实现合模、注射、保压、顶出等液压控制流程。

### 1.1 适用场景

- 伺服泵驱动的注塑机液压系统
- 支持位置控制、速度/流量控制、压力闭环控制三种模式
- 支持 Recipe（配方多段）模式和 Direct（直接单段）模式

### 1.2 架构定位

IEC 接口层位于 PLC 运行时与核心控制引擎之间，负责：
- FB 实例池管理与轴资源分配
- 命令仲裁（Recipe/Direct 模式互斥）
- 所有权追踪（`_EXEC_ID` / `_PENDING` 两阶段模型）
- 信号映射（PLCopen 标准输出：ACTIVE / BUSY / DONE / ERROR / COMMANDABORTED）

> 接口定义参见 [motion_interface.h](include/motion_interface.h#L1-L381)，命令分发实现参见 [motion_interface.c](src/motion_interface.c#L1-L1035)

---

## 2. 公共数据结构

### 2.1 HYD_AXISMOTION — 轴运动数据通道

`HYD_AXISMOTION` 是 MoveProfile 功能块的双向数据通道，包含输入侧（设定参数）和输出侧（反馈数据）。

| 字段 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `SEGMENTTAG` | USINT | IN/OUT | 段标签，对应工艺类型枚举值 |
| `PLANNER` | USINT | IN/OUT | 规划器类型：0=位置规划，1=时间规划 |
| `MODE` | USINT | IN/OUT | 控制模式：0=位置，1=速度斜坡，2=压力闭环 |
| `ENDCONDITION` | USINT | IN/OUT | 结束条件：0=位置，1=时间，2=压力，3=流量，4=手动 |
| `DIRECTION` | USINT | IN/OUT | 方向：0=自动，1=伸出，2=缩回，3=保持 |
| `SETPOSITION` | REAL | IN | 目标位置 |
| `SETVELOCITY` | REAL | IN | 目标速度（速度/流量模式下表示目标速度） |
| `SETFLOW` | REAL | IN | 目标流量（速度/流量模式下表示目标流量） |
| `SETPRESSURE` | REAL | IN | 目标压力 |
| `ACCELERATION` | REAL | IN | 加速度 |
| `DECELERATION` | REAL | IN | 减速度 |
| `DURATION` | REAL | IN | 持续时间 |
| `PRESSURERAMPRATE` | REAL | IN | 压力斜坡速率 |
| `ACTPOSITION` | REAL | OUT | 实际位置反馈 → AXIS_REF |
| `ACTVELOCITY` | REAL | OUT | 实际速度反馈 → AXIS_REF |
| `ACTFLOW` | REAL | OUT | 实际流量反馈 → AXIS_REF |
| `ACTPRESSURE` | REAL | OUT | 实际压力反馈 → AXIS_REF |
| `TIMESTAMP` | REAL | OUT | 时间戳 |

> 枚举值定义参见 [common_types.h](include/common_types.h#L29-L69)

### 2.2 枚举常量速查

**控制模式 (MODE)**
| 值 | 含义 |
|----|------|
| 0 | 位置控制 `HYD_MODE_POSITION` |
| 1 | 速度斜坡 `HYD_MODE_SPEED_RAMP` |
| 2 | 压力闭环 `HYD_MODE_PRESSURE_CLOSED_LOOP` |

**结束条件 (ENDCONDITION)**
| 值 | 含义 |
|----|------|
| 0 | 到达目标位置 `HYD_END_POSITION` |
| 1 | 时间到期 `HYD_END_TIME` |
| 2 | 达到目标压力 `HYD_END_PRESSURE` |
| 3 | 达到目标流量 `HYD_END_FLOW` |
| 4 | 手动/持续运行 `HYD_END_MANUAL` |

**方向 (DIRECTION)**
| 值 | 含义 |
|----|------|
| 0 | 自动 `HYD_DIRECTION_AUTO` |
| 1 | 伸出（合模/注射）`HYD_DIRECTION_EXTEND` |
| 2 | 缩回（开模/退回）`HYD_DIRECTION_RETRACT` |
| 3 | 保持（保压）`HYD_DIRECTION_HOLD` |

**缓冲模式 (BUFFERMODE)**
| 值 | 含义 |
|----|------|
| 0 | 中止当前运动并立即执行 `HYD_BUFFER_MODE_ABORT` |
| 1 | 等轴空闲后执行 `HYD_BUFFER_MODE_BUFFER` |

**控制器状态 (STATE)**
| 值 | 含义 |
|----|------|
| 0 | 空闲 `HYD_STATUS_IDLE` |
| 1 | 就绪 `HYD_STATUS_READY` |
| 2 | 运行中 `HYD_STATUS_RUNNING` |
| 3 | 保持 `HYD_STATUS_HOLD` |
| 4 | 降级 `HYD_STATUS_DEGRADED` |
| 5 | 段完成 `HYD_STATUS_SEGMENT_COMPLETE` |
| 6 | 完成 `HYD_STATUS_FINISHED` |
| 7 | 故障 `HYD_STATUS_FAULT` |

---

## 3. 框架生命周期函数

PLC 主程序必须在每个扫描周期按以下顺序调用框架函数：

```
Init → [Retrieve → Publish] 循环 → Cleanup
```

| 函数 | 调用时机 | 说明 |
|------|----------|------|
| `__HydMotion_framework_Init()` | PLC 启动时调用一次 | 初始化 FB 实例池，清零所有轴实例 |
| `__HydMotion_framework_Retrieve()` | 每周期起始 | 反馈数据由各命令函数或工艺层直接更新 |
| `__HydMotion_framework_Publish()` | 每周期末尾 | 驱动所有 FB 实例的 Scan() 运算，输出 PUMP_SPEED；仿真模式下自动闭合反馈环 |
| `__HydMotion_framework_Cleanup()` | PLC 退出时调用 | 释放资源（当前为空操作） |

> 参见 [motion_interface.c](src/motion_interface.c#L222-L281)

---

## 4. 功能块接口详细说明

### 4.1 HYD_CREATEMOTION — 创建运动控制实例

创建并初始化一个运动控制 FB 实例，返回轴 ID 供后续 FB 使用。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能（IEC 标准） |
| `ENO` | BOOL | OUT | 使能输出 |
| `USE_RECIPE` | BOOL | IN | `TRUE`=配方模式，`FALSE`=直接模式 |
| `FLOW_TO_PUMPSPEED` | REAL | IN | 流量→泵速转换增益 |
| `PUMPSPEED_LIMIT` | REAL | IN | 泵速上限 |
| `USE_SIMULATION` | BOOL | IN | `TRUE`=启用内部仿真反馈 |
| `AXISID` | SINT | OUT | 分配的轴 ID（后续 FB 引用此 ID） |
| `BUSY` | BOOL | OUT | 创建中 |
| `DONE` | BOOL | OUT | 创建完成 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例**

```pascal
// 声明
fbCreateMotion : HYD_CREATEMOTION;
axisId : SINT;

// 代码
fbCreateMotion(EN := TRUE,
               USE_RECIPE := TRUE,
               FLOW_TO_PUMPSPEED := 0.2,
               PUMPSPEED_LIMIT := 3000.0,
               USE_SIMULATION := FALSE);

IF fbCreateMotion.DONE THEN
    axisId := fbCreateMotion.AXISID;  // 获取轴ID
END_IF
```

**注意事项**
- 每个 `CREATEMOTION` 分配一个 FB 实例，最大实例数由 `HYD_MAX_AXIS_MOTION` 配置
- `DONE=TRUE` 后才能使用返回的 `AXISID` 进行后续操作
- 仅调用一次，重复调用无效（`DONE` 已为 TRUE 时跳过）

---

### 4.2 HYD_SETAXISFEEDBACK — 设置轴反馈数据

将外部传感器反馈数据写入运动控制 FB 实例，每个周期必须调用。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `ENABLE` | BOOL | IN | 反馈使能，`TRUE`=写入反馈数据 |
| `AXISID` | SINT | IN | 轴 ID（由 CREATEMOTION 分配） |
| `ACT_POSITION` | REAL | IN | 实际位置 |
| `ACT_VELOCITY` | REAL | IN | 实际速度 |
| `ACT_FLOW` | REAL | IN | 实际流量 |
| `ACT_PRESSURE` | REAL | IN | 实际压力 |
| `TIMESTAMP` | REAL | IN | 时间戳 |
| `BUSY` | BOOL | OUT | 处理中 |
| `DONE` | BOOL | OUT | 写入完成 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例**

```pascal
fbSetFeedback(EN := TRUE,
              ENABLE := TRUE,
              AXISID := axisId,
              ACT_POSITION := encoder_pos,
              ACT_VELOCITY := encoder_vel,
              ACT_FLOW := flow_sensor,
              ACT_PRESSURE := pressure_transducer,
              TIMESTAMP := system_time);
```

**注意事项**
- 实际部署中每个 PLC 扫描周期必须调用，否则时间戳不更新将触发诊断告警
- 仿真模式下（`USE_SIMULATION=TRUE`），此 FB 可不调用，Publish 阶段自动闭合反馈环

---

### 4.3 HYD_GETPUMPREQUEST — 获取泵速输出

读取当前周期的泵速控制输出，写入伺服泵驱动器。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `ENABLE` | BOOL | IN | 读取使能 |
| `STRATEGY` | SINT | IN | 读取策略（预留） |
| `PUMPSPEED` | REAL | OUT | 泵速输出值（非负） |
| `BUSY` | BOOL | OUT | 处理中 |
| `DONE` | BOOL | OUT | 读取完成 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例**

```pascal
fbGetPump(EN := TRUE,
          ENABLE := TRUE,
          STRATEGY := 0);

IF fbGetPump.DONE THEN
    servo_pump_command := fbGetPump.PUMPSPEED;  // 写入伺服驱动器
END_IF
```

**注意事项**
- `PUMPSPEED` 为非负值，方向由阀控逻辑在工艺层处理
- 故障或空闲状态下输出 0.0（安全零输出）

---

### 4.4 HYD_MOVEPROFILE — 配方驱动的多段运动控制

PLCopen 风格的配方运动 FB，支持从 `MOTION` 字段自动构建单段配方，也支持预加载多段配方。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `EXECUTE` | BOOL | IN | 上升沿触发运动启动 |
| `MOTION` | HYD_AXISMOTION | IN/OUT | 双向运动数据通道 |
| `AXISID` | SINT | IN | 轴 ID |
| `BUFFERMODE` | INT | IN | 缓冲模式（0=ABORT, 1=BUFFER） |
| `ACTIVE` | BOOL | OUT | 运动激活中 |
| `BUSY` | BOOL | OUT | 功能块忙碌 |
| `DONE` | BOOL | OUT | 运动完成 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例 — 单段合模运动**

```pascal
fbMoveProfile(EN := TRUE,
              EXECUTE := startClamp,
              AXISID := axisId,
              BUFFERMODE := 0,  // ABORT模式
              MOTION := (SEGMENTTAG := 0,   // CLAMPING
                         PLANNER := 0,       // 位置规划
                         MODE := 0,          // 位置控制
                         ENDCONDITION := 0,  // 到位置结束
                         DIRECTION := 1,     // 伸出(合模)
                         SETPOSITION := 100.0,
                         SETVELOCITY := 50.0,
                         SETPRESSURE := 0.0,
                         ACCELERATION := 100.0,
                         DECELERATION := 100.0,
                         DURATION := 0.0,
                         PRESSURERAMPRATE := 0.0,
                         ACTPOSITION := encoder_pos,
                         ACTVELOCITY := encoder_vel,
                         ACTFLOW := flow_val,
                         ACTPRESSURE := press_val,
                         TIMESTAMP := sys_time));

IF fbMoveProfile.DONE THEN
    // 合模到位，切换下一工序
END_IF
```

**信号时序**

```
EXECUTE:  ___/‾‾‾‾‾\___
ACTIVE:   ______/‾‾‾‾‾\____
BUSY:     ______/‾‾‾‾‾‾‾‾‾\______
DONE:     ______________/‾‾‾\___
```

1. `EXECUTE` 上升沿 → 构建/加载配方 → 启动段执行
2. 核心引擎确认激活 → `ACTIVE=TRUE`
3. 段运行中 → `ACTIVE=TRUE, BUSY=TRUE`
4. 段结束条件满足 → `DONE=TRUE, BUSY=FALSE, ACTIVE=FALSE`

**注意事项**
- 若无预加载配方（`RECIPE_SIZE=0`），EXECUTE 上升沿自动从 `MOTION` 输入侧构建单段配方
- `MOTION` 的 `ACT*` 字段作为反馈输入写入 `AXIS_REF`
- `MOTION` 的 `SET*` 字段在执行中作为输出读回当前活动段参数

---

### 4.5 HYD_LOADPROFILE — 预加载配方

预加载多段配方到 FB 实例，不启动执行。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `EXECUTE` | BOOL | IN | 上升沿触发加载 |
| `MOTION` | HYD_AXISMOTION | IN | 配方数据 |
| `AXISID` | SINT | IN | 轴 ID |
| `BUSY` | BOOL | OUT | 加载中 |
| `DONE` | BOOL | OUT | 加载完成 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**注意事项**
- 当前版本（V1.0）此 FB 为预留接口，完整多段配方加载功能待后续版本实现
- 单段配方可直接通过 `MOVEPROFILE` 的 EXECUTE 自动构建

---

### 4.6 HYD_MOVEABSOLUTE — 绝对位置运动

Direct 模式下的绝对位置定位，自动构建位置控制段并启动执行。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `EXECUTE` | BOOL | IN | 上升沿触发定位 |
| `AXISID` | SINT | IN | 轴 ID |
| `CONTINUOUSUPDATE` | BOOL | IN | 运行中更新参数（预留） |
| `POSITION` | REAL | IN | 目标绝对位置 |
| `VELOCITY` | REAL | IN | 最大速度 |
| `ACCELERATION` | REAL | IN | 加速度 |
| `DECELERATION` | REAL | IN | 减速度 |
| `JERK` | REAL | IN | 加加速度（预留） |
| `DIRECTION` | SINT | IN | 方向：正=伸出，负=缩回，0=自动 |
| `BUFFERMODE` | INT | IN | 缓冲模式 |
| `DONE` | BOOL | OUT | 到达目标位置 |
| `BUSY` | BOOL | OUT | 运动中 |
| `ACTIVE` | BOOL | OUT | 运动激活 |
| `COMMANDABORTED` | BOOL | OUT | 命令被新命令取代 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例 — 注射前进到位**

```pascal
fbMoveAbs(EN := TRUE,
          EXECUTE := injectStart,
          AXISID := axisId,
          POSITION := 200.0,     // 目标位置 200mm
          VELOCITY := 80.0,      // 最大速度 80mm/s
          ACCELERATION := 200.0, // 加速度 200mm/s²
          DECELERATION := 200.0,
          JERK := 0.0,
          DIRECTION := 1,        // 伸出
          BUFFERMODE := 0,       // ABORT
          CONTINUOUSUPDATE := FALSE);

IF fbMoveAbs.DONE THEN
    // 到位，切换保压
END_IF
```

**信号时序**

```
EXECUTE:        ___/‾‾‾\___
ACTIVE:         ______/‾‾‾‾‾‾\____
BUSY:           ______/‾‾‾‾‾‾‾‾‾\______
DONE:           ______________/‾‾\___
COMMANDABORTED: ________________ (若被取代则置TRUE)
```

**注意事项**
- 内部自动构建位置控制段：`MODE=POSITION`, `ENDCONDITION=POSITION`
- `POSITION` 到达容差默认 0.1mm，超时保护默认 30s
- `DIRECTION` 映射：正值→`EXTEND`，负值→`RETRACT`，零→`AUTO`
- 被新命令取代时 `COMMANDABORTED=TRUE`

---

### 4.7 HYD_MOVEVELOCITY — 速度运动

Direct 模式下的持续速度运动，运行直到被 Stop 或其他命令取代。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `EXECUTE` | BOOL | IN | 上升沿触发 |
| `AXISID` | SINT | IN | 轴 ID |
| `CONTINUOUSUPDATE` | BOOL | IN | 运行中更新参数（预留） |
| `VELOCITY` | REAL | IN | 目标速度 |
| `ACCELERATION` | REAL | IN | 加速度 |
| `DECELERATION` | REAL | IN | 减速度 |
| `JERK` | REAL | IN | 加加速度（预留） |
| `DIRECTION` | SINT | IN | 方向 |
| `BUFFERMODE` | INT | IN | 缓冲模式 |
| `INVELOCITY` | BOOL | OUT | 已达到目标速度 |
| `BUSY` | BOOL | OUT | 运动中 |
| `ACTIVE` | BOOL | OUT | 运动激活 |
| `COMMANDABORTED` | BOOL | OUT | 命令被取代 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例 — 慢速合模**

```pascal
fbMoveVel(EN := TRUE,
          EXECUTE := slowCloseStart,
          AXISID := axisId,
          VELOCITY := 10.0,      // 10mm/s 慢速
          ACCELERATION := 50.0,
          DECELERATION := 50.0,
          JERK := 0.0,
          DIRECTION := 1,        // 伸出
          BUFFERMODE := 0,
          CONTINUOUSUPDATE := FALSE);

IF fbMoveVel.INVELOCITY THEN
    // 已达目标速度，持续运行
END_IF
```

**注意事项**
- 内部构建速度斜坡段：`MODE=SPEED_RAMP`, `ENDCONDITION=MANUAL`
- 无超时保护（`timeoutLimit=0`），持续运行直到被 Stop 取代
- 不输出 `DONE`，而输出 `INVELOCITY` 表示已达到目标速度

---

### 4.8 HYD_PRESSUREHANDLE — 压力闭环控制

Direct 模式下的压力闭环控制，支持保压和压力斜坡。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `EXECUTE` | BOOL | IN | 上升沿触发 |
| `AXISID` | SINT | IN | 轴 ID |
| `PRESSURE` | REAL | IN | 目标压力 |
| `PRESSURERAMPRATE` | REAL | IN | 压力斜坡速率 |
| `DURATION` | REAL | IN | 持续时间（0=持续到手动停止） |
| `BUFFERMODE` | INT | IN | 缓冲模式 |
| `INPRESSURE` | BOOL | OUT | 已达到目标压力 |
| `BUSY` | BOOL | OUT | 控制中 |
| `ACTIVE` | BOOL | OUT | 控制激活 |
| `COMMANDABORTED` | BOOL | OUT | 命令被取代 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例 — 保压阶段**

```pascal
fbPressHandle(EN := TRUE,
              EXECUTE := holdPressStart,
              AXISID := axisId,
              PRESSURE := 80.0,          // 目标压力 80MPa
              PRESSURERAMPRATE := 50.0,   // 斜坡速率 50MPa/s
              DURATION := 5.0,            // 保压 5 秒
              BUFFERMODE := 0);

IF fbPressHandle.INPRESSURE THEN
    // 压力已达目标值
END_IF
```

**调用示例 — 持续保压（无时间限制）**

```pascal
fbPressHandle(EN := TRUE,
              EXECUTE := holdPressStart,
              AXISID := axisId,
              PRESSURE := 60.0,
              PRESSURERAMPRATE := 30.0,
              DURATION := 0.0,   // 0 = 持续到手动停止
              BUFFERMODE := 0);
```

**注意事项**
- `DURATION > 0` → 时间结束条件，`DURATION = 0` → 手动结束条件
- 内部默认 PI 压力控制器（Kp=0.5, Ki=0.1）
- 压力容差默认 0.5MPa，超时保护 30s
- 保压流量默认 5.0 L/min，最大流量限制 20.0 L/min

---

### 4.9 HYD_STOP — 停止运动

中止当前运动，使轴减速停止。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `EXECUTE` | BOOL | IN | 上升沿触发停止 |
| `AXISID` | SINT | IN | 轴 ID |
| `DECELERATION` | REAL | IN | 减速度（预留） |
| `DONE` | BOOL | OUT | 停止完成 |
| `BUSY` | BOOL | OUT | 减速中 |
| `COMMANDABORTED` | BOOL | OUT | （此 FB 不使用） |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例**

```pascal
fbStop(EN := TRUE,
       EXECUTE := emergencyStop,
       AXISID := axisId,
       DECELERATION := 500.0);

IF fbStop.DONE THEN
    // 轴已安全停止
END_IF
```

**注意事项**
- 内部调用 `Abort()`，立即中止当前运动段
- `DONE=TRUE` 表示轴已完全停止

---

### 4.10 HYD_RESET — 故障复位

清除故障状态，将 FB 恢复到就绪/空闲状态。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `EXECUTE` | BOOL | IN | 上升沿触发复位 |
| `AXISID` | SINT | IN | 轴 ID |
| `DONE` | BOOL | OUT | 复位完成 |
| `BUSY` | BOOL | OUT | 复位中 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例**

```pascal
fbReset(EN := TRUE,
        EXECUTE := resetButton,
        AXISID := axisId);

IF fbReset.DONE THEN
    // 故障已清除，可重新启动
END_IF
```

**注意事项**
- 软复位：清除运行状态、活动段、故障/诊断状态，保留配方、增益配置和诊断判据设置
- 复位后 FB 返回 `READY`（有配方/直接段时）或 `IDLE`

---

### 4.11 HYD_READSTATUS — 读取控制器状态

读取指定轴的当前控制器状态。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `ENABLE` | BOOL | IN | 读取使能 |
| `AXISID` | SINT | IN | 轴 ID |
| `VALID` | BOOL | OUT | 数据有效 |
| `BUSY` | BOOL | OUT | 读取中 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |
| `STATE` | UINT | OUT | 控制器状态值（参见2.2节状态枚举） |

---

### 4.12 HYD_READERROR — 读取错误信息

读取指定轴的当前错误码。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `ENABLE` | BOOL | IN | 读取使能 |
| `AXISID` | SINT | IN | 轴 ID |
| `VALID` | BOOL | OUT | 数据有效 |
| `BUSY` | BOOL | OUT | 读取中 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 当前错误码 |

---

### 4.13 HYD_READSIMFEEDBACK — 读取仿真反馈

读取内部仿真器的物理反馈数据（仅仿真模式可用）。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `ENABLE` | BOOL | IN | 读取使能 |
| `AXISID` | SINT | IN | 轴 ID |
| `VALID` | BOOL | OUT | 数据有效 |
| `BUSY` | BOOL | OUT | 读取中 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |
| `POSITION` | REAL | OUT | 仿真位置 |
| `VELOCITY` | REAL | OUT | 仿真速度 |
| `FLOW` | REAL | OUT | 仿真流量 |
| `PRESSURE` | REAL | OUT | 仿真压力 |

---

## 5. 典型注塑周期调用流程

### 5.1 完整注塑周期（Direct 模式）

```pascal
// ═══════════════════════════════════════
// 阶段0: 初始化（PLC启动时执行一次）
// ═══════════════════════════════════════
__HydMotion_framework_Init();

fbCreateMotion(EN := TRUE, USE_RECIPE := FALSE,
               FLOW_TO_PUMPSPEED := 0.2, PUMPSPEED_LIMIT := 3000.0,
               USE_SIMULATION := FALSE);
// 获取 axisId

// ═══════════════════════════════════════
// 阶段1: 每周期必调用
// ═══════════════════════════════════════
__HydMotion_framework_Retrieve();

fbSetFeedback(EN := TRUE, ENABLE := TRUE, AXISID := axisId,
              ACT_POSITION := ..., ACT_VELOCITY := ...,
              ACT_FLOW := ..., ACT_PRESSURE := ..., TIMESTAMP := ...);

// ═══════════════════════════════════════
// 阶段2: 工艺控制（状态机）
// ═══════════════════════════════════════
CASE step OF
  0: // 快速合模
     fbMoveAbs(EXECUTE := TRUE, POSITION := 100.0, VELOCITY := 80.0, ...);
     IF fbMoveAbs.DONE THEN step := 1; END_IF

  1: // 慢速合模（低压保护）
     fbMoveVel(EXECUTE := TRUE, VELOCITY := 10.0, DIRECTION := 1, ...);
     // 检测到模具接触信号后:
     IF moldTouch THEN step := 2; END_IF

  2: // 高压锁模
     fbPressHandle(EXECUTE := TRUE, PRESSURE := 120.0,
                   PRESSURERAMPRATE := 100.0, DURATION := 0, ...);
     IF fbPressHandle.INPRESSURE THEN step := 3; END_IF

  3: // 注射
     fbMoveAbs(EXECUTE := TRUE, POSITION := 200.0, VELOCITY := 100.0,
               DIRECTION := 1, ...);
     IF fbMoveAbs.DONE THEN step := 4; END_IF

  4: // 保压
     fbPressHandle(EXECUTE := TRUE, PRESSURE := 80.0,
                   PRESSURERAMPRATE := 50.0, DURATION := 5.0, ...);
     IF fbPressHandle.DONE THEN step := 5; END_IF

  5: // 开模
     fbMoveAbs(EXECUTE := TRUE, POSITION := 0.0, VELOCITY := 60.0,
               DIRECTION := -1, ...);
     IF fbMoveAbs.DONE THEN step := 6; END_IF

  6: // 顶出
     fbMoveAbs(EXECUTE := TRUE, POSITION := 50.0, VELOCITY := 30.0,
               DIRECTION := 1, ...);
     IF fbMoveAbs.DONE THEN step := 0; END_IF  // 循环
END_CASE

// ═══════════════════════════════════════
// 阶段3: 输出泵速
// ═══════════════════════════════════════
fbGetPump(EN := TRUE, ENABLE := TRUE, STRATEGY := 0);
servo_output := fbGetPump.PUMPSPEED;

__HydMotion_framework_Publish();
```

### 5.2 Recipe 模式（配方驱动多段运动）

```pascal
// 创建时选择 Recipe 模式
fbCreateMotion(USE_RECIPE := TRUE, ...);

// 使用 MoveProfile 按段执行
fbMoveProfile(EXECUTE := segStart,
              AXISID := axisId,
              BUFFERMODE := 0,
              MOTION := (SEGMENTTAG := 0, MODE := 0, ...));
```

---

## 6. 命令仲裁与所有权机制

### 6.1 两阶段所有权模型

| 阶段 | 条件 | 说明 |
|------|------|------|
| PENDING | `_PENDING=TRUE` | EXECUTE 上升沿后等待核心引擎确认 |
| OWNED | `_EXEC_ID != 0` | 持有 executionId，与核心 FB 比对确认活跃性 |

### 6.2 仲裁规则

- **Recipe/Direct 互斥**：`MoveProfile`（Recipe模式）与 `MoveAbsolute`/`MoveVelocity`/`PressureHandle`（Direct模式）不能同时活跃
- **新命令取代旧命令**：`BUFFERMODE=ABORT(0)` 时新命令可接管正在执行的旧命令
- **被接管输出**：旧命令的 `COMMANDABORTED=TRUE, BUSY=FALSE`

### 6.3 状态转换矩阵

| 命令 | 允许的起始状态 |
|------|---------------|
| START | IDLE / READY / SEGMENT_COMPLETE / DONE / ABORTED |
| NEXT | SEGMENT_COMPLETE |
| HOLD | STARTING / RUNNING |
| RESUME | HOLD |
| ABORT | STARTING / RUNNING / SEGMENT_COMPLETE / HOLD |
| ACK | DISABLED / IDLE / READY / SEGMENT_COMPLETE / HOLD / DONE / ABORTED |
| RESET | 任意状态 |

---

## 7. 错误码参考

| 错误码 | 常量名 | 说明 |
|--------|--------|------|
| 0 | `HYD_DIAG_CODE_NONE` | 无错误 |
| 1 | `HYD_DIAG_CODE_RECIPE_EMPTY` | 配方为空 |
| 2 | `HYD_DIAG_CODE_RECIPE_TOO_LARGE` | 配方段数超限 |
| 3 | `HYD_DIAG_CODE_SEGMENT_INVALID` | 段参数无效 |
| 4 | `HYD_DIAG_CODE_RUNTIME_CONFIG_INVALID` | 运行时配置无效 |
| 5 | `HYD_DIAG_CODE_START_CONTEXT_INVALID` | 启动上下文非法（轴ID无效或状态不允许） |
| 6 | `HYD_DIAG_CODE_COMMAND_NOT_ALLOWED` | 命令在当前状态不允许 |
| 7 | `HYD_DIAG_CODE_NO_RECIPE` | 无配方可执行 |
| 8 | `HYD_DIAG_CODE_NO_DIRECT_SEGMENT` | 无直接段可执行 |
| 11 | `HYD_DIAG_CODE_SEGMENT_NOT_COMPLETED` | 段未完成 |
| 12 | `HYD_DIAG_CODE_RECIPE_ALREADY_FINISHED` | 配方已全部完成 |
| 13 | `HYD_DIAG_CODE_ABORTED` | 被中止 |
| 14 | `HYD_DIAG_CODE_TIMEOUT` | 运行超时 |
| 15 | `HYD_DIAG_CODE_OVER_PRESSURE` | 超压 |
| 16 | `HYD_DIAG_CODE_UNDER_PRESSURE` | 低压 |
| 17 | `HYD_DIAG_CODE_FLOW_DEVIATION` | 流量偏差 |
| 18 | `HYD_DIAG_CODE_POSITION_DEVIATION` | 位置偏差 |
| 19 | `HYD_DIAG_CODE_VELOCITY_DEVIATION` | 速度偏差 |
| 20 | `HYD_DIAG_CODE_SENSOR_FAULT` | 传感器故障 |
| 21 | `HYD_DIAG_CODE_TIMESTAMP_ROLLBACK` | 时间戳回滚 |
| 22 | `HYD_DIAG_CODE_INTERNAL_ERROR` | 内部错误 |

> 完整定义参见 [common_types.h](include/common_types.h#L117-L139)

---

## 8. 平台配置要点

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `HYD_MAX_SEGMENTS` | 16 | 最大配方段数 |
| `HYD_MAX_AXIS_MOTION` | — | 最大轴实例数（FB池大小） |
| `HYD_REAL_PRECISION_FLOAT` | 1 | 实数精度（1=float, 0=double） |
| `HYD_ENABLE_PRESSURE_LOOP_TELEMETRY` | 1 | 压力环遥测开关 |
| `HYD_ENABLE_PROTECTION` | 1 | 保护功能开关 |
| `dfCycleTime` | 0.001s | 默认扫描周期 |

> 参见 [hyd_config.h](include/hyd_config.h#L1-L320)

---

## 9. 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| V1.0 | 2026-06 | 初始版本，覆盖全部 13 个 IEC 功能块接口 |

---

如需进一步了解命令仲裁细节、诊断系统架构或嵌入式部署流程，建议继续阅读：

- [IEC61131-3 适配层](8-iec61131-3-adapter-layer) — 适配层整体架构与数据流
- [指令仲裁与 FB 池](9-command-arbitration-and-fb-pool) — 多实例仲裁与所有权追踪细节
- [配方与直接模式](14-recipe-vs-direct-mode) — 两种运动模式的完整对比
- [执行周期与数据流](22-execution-cycle-and-data-flow) — 框架 Retrieve/Publish 调用时机