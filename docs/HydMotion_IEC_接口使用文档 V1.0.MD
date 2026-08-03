# HydMotionLight IEC 接口使用文档 V1.1

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

> 接口定义参见 [motion_interface.h](include/motion_interface.h)，命令分发实现参见 [motion_interface.c](src/motion_interface.c)

---

## 2. 公共数据结构

### 2.1 HYD_AXISMOTION — 轴运动数据通道

`HYD_AXISMOTION` 是 MoveProfile 功能块的双向数据通道，包含输入侧（设定参数）和输出侧（反馈数据）。

| 字段 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `SEGMENTTAG` | USINT | IN | 段标签，对应工艺类型枚举值 |
| `SEGMENTTYPE` | USINT | IN | 段类型（新增，用于内部分类，通常与 SEGMENTTAG 一致） |
| `PLANNER` | USINT | IN | 规划器类型：0=位置规划，1=时间规划 |
| `MODE` | USINT | IN | 控制模式：0=位置，1=速度斜坡，2=压力闭环 |
| `ENDCONDITION` | USINT | IN | 结束条件：0=位置，1=时间，2=压力，3=流量，4=手动 |
| `DIRECTION` | USINT | IN | 方向枚举值，见 2.2 节方向说明 |
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

> **HYD_AXISMOTION 半区所有权契约**：Setpoint 半区（`SET*`、`DIRECTION`、`MODE` 等）由 PLC 写入，runtime 仅在 EXECUTE 上升沿读取一次，运行中不回写；Actual 半区（`ACT*`、`TIMESTAMP`）由 runtime/传感器写入，PLC 不回写。

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

**方向 (DIRECTION) — PLCopen / Beckhoff TF5810 兼容**
| 值 | 枚举名 | 别名 | 含义 |
|----|--------|------|------|
| 0 | `HYD_DIRECTION_SHORTEST_WAY` | `HYD_DIRECTION_AUTO` | 自动最短路径：由运行时根据目标位置与当前位置的差值推导方向 |
| 1 | `HYD_DIRECTION_POSITIVE` | `HYD_DIRECTION_EXTEND` | 强制正向（伸出/合模/注射）|
| 2 | `HYD_DIRECTION_NEGATIVE` | `HYD_DIRECTION_RETRACT` | 强制负向（缩回/开模/退回）|
| 3 | `HYD_DIRECTION_CURRENT` | — | 继承当前方向：复用上一次 ACTIVE 运动的方向；轴静止时默认正向 |
| 4 | `HYD_DIRECTION_HOLD` | — | 无运动（保压专用）：由 PressureHandle 内部自动设置，PLC 一般不需要手动填写 |

> **方向参数详细说明见第 2.3 节。**

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

### 2.3 方向参数详细说明

方向参数（`DIRECTION`）遵循 PLCopen MC 规范并与 Beckhoff TF5810 `MC_Direction` 保持兼容，共 5 个枚举值。以下说明各值的含义、推导规则和使用场景：

#### 0 — Shortest_Way（自动最短路径）

- **适用 FB**：`MoveAbsolute`、`MoveVelocity`（`HYD_AXISMOTION.DIRECTION`）
- **规则**：运行时根据目标位置与当前轴位置的差值自动决定正向还是负向
  - 目标 > 当前：选择正向（EXTEND）
  - 目标 < 当前：选择负向（RETRACT）
  - 目标 ≈ 当前（位置差 ≤ 容差）：选择正向（默认）
- **MoveVelocity 特例**：`Shortest_Way` 时，方向由 `VELOCITY` 参数的正负号推导
  - `VELOCITY > 0` → 正向；`VELOCITY < 0` → 负向；`VELOCITY = 0` → 沿用上次运动方向
  - 推导后 `VELOCITY` 自动取绝对值，段描述符以纯幅值存储
- **推荐场景**：目标位置明确时优先使用，无需手动指定方向

#### 1 — Positive_Direction（强制正向 / 伸出）

- **适用 FB**：`MoveAbsolute`、`MoveVelocity`、`MoveProfile`
- **规则**：强制以正向执行运动，`VELOCITY` 无论正负均取绝对值使用
- **MoveAbsolute 校验**：若目标位置 < 当前位置 − `positionTolerance`，则拒绝执行并输出 `ERRORID = HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT`
- **推荐场景**：合模、注射推进、顶针伸出等需明确伸出方向的工序

#### 2 — Negative_Direction（强制负向 / 缩回）

- **适用 FB**：`MoveAbsolute`、`MoveVelocity`、`MoveProfile`
- **规则**：强制以负向执行运动，`VELOCITY` 无论正负均取绝对值使用
- **MoveAbsolute 校验**：若目标位置 > 当前位置 + `positionTolerance`，则拒绝执行并输出 `ERRORID = HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT`
- **推荐场景**：开模、螺杆退回、顶针缩回等需明确缩回方向的工序

#### 3 — Current_Direction（继承当前方向）

- **适用 FB**：`MoveAbsolute`、`MoveVelocity`、`MoveProfile`
- **规则**：继承轴上一次 ACTIVE 运动的方向（记录在 `fb->_lastActiveDirection`）
  - 若轴当前静止（未曾运动）：默认为正向
- **推荐场景**：多段连续运动中需维持同一方向时使用，避免重复指定

#### 4 — Hold（无运动 / 保压专用）

- **适用 FB**：由 `PressureHandle` 内部自动设置，**PLC 层通常不需要直接使用**
- **规则**：无位置运动输出，仅输出压力闭环所需流量
- **泵仲裁**：`GetPumpRequest` 不将 `HOLD` 轴计入方向冲突检测

#### 方向参数与速度符号的关系

| FB | DIRECTION | VELOCITY 正负 | 实际执行方向 |
|----|-----------|--------------|------------|
| MoveAbsolute | 0（Auto） | 正或负均可 | 由目标位置相对当前位置推导 |
| MoveAbsolute | 1（Positive） | 自动取绝对值 | 强制正向；目标位置 < 当前则报错 |
| MoveAbsolute | 2（Negative） | 自动取绝对值 | 强制负向；目标位置 > 当前则报错 |
| MoveAbsolute | 3（Current） | 自动取绝对值 | 继承上次方向 |
| MoveVelocity | 0（Auto） | `+` → 正向 / `−` → 负向 | 由速度符号推导 |
| MoveVelocity | 1（Positive） | 自动取绝对值 | 强制正向 |
| MoveVelocity | 2（Negative） | 自动取绝对值 | 强制负向 |
| MoveVelocity | 3（Current） | 自动取绝对值 | 继承上次方向 |
| PressureHandle | 自动为 4（Hold） | 不适用 | 无位置运动 |

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

读取当前周期的泵速控制输出，写入伺服泵驱动器。支持多轴冲突检测。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `ENABLE` | BOOL | IN | 读取使能 |
| `STRATEGY` | SINT | IN | 读取策略（当前仅支持 0=MAX 仲裁） |
| `PUMPSPEED` | REAL | OUT | 泵速输出值（非负，所有活跃轴的最大值） |
| `CONFLICT` | BOOL | OUT | **方向冲突标志**（新增）：同时存在 EXTEND 和 RETRACT 轴时置 TRUE |
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
    IF fbGetPump.CONFLICT THEN
        // 处理多轴方向冲突警告
    END_IF
END_IF
```

**方向冲突检测规则**
- 遍历所有 ACTIVE 轴的 `STATE.plannedDirection`
- 同时存在 `EXTEND` 轴和 `RETRACT` 轴 → `CONFLICT=TRUE`
- `HOLD`（保压）和 `AUTO`（未解析方向）的轴**不参与**冲突检测
- `CONFLICT=TRUE` 时 PUMPSPEED 仍输出（最大值仲裁），工艺层负责处理告警响应

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
| `MOTION` | HYD_AXISMOTION | IN/OUT | 双向运动数据通道（Setpoint半区：PLC→runtime；Actual半区：runtime→PLC） |
| `AXISID` | SINT | IN | 轴 ID |
| `BUFFERMODE` | INT | IN | 缓冲模式（0=ABORT, 1=BUFFER） |
| `ACTIVE` | BOOL | OUT | 运动激活中 |
| `BUSY` | BOOL | OUT | 功能块忙碌 |
| `DONE` | BOOL | OUT | 运动完成 |
| `COMMANDABORTED` | BOOL | OUT | 命令被新命令取代 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例 — 单段合模运动**

```pascal
fbMoveProfile(EN := TRUE,
              EXECUTE := startClamp,
              AXISID := axisId,
              BUFFERMODE := 0,  // ABORT模式
              MOTION := (SEGMENTTAG := 0,   // CLAMPING
                         SEGMENTTYPE := 0,   // 对应工艺类型
                         PLANNER := 0,       // 位置规划
                         MODE := 0,          // 位置控制
                         ENDCONDITION := 0,  // 到位置结束
                         DIRECTION := 1,     // Positive(合模伸出)
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
- `MOTION.DIRECTION` 直接传递给内部段描述符，支持 0~4 的全部方向枚举值
- `MOTION` 的 `ACT*` 字段作为反馈输入写入 `AXIS_REF`

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
- 当前版本此 FB 为预留接口，完整多段配方加载功能待后续版本实现
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
| `CONTINUOUSUPDATE` | BOOL | IN | 运行中持续更新参数（置 TRUE 时每周期推送 LiveUpdate） |
| `POSITION` | REAL | IN | 目标绝对位置 |
| `VELOCITY` | REAL | IN | 最大速度（始终取绝对值，方向由 DIRECTION 决定） |
| `ACCELERATION` | REAL | IN | 加速度 |
| `DECELERATION` | REAL | IN | 减速度 |
| `JERK` | REAL | IN | 加加速度（预留，目前非零值将被拒绝） |
| `DIRECTION` | SINT | IN | 方向枚举值（0~3，见 2.3 节；4=Hold 不适用于此 FB） |
| `BUFFERMODE` | INT | IN | 缓冲模式 |
| `DONE` | BOOL | OUT | 到达目标位置 |
| `BUSY` | BOOL | OUT | 运动中 |
| `ACTIVE` | BOOL | OUT | 运动激活 |
| `COMMANDABORTED` | BOOL | OUT | 命令被新命令取代 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例 — 注射前进到位（强制正向）**

```pascal
fbMoveAbs(EN := TRUE,
          EXECUTE := injectStart,
          AXISID := axisId,
          POSITION := 200.0,     // 目标位置 200mm
          VELOCITY := 80.0,      // 最大速度 80mm/s（自动取绝对值）
          ACCELERATION := 200.0,
          DECELERATION := 200.0,
          JERK := 0.0,
          DIRECTION := 1,        // Positive_Direction（强制正向）
          BUFFERMODE := 0,
          CONTINUOUSUPDATE := FALSE);

IF fbMoveAbs.DONE THEN
    // 到位，切换保压
END_IF
```

**调用示例 — 自动方向（推荐）**

```pascal
fbMoveAbs(EN := TRUE,
          EXECUTE := startMove,
          AXISID := axisId,
          POSITION := 50.0,      // 目标位置 < 当前位置 → 运行时自动选负向
          VELOCITY := 60.0,
          ACCELERATION := 150.0,
          DECELERATION := 150.0,
          JERK := 0.0,
          DIRECTION := 0,        // Shortest_Way（自动推导方向）
          BUFFERMODE := 0,
          CONTINUOUSUPDATE := FALSE);
```

**信号时序**

```
EXECUTE:        ___/‾‾‾\___
ACTIVE:         ______/‾‾‾‾‾‾\____
BUSY:           ______/‾‾‾‾‾‾‾‾‾\______
DONE:           ______________/‾‾\___
COMMANDABORTED: ________________ (若被取代则置TRUE)
```

**方向校验规则**

| DIRECTION | 校验行为 |
|-----------|---------|
| 0（Shortest_Way） | 无校验，运行时根据位置差推导方向 |
| 1（Positive） | 目标位置 < 当前位置 − positionTolerance 时，报错 `PUMP_DIRECTION_CONFLICT` |
| 2（Negative） | 目标位置 > 当前位置 + positionTolerance 时，报错 `PUMP_DIRECTION_CONFLICT` |
| 3（Current） | 无校验，继承上次方向 |

**注意事项**
- 内部自动构建位置控制段：`MODE=POSITION`, `ENDCONDITION=POSITION`
- `VELOCITY` 无论正负始终取绝对值，实际方向完全由 `DIRECTION` 参数决定
- `POSITION` 到达容差默认 0.1mm，超时保护默认 30s
- `CONTINUOUSUPDATE=TRUE` 时每周期向运行时推送 LiveUpdate，支持运行中修改目标位置、速度、方向

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
| `CONTINUOUSUPDATE` | BOOL | IN | 运行中持续更新参数 |
| `VELOCITY` | REAL | IN | 目标速度（`DIRECTION=0` 时正/负值决定方向；否则始终取绝对值） |
| `ACCELERATION` | REAL | IN | 加速度 |
| `DECELERATION` | REAL | IN | 减速度 |
| `JERK` | REAL | IN | 加加速度（预留） |
| `DIRECTION` | SINT | IN | 方向枚举值（0~3） |
| `BUFFERMODE` | INT | IN | 缓冲模式 |
| `INVELOCITY` | BOOL | OUT | 已达到目标速度 |
| `BUSY` | BOOL | OUT | 运动中 |
| `ACTIVE` | BOOL | OUT | 运动激活 |
| `COMMANDABORTED` | BOOL | OUT | 命令被取代 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例 — 慢速合模（正向）**

```pascal
fbMoveVel(EN := TRUE,
          EXECUTE := slowCloseStart,
          AXISID := axisId,
          VELOCITY := 10.0,      // 10mm/s 正值表示正向（Shortest_Way时）
          ACCELERATION := 50.0,
          DECELERATION := 50.0,
          JERK := 0.0,
          DIRECTION := 1,        // 也可使用 Positive 明确正向
          BUFFERMODE := 0,
          CONTINUOUSUPDATE := FALSE);

IF fbMoveVel.INVELOCITY THEN
    // 已达目标速度，持续运行
END_IF
```

**调用示例 — 速度符号决定方向（Shortest_Way）**

```pascal
// DIRECTION=0 时，VELOCITY 正负号决定方向
fbMoveVel(VELOCITY := -20.0,  // 负值 → 负向（缩回）
          DIRECTION := 0,      // Shortest_Way，由速度符号推导
          ...);
```

**方向推导规则（DIRECTION=0 Shortest_Way 时）**

| VELOCITY 值 | 推导的方向 |
|------------|-----------|
| > 0 | 正向（EXTEND） |
| < 0 | 负向（RETRACT） |
| = 0 | 沿用上次运动方向；若从未运动则默认正向 |

**注意事项**
- 内部构建速度斜坡段：`MODE=SPEED_RAMP`, `ENDCONDITION=MANUAL`
- 无超时保护（`timeoutLimit=0`），持续运行直到被 Stop 取代
- 不输出 `DONE`，而输出 `INVELOCITY` 表示已达到目标速度
- **对所有 DIRECTION 值**：段描述符中 `maxVelocity` 始终存储绝对值，方向由 `direction` 字段单独记录

---

### 4.8 HYD_PRESSUREHANDLE — 压力闭环控制

Direct 模式下的压力闭环控制，支持保压和压力斜坡。内部自动将方向设置为 `HOLD`（值 4），不输出位置运动。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `EXECUTE` | BOOL | IN | 上升沿触发 |
| `AXISID` | SINT | IN | 轴 ID |
| `CONTINUOUSUPDATE` | BOOL | IN | 运行中持续更新压力目标和斜坡速率（新增） |
| `PRESSURE` | REAL | IN | 目标压力 |
| `PRESSURERAMPRATE` | REAL | IN | 压力斜坡速率 |
| `DURATION` | REAL | IN | 持续时间（0=持续到手动停止） |
| `BUFFERMODE` | INT | IN | 缓冲模式 |
| `INPRESSURE` | BOOL | OUT | 已达到目标压力 |
| `DONE` | BOOL | OUT | 保压时间结束（DURATION > 0 时） |
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
              BUFFERMODE := 0,
              CONTINUOUSUPDATE := FALSE);

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
              BUFFERMODE := 0,
              CONTINUOUSUPDATE := FALSE);
```

**调用示例 — 运行中在线修改目标压力**

```pascal
// CONTINUOUSUPDATE=TRUE 时，每周期推送新目标压力（无需重新触发 EXECUTE）
fbPressHandle(EN := TRUE,
              EXECUTE := pressActive,      // 保持 TRUE
              CONTINUOUSUPDATE := TRUE,    // 开启在线更新
              PRESSURE := newPressure,     // 每周期更新目标压力
              PRESSURERAMPRATE := 40.0,
              ...);
```

**注意事项**
- `DURATION > 0` → 时间结束条件（DONE 输出），`DURATION = 0` → 手动结束条件
- 内部默认 PI 压力控制器（Kp=0.5, Ki=0.1）
- 压力容差默认 0.5MPa，超时保护 30s
- 保压流量默认 5.0 L/min，最大流量限制 20.0 L/min
- **方向固定为 HOLD**：`GetPumpRequest` 的冲突检测不计入此轴
- `CONTINUOUSUPDATE=TRUE` 时每周期同步 `PRESSURE` 和 `PRESSURERAMPRATE`，无需重新上升沿

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

### 4.10 HYD_HOLD — 暂停运动

暂停当前运动，轴保持当前位置/状态，等待 Resume。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `EXECUTE` | BOOL | IN | 上升沿触发暂停 |
| `AXISID` | SINT | IN | 轴 ID |
| `DONE` | BOOL | OUT | 暂停完成 |
| `BUSY` | BOOL | OUT | 暂停中 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例**

```pascal
fbHold(EN := TRUE,
       EXECUTE := pauseTrigger,
       AXISID := axisId);

IF fbHold.DONE THEN
    // 轴已暂停，可安全检查状态
END_IF
```

**注意事项**
- 仅在 `STARTING` 或 `RUNNING` 状态下有效
- 暂停后轴进入 `HOLD` 状态，需调用 `HYD_RESUME` 恢复
- 不同于 Stop：Hold 后可 Resume 继续执行；Stop 后须重新启动段

---

### 4.11 HYD_RESUME — 恢复运动

从 Hold 状态恢复运动执行。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `EXECUTE` | BOOL | IN | 上升沿触发恢复 |
| `AXISID` | SINT | IN | 轴 ID |
| `DONE` | BOOL | OUT | 恢复完成 |
| `BUSY` | BOOL | OUT | 恢复中 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例**

```pascal
fbResume(EN := TRUE,
         EXECUTE := resumeTrigger,
         AXISID := axisId);

IF fbResume.DONE THEN
    // 运动已恢复
END_IF
```

**注意事项**
- 仅在 `HOLD` 状态下有效
- 恢复后轴继续执行 Hold 前的运动段

---

### 4.12 HYD_RESET — 故障复位

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

### 4.13 HYD_READSTATUS — 读取控制器状态

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
| `STATE` | UINT | OUT | 控制器状态值（参见 2.2 节状态枚举） |

---

### 4.14 HYD_READERROR — 读取错误信息

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

### 4.15 HYD_READSIMFEEDBACK — 读取仿真反馈

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

### 4.16 HYD_READPARAMETER — IEC 方式读取参数

通过参数编号读取指定轴的 REAL 类型参数值（电平触发型）。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `AXISID` | SINT | IN | 轴 ID |
| `ENABLE` | BOOL | IN | 读取使能（电平触发，持续为 TRUE 时每周期更新） |
| `PARAMETERNUMBER` | INT | IN | 参数编号（见参数读写接口设计文档） |
| `VALID` | BOOL | OUT | 数据有效 |
| `BUSY` | BOOL | OUT | 读取中 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |
| `VALUE` | LREAL | OUT | 参数值（REAL→LREAL 宽化输出） |

**调用示例**

```pascal
fbReadParam(EN := TRUE,
            AXISID := axisId,
            ENABLE := TRUE,
            PARAMETERNUMBER := 11);  // 读取压力比例增益

IF fbReadParam.VALID THEN
    currentKp := LREAL_TO_REAL(fbReadParam.VALUE);
END_IF
```

**注意事项**
- 电平触发型：`ENABLE=TRUE` 期间每周期输出最新值
- 仅支持 REAL 类型参数（索引 0~26, 28~33）；BOOL 参数（索引 27）须用 `HYD_READBOOLPARAMETER`

---

### 4.17 HYD_WRITEPARAMETER — IEC 方式写入参数

通过参数编号写入指定轴的 REAL 类型参数值（上升沿触发型）。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `AXISID` | SINT | IN | 轴 ID |
| `EXECUTE` | BOOL | IN | 上升沿触发写入 |
| `PARAMETERNUMBER` | INT | IN | 参数编号 |
| `VALUE` | LREAL | IN | 要写入的值（LREAL→REAL 截断） |
| `DONE` | BOOL | OUT | 写入完成 |
| `BUSY` | BOOL | OUT | 写入中 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例**

```pascal
// 在线修改压力比例增益（索引 11）
fbWriteParam(EN := TRUE,
             AXISID := axisId,
             EXECUTE := writeKpTrigger,
             PARAMETERNUMBER := 11,
             VALUE := 12.5);

IF fbWriteParam.DONE THEN
    // 参数已写入
END_IF
```

---

### 4.18 HYD_READBOOLPARAMETER — IEC 方式读取 BOOL 参数

读取 BOOL 类型参数（当前仅索引 27 = 仿真使能）。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `AXISID` | SINT | IN | 轴 ID |
| `ENABLE` | BOOL | IN | 读取使能（电平触发） |
| `PARAMETERNUMBER` | INT | IN | 参数编号（目前仅 27 有效） |
| `VALID` | BOOL | OUT | 数据有效 |
| `BUSY` | BOOL | OUT | 读取中 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |
| `VALUE` | BOOL | OUT | 参数值 |

---

### 4.19 HYD_WRITEBOOLPARAMETER — IEC 方式写入 BOOL 参数

写入 BOOL 类型参数（当前仅索引 27 = 仿真使能）。

**接口定义**

| 参数 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `EN` | BOOL | IN | 使能 |
| `ENO` | BOOL | OUT | 使能输出 |
| `AXISID` | SINT | IN | 轴 ID |
| `EXECUTE` | BOOL | IN | 上升沿触发写入 |
| `PARAMETERNUMBER` | INT | IN | 参数编号（目前仅 27 有效） |
| `VALUE` | BOOL | IN | 要写入的值 |
| `DONE` | BOOL | OUT | 写入完成 |
| `BUSY` | BOOL | OUT | 写入中 |
| `ERROR` | BOOL | OUT | 错误标志 |
| `ERRORID` | WORD | OUT | 错误码 |

**调用示例**

```pascal
// 开启仿真模式（索引 27）
fbWriteBoolParam(EN := TRUE,
                 AXISID := axisId,
                 EXECUTE := simEnableTrigger,
                 PARAMETERNUMBER := 27,
                 VALUE := TRUE);
```

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
  0: // 快速合模（强制正向）
     fbMoveAbs(EXECUTE := TRUE, POSITION := 100.0, VELOCITY := 80.0,
               DIRECTION := 1, ...);  // 1=Positive
     IF fbMoveAbs.DONE THEN step := 1; END_IF

  1: // 慢速合模（低压保护，沿用正向）
     fbMoveVel(EXECUTE := TRUE, VELOCITY := 10.0,
               DIRECTION := 3, ...);  // 3=Current（继承上次正向）
     IF moldTouch THEN step := 2; END_IF

  2: // 高压锁模
     fbPressHandle(EXECUTE := TRUE, PRESSURE := 120.0,
                   PRESSURERAMPRATE := 100.0, DURATION := 0, ...);
     IF fbPressHandle.INPRESSURE THEN step := 3; END_IF

  3: // 注射（强制正向）
     fbMoveAbs(EXECUTE := TRUE, POSITION := 200.0, VELOCITY := 100.0,
               DIRECTION := 1, ...);  // 1=Positive
     IF fbMoveAbs.DONE THEN step := 4; END_IF

  4: // 保压
     fbPressHandle(EXECUTE := TRUE, PRESSURE := 80.0,
                   PRESSURERAMPRATE := 50.0, DURATION := 5.0, ...);
     IF fbPressHandle.DONE THEN step := 5; END_IF

  5: // 开模（强制负向）
     fbMoveAbs(EXECUTE := TRUE, POSITION := 0.0, VELOCITY := 60.0,
               DIRECTION := 2, ...);  // 2=Negative
     IF fbMoveAbs.DONE THEN step := 6; END_IF

  6: // 顶出（强制正向）
     fbMoveAbs(EXECUTE := TRUE, POSITION := 50.0, VELOCITY := 30.0,
               DIRECTION := 1, ...);  // 1=Positive
     IF fbMoveAbs.DONE THEN step := 0; END_IF  // 循环
END_CASE

// ═══════════════════════════════════════
// 阶段3: 输出泵速（含冲突检测）
// ═══════════════════════════════════════
fbGetPump(EN := TRUE, ENABLE := TRUE, STRATEGY := 0);
servo_output := fbGetPump.PUMPSPEED;
IF fbGetPump.CONFLICT THEN
    // 可选：触发多轴方向冲突告警处理
END_IF

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
              MOTION := (SEGMENTTAG := 0,
                         SEGMENTTYPE := 0,
                         MODE := 0,
                         DIRECTION := 1,  // 1=Positive
                         ...));
```

### 5.3 在线参数调整（IEC 参数接口）

```pascal
// 运行中在线修改压力控制器增益（不中断运动）
fbWriteParam(EXECUTE := tuneKpTrigger,
             PARAMETERNUMBER := 11,    // 压力Kp
             VALUE := 15.0,
             AXISID := axisId);

// 读取当前状态
fbReadParam(ENABLE := TRUE,
            PARAMETERNUMBER := 0,     // 位置容差
            AXISID := axisId);
currentPosTolerance := LREAL_TO_REAL(fbReadParam.VALUE);
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

### 7.1 诊断代码（HYD_DiagnosticCode）

枚举值为连续整数，顺序与 `common_types.h` 中 `HYD_DiagnosticCode` 的定义完全一致。

| 错误码 | 常量名 | 严重级别 | 说明 |
|--------|--------|----------|------|
| 0  | `HYD_DIAG_CODE_NONE`                      | —       | 无错误 |
| 1  | `HYD_DIAG_CODE_RECIPE_EMPTY`              | FAULT   | 配方为空 |
| 2  | `HYD_DIAG_CODE_RECIPE_TOO_LARGE`          | FAULT   | 配方段数超过 `HYD_MAX_SEGMENTS` 限制 |
| 3  | `HYD_DIAG_CODE_SEGMENT_INVALID`           | FAULT   | 段参数无效（速度/压力/位置等超界或缺失） |
| 4  | `HYD_DIAG_CODE_RUNTIME_CONFIG_INVALID`    | FAULT   | 运行时配置无效（泵增益/面积等未设置） |
| 5  | `HYD_DIAG_CODE_START_CONTEXT_INVALID`     | FAULT   | 启动上下文非法（轴 ID 越界或当前状态不允许启动） |
| 6  | `HYD_DIAG_CODE_COMMAND_NOT_ALLOWED`       | FAULT   | 命令在当前状态不被允许 |
| 7  | `HYD_DIAG_CODE_NO_RECIPE`                 | FAULT   | 无配方可执行（需先写入配方再启动） |
| 8  | `HYD_DIAG_CODE_NO_DIRECT_SEGMENT`         | FAULT   | 无直接段可执行（直接模式下段未准备好） |
| 9  | `HYD_DIAG_CODE_SEGMENT_INDEX_OUT_OF_RANGE`| FAULT   | 段索引越界 |
| 10 | `HYD_DIAG_CODE_SEGMENT_NOT_COMPLETED`     | WARNING | 段未完成即收到 NEXT 命令 |
| 11 | `HYD_DIAG_CODE_RECIPE_ALREADY_FINISHED`   | INFO    | 配方已全部完成，无法继续 NEXT |
| 12 | `HYD_DIAG_CODE_ABORTED`                   | INFO    | 运动被 ABORT 命令中止 |
| 13 | `HYD_DIAG_CODE_TIMEOUT`                   | FAULT   | 段运行超出 `timeoutLimit` |
| 14 | `HYD_DIAG_CODE_OVER_PRESSURE`             | FAULT   | 实际压力超过 `pressureTolerance` 上界 |
| 15 | `HYD_DIAG_CODE_UNDER_PRESSURE`            | FAULT   | 实际压力低于 `pressureTolerance` 下界 |
| 16 | `HYD_DIAG_CODE_FLOW_DEVIATION`            | FAULT   | 实际流量偏差超过 `flowTolerance` |
| 17 | `HYD_DIAG_CODE_POSITION_DEVIATION`        | FAULT   | 实际位置偏差超过 `positionTolerance` |
| 18 | `HYD_DIAG_CODE_VELOCITY_DEVIATION`        | FAULT   | 实际速度偏差超过 `velocityTolerance` |
| 19 | `HYD_DIAG_CODE_SENSOR_FAULT`              | FAULT   | 传感器故障（压力/位置反馈无效） |
| 20 | `HYD_DIAG_CODE_TIMESTAMP_ROLLBACK`        | FAULT   | 时间戳回滚（扫描周期异常或系统时间跳变） |
| 21 | `HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED` | WARNING | 压力超出 `pressureCeiling` 上限（触发比例限速，保护中） |
| 22 | `HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED` | FAULT   | 压力持续超出天花板且超过容差，停机 |
| 23 | `HYD_DIAG_CODE_OVER_PRESSURE_LIMIT`       | WARNING | 超过段级最大压力限制 `maxPressure`（比例限速中） |
| 24 | `HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT` | FAULT   | 持续超过最大压力限制，触发停机保护 |
| 25 | `HYD_DIAG_CODE_SOFT_LIMIT_REACHED`        | WARNING | 到达软限位减速带边界（正在减速） |
| 26 | `HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED`       | FAULT   | 超出软限位边界（行程溢出） |
| 27 | `HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT`   | FAULT   | 方向冲突：强制方向（POSITIVE/NEGATIVE）与目标位置不匹配 |
| 28 | `HYD_DIAG_CODE_INTERNAL_ERROR`            | FAULT   | 库内部逻辑错误（不可恢复） |

> **注意**：枚举值从 0 开始连续递增，与代码中 `HYD_DiagnosticCode` 枚举定义完全对应。  
> 完整定义参见 [common_types.h](include/common_types.h)

---

### 7.2 诊断标志位（HYD_DiagnosticFlags）

`HYD_GETAXISINFO` 输出的 `DIAGFLAGS`（WORD）是各运行时异常的位掩码汇总，多个异常可同时置位。编译时需开启 `HYD_ENABLE_DIAGNOSTIC_FLAGS=1`。

| 位掩码（十六进制） | 常量名 | 对应诊断代码 | 说明 |
|-------------------|--------|-------------|------|
| `0x0001` | `HYD_DIAG_FLAG_OVER_PRESSURE`              | 14 | 超压 |
| `0x0002` | `HYD_DIAG_FLAG_UNDER_PRESSURE`             | 15 | 低压 |
| `0x0004` | `HYD_DIAG_FLAG_FLOW_DEVIATION`             | 16 | 流量偏差 |
| `0x0008` | `HYD_DIAG_FLAG_POSITION_DEVIATION`         | 17 | 位置偏差 |
| `0x0010` | `HYD_DIAG_FLAG_VELOCITY_DEVIATION`         | 18 | 速度偏差 |
| `0x0020` | `HYD_DIAG_FLAG_TIMEOUT`                    | 13 | 超时 |
| `0x0040` | `HYD_DIAG_FLAG_SENSOR_FAULT`               | 19 | 传感器故障 |
| `0x0080` | `HYD_DIAG_FLAG_TIMESTAMP_ROLLBACK`         | 20 | 时间戳回滚 |
| `0x0100` | `HYD_DIAG_FLAG_PRESSURE_CEILING_EXCEEDED`  | 21 | 压力超天花板（WARNING） |
| `0x0200` | `HYD_DIAG_FLAG_PRESSURE_CEILING_VIOLATED`  | 22 | 压力天花板持续违规（FAULT） |
| `0x0400` | `HYD_DIAG_FLAG_PUMP_DIRECTION_CONFLICT`    | 27 | 方向冲突 |
| `0x0800` | `HYD_DIAG_FLAG_OVER_PRESSURE_LIMIT`        | 23 | 超过段级最大压力限制 |
| `0x1000` | `HYD_DIAG_FLAG_SOFT_LIMIT`                 | 25/26 | 软限位触发（WARNING 或 FAULT） |

**PLC 读取示例：**
```pascal
(* 检查是否存在任意压力类报警 *)
bPressureAlarm := (fbAxisInfo.DIAGFLAGS AND 16#0303) <> 0;

(* 检查方向冲突 *)
bDirectionConflict := (fbAxisInfo.DIAGFLAGS AND 16#0400) <> 0;

(* 检查软限位 *)
bSoftLimit := (fbAxisInfo.DIAGFLAGS AND 16#1000) <> 0;
```

> 完整定义参见 [common_types.h](include/common_types.h) — `HYD_DiagnosticFlag` 枚举

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

> 参见 [hyd_config.h](include/hyd_config.h)

---

## 9. 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| V1.0 | 2026-06 | 初始版本，覆盖全部 13 个 IEC 功能块接口 |
| V1.1 | 2026-06 | 增补 HYD_HOLD / HYD_RESUME FB；新增 4 个参数读写 IEC FB（READPARAMETER / WRITEPARAMETER / READBOOLPARAMETER / WRITEBOOLPARAMETER）；GETPUMPREQUEST 新增 CONFLICT 输出引脚；PRESSUREHANDLE 新增 CONTINUOUSUPDATE 引脚；AXISMOTION 结构体新增 SEGMENTTYPE 字段；**全面更新方向枚举**至 PLCopen / Beckhoff TF5810 五值模型（含 CURRENT=3 新值），补充各 FB 方向校验规则和速度符号推导说明 |
| V1.2 | 2026-06 | **全面修订第 7 章错误码表**：修正枚举值偏移错误（原表跳过索引 9 导致 10~23 全部错位）；补齐 6 个缺失错误码（9=SEGMENT_INDEX_OUT_OF_RANGE、21=PRESSURE_CEILING_EXCEEDED、22=PRESSURE_CEILING_VIOLATED、23=OVER_PRESSURE_LIMIT、24=OVER_PRESSURE_LIMIT_FAULT、25=SOFT_LIMIT_REACHED、26=SOFT_LIMIT_VIOLATED）；修正 PUMP_DIRECTION_CONFLICT=27、INTERNAL_ERROR=28；新增诊断标志位（HYD_DiagnosticFlags）完整位掩码表及 PLC 读取示例 |

---

如需进一步了解命令仲裁细节、诊断系统架构或嵌入式部署流程，建议继续阅读：

- [IEC61131-3 适配层](docs/architecture) — 适配层整体架构与数据流
- [配方与直接模式](docs/) — 两种运动模式的完整对比
- [液压库参数读写接口设计文档](docs/液压库参数读写接口设计文档.md) — 参数编号、分组与调用规范
