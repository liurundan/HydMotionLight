# GEN 参数消除与命令校验统一化设计

## 背景

当前 IEC 命令接口存在两套机制各自判断命令接受与否：

| 机制 | 位置 | 职责 |
|------|------|------|
| `HYD_IsCommandAllowedInState` + 状态掩码表 | 核心层 `motion_control.c` | 命令 X 在状态 Y 下是否合法 |
| `_commandGeneration` + `GEN : WORD` 参数 | IEC 层 `motion_interface.c` | 此 IEC FB 实例是否还拥有轴控制权（多 FB 抢占检测） |

问题：
1. GEN 作为 WORD 参数暴露给 PLC 程序员，增加不必要的理解负担
2. 抢占检测（GEN）和状态校验（掩码表）两套机制有概念重叠
3. 所有运动 FB 无条件抢占当前运动，与 PLCopen 标准的 BufferMode 机制不符

## 设计目标

1. **PLCopen 对齐**：状态机是命令准入的唯一判定依据；增加 BufferMode 参数控制抢占/缓冲
2. **GEN 消除**：从所有 IEC FB 公共接口删除 GEN 参数，用内部 `_executionId` 替代
3. **校验统一**：命令可否执行由核心层 `HYD_IsCommandAllowedInState` 单一口径判决；IEC 层不做重复校验
4. **失败即报错**：命令不被接受时 IEC FB 立即设置 ERROR=true，不做静默入队

## 公共接口变更

### BufferMode 枚举（common_types.h 新增）

```c
typedef enum {
    HYD_BUFFER_MODE_ABORT  = 0,  // 抢占当前运动，立即执行本命令
    HYD_BUFFER_MODE_BUFFER = 1,  // 缓冲：仅当轴空闲时执行；忙时返回错误
    // 2-5 保留给后续 Blending 扩展
} HYD_BufferMode;
```

本期仅实现 ABORT 和 BUFFER。BUFFER 在轴忙时直接返回 ERROR（无内部缓冲队列）。

### 受影响的 IEC FB 引脚变更

| FB | 删除 | 新增 |
|----|------|------|
| MoveAbsolute | `GEN : WORD` | `BUFFERMODE : INT` |
| MoveVelocity | `GEN : WORD` | `BUFFERMODE : INT` |
| PressureHandle | `GEN : WORD` | `BUFFERMODE : INT` |
| MoveProfile | `GEN : WORD` | `BUFFERMODE : INT` |
| Stop | `GEN : WORD` | —（始终硬抢占） |
| Reset | `GEN : WORD` | —（不涉及命令抢占） |

## 核心层改动

### `_commandGeneration` → `_executionId`

递增时机调整为 **双点递增**——在 `HYD_AbortNow()` 和 `HYD_BeginSegment()` 两处均递增：

```
旧：仅在 Abort() 时递增
新：Abort() 时递增一次 + BeginSegment() 成功时再递增一次
```

**为何需要双点递增：** 仅在 BeginSegment 递增存在竞态窗口——pending FB 在同一扫描周期内检测 `FB_STATE == ABORTED` 时，若新段恰好在同周期通过 BeginSegment 启动，pending FB 会错误地捕获新段的 `_executionId`，导致两个 FB 同时认为拥有同一个段。AbortNow 中的递增闭合了这个窗口：abort 瞬间改变计数值，pending FB 的下一次检查必然看到不匹配。

对于 Stop（只 Abort 不启动新段）：`_executionId` 在 AbortNow 中递增一次（无后续 BeginSegment）。被 Stop 的 FB 通过 `_executionId` 不匹配检测抢占。

### 不改动的部分

- `HYD_IsCommandAllowedInState` 状态掩码表完全保留
- `HYD_RequestCommandQueue` 的状态校验逻辑保留
- 所有运动控制算法模块不变
- 诊断、保护、段完成模块不变

## IEC 层统一模板

### 运动命令 FB 通用流程

```
execRising:
  1. BufferMode == ABORT → Abort(fb) + Scan(fb)
     BufferMode == BUFFER → 跳过，由 StartSegment 状态校验把关
  2. buildXxxSegment() 构建段参数（各 FB 不同）
  3. LoadDirectSegment(fb, &seg) → 失败则 ERROR
  4. StartSegment(fb, 0, timestamp) → 失败则 ERROR（状态不允许等）
     → 成功则标记 _pending = true

后续周期（_pending == true）:
  等待核心在 Publish/Scan 中执行 BeginSegment：
  - active && 段源匹配 → 捕获 _myExecId = fb->_executionId, _pending = false
  - FB_STATE == ABORTED  → 入队后被抢占，COMMANDABORTED

后续周期（_pending == false && _myExecId != 0）:
  _myExecId == fb->_executionId → 正常映射 BUSY/ACTIVE/DONE/ERROR
  _myExecId != fb->_executionId → COMMANDABORTED
```

### 各 FB 差异

| FB | 段构建 | 特殊输出 |
|----|-------|---------|
| MoveAbsolute | `buildPositionSegment(pos, vel, accel, dir)` | — |
| MoveVelocity | `buildVelocitySegment(vel, accel, dir)` | `INVELOCITY`：\|速度误差\| < 5% |
| PressureHandle | `buildPressureSegment(press, ramp, dur)` | `INPRESSURE`：\|压力误差\| < 0.5 MPa |
| MoveProfile | `buildSegmentFromMotion(&motion)` | `PUMP_SPEED`, `STATE`, MOTION 写回 |

### Stop — 不使用 _executionId

Stop 始终硬抢占，无段启动。完成判断直接读 `FB_STATE` 和 `STATE.active`。
被 Stop 抢占的其他 FB 通过 `_executionId` 不匹配（若有新段）或 `FB_STATE == ABORTED` 检测。

### Reset — 不使用 _executionId

Reset 直接调用 `SoftReset()`，不涉及段或抢占。

## 状态迁移校验表

### BufferMode = ABORT

| 当前 FB_STATE | Abort 允许 | Abort 后状态 | StartSegment 允许 | 结果 |
|--------------|-----------|-------------|-------------------|------|
| DISABLED | 否 | — | — | ERROR |
| IDLE | 是 | ABORTED | 是 | 执行新段 |
| READY | 是 | ABORTED | 是 | 执行新段 |
| STARTING | 是 | ABORTED | 是 | 抢占，执行新段 |
| RUNNING | 是 | ABORTED | 是 | 抢占，执行新段 |
| SEGMENT_COMPLETE | 是 | ABORTED | 是 | 执行新段 |
| HOLD | 是 | ABORTED | 是 | 抢占，执行新段 |
| DONE | 是 | ABORTED | 是 | 执行新段 |
| ABORTED | 是 | ABORTED | 是 | 执行新段 |
| FAULT | 否 | — | — | ERROR（需先 Reset） |

### BufferMode = BUFFER

| 当前 FB_STATE | StartSegment 允许 | 结果 |
|--------------|-------------------|------|
| DISABLED | 否 | ERROR |
| IDLE | 是 | 执行新段 |
| READY | 是 | 执行新段 |
| STARTING | 否 | ERROR（轴忙） |
| RUNNING | 否 | ERROR（轴忙） |
| SEGMENT_COMPLETE | 是 | 执行新段 |
| HOLD | 否 | ERROR（轴保持中） |
| DONE | 是 | 执行新段 |
| ABORTED | 是 | 执行新段 |
| FAULT | 否 | ERROR（需先 Reset） |

### Stop 始终硬抢占

| 当前 FB_STATE | Abort 允许 | 结果 |
|--------------|-----------|------|
| DISABLED | 否 | ERROR |
| FAULT | 否 | ERROR |
| 其余全部 | 是 | Abort → ABORTED → DONE |

## 错误码

无需新增诊断码。核心层状态校验已产生对应诊断码，IEC 层直接映射：

| 拒绝场景 | 诊断码 |
|---------|-------|
| 状态不允许该命令 | `HYD_DIAG_CODE_COMMAND_NOT_ALLOWED` |
| 轴未创建 (DISABLED) | `HYD_DIAG_CODE_START_CONTEXT_INVALID` |
| 段参数无效 | `HYD_DIAG_CODE_SEGMENT_INVALID` |
| 无配方 (MoveProfile) | `HYD_DIAG_CODE_NO_RECIPE` |
| 轴索引越界（IEC 层） | `HYD_DIAG_CODE_START_CONTEXT_INVALID` |

## 改动文件清单

| 文件 | 改动 | 规模 |
|------|------|------|
| `include/common_types.h` | 新增 `HYD_BufferMode` 枚举 | +8 |
| `include/motion_control.h` | `_commandGeneration` → `_executionId`；注释调整 | ±2 |
| `src/motion_control.c` | `AbortNow` 保留递增；`BeginSegment` 末尾新增递增 | ±2 |
| `include/motion_interface.h` | 5 个 FB 删除 GEN、增加 BUFFERMODE（INT） | -6 +5 |
| `src/motion_interface.c` | 5 个 IEC FB 实现按统一模板重写 | -200 +250 |
| **总计** | | **~+52 净行** |

不改动的文件：`state_reporter.c`, `protection_manager.c`, `motion_planner.c`, `pressure_controller.c`, `segment_completion.c`, `diagnostics*.c`, `CreateMotion`, 所有测试文件。

## 边界情况

### _pending 期间 EXECUTE 提前断开

若 PLC 程序在 execRising 入队成功（_pending=true）后、段尚未启动（_myExecId 未捕获）前将 EXECUTE 置为 false：

- FB 继续保持 _pending=true，等待段启动或超时
- 段启动后正常捕获 _myExecId，后续输出由 EXECUTE=false 时的复位逻辑（各 FB 现有行为）处理
- 若段永远不启动（如被其他 Abort 抢占），下一周期检测到 FB_STATE==ABORTED → COMMANDABORTED=true 持续一个周期，然后因 EXECUTE=false 全部输出清零

### COMMANDABORTED 锁存

COMMANDABORTED 在检测到不匹配时输出 true，持续到 EXECUTE 下降沿清零。与当前 GEN 方案行为一致：不匹配一旦发生，只要 EXECUTE 保持 true，COMMANDABORTED 保持为 true。

### 同周期多 FB 竞争

同一 PLC 扫描周期内两个 FB (A, B) 同时对同一轴 execRising（ABORT 模式）：

1. FB A: Abort + Scan → StartSegment 入队，_pending=true
2. FB B: Abort + Scan（FB A 的段已入队但尚未 BeginSegment，状态可能还是 ABORTED）→ StartSegment 入队，_pending=true
3. 下一周期 Publish: Scan 执行 BeginSegment，启动 FB B 的段（后入队的覆盖先入队的，因为 LoadDirectSegment 覆盖了 DIRECT_SEGMENT）
4. FB A 检测 _executionId 不匹配 → COMMANDABORTED

此行为与当前 GEN 方案一致。

## 时序

```
周期    FB A (MoveAbsolute, ABORT)            FB B (MoveVelocity, ABORT)       核心 _executionId
────    ──────────────────────────            ─────────────────────────         ───────────────
  0     execRising: Abort(+1)+Scan              空闲                                6 (5→6)
        LoadDirect + StartSegment 入队
        _pending = true

  1     Framework Publish: Scan
        BeginSegment(+1) → _executionId = 7                                        7 (6→7)

  2     检测 active && 段源匹配
        → _myExecId = 7, _pending = false
        BUSY, ACTIVE 正常输出

 ...    运动执行中...                          execRising (ABORT):
                                              Abort(+1)→ _executionId = 8          8 (7→8)
                                              Scan+StartSeg 入队
                                              _pending = true

 N+1    _myExecId(7) != _executionId(8)       BeginSegment(+1)→ _executionId = 9   9 (8→9)
        → COMMANDABORTED

 N+2                                           检测 active → _myExecId = 9
                                              BUSY, ACTIVE 正常输出
```
