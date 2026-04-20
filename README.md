# HDY Motion Control Library

面向注塑机液压场景的轻量级运动控制库，目标是把**工艺层**与**运动控制层**清晰分离，并保持纯 C99、静态内存、易于嵌入式部署的实现风格。

## 设计边界

- **工艺层负责**：动作配方组织、阀逻辑、段切换决策、机构联锁。
- **运动控制层负责**：速度/流量/压力参考计算、泵速换算、状态与诊断输出。
- 库本身**不直接控制电磁阀**，仅输出泵侧命令与可供上层判定的状态信息。

## 目录结构

- `include/`：对外头文件，包含 `motion_control.h`、`common_types.h`、`diagnostics.h` 等公共接口。
- `src/`：核心实现，包括运动规划、压力控制、泵换算、段完成判定、状态上报、诊断与保护。
- `tests/`：单元测试、场景回归测试与示例程序。
- `项目需求与设计说明书.md`：需求、分层边界与总体设计说明。
- `开发阶段与下一步计划.MD`：Sprint 规划与阶段评审记录。
- `正式发布说明模板.md`：对外发布、版本交付、联调移交时可直接复用的 Release Notes 模板。
- `HMI诊断对照表.md`：供 HMI / 上位机直接映射中文告警文案、分级和现场处置建议。

## 构建与测试

仓库根目录执行：

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc
ctest --test-dir out/build/unixgcc --output-on-failure
```

单独运行端到端示例：

```bash
./out/build/unixgcc/main
```

## 工艺层对接时序

推荐按 PLCopen function block 语义集成：

1. `HDY_MotionControlFB_Init()` 初始化函数块。
2. 配置 `FLOW_TO_PUMP_SPEED_GAIN`、`PUMP_SPEED_LIMIT`。
3. 调用 `HDY_MotionControlFB_LoadRecipe()` 装载配方。
4. 工艺层在合适时机调用：
   - `HDY_MotionControlFB_StartSegment()`；或
   - 置位 `START_SEGMENT/START_SEGMENT_INDEX` 后再进入周期调用。
5. 每周期刷新 `AXIS_REF` 后调用 `HDY_MotionControlFB_Execute()`。
6. 读取：
   - `PUMP_SPEED`
   - `STATE`
   - `DIAGNOSTIC`
   - `SEGMENT_COMPLETED`
   - `SEGMENT_CHANGED`
7. 当 `SEGMENT_COMPLETED=true` 时，由工艺层决定是否调用 `HDY_MotionControlFB_NextSegment()`。
8. 需要安全停机时调用 `HDY_MotionControlFB_Abort()`；需要彻底重置时置位 `RESET`。

## 段切换约定

- `LoadRecipe()` 只完成配方装载，不会自动开始执行。
- `SEGMENT_CHANGED` 是段启动后的**单周期脉冲**。
- `SEGMENT_COMPLETED` 在段完成后保持，直到上层切换下一段或进入最终完成态。
- 最后一段完成后，控制器进入 `FINISHED=true`。
- `EN=false` 会立即输出安全零值，重新使能后不会自动恢复，需要上层重新启动段。

## Direct / Recipe 模式语义（补充）

- 概述：FB 同时持有 `RECIPE[]` 与 `DIRECT_SEGMENT` 两套段参数缓存。启动时由 `USE_RECIPE` 决定使用哪套来源；当 `StartSegment()` / `START_SEGMENT` 被消费并进入运行态后，库会将活动段的参数与来源锁存（latched），后续外部对 `USE_RECIPE` 或 `DIRECT_SEGMENT` 的修改不会影响当前已启动段。

- `USE_RECIPE = true`：`StartSegment()` 使用 `RECIPE[segmentIndex]` 作为启动段；`segmentIndex` 指向配方索引；`NextSegment()` 可用于推进到 recipe 的下一段（若存在）。

- `USE_RECIPE = false`：`StartSegment()` 忽略 `segmentIndex`，使用 `DIRECT_SEGMENT` 作为单段直接执行来源。必须先通过 `HDY_MotionControlFB_LoadDirectSegment()` 装载 `DIRECT_SEGMENT` 并使 `DIRECT_SEGMENT_VALID=true`。Direct 模式下 `NextSegment()` 是不被允许的，库会拒绝并上报诊断（例如 `HDY_DIAG_CODE_COMMAND_NOT_ALLOWED` 或 `HDY_DIAG_CODE_NO_DIRECT_SEGMENT`）。

- Direct 模式语义：
  - 启动：`StartSegment()` 将 `DIRECT_SEGMENT` 的参数锁存为当前活动段。
  - 执行：按段内定义执行，状态与诊断行为与 recipe 段相同。
  - 完成：Direct 模式为单段语义，段完成后控制器直接进入终端完成态（`FB_STATE=DONE` / `FINISHED=true`），不会等待或依赖 `NextSegment()`。

- `STATE.segmentSource`：控制器通过 `STATE.segmentSource`（枚举值 `HDY_SEGMENT_SOURCE_NONE/RECIPE/DIRECT`）对外暴露当前（或最近）活动段的来源。HMI 与工艺层可据此判断当前执行语义（例如禁止在 `DIRECT` 来源下调用 `NextSegment()`）。

- `DIRECT_SEGMENT_VALID`：当 `DIRECT_SEGMENT` 已装载并通过校验时为 true；`HDY_MotionControlFB_LoadDirectSegment()` 会设置该标志并更新 Ready/Idle 判定。

示例：Direct 模式调用

```c
fb->USE_RECIPE = HDY_FALSE;
HDY_MotionControlFB_LoadDirectSegment(&fb, &segment);
HDY_MotionControlFB_StartSegment(&fb, 0, timestamp); // index 被忽略
```

注意：Direct 模式下 `segmentIndex` 仅为兼容接口保留，不代表 recipe 索引。

## 当前发布基线说明

### 基线定位

当前 `master` 可作为本项目的**首个工程化发布基线**使用，适合以下集成前提：

- 上层工艺层已经独立实现动作编排、阀逻辑、机构联锁与段切换决策
- 本库只承担多段运动/压力参考计算、泵速换算、状态与诊断输出
- 目标平台要求纯 C99、静态内存、可在周期任务中稳定调用

### 基线范围

当前基线已纳入并验证的能力包括：

- PLCopen 风格函数块：`Init / LoadRecipe / StartSegment / NextSegment / Abort / Execute`
- 多段配方执行：`HDY_MAX_SEGMENTS = 16`
- 控制模式：
  - `HDY_MODE_POSITION`
  - `HDY_MODE_SPEED_RAMP`
  - `HDY_MODE_PRESSURE_CLOSED_LOOP`
- 压力闭环策略：`P / PI / PID`
- 结束条件：
  - `HDY_END_POSITION`
  - `HDY_END_TIME`
  - `HDY_END_PRESSURE`
  - `HDY_END_FLOW`
  - `HDY_END_MANUAL`
- 运行期模块：运动规划、压力控制、泵速换算、段完成判定、状态上报、保护管理、诊断快照/历史

### 发布语义约束

集成时请按以下语义理解当前发布基线：

- `LoadRecipe()` 仅完成装载，不自动执行
- `StartSegment()` 或 `START_SEGMENT` 是进入运行态的唯一触发入口
- `EN=false` 会立即输出安全零值，且重新使能后**不会自动续跑**
- `RESET=true` 等价于完整 `Init()`，会清空配方、配置与保留诊断
- `SEGMENT_CHANGED` 为单周期脉冲，`SEGMENT_COMPLETED` 为保持信号
- 最后一段完成后进入 `FINISHED=true`
- `PUMP_SPEED` 始终为泵侧非负幅值，方向由 `STATE.plannedDirection` 和工艺层阀动作共同决定

### 当前未纳入发布承诺的内容

以下内容当前**不属于本次发布基线承诺范围**：

- 电磁阀直接控制
- 工艺层自动切段决策
- 机理级安全联锁
- `RBF_PID` 主执行链路集成

其中 `RBF_PID` 模块已保留并具备独立测试价值，但配方校验当前仍将其视为**未支持的主链路压力策略**。

## 诊断接口使用建议

运行期优先读取：

- `DIAGNOSTIC.code`：当前主要诊断码
- `DIAGNOSTIC.severity`：严重级别
- `DIAGNOSTIC.source`：来源模块
- `DIAGNOSTIC.recovery`：建议恢复动作
- `DIAGNOSTIC.protectionAction`：告警 / 降级 / 停止
- `DIAGNOSTIC.flags`：面向嵌入式/HMI 的紧凑标志位摘要

调试与售后可结合：

- `DIAGNOSTIC_LATCH`
- `LAST_DIAGNOSTIC_SNAPSHOT`
- `LAST_FAULT_SNAPSHOT`
- `DIAGNOSTIC_HISTORY`

保留语义说明：

- `DIAGNOSTIC`：当前周期/当前命令的实时结果，在非故障保持态会自动清除
- `DIAGNOSTIC_LATCH`：最近一次非 `NONE` 事件
- `LAST_DIAGNOSTIC_SNAPSHOT`：最近一次诊断事件的上下文快照
- `LAST_FAULT_SNAPSHOT`：最近一次故障停机快照，仅 `FAULT` 事件更新
- `DIAGNOSTIC_HISTORY`：循环历史，当前深度为 `HDY_DIAG_HISTORY_DEPTH = 4`

在实时故障已清除且控制器不处于故障态时，可调用 `HDY_MotionControlFB_AcknowledgeDiagnostics()` 清除保留诊断；故障停机保留信息仍应通过 `RESET` 清除。

现场联调 / 售后场景建议直接复用仓库根目录 `HMI诊断对照表.md` 中的中文显示文案和处置建议。

## 诊断码表

> 表中“恢复建议”“保护动作”直接对应 `DIAGNOSTIC.recovery` 与 `DIAGNOSTIC.protectionAction`。

| 诊断码 | 严重级别 | 来源 | 恢复建议 | 保护动作 | 典型触发场景 |
| --- | --- | --- | --- | --- | --- |
| `NONE` | `NONE` | `NONE` | `NONE` | `NONE` | 当前无活动诊断 |
| `RECIPE_EMPTY` | `WARNING` | `RECIPE` | `RELOAD_RECIPE` | `WARNING` | 装载空配方 |
| `RECIPE_TOO_LARGE` | `WARNING` | `RECIPE` | `RELOAD_RECIPE` | `WARNING` | 配方段数超过 `HDY_MAX_SEGMENTS` |
| `SEGMENT_INVALID` | `WARNING` | `RECIPE` | `RELOAD_RECIPE` | `WARNING` | 段参数非法，或配置了当前主链路未支持的策略/参数组合 |
| `RUNTIME_CONFIG_INVALID` | `FAULT` | `RUNTIME` | `RESET_CONTROLLER` | `STOP` | 运行期配置无效，如泵速增益/限幅非法 |
| `START_CONTEXT_INVALID` | `WARNING` | `COMMAND` | `CHECK_COMMAND` | `WARNING` | 启动段时上下文不满足要求 |
| `NO_RECIPE` | `WARNING` | `COMMAND` | `RELOAD_RECIPE` | `WARNING` | 未装载配方即启动或切段 |
| `SEGMENT_INDEX_OUT_OF_RANGE` | `WARNING` | `COMMAND` | `CHECK_COMMAND` | `WARNING` | 启动段索引越界 |
| `SEGMENT_NOT_COMPLETED` | `WARNING` | `COMMAND` | `CHECK_COMMAND` | `WARNING` | 当前段未完成就请求 `NextSegment()` |
| `RECIPE_ALREADY_FINISHED` | `INFO` | `COMMAND` | `CHECK_COMMAND` | `NONE` | 已完成后再次请求切段 |
| `ABORTED` | `INFO` | `COMMAND` | `NONE` | `NONE` | 调用 `Abort()` 主动终止 |
| `TIMEOUT` | `FAULT` | `EXECUTION` | `RESTART_SEGMENT` | `STOP` | 段执行时间超过 `timeoutLimit` |
| `OVER_PRESSURE` | `WARNING` | `EXECUTION` | `CHECK_COMMAND` | `DERATE` | 实测压力高于参考值加容差 |
| `UNDER_PRESSURE` | `WARNING` | `EXECUTION` | `CHECK_COMMAND` | `WARNING` | 实测压力低于参考值减容差 |
| `FLOW_DEVIATION` | `WARNING` | `EXECUTION` | `CHECK_COMMAND` | `DERATE` | 实测流量与参考流量偏差超限 |
| `POSITION_DEVIATION` | `WARNING` | `EXECUTION` | `CHECK_COMMAND` | `WARNING` | 位置结束型段中，目标位置与实测位置偏差超限 |
| `VELOCITY_DEVIATION` | `WARNING` | `EXECUTION` | `CHECK_COMMAND` | `WARNING` | 实测速度与参考速度偏差超限 |
| `SENSOR_FAULT` | `FAULT` | `SENSOR` | `CHECK_SENSOR` | `STOP` | 反馈值非有限数、压力为负、时间戳为负等 |
| `TIMESTAMP_ROLLBACK` | `FAULT` | `SENSOR` | `CHECK_SENSOR` | `STOP` | 反馈时间戳倒退 |
| `INTERNAL_ERROR` | `FAULT` | `INTERNAL` | `RESET_CONTROLLER` | `STOP` | 内部状态损坏或运行索引异常 |

### 偏差聚合优先级

当同一周期同时出现多个**执行偏差**时，`DIAGNOSTIC.code` 会按以下优先级保留主诊断码：

1. `TIMEOUT`
2. `OVER_PRESSURE`
3. `UNDER_PRESSURE`
4. `FLOW_DEVIATION`
5. `POSITION_DEVIATION`
6. `VELOCITY_DEVIATION`

补充说明：

- `SENSOR_FAULT` 与 `TIMESTAMP_ROLLBACK` 由主控流程前置判定，并直接按故障事件上报
- 上位机/HMI 建议同时读取 `DIAGNOSTIC.code` 与 `DIAGNOSTIC.flags`

## 诊断标志位摘要

`DIAGNOSTIC.flags` 是面向嵌入式/HMI 的位掩码，不保证与 `code` 一一对应。命令类、配方类、人工终止类事件通常可能只有 `code`，而没有任何 `flags` 置位。

| 标志位 | 十六进制 | 含义 |
| --- | --- | --- |
| `HDY_DIAG_FLAG_OVER_PRESSURE` | `0x01` | 当前周期检测到超压 |
| `HDY_DIAG_FLAG_UNDER_PRESSURE` | `0x02` | 当前周期检测到欠压 |
| `HDY_DIAG_FLAG_FLOW_DEVIATION` | `0x04` | 当前周期检测到流量偏差 |
| `HDY_DIAG_FLAG_POSITION_DEVIATION` | `0x08` | 当前周期检测到位置偏差 |
| `HDY_DIAG_FLAG_VELOCITY_DEVIATION` | `0x10` | 当前周期检测到速度偏差 |
| `HDY_DIAG_FLAG_TIMEOUT` | `0x20` | 当前周期检测到超时 |
| `HDY_DIAG_FLAG_SENSOR_FAULT` | `0x40` | 当前周期检测到反馈异常 |
| `HDY_DIAG_FLAG_TIMESTAMP_ROLLBACK` | `0x80` | 当前周期检测到时间戳倒退 |

## 当前自动化测试基线

当前 CTest 已纳入以下测试：

- `test_motion_planner`
- `test_motion_control`
- `test_recipe_validator`
- `test_pressure_controller`
- `test_pump_converter`
- `test_segment_completion`
- `test_rbf_pid`
- `test_ramp_controller`
- `test_scenario_matrix`

截至 **2026-04-16**，当前回归基线结果为：**9/9 测试通过**。

其中 `test_scenario_matrix` 覆盖典型注塑机场景矩阵，包括：

- 慢速合模
- 快速合模
- 低压保护
- 高压锁模
- 多段注射
- 多级保压
- 射退
- 开模
- 顶针
- 抽芯
- 长周期回归
- 最大段数边界回归

## 嵌入式适配特性

- 纯 C99
- 无动态内存分配
- 固定上限数组与有界诊断历史
- 统一 `HDY_` 前缀的数据模型与接口
- 适合在 PLC/嵌入式控制器的周期任务中调用
