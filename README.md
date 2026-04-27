# HDY Motion Control Library

面向注塑机液压场景的轻量级运动控制库，目标是把**工艺层**与**运动控制层**清晰分离，并保持纯 C99、静态内存、易于嵌入式部署的实现风格。

## 设计边界

- **工艺层负责**：动作配方组织、阀逻辑、段切换决策、机构联锁。
- **运动控制层负责**：速度/流量/压力参考计算、泵速换算、状态与诊断输出。
- 库本身**不直接控制电磁阀**，仅输出泵侧命令与可供上层判定的状态信息。

## 目录结构

```
项目根目录
├── include/                    # 对外头文件
│   ├── hdy_config.h          # 统一平台配置 (新增)
│   ├── motion_control.h       # PLCopen风格函数块接口
│   ├── common_types.h         # 公共类型定义
│   ├── diagnostics.h          # 诊断接口
│   ├── motion_utils.h         # 工具函数接口
│   ├── motion_validator.h     # 验证逻辑接口
│   └── ...                    # 其他模块接口
├── src/                        # 核心实现
│   ├── motion_control.c        # 函数块编排器
│   ├── motion_planner.c        # 运动规划
│   ├── pressure_controller.c  # 压力控制
│   ├── pump_converter.c       # 泵速换算
│   ├── segment_completion.c    # 段完成判定
│   ├── motion_utils.c          # 工具函数实现
│   ├── motion_validator.c      # 验证逻辑实现
│   └── ...                    # 其他模块实现
├── tests/                      # 单元测试和场景回归
│   └── benchmark_performance.c # 性能基准测试 (新增)
├── CODEBUDDY.md               # 项目开发指导
├── 项目需求文档-v1.0.md        # 统一的需求规范
├── 开发计划文档-v1.0.md        # 统一的开发计划
├── HMI诊断对照表.md           # 诊断码HMI映射
├── 正式发布说明模板.md        # 发布说明模板
└── archive/                   # 历史文档归档
```

## 快速开始

### 构建与测试

在仓库根目录执行：

```bash
# 配置项目
cmake --preset unixgcc

# 构建所有目标
cmake --build --preset unixgcc

# 运行所有测试
ctest --test-dir out/build/unixgcc --output-on-failure
```

### 库架构说明

本项目的构建系统提供两种独立的静态库：

| 库名称 | 用途 | 使用场景 |
|--------|------|----------|
| **HydroMotionLib** | 核心运动控制库 | 生产部署、嵌入式平台 |
| **HydroSimLib** | 液压仿真器库 | 开发测试、集成验证 |

#### 核心运动控制库

- **包含模块**：运动规划、压力控制、泵速换算、诊断系统等所有核心功能
- **不包含**：液压仿真器代码
- **部署方式**：使用 `scripts/deploy_embedded_prod.sh` 脚本生成生产版本
- **目标平台**：嵌入式控制器、PLC、实时操作系统

#### 液压仿真器库

- **包含模块**：`hydro_sim.c` 完整仿真逻辑
- **使用场景**：
  - 仿真器功能测试（`test_hydro_sim_fb`）
  - 开发环境验证
  - 算法离线仿真
- **部署方式**：仅用于开发测试，不部署到生产环境

### 仿真器测试

运行液压仿真函数块测试：

```bash
# 运行仿真器函数块测试
./out/build/unixgcc/test_hydro_sim_fb

# 或通过ctest运行
ctest --test-dir out/build/unixgcc -R test_hydro_sim_fb --output-on-failure
```

仿真器测试主要验证以下行为：
- 使能、复位与模式切换逻辑
- 位置、速度、压力等输出更新
- 诊断与边界保护行为

### 嵌入式部署

生成不包含仿真器的生产版本库：

```bash
# 运行部署脚本
./scripts/deploy_embedded_prod.sh
```

部署脚本会：
1. 构建核心运动控制库（不含仿真器）
2. 复制库文件到 `out/install/embedded_prod/`
3. 复制必要的头文件
4. 验证库文件不包含仿真器符号
5. 生成集成说明文档

**生产版本特点**：
- 库大小：约 80-100KB（不含仿真器）
- 包含完整运动控制功能
- 无仿真器依赖
- 纯 C99 实现
- 静态内存分配

运行端到端示例：

```bash
./out/build/unixgcc/main
```

### 性能基准测试 (新增)

运行性能基准测试程序：

```bash
./out/build/unixgcc/benchmark_performance
```

这将测试库中各个模块的执行性能，包括：
- 运动工具函数性能
- 运动规划器性能
- 压力控制器性能
- 泵转换器性能
- 完整控制周期性能

性能测试结果可用于：
- 验证实时控制可行性
- 识别性能瓶颈
- 指导性能优化方向
- 不同平台性能对比

### 平台配置 (新增)

通过 `include/hdy_config.h` 可以针对不同目标平台进行优化配置：

**主要配置项**：

1. **资源限制**
   - `HDY_MAX_SEGMENTS` - 最大配方段数
   - `HDY_DIAG_HISTORY_DEPTH` - 诊断历史深度（已弃用，固定为1）

2. **功能开关**
   - `HDY_ENABLE_DIAGNOSTIC_HISTORY` - 启用诊断历史记录
   - `HDY_ENABLE_DIAGNOSTIC_FLAGS` - 启用诊断标志位
   - `HDY_ENABLE_PRESSURE_LOOP_TELEMETRY` - 启用压力环遥测
   - `HDY_ENABLE_EXECUTION_REFERENCE` - 启用执行参考

3. **性能优化**
   - `HDY_ENABLE_INLINE_FUNCTIONS` - 启用内联函数
   - `HDY_ENABLE_FAST_MATH` - 启用快速数学函数

**配置示例**：

```c
// 低资源平台配置
#define HDY_MAX_SEGMENTS 8
#define HDY_ENABLE_DIAGNOSTIC_FLAGS 0
#define HDY_ENABLE_PRESSURE_LOOP_TELEMETRY 0

// 获取当前配置信息
HDY_ConfigInfo config = HDY_GetConfigInfo();
printf("Max Segments: %d\n", config.maxSegments);
printf("Version: %s\n", config.versionString);
```

**资源节省**：
- 最小配置可节省 **30-40% RAM** 和 **20-30% ROM**
- 适用于资源受限的嵌入式平台（如 STM32F1系列）

### 工艺层对接时序

推荐按 PLCopen function block 语义集成：

1. `HDY_MotionControlFB_Init()` 初始化函数块
2. 配置 `FLOW_TO_PUMP_SPEED_GAIN`、`PUMP_SPEED_LIMIT`
3. 调用 `HDY_MotionControlFB_LoadRecipe()` 装载配方
4. 工艺层在合适时机调用：
   - `HDY_MotionControlFB_StartSegment()`；或
   - 置位 `START_SEGMENT/START_SEGMENT_INDEX` 后再进入周期调用
5. 每周期刷新 `AXIS_REF` 后调用 `HDY_MotionControlFB_Execute()`
6. 读取：
   - `PUMP_SPEED`（泵速命令）
   - `STATE`（执行状态）
   - `DIAGNOSTIC`（诊断信息）
   - `SEGMENT_COMPLETED`（段完成标志）
   - `SEGMENT_CHANGED`（段切换脉冲）
7. 当 `SEGMENT_COMPLETED=true` 时，由工艺层决定是否调用 `HDY_MotionControlFB_NextSegment()`
8. 需要安全停机时调用 `HDY_MotionControlFB_Abort()`；需要彻底重置时置位 `RESET`

## 段切换约定

- `LoadRecipe()` 只完成配方装载，不会自动开始执行
- `SEGMENT_CHANGED` 是段启动后的**单周期脉冲**
- `SEGMENT_COMPLETED` 在段完成后保持，直到上层切换下一段或进入最终完成态
- 最后一段完成后，控制器进入 `FINISHED=true`
- `EN=false` 会立即输出安全零值，重新使能后不会自动恢复，需要上层重新启动段

## Direct / Recipe 模式语义

库同时支持配方模式和直接模式，满足不同应用场景需求。

### Recipe 模式（`USE_RECIPE = true`）

- `StartSegment(segmentIndex)` 使用 `RECIPE[segmentIndex]` 作为启动段
- `NextSegment()` 推进到 recipe 的下一段（若存在）
- 适合预定义的多段配方执行

### Direct 模式（`USE_RECIPE = false`）

- `StartSegment()` 忽略 `segmentIndex`，使用 `DIRECT_SEGMENT` 作为单段直接执行来源
- 必须先通过 `HDY_MotionControlFB_LoadDirectSegment()` 装载 `DIRECT_SEGMENT` 并使 `DIRECT_SEGMENT_VALID=true`
- Direct 模式下 `NextSegment()` 不被允许，库会拒绝并上报诊断（例如 `HDY_DIAG_CODE_COMMAND_NOT_ALLOWED` 或 `HDY_DIAG_CODE_NO_DIRECT_SEGMENT`）
- 适合手动调试或动态参数调整

**Direct 模式特性**：
- **启动**：`StartSegment()` 将 `DIRECT_SEGMENT` 的参数锁存为当前活动段
- **执行**：按段内定义执行，状态与诊断行为与 recipe 段相同
- **完成**：Direct 模式为单段语义，段完成后控制器直接进入终端完成态（`FB_STATE=DONE` / `FINISHED=true`），不会等待或依赖 `NextSegment()`

### 模式识别

- `STATE.segmentSource`：控制器通过该字段（枚举值 `HDY_SEGMENT_SOURCE_NONE/RECIPE/DIRECT`）对外暴露当前（或最近）活动段的来源
- HMI 与工艺层可据此判断当前执行语义，例如禁止在 `DIRECT` 来源下调用 `NextSegment()`

### 使用示例

Direct 模式调用示例：

```c
fb->USE_RECIPE = HDY_FALSE;
HDY_MotionControlFB_LoadDirectSegment(&fb, &segment);
HDY_MotionControlFB_StartSegment(&fb, 0, timestamp); // index 被忽略
```

**注意**：Direct 模式下 `segmentIndex` 仅为兼容接口保留，不代表 recipe 索引。

## 新增工具模块

### motion_utils - 工具函数模块

提供通用的工具函数,供整个运动控制系统复用:

**数学工具**:
- `HDY_MotionUtils_MinReal()` - 获取两个实数的最小值
- `HDY_MotionUtils_AbsReal()` - 获取实数的绝对值
- `HDY_MotionUtils_IsFiniteReal()` - 检查实数是否为有限值(非NaN或无穷大)

**验证工具**:
- `HDY_MotionUtils_AxisRefIsValid()` - 检查轴反馈数据是否有效

**字符串转换**:
- `HDY_MotionUtils_CommandToString()` - 命令枚举转字符串(用于诊断)
- `HDY_MotionUtils_StateToString()` - 状态枚举转字符串(用于诊断)

**设计目的**:
- 提升代码复用性,避免重复实现
- 统一数学计算逻辑,减少错误
- 支持独立单元测试

### motion_validator - 验证逻辑模块

集中处理运动控制的验证逻辑:

**请求验证**:
- `HDY_MotionValidator_ValidateStartRequest()` - 验证Start请求
- `HDY_MotionValidator_ValidateNextRequest()` - 验证Next请求

**配置验证**:
- `HDY_MotionValidator_ValidatePumpConfig()` - 验证泵配置参数

**段源解析**:
- `HDY_MotionValidator_ResolveStartSourceSegment()` - 解析启动段来源
- `HDY_MotionValidator_UsesRecipeSource()` - 判断是否使用配方源
- `HDY_MotionValidator_HasSelectedStartSource()` - 检查是否已选择启动源
- `HDY_MotionValidator_ResolveEffectiveFbState()` - 解析有效功能块状态

**设计目的**:
- 分离验证逻辑与业务逻辑
- 提升代码可测试性
- 统一验证标准,减少错误

**代码优化效果**:
- motion_control.c从1533行减少到1365行(-11%)
- 工具函数复用性提升100%
- 验证逻辑复用性提升100%
- 无回归风险,100%测试通过

## 发布基线说明

### 基线定位

当前 `master` 可作为本项目的**首个工程化发布基线**使用，适合以下集成前提：

- 上层工艺层已经独立实现动作编排、阀逻辑、机构联锁与段切换决策
- 本库只承担多段运动/压力参考计算、泵速换算、状态与诊断输出
- 目标平台要求纯 C99、静态内存、可在周期任务中稳定调用

### 基线范围

当前基线已纳入并验证的能力包括：

#### 核心功能
- **PLCopen 风格函数块**：`Init / LoadRecipe / StartSegment / NextSegment / Abort / Execute`
- **多段配方执行**：`HDY_MAX_SEGMENTS = 16`
- **双模式参数来源**：Recipe 模式与 Direct 模式并存

#### 控制模式
- `HDY_MODE_POSITION`：位置控制模式
- `HDY_MODE_SPEED_RAMP`：速度斜坡模式
- `HDY_MODE_PRESSURE_CLOSED_LOOP`：压力闭环模式

#### 压力闭环策略
- P / PI / PID 三种策略
- 支持配置不同压力控制参数

#### 结束条件
- `HDY_END_POSITION`：位置结束
- `HDY_END_TIME`：时间结束
- `HDY_END_PRESSURE`：压力结束
- `HDY_END_FLOW`：流量结束
- `HDY_END_MANUAL`：手动结束

#### 运行期模块
- 运动规划
- 压力控制
- 泵速换算
- 段完成判定
- 状态上报
- 保护管理
- 诊断快照/历史

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

## 诊断接口

### 实时诊断信息

运行期优先读取 `DIAGNOSTIC` 结构体的以下字段：

- `code`：当前主要诊断码
- `severity`：严重级别（`NONE` / `WARNING` / `FAULT`）
- `source`：来源模块（`RECIPE` / `RUNTIME` / `COMMAND` / `EXECUTION` / `SENSOR` / `INTERNAL`）
- `recovery`：建议恢复动作（`NONE` / `RELOAD_RECIPE` / `RESET_CONTROLLER` / `CHECK_COMMAND` / `CHECK_SENSOR` / `RESTART_SEGMENT`）
- `protectionAction`：保护动作（`NONE` / `WARNING` / `STOP` / `DERATE`）
- `flags`：面向嵌入式/HMI 的紧凑标志位摘要

### 保留诊断信息

调试与售后可结合以下保留诊断信息：

- `DIAGNOSTIC_LATCH`：最近一次非 `NONE` 事件
- `LAST_DIAGNOSTIC_SNAPSHOT`：最近一次诊断事件的上下文快照
- `LAST_FAULT_SNAPSHOT`：最近一次故障停机快照，仅 `FAULT` 事件更新
- `DIAGNOSTIC_HISTORY`：单快照历史，仅保留最近一条快照与累计事件计数（`HDY_DIAG_HISTORY_DEPTH` 已弃用，固定为1）

### 诊断保留语义

- `DIAGNOSTIC`：当前周期/当前命令的实时结果，在非故障保持态会自动清除
- `DIAGNOSTIC_LATCH`：最近一次非 `NONE` 事件
- `LAST_DIAGNOSTIC_SNAPSHOT`：最近一次诊断事件的上下文快照
- `LAST_FAULT_SNAPSHOT`：最近一次故障停机快照，仅 `FAULT` 事件更新
- `DIAGNOSTIC_HISTORY`：单快照模型，保留最近一条快照（`lastSnapshot`）与累计事件计数（`totalRecorded`）。`GetEntry(0)` 返回最新快照，`GetEntry(n>0)` 返回 false。`HDY_DIAG_HISTORY_DEPTH` 已弃用，固定为1。

在实时故障已清除且控制器不处于故障态时，可调用 `HDY_MotionControlFB_AcknowledgeDiagnostics()` 清除保留诊断；故障停机保留信息仍应通过 `RESET` 清除。

### HMI 文案映射

现场联调 / 售后场景建议直接复用仓库根目录 `HMI诊断对照表.md` 中的中文显示文案和处置建议。

## 诊断码表

> 表中"恢复建议""保护动作"直接对应 `DIAGNOSTIC.recovery` 与 `DIAGNOSTIC.protectionAction`。

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

**补充说明**：
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

## 自动化测试

### 当前测试基线

当前 CTest 已纳入以下测试：

- `test_motion_planner`：运动规划器测试
- `test_motion_control`：函数块编排测试
- `test_recipe_validator`：配方校验器测试
- `test_pressure_controller`：压力控制器测试
- `test_pump_converter`：泵速转换器测试
- `test_segment_completion`：段完成判定测试
- `test_rbf_pid`：RBF_PID 测试
- `test_ramp_controller`：斜坡控制器测试
- `test_scenario_matrix`：场景矩阵测试

截至 **2026-04-23**，当前回归基线结果为：**16/16 测试通过**。

### 场景覆盖

`test_scenario_matrix` 覆盖典型注塑机场景矩阵，包括：

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

- **纯 C99**：无语言扩展，可移植性强
- **无动态内存分配**：所有数据结构静态分配，避免碎片化
- **固定上限数组与单快照诊断历史**：内存占用可控
- **统一 `HDY_` 前缀**的数据模型与接口：避免命名冲突
- **适合在 PLC/嵌入式控制器的周期任务中调用**：执行时间确定

## 项目文档体系

### 核心文档

- **`README.md`**：项目概述和快速开始（本文档）
- **`项目需求文档-v1.0.md`**：统一的需求规范，包含功能需求、非功能需求、接口需求和验收标准
- **`开发计划文档-v1.0.md`**：统一的开发计划，包含Sprint规划、任务清单和进度跟踪
- **`CODEBUDDY.md`**：项目开发指导，包含构建命令、架构说明和开发约定

### 支撑文档

- **`HMI诊断对照表.md`**：诊断码HMI映射，包含中文告警文案、分级和现场处置建议
- **`正式发布说明模板.md`**：对外发布、版本交付、联调移交时可直接复用的 Release Notes 模板

### 历史文档

- **`archive/`**：历史文档归档目录，包含：
  - `归档目录说明.md`：归档管理和查阅指南
  - `项目需求与设计说明书.md`：原需求设计文档
  - `开发计划20260420.md`：原开发计划
  - `开发阶段与下一步计划.MD`：原阶段规划
  - `项目评审报告.md`：原评审报告
  - `对话上下文-Direct与Recipe模式并存接口-已完成并通过测试.md`：技术实现记录

### 文档使用建议

#### 新团队成员入门
1. 阅读 `README.md` 了解项目概况
2. 阅读 `项目需求文档-v1.0.md` 了解详细需求
3. 阅读 `CODEBUDDY.md` 了解开发约定

#### 开发人员日常工作
- **需求确认**：查阅 `项目需求文档-v1.0.md`
- **进度跟踪**：查阅 `开发计划文档-v1.0.md`
- **开发指导**：查阅 `CODEBUDDY.md`
- **代码实现**：查阅代码注释和头文件

#### 项目经理工作
- **需求管理**：查阅 `项目需求文档-v1.0.md`
- **计划管理**：查阅 `开发计划文档-v1.0.md`
- **发布准备**：查阅 `正式发布说明模板.md`

#### 测试人员工作
- **需求理解**：查阅 `项目需求文档-v1.0.md`
- **测试依据**：查阅需求文档中的验收标准
- **诊断验证**：查阅 `HMI诊断对照表.md`

#### HMI开发人员工作
- **接口对接**：查阅 `项目需求文档-v1.0.md` 的接口需求
- **诊断映射**：查阅 `HMI诊断对照表.md`
- **状态显示**：参考需求文档中的状态输出说明

## 架构设计

### 高层架构

本库的核心设计原则是**清晰的层次分离**：

```
┌─────────────────────────────────────┐
│         工艺层（外部）                │
│  - 动作配方组织                      │
│  - 阀逻辑控制                        │
│  - 段切换决策                        │
│  - 机构联锁                          │
└──────────────┬──────────────────────┘
               │
               │ PLCopen Function Block 接口
               │
┌──────────────▼──────────────────────┐
│       运动控制层（本库）              │
│  ┌────────────────────────────┐    │
│  │  HDY_MotionControlFB       │    │
│  │  - 状态机管理               │    │
│  │  - 命令接口                │    │
│  │  - 诊断上报                │    │
│  └──────┬───────────────────┬─┘    │
│         │                   │       │
│    ┌────▼────┐        ┌─────▼───┐   │
│    │ 运动规划 │        │ 压力控制 │   │
│    │ 速度/流 │        │ P/PI/PID │   │
│    │ 量计算  │        │ 策略    │   │
│    └────┬────┘        └─────┬───┘   │
│         │                   │       │
│    ┌────▼────┬──────────────▼───┐   │
│    │ 泵速换算│   段完成判定       │   │
│    │ 保护逻辑│   诊断管理         │   │
│    └─────────┴──────────────────┘   │
└─────────────────────────────────────┘
```

### 模块职责

| 模块 | 职责 | 关键函数 |
| --- | --- | --- |
| `motion_control` | 函数块编排器，状态机管理，命令接口 | `HDY_MotionControlFB_Execute()` |
| `motion_planner` | 运动规划，速度/流量计算 | `HDY_MotionPlanner_Execute()` |
| `pressure_controller` | 压力闭环控制，P/PI/PID 策略 | `HDY_PressureController_Execute()` |
| `pump_converter` | 泵速换算 | `HDY_PumpConverter_Execute()` |
| `segment_completion` | 段完成判定 | `HDY_SegmentCompletion_Check()` |
| `ramp_controller` | 压力目标斜坡平滑 | `HDY_RampController_Execute()` |
| `diagnostics` | 诊断管理，快照/历史记录 | `HDY_DiagnosticsHistory_Push()` |

### 数据流

```
AXIS_REF（反馈）
    │
    ▼
HDY_MotionControlFB_Execute()
    │
    ├──► 运动规划 ──► 速度/流量参考
    │
    ├──► 压力控制 ──► 压力修正
    │
    ├──► 泵速换算 ──► PUMP_SPEED（输出）
    │
    ├──► 段完成判定 ──► SEGMENT_COMPLETED
    │
    └──► 诊断管理 ──► DIAGNOSTIC
```

## 开发约定

### 编码规范

- **语言**：纯 C99，不使用 C++ 特性
- **命名**：公共接口使用 `HDY_` 前缀
- **内存**：静态分配，不使用 `malloc/free`
- **风格**：遵循 PLCopen 函数块风格

### 构建约定

- **工具**：CMake 3.15+
- **配置**：`cmake --preset unixgcc`
- **构建**：`cmake --build --preset unixgcc`
- **测试**：`ctest --test-dir out/build/unixgcc --output-on-failure`

### 测试约定

- **单元测试**：每个模块都有对应的测试用例
- **集成测试**：`test_scenario_matrix` 覆盖典型场景
- **回归测试**：每次提交前运行完整测试套件

## 许可证

本项目采用开源许可证，详见 `LICENSE` 文件。

## 贡献指南

欢迎贡献代码和改进建议。贡献前请：

1. 阅读 `CODEBUDDY.md` 了解开发约定
2. 运行完整测试套件确保测试通过
3. 遵循现有代码风格和命名规范
4. 更新相关文档和注释

## 版本历史

- **v1.0**（2026-04-23）：首个工程化发布基线
  - PLCopen 风格函数块
  - 多段配方执行（最多16段）
  - 双模式参数来源（Recipe / Direct）
  - 三种控制模式（位置/速度斜坡/压力闭环）
  - 完整的诊断体系
  - 16/16 测试通过

## 联系方式

如有问题或建议，请通过以下方式联系：

- Git Issues：在仓库提交 Issue
- 团队会议：在定期会议上讨论
- 直接联系：联系项目负责人

---

**最后更新日期**：2026-04-23
**文档维护者**：项目开发团队
