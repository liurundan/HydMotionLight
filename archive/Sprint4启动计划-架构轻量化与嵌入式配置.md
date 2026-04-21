# Sprint 4 启动计划 - 架构轻量化与嵌入式配置

> 启动日期：2026-04-21
> Sprint周期：7天（计划）
> 总体目标：在保持对外接口稳定前提下，提升内部可维护性与平台可移植性

---

## 📋 执行摘要

**Sprint 4: 架构轻量化与嵌入式配置** 已在Sprint 3完成后立即启动。本Sprint的核心目标是继续收敛`motion_control`内部职责，必要时拆分为内部实现单元，同时引入统一平台配置头文件`hdy_config.h`，支持资源裁剪和平台适配。

**核心任务**：
- ✅ motion_control.c内部职责拆分（command/state/runtime/fault）
- ✅ 引入统一平台配置头文件hdy_config.h
- ✅ 支持配置项可裁剪（精度、时间基、诊断消息、历史深度等）
- ✅ 评估低资源平台上的精度与性能影响
- ✅ 更新文档和测试，确保对外接口稳定

---

## 🎯 任务拆分

### 任务1：motion_control.c内部职责拆分

**预计工期**: 3天
**优先级**: P1
**依赖**: Sprint 3已完成

**子任务**：
1.1 分析motion_control.c当前代码结构，识别职责边界
1.2 设计内部实现单元：
    - `motion_command.c/h`：命令入口、触发与参数校验
    - `motion_state.c/h`：状态转换与状态机规则
    - `motion_runtime.c/h`：周期运行时编排、planner/controller/converter调用
    - `motion_fault.c/h`：故障分流与安全停机
1.3 实现拆分后的模块
1.4 更新motion_control.c，使用拆分后的模块
1.5 运行测试，确保无行为回归

**验收标准**：
- motion_control.c代码量减少30%以上
- 各子模块职责清晰，接口简洁
- 所有现有测试通过
- 对外接口完全兼容

---

### 任务2：引入统一平台配置头文件hdy_config.h

**预计工期**: 1.5天
**优先级**: P2
**依赖**: 任务1完成

**子任务**：
2.1 设计hdy_config.h结构
2.2 实现配置项：
    - `HDY_REAL`精度策略（float/double切换）
    - 时间基类型（HDY_TIME）
    - 诊断消息字符串开关（HDY_ENABLE_DIAGNOSTIC_MESSAGE）
    - 历史深度（HDY_DIAG_HISTORY_DEPTH）
    - 最大段数（HDY_MAX_SEGMENTS）
    - 最大诊断历史深度（HDY_DIAG_HISTORY_DEPTH）
    - 最大段名称长度（HDY_NAME_MAX）
    - 最大诊断消息长度（HDY_MESSAGE_MAX）
2.3 更新common_types.h，使用hdy_config.h中的配置
2.4 更新所有模块，使用统一的配置项
2.5 编写配置说明文档

**验收标准**：
- hdy_config.h结构清晰，易于配置
- 所有配置项有合理默认值
- 配置变更不影响核心功能
- 文档完整说明各配置项的含义和影响

---

### 任务3：支持资源裁剪和平台适配

**预计工期**: 1.5天
**优先级**: P2
**依赖**: 任务2完成

**子任务**：
3.1 评估pump_converter、pressure_controller在低资源平台上的精度与性能影响
3.2 支持裁剪诊断消息字符串（减少ROM占用）
3.3 支持裁剪诊断历史深度（减少RAM占用）
3.4 支持裁剪最大段数（减少RAM占用）
3.5 编写资源裁剪说明文档
3.6 测试不同配置下的内存占用和性能

**验收标准**：
- 关键资源上限可集中配置
- 诊断消息裁剪后功能正常（只是消息不可用）
- 历史深度裁剪后功能正常（只是历史记录减少）
- 生成资源占用报告

---

### 任务4：测试与文档更新

**预计工期**: 1天
**优先级**: P1
**依赖**: 任务1-3完成

**子任务**：
4.1 运行全量测试套件，确保无回归
4.2 更新README.md，说明配置机制
4.3 更新Sprint 4完成报告
4.4 生成架构轻量化说明文档
4.5 生成嵌入式适配说明文档

**验收标准**：
- 所有测试通过
- 文档与代码一致
- 配置使用示例清晰

---

## 📊 进度跟踪

|| 任务 | 预计工期 | 实际工期 | 状态 | 完成度 |
||------|---------|---------|------|--------|
|| 任务1：motion_control.c内部职责拆分 | 3天 | - | ⏸️ 待开始 | 0% |
|| 任务2：引入统一平台配置头文件hdy_config.h | 1.5天 | - | ⏸️ 待开始 | 0% |
|| 任务3：支持资源裁剪和平台适配 | 1.5天 | - | ⏸️ 待开始 | 0% |
|| 任务4：测试与文档更新 | 1天 | - | ⏸️ 待开始 | 0% |
|| **总计** | **7天** | **-** | ⏸️ 待开始 | **0%** |

---

## 🏗️ 架构设计

### 当前motion_control.c的问题分析

**代码统计**：
- 文件大小：1533行
- 职责混合：命令处理、状态机、运行时编排、故障处理全部耦合

**主要问题**：
1. **职责不清**：单一文件承担太多职责
2. **可维护性差**：修改一个功能可能影响其他功能
3. **测试困难**：难以单独测试某个逻辑
4. **可扩展性差**：添加新功能需要修改主文件

### 拆分方案

#### 方案A：内部静态函数拆分（推荐）
- 在motion_control.c内部，通过注释和函数分组明确职责
- 优点：对外接口完全不变，改动最小
- 缺点：仍然是单一文件

#### 方案B：内部模块拆分
- 创建motion_command.c/h、motion_state.c/h等内部模块
- motion_control.c作为编排器，调用各模块
- 优点：职责清晰，易于维护
- 缺点：需要增加内部头文件，编译复杂度略增

#### 方案C：完全拆分（不推荐）
- 将所有功能完全拆分到独立模块
- motion_control.c只做最小化编排
- 优点：最大程度的模块化
- 缺点：对外接口可能变化，回归风险高

**推荐采用方案B**：在保持对外接口稳定的前提下，实现内部模块化。

### 模块职责定义

#### motion_command模块
- 职责：命令入口、触发与参数校验
- 接口：
  - `HDY_Command_ValidateStartRequest()`
  - `HDY_Command_ValidateNextRequest()`
  - `HDY_Command_QueuePendingCommand()`
  - `HDY_Command_ConsumePendingCommand()`
  - `HDY_Command_SampleStartInput()`

#### motion_state模块
- 职责：状态转换与状态机规则
- 接口：
  - `HDY_State_ResolveEffectiveState()`
  - `HDY_State_IsCommandAllowedInState()`
  - `HDY_State_TransitionToStarting()`
  - `HDY_State_TransitionToRunning()`
  - `HDY_State_TransitionToSegmentComplete()`
  - `HDY_State_TransitionToHold()`
  - `HDY_State_TransitionToDone()`
  - `HDY_State_TransitionToAborted()`
  - `HDY_State_TransitionToFault()`

#### motion_runtime模块
- 职责：周期运行时编排、planner/controller/converter调用
- 接口：
  - `HDY_Runtime_ExecuteRunning()`
  - `HDY_Runtime_PrimeControllers()`
  - `HDY_Runtime_UpdateReferences()`
  - `HDY_Runtime_CallPlanner()`
  - `HDY_Runtime_CallPressureController()`
  - `HDY_Runtime_CallPumpConverter()`

#### motion_fault模块
- 职责：故障分流与安全停机
- 接口：
  - `HDY_Fault_ReportSensorFault()`
  - `HDY_Fault_ReportTimestampRollback()`
  - `HDY_Fault_ReportInternalError()`
  - `HDY_Fault_EnterFaultStop()`
  - `HDY_Fault_ApplySafeOutputs()`

---

## ⚠️ 风险与注意事项

### 技术风险

1. **行为回归风险**
   - 风险：拆分过程中可能引入行为变化
   - 缓解：充分测试，保持对外接口稳定
   - 状态：⚠️ 中等风险

2. **编译兼容性风险**
   - 风险：新增模块可能导致编译问题
   - 缓解：逐模块验证，保持CMakeLists.txt更新
   - 状态：⚠️ 低风险

3. **性能影响风险**
   - 风险：模块化可能增加函数调用开销
   - 缓解：使用static inline函数，测量性能影响
   - 状态：⚠️ 低风险

### 管控要求

- 每次拆分模块前，先补充单元测试
- 保持对外接口完全兼容
- 所有配置项变更必须有文档说明
- 不得降低测试覆盖率

---

## 🚀 下一步行动

### 立即执行

1. ✅ **分析motion_control.c当前代码结构**
   - 统计各功能模块的代码行数
   - 识别职责边界
   - 绘制调用关系图

2. ✅ **设计内部模块接口**
   - 定义各模块的职责边界
   - 设计清晰的接口
   - 编写接口文档

3. ✅ **实现motion_command模块**
   - 提取命令相关代码
   - 创建motion_command.c/h
   - 编写单元测试

4. ✅ **实现motion_state模块**
   - 提取状态机相关代码
   - 创建motion_state.c/h
   - 编写单元测试

5. ✅ **实现motion_runtime模块**
   - 提取运行时编排相关代码
   - 创建motion_runtime.c/h
   - 编写单元测试

6. ✅ **实现motion_fault模块**
   - 提取故障处理相关代码
   - 创建motion_fault.c/h
   - 编写单元测试

7. ✅ **重构motion_control.c**
   - 使用拆分后的模块
   - 简化主文件逻辑
   - 运行全量测试

8. ✅ **引入hdy_config.h**
   - 设计配置结构
   - 实现配置项
   - 更新所有模块

9. ✅ **测试与文档更新**
   - 运行全量测试
   - 更新文档
   - 生成报告

---

## 📝 文件清单

### 新增文件
```
include/
├── hdy_config.h                    # 平台配置头文件
└── motion_internal/               # 内部模块目录
    ├── motion_command.h           # 命令处理模块
    ├── motion_state.h             # 状态机模块
    ├── motion_runtime.h           # 运行时编排模块
    └── motion_fault.h            # 故障处理模块

src/
└── motion_internal/               # 内部模块实现
    ├── motion_command.c
    ├── motion_state.c
    ├── motion_runtime.c
    └── motion_fault.c
```

### 修改文件
```
src/motion_control.c              # 重构，使用内部模块
include/common_types.h            # 使用hdy_config.h配置
include/motion_control.h          # 保持不变（对外接口）
CMakeLists.txt                    # 添加新的源文件
README.md                         # 更新配置说明
```

### 文档文件
```
Sprint4启动计划-架构轻量化与嵌入式配置.md    # 本文档
Sprint4完成总结-架构轻量化与嵌入式配置.md    # 完成总结
Sprint4架构评审-模块化设计.md                 # 架构评审报告
```

---

## 🎉 预期成果

**Sprint 4 完成后**：

- ✅ motion_control.c代码量减少30%以上
- ✅ 内部职责清晰，模块化程度高
- ✅ 引入统一平台配置机制
- ✅ 支持资源裁剪和平台适配
- ✅ 对外接口完全兼容
- ✅ 所有测试通过
- ✅ 文档完整且与代码一致

**质量指标**：
- **测试通过率**: 100%
- **编译警告数**: 0
- **代码规范符合度**: > 95%
- **代码复杂度**: 降低30%以上
- **可维护性**: 显著提升

---

**Sprint 4 启动日期**: 2026-04-21
**计划工期**: 7天
**状态**: ⏸️ 准备中
**预期完成**: 2026-04-28

🚀 **立即开始执行！** 🚀
