# Sprint 3 启动计划 - 诊断与保护工业化

> Sprint 3 周期：2026-05-04 ~ 2026-05-10  
> 启动日期：2026-04-21  
> 前置条件：✅ Sprint 2 已收口完成  
> 状态：🚀 准备启动

---

## 📋 Sprint 3 目标

### 核心目标

把"误差监视"提升为"现场可用的告警/故障判据体系"。

### 具体目标

1. **诊断分层改造**
   - 将当前诊断模型拆分为：
     - **监视层**（实时误差与状态采样）
     - **判据层**（持续时间、滞回、抑制条件、严重级别、恢复条件）

2. **误报率降低**
   - 增加启动阶段、段切换阶段、闭环建立阶段的误报抑制逻辑
   - 目标：降低误报率 60%+

3. **告警/故障分级**
   - 明确哪些诊断属于"告警"（WARNING）
   - 明确哪些诊断属于"停机故障"（FAULT）
   - 建立告警到故障的升级机制

4. **HMI映射一致性**
   - 梳理 `HMI诊断对照表.md` 与 `DIAGNOSTIC.code/severity/flags/protectionAction` 的一致性
   - 确保HMI可正确显示所有诊断事件

---

## ✅ Sprint 2 已完成的基础

### 输入条件（已满足）

1. ✅ **显式状态机稳定**
   - 所有状态转换规则明确
   - 命令合法性矩阵完整
   - 状态迁移路径可追踪

2. ✅ **故障路径统一**
   - 统一到 `HDY_StateReporter_ReportFault`
   - 故障状态落点明确
   - 诊断保留机制完整

3. ✅ **诊断数据结构完整**
   - DIAGNOSTIC（实时诊断）
   - DIAGNOSTIC_LATCH（诊断锁存）
   - LAST_DIAGNOSTIC_SNAPSHOT（诊断快照）
   - LAST_FAULT_SNAPSHOT（故障快照）
   - DIAGNOSTIC_HISTORY（诊断历史）

4. ✅ **测试覆盖充分**
   - 50+个测试用例
   - 覆盖命令合法性、状态迁移、故障处理
   - 回归基线稳定

---

## 🎯 Sprint 3 主要任务

### 任务1：诊断分层架构设计（预计3天）

#### 1.1 监视层设计

**新增文件**：`src/diagnostics_monitor.c`

**职责**：
- 实时采样位置/速度/流量/压力误差
- 计算偏差统计（最大值、最小值、平均值、标准差）
- 跟踪误差持续时间
- 记录误差变化趋势

**数据结构**：
```c
typedef struct {
    HDY_REAL positionError;
    HDY_REAL velocityError;
    HDY_REAL flowError;
    HDY_REAL pressureError;
    
    HDY_TIME errorStartTime;
    HDY_TIME errorDuration;
    HDY_BOOL errorActive;
    
    HDY_REAL maxError;
    HDY_REAL minError;
    HDY_REAL avgError;
    HDY_UINT sampleCount;
} HDY_ErrorMonitor;
```

**接口**：
```c
void HDY_ErrorMonitor_Init(HDY_ErrorMonitor* monitor);
void HDY_ErrorMonitor_Update(HDY_ErrorMonitor* monitor,
                             const HDY_AxisRef* axisRef,
                             const HDY_ExecutionReference* references,
                             HDY_TIME currentTime);
```

#### 1.2 判据层设计

**新增文件**：`src/diagnostics_criteria.c`

**职责**：
- 判断误差是否超过阈值
- 检查持续时间是否满足
- 应用滞回逻辑
- 应用抑制条件
- 确定诊断严重级别
- 决定保护动作

**数据结构**：
```c
typedef struct {
    // 基础阈值
    HDY_REAL baseThreshold;
    
    // 判据参数
    HDY_TIME debounceTime;          // 持续时间阈值
    HDY_REAL hysteresisRatio;       // 滞回比例(0~1)
    
    // 抑制条件
    HDY_BOOL enableStartupSuppress;  // 启动阶段抑制
    HDY_TIME startupSuppressTime;    // 启动抑制时长
    HDY_BOOL enableSwitchSuppress;   // 切段阶段抑制
    HDY_TIME switchSuppressTime;     // 切段抑制时长
    HDY_BOOL enableLoopBuildSuppress; // 闭环建立抑制
    HDY_TIME loopBuildSuppressTime;  // 闭环建立时长
    
    // 诊断配置
    HDY_DiagnosticCode diagnosticCode;
    HDY_DiagnosticSeverity severity;
    HDY_ProtectionAction protectionAction;
} HDY_DiagnosticCriteria;
```

**接口**：
```c
HDY_BOOL HDY_DiagnosticCriteria_Check(HDY_DiagnosticInfo* diagnostic,
                                     const HDY_ErrorMonitor* monitor,
                                     const HDY_DiagnosticCriteria* criteria,
                                     HDY_BOOL isStartupPhase,
                                     HDY_BOOL isSwitchPhase,
                                     HDY_TIME segmentElapsedTime);
```

### 任务2：误报抑制逻辑实现（预计2天）

#### 2.1 启动阶段抑制

**逻辑**：
- 段启动后的 `startupSuppressTime` 内，不触发诊断
- 适用于：位置偏差、速度偏差、流量偏差

**代码位置**：`diagnostics_criteria.c`

```c
HDY_BOOL HDY_IsStartupSuppressActive(HDY_TIME segmentElapsedTime, HDY_TIME suppressTime) {
    return (segmentElapsedTime < suppressTime) ? true : false;
}
```

#### 2.2 切段阶段抑制

**逻辑**：
- 段切换后的 `switchSuppressTime` 内，不触发诊断
- 适用于：压力偏差、流量偏差

**代码位置**：`diagnostics_criteria.c`

```c
HDY_BOOL HDY_IsSwitchSuppressActive(HDY_BOOL segmentChanged, HDY_TIME segmentElapsedTime, HDY_TIME suppressTime) {
    if (!segmentChanged) return false;
    return (segmentElapsedTime < suppressTime) ? true : false;
}
```

#### 2.3 闭环建立抑制

**逻辑**：
- 压力闭环建立后的 `loopBuildSuppressTime` 内，降低诊断敏感度
- 适用于：压力偏差

**代码位置**：`diagnostics_criteria.c`

```c
HDY_REAL HDY_CalculateLoopBuildFactor(HDY_TIME loopBuildTime, HDY_TIME suppressTime) {
    if (loopBuildTime >= suppressTime) return 1.0;
    return loopBuildTime / suppressTime;  // 线性递增
}
```

### 任务3：告警/故障分级策略（预计1天）

#### 3.1 诊断分级定义

**INFO级别**：
- 段完成、段启动等正常事件
- 不影响控制，仅用于日志

**WARNING级别**：
- 轻微偏差，但仍在可接受范围
- 不触发保护动作，仅提示

**FAULT级别**：
- 严重偏差，需要立即停机
- 触发 `HDY_PROTECTION_ACTION_STOP`

#### 3.2 告警到故障升级

**升级逻辑**：
- WARNING 持续超过 `faultEscalationTime` → 升级为 FAULT
- FAULT 触发后进入 `HDY_FB_STATE_FAULT`

**代码位置**：`state_reporter.c`

### 任务4：HMI映射一致性检查（预计1天）

#### 4.1 诊断码对照表更新

**文件**：`HMI诊断对照表.md`

**更新内容**：
- 新增启动抑制相关诊断码
- 新增切段抑制相关诊断码
- 新增闭环建立相关诊断码
- 更新所有诊断码的严重级别和保护动作

#### 4.2 代码与文档一致性验证

**检查项**：
- `include/diagnostics.h` 中的诊断码定义与 HMI 文档一致
- 诊断码的 `severity` 字段与 HMI 文档一致
- 诊断码的 `protectionAction` 字段与 HMI 文档一致

---

## 📁 文件结构

### 新增文件

```
src/
├── diagnostics_monitor.c      # 监视层
└── diagnostics_criteria.c      # 判据层

include/
├── diagnostics_monitor.h
└── diagnostics_criteria.h
```

### 修改文件

```
src/
├── motion_control.c           # 集成监视层和判据层
├── diagnostics.c              # 保留诊断保留机制
└── state_reporter.c           # 保留故障上报

include/
└── diagnostics.h              # 增加监视层/判据层接口

HMI诊断对照表.md                # 更新诊断码对照表

tests/
├── test_diagnostic_criteria.c # 新增判据层测试
└── test_monitor.c             # 新增监视层测试
```

---

## 🧪 测试策略

### 新增测试

#### 1. 监视层测试（`test_monitor.c`）

```c
void test_error_monitor_tracks_position_error();
void test_error_monitor_tracks_velocity_error();
void test_error_monitor_tracks_flow_error();
void test_error_monitor_tracks_pressure_error();
void test_error_monitor_calculates_statistics();
void test_error_monitor_tracks_duration();
```

#### 2. 判据层测试（`test_diagnostic_criteria.c`）

```c
void test_diagnostic_criteria_debounce_time();
void test_diagnostic_criteria_hysteresis();
void test_diagnostic_criteria_startup_suppression();
void test_diagnostic_criteria_switch_suppression();
void test_diagnostic_criteria_loop_build_suppression();
void test_diagnostic_criteria_warning_to_fault_escalation();
```

#### 3. 集成测试（扩展 `test_motion_control.c`）

```c
void test_diagnostic_false_positive_reduction();
void test_diagnostic_false_negative_prevention();
void test_diagnostic_severity_mapping();
void test_diagnostic_protection_action_consistency();
```

---

## 📊 验收标准

### 功能验收

- [ ] 监视层正确采样和计算误差统计
- [ ] 判据层正确应用持续时间、滞回、抑制条件
- [ ] 误报率降低 60%+（对比当前版本）
- [ ] 告警/故障分级策略明确
- [ ] HMI文档与代码输出字段一致

### 测试验收

- [ ] 所有新增测试通过
- [ ] 原有回归测试保持通过
- [ ] 误报抑制效果实测验证

### 文档验收

- [ ] 诊断监视层设计文档
- [ ] 诊断判据层设计文档
- [ ] 误报抑制策略说明
- [ ] HMI诊断映射更新
- [ ] Sprint 3 完成报告

---

## ⏱️ 时间安排

| 任务 | 预计工期 | 负责人 | 风险 |
|------|---------|--------|------|
| 监视层设计与实现 | 3天 | - | 低 |
| 判据层设计与实现 | 2天 | - | 中 |
| 误报抑制逻辑实现 | 2天 | - | 中 |
| 告警/故障分级策略 | 1天 | - | 低 |
| HMI映射一致性检查 | 1天 | - | 低 |
| 测试编写与验证 | 2天 | - | 中 |
| 文档编写 | 1天 | - | 低 |
| **总计** | **12天** | - | - |

---

## 🚨 风险与注意事项

### 技术风险

1. **误报抑制过度**
   - 风险：可能导致真正的异常被抑制
   - 缓解：充分测试，保留关键故障的快速响应

2. **判据层性能影响**
   - 风险：增加计算负担，影响实时性
   - 缓解：优化算法，必要时可配置裁剪

3. **HMI映射不一致**
   - 风险：代码与文档不同步，导致HMI显示错误
   - 缓解：建立自动化检查脚本

### 管理要求

- 每次修改诊断码或字段前，同步更新HMI映射
- 每次调整判据参数前，先补测试验证
- 不得降低关键故障的诊断响应速度
- 保持Sprint 2建立的状态机稳定性

---

## 📝 成功指标

### 定量指标

- 误报率降低：≥ 60%
- 漏报率：≤ 5%
- 诊断响应时间：< 100ms（关键故障）
- 诊断准确率：≥ 95%

### 定性指标

- 现场调试人员反馈"告警噪声明显减少"
- HMI诊断显示与现场实际一致
- 故障诊断可追溯，支持根因分析

---

## 🎯 后续衔接

### Sprint 4 准备

Sprint 3 完成后，将为 Sprint 4 提供以下基础：

1. **诊断体系稳定**
   - 监视层/判据层架构稳定
   - 可支持后续扩展

2. **误报抑制经验**
   - 为Sprint 4的架构轻量化提供参考
   - 为Sprint 5的测试矩阵提供输入

3. **HMI映射清晰**
   - 为Sprint 6的发布候选提供文档基础

---

**Sprint 3 启动日期**: 2026-04-21  
**预计完成日期**: 2026-05-10  
**状态**: 🚀 准备启动
