# Direct模式API使用说明

## 概述

Direct模式允许工艺层完全控制段切换，无需使用配方机制。工艺层通过`DIRECT_SEGMENT`字段直接提供段参数，运动控制库负责执行运动规划、压力控制和泵速转换。

## 主要API接口

### 1. 初始化与配置

```c
// 初始化功能块
void HDY_MotionControlFB_Init(HDY_MotionControlFB* fb);

// 配置Direct模式
fb.EN = true;                    // 使能功能块
fb.USE_RECIPE = false;          // 使用Direct模式
fb.FLOW_TO_PUMP_SPEED_GAIN = 1.2;  // 流量到泵速的转换增益
fb.PUMP_SPEED_LIMIT = 3000.0;   // 泵速限制
```

### 2. 段参数配置

```c
// 加载Direct段参数
HDY_BOOL HDY_MotionControlFB_LoadDirectSegment(
    HDY_MotionControlFB* fb, 
    const HDY_MotionSegment* segment
);

// 清除Direct段参数
void HDY_MotionControlFB_ClearDirectSegment(HDY_MotionControlFB* fb);
```

### 3. 执行控制

```c
// 启动段执行
HDY_BOOL HDY_MotionControlFB_StartSegment(
    HDY_MotionControlFB* fb, 
    size_t segmentIndex,  // 在Direct模式下忽略此参数
    HDY_TIME timestamp
);

// 周期性执行（推荐使用Execute）
void HDY_MotionControlFB_Execute(HDY_MotionControlFB* fb);

// 或者使用Scan（自动处理边沿触发）
void HDY_MotionControlFB_Scan(HDY_MotionControlFB* fb);
```

### 4. 状态监控

```c
// 主要状态输出
fb.ACTIVE              // 段是否在执行中
fb.SEGMENT_COMPLETED    // 段是否完成
fb.SEGMENT_CHANGED     // 段是否刚切换（单周期脉冲）
fb.FINISHED            // 所有段是否完成
fb.FAULT               // 是否故障
fb.PUMP_SPEED          // 泵速命令输出
fb.FB_STATE            // 功能块状态机状态

// 详细状态信息
fb.STATE.plannedVelocity         // 规划速度
fb.STATE.plannedFlow             // 规划流量
fb.STATE.commandedPumpSpeed      // 命令泵速
fb.STATE.currentSegmentTag       // 当前段标签（uint8_t，替代原currentSegmentName）
fb.STATE.pressureLoop            // 压力循环遥测数据（需启用HDY_ENABLE_PRESSURE_LOOP_TELEMETRY）
```

### 5. 诊断信息

```c
// 当前周期诊断
fb.DIAGNOSTIC.code              // 诊断代码
fb.DIAGNOSTIC.severity          // 严重程度
fb.DIAGNOSTIC.source            // 来源
fb.DIAGNOSTIC.pressureError     // 压力误差
fb.DIAGNOSTIC.flowError         // 流量误差

// 诊断保持信息
fb.DIAGNOSTIC_LATCH             // 保持的诊断事件
fb.LAST_DIAGNOSTIC_SNAPSHOT     // 最后诊断快照
fb.LAST_FAULT_SNAPSHOT          // 最后故障快照
```

## 调用链路

### 标准调用流程

```
工艺层PLC程序 (1ms周期)
    ↓
1. 初始化
    HDY_MotionControlFB_Init()
    配置参数
    ↓
2. 段配置
    HDY_MotionControlFB_LoadDirectSegment()
    ↓
3. 段启动
    HDY_MotionControlFB_StartSegment()
    ↓
4. 周期循环 (每个PLC周期)
    更新AXIS_REF反馈
    HDY_MotionControlFB_Execute()
    读取PUMP_SPEED输出
    检查SEGMENT_COMPLETED
    ↓
5. 段切换 (工艺层决策)
    检查SEGMENT_COMPLETED
    配置新段参数
    HDY_MotionControlFB_LoadDirectSegment()
    HDY_MotionControlFB_StartSegment()
    ↓
6. 重复步骤4-5
```

### 内部执行流程

```
HDY_MotionControlFB_Execute()
    ↓
1. 输入处理
    - 检查EN/RESET信号
    - 处理启动段信号
    - 更新时间戳
    ↓
2. 状态机处理
    - 解析有效状态
    - 检查命令合法性
    - 执行状态转换
    ↓
3. 压力斜坡控制
    - HDY_RampController_Execute()
    - 平滑压力目标变化
    ↓
4. 运动规划
    - HDY_MotionPlanner_Execute()
    - 根据控制模式计算速度/流量
    - 应用位置/时间约束
    ↓
5. 压力控制
    - HDY_PressureController_Execute()
    - P/PI/PID/RBF-PID策略
    - 前馈+反馈控制
    ↓
6. 泵速转换
    - HDY_PumpConverter_Execute()
    - 流量到泵速转换
    - 应用泵速限制
    ↓
7. 段完成检查
    - HDY_SegmentCompletion_CheckWithContext()
    - 检查位置/时间/压力/流量条件
    ↓
8. 诊断更新
    - 更新诊断信息
    - 检查保护动作
    ↓
9. 输出刷新
    - 更新STATE结构
    - 刷新状态输出
    - 设置完成标志
```

## Direct模式完整示例

```c
#include "motion_control.h"
#include "common_types.h"

#define CYCLE_PERIOD 0.001  // 1ms PLC周期

HDY_MotionControlFB fb;
HDY_TIME currentTime = 0.0;

// 1. 初始化
HDY_MotionControlFB_Init(&fb);
fb.EN = true;
fb.USE_RECIPE = false;  // Direct模式
fb.FLOW_TO_PUMP_SPEED_GAIN = 1.2;
fb.PUMP_SPEED_LIMIT = 3000.0;

// 2. 配置并启动注射段
HDY_MotionSegment injectionSeg;
memset(&injectionSeg, 0, sizeof(injectionSeg));
injectionSeg.segmentTag = 1;  // 段标签（替代原name字段）
injectionSeg.mode = HDY_MODE_SPEED_RAMP;
injectionSeg.endCondition = HDY_END_POSITION;
injectionSeg.direction = HDY_DIRECTION_EXTEND;
injectionSeg.targetPosition = 100.0;  // mm
injectionSeg.maxVelocity = 200.0;    // mm/s
injectionSeg.maxAcceleration = 500.0; // mm/s²
injectionSeg.maxFlow = 50.0;         // L/min
injectionSeg.targetFlow = 40.0;      // L/min
injectionSeg.velocityToFlowGain = 0.2;
injectionSeg.positionTolerance = 0.1;

HDY_MotionControlFB_LoadDirectSegment(&fb, &injectionSeg);
HDY_MotionControlFB_StartSegment(&fb, 0, currentTime);

// 3. 周期循环
while (!fb.SEGMENT_COMPLETED) {
    // 更新传感器反馈
    fb.AXIS_REF.position = GetCurrentPosition();  // mm
    fb.AXIS_REF.velocity = GetCurrentVelocity();  // mm/s
    fb.AXIS_REF.flow = GetCurrentFlow();          // L/min
    fb.AXIS_REF.pressure = GetCurrentPressure();  // MPa
    fb.AXIS_REF.timestamp = currentTime;
    
    // 执行控制周期
    HDY_MotionControlFB_Execute(&fb);
    
    // 输出泵速命令
    SetPumpSpeed(fb.PUMP_SPEED);
    
    // 检查故障
    if (fb.FAULT) {
        HandleFault(fb.DIAGNOSTIC);
        break;
    }
    
    currentTime += CYCLE_PERIOD;
}

// 4. 工艺层决定切换到保压段
HDY_MotionSegment holdingSeg;
memset(&holdingSeg, 0, sizeof(holdingSeg));
holdingSeg.segmentTag = 2;  // 段标签（替代原name字段）
holdingSeg.mode = HDY_MODE_PRESSURE_CLOSED_LOOP;
holdingSeg.endCondition = HDY_END_TIME;
holdingSeg.direction = HDY_DIRECTION_HOLD;
holdingSeg.targetPressure = 80.0;   // MPa
holdingSeg.targetFlow = 5.0;       // L/min
holdingSeg.maxFlow = 20.0;
holdingSeg.duration = 2.0;         // s
holdingSeg.pressureController = HDY_PRESSURE_CONTROLLER_PI;
holdingSeg.pressureKp = 0.5;
holdingSeg.pressureKi = 0.1;
holdingSeg.pressureIntegralLimit = 10.0;
holdingSeg.pressureRampRate = 5.0;
holdingSeg.pressureTolerance = 0.5;

HDY_MotionControlFB_LoadDirectSegment(&fb, &holdingSeg);
HDY_MotionControlFB_StartSegment(&fb, 0, currentTime);

// 5. 继续周期循环...
```

## 关键特性

### 1. 速度连续性保证

运动规划器通过以下机制保证速度连续性：

- **位置模式**: 使用 `v = √(2as)` 或 `v = min(at, √(2as))` 确保速度平滑
- **速度斜坡模式**: 使用 `v = at` 时间斜坡，配合位置制动保护
- **段切换**: 工艺层可以在任意时刻切换段，运动规划器会从当前速度状态继续

### 2. 加速度连续性保证

- **恒定加速度**: 段内使用恒定加速度 `a = constant`
- **段切换**: 新段从当前速度状态开始，加速度可能跳变但速度连续
- **制动保护**: 在目标位置附近自动应用制动包络

### 3. 压力平滑性保证

压力控制通过以下机制保证平滑性：

- **压力斜坡控制器**: `HDY_RampController` 平滑压力目标变化
- **斜坡率限制**: `pressureRampRate` 限制压力变化率 `dP/dt`
- **测量滤波**: `pressureFilterAlpha` 压力测量滤波
- **微分滤波**: `pressureDerivativeFilterAlpha` 微分项滤波
- **PID控制**: 积分抗饱和和扰动跟踪

## 连续性验证

基于代码分析和测试验证：

### 速度连续性 ✅

```c
// motion_planner.c
// 速度计算基于连续的物理模型
velocityMagnitude = acceleration * elapsedTime;  // 连续函数
velocityMagnitude = sqrt(2 * acceleration * remainingDistance);  // 连续函数

// 段切换时，elapsedTime从0开始，但状态保持连续
// 新段会基于当前feedback继续规划
```

### 加速度连续性 ⚠️

```c
// 段内加速度恒定
const HDY_REAL maxAcceleration = segment->maxAcceleration;

// 段切换时加速度可能跳变（不同段的加速度参数不同）
// 但速度保持连续，这是工业控制中常见且可接受的
```

### 压力平滑性 ✅

```c
// ramp_controller.c
// 压力斜坡保证平滑性
if (input->rampRate > 0.0) {
    HDY_REAL maxChange = input->rampRate * deltaTime;
    // 限制压力变化率，确保平滑
    controller->rampedPressure = HDY_ClampReal(
        controller->rampedPressure + maxChange,
        controller->rampedPressure,
        input->targetPressure
    );
}

// 压力控制器带滤波
filteredPressure = alpha * currentPressure + (1-alpha) * previousPressure
```

## 控制模式选择

### 1. 位置模式 (HDY_MODE_POSITION)

```c
segment.mode = HDY_MODE_POSITION;
segment.planner = HDY_PLANNER_POSITION_BASED;  // 或 TIME_BASED
segment.endCondition = HDY_END_POSITION;
segment.targetPosition = 100.0;
```

**适用场景**:
- 精确定位控制
- 需要位置制动
- 合模、开模等动作

**特点**:
- 速度基于制动曲线计算
- 保证准确停在目标位置
- 支持EXTEND/RETRACT方向

### 2. 速度斜坡模式 (HDY_MODE_SPEED_RAMP)

```c
segment.mode = HDY_MODE_SPEED_RAMP;
segment.planner = HDY_PLANNER_TIME_BASED;
segment.endCondition = HDY_END_POSITION;  // 或 HDY_END_TIME
segment.maxVelocity = 200.0;
segment.maxAcceleration = 500.0;
```

**适用场景**:
- 注射、射退等动作
- 需要控制速度 profile
- 时间或位置结束

**特点**:
- 速度按时间斜坡增加
- 可选位置制动保护
- 灵活的结束条件

### 3. 压力闭环模式 (HDY_MODE_PRESSURE_CLOSED_LOOP)

```c
segment.mode = HDY_MODE_PRESSURE_CLOSED_LOOP;
segment.endCondition = HDY_END_TIME;  // 或 HDY_END_PRESSURE
segment.targetPressure = 80.0;
segment.targetFlow = 5.0;
segment.pressureController = HDY_PRESSURE_CONTROLLER_PI;
segment.pressureKp = 0.5;
segment.pressureKi = 0.1;
segment.pressureRampRate = 5.0;
```

**适用场景**:
- 保压控制
- 需要精确压力控制
- 压力或时间结束

**特点**:
- 压力闭环控制
- 支持P/PI/PID/RBF-PID
- 前馈流量设定
- 压力斜坡平滑

## 工艺层集成要点

### 1. 反馈更新频率

```c
// 建议每个PLC周期更新一次反馈
fb.AXIS_REF.position = encoder.position;      // 位置反馈
fb.AXIS_REF.velocity = encoder.velocity;      // 速度反馈
fb.AXIS_REF.flow = flowSensor.flow;            // 流量反馈
fb.AXIS_REF.pressure = pressureSensor.pressure; // 压力反馈
fb.AXIS_REF.timestamp = systemTime;           // 时间戳
```

### 2. 段切换时机

```c
// 在SEGMENT_COMPLETED为true时切换
if (fb.SEGMENT_COMPLETED) {
    // 配置新段
    HDY_MotionControlFB_LoadDirectSegment(&fb, &newSegment);
    HDY_MotionControlFB_StartSegment(&fb, 0, currentTime);
}
```

### 3. 故障处理

```c
// 检查故障状态
if (fb.FAULT || fb.ERROR) {
    HDY_DiagnosticCode code = fb.DIAGNOSTIC.code;
    switch (code) {
        case HDY_DIAG_CODE_OVER_PRESSURE:
            // 处理超压
            break;
        case HDY_DIAG_CODE_TIMEOUT:
            // 处理超时
            break;
        // ...其他故障
    }
    
    // 确认并清除诊断
    HDY_MotionControlFB_AcknowledgeDiagnostics(&fb);
}
```

### 4. 性能监控

```c
// 监控压力循环状态
#if HDY_ENABLE_PRESSURE_LOOP_TELEMETRY
HDY_PressureLoopState* loop = &fb.STATE.pressureLoop;
printf("Pressure: %.2f MPa, Error: %.2f MPa, Output: %.2f L/min\n",
       loop->filteredPressure, loop->controlError, loop->outputFlow);
printf("Adaptive: Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
       loop->adaptiveKp, loop->adaptiveKi, loop->adaptiveKd);
#endif
```

## 参数调优建议

### 1. 运动参数

```c
// 根据机械特性调整
segment.maxVelocity = 200.0;      // mm/s，受机械限制
segment.maxAcceleration = 500.0;   // mm/s²，受电机能力限制
segment.velocityToFlowGain = 0.2;  // L/min per mm/s，受液压缸面积影响
```

### 2. 压力控制参数

```c
// 从保守参数开始，逐步调整
segment.pressureKp = 0.5;          // 先调比例
segment.pressureKi = 0.0;          // 积分从0开始
segment.pressureKd = 0.0;          // 微分最后调
segment.pressureRampRate = 5.0;    // MPa/s，压力变化率
segment.pressureFilterAlpha = 0.8;  // 测量滤波系数
```

### 3. 安全限制

```c
segment.maxFlow = 50.0;            // L/min，最大流量
fb.PUMP_SPEED_LIMIT = 3000.0;      // rpm，泵速限制
segment.timeoutLimit = 5.0;         // s，超时保护
```

## 常见问题

### Q1: Direct模式和Recipe模式如何选择？

**A**: 
- **Direct模式**: 工艺层需要动态控制段切换，段参数可能变化
- **Recipe模式**: 段序列固定，可以预先配置整个工艺流程

### Q2: 段切换时会有冲击吗？

**A**: 
- **速度**: 连续，无冲击
- **加速度**: 可能跳变，但在工业可接受范围内
- **压力**: 通过斜坡控制器保证平滑

### Q3: 如何保证段切换的安全性？

**A**:
1. 在SEGMENT_COMPLETED确认后再切换
2. 新段参数经过验证
3. 监控FAULT和ERROR状态
4. 合理设置超时和保护限制

### Q4: 压力控制参数如何整定？

**A**:
1. 先设置合理的pressureRampRate（如5-10 MPa/s）
2. 从小压力Kp开始（如0.1-0.5）
3. 逐步增加Ki，观察积分饱和
4. 最后调整Kd改善动态响应
5. 使用压力滤波减少噪声影响

## 总结

Direct模式为工艺层提供了灵活的段控制能力：

✅ **优势**:
- 工艺层完全控制段切换
- 无需预先配置配方
- 支持动态段参数
- 简化集成复杂度

✅ **保证**:
- 速度连续性: ✅ 保证
- 加速度连续性: ⚠️ 段内连续，段间可能跳变
- 压力平滑性: ✅ 保证（通过斜坡和滤波）

✅ **安全**:
- 完整的诊断机制
- 多层保护动作
- 参数验证和约束
- 超时和偏差检测

通过合理使用Direct模式API，工艺开发工程师可以实现精确、安全、高效的运动控制。
