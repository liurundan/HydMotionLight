# 工艺层Direct模式集成指南

## 📋 快速参考

### Direct模式核心API
```c
// 1. 初始化
HDY_MotionControlFB_Init(&fb);

// 2. 配置Direct模式
fb.EN = true;
fb.USE_RECIPE = false;  // 关键：使用Direct模式

// 3. 加载段参数
HDY_MotionControlFB_LoadDirectSegment(&fb, &segment);

// 4. 启动执行
HDY_MotionControlFB_StartSegment(&fb, 0, timestamp);

// 5. 周期循环（每个PLC周期）
fb.AXIS_REF = GetCurrentFeedback();  // 更新传感器反馈
HDY_MotionControlFB_Execute(&fb);     // 执行控制
SetPumpSpeed(fb.PUMP_SPEED);           // 输出泵速命令

// 6. 段切换（工艺层决策）
if (fb.SEGMENT_COMPLETED) {
    HDY_MotionControlFB_LoadDirectSegment(&fb, &nextSegment);
    HDY_MotionControlFB_StartSegment(&fb, 0, timestamp);
}
```

---

## 🎯 核心优势

### 1. 工艺层完全控制
- ✅ 自由决定段切换时机
- ✅ 动态配置段参数
- ✅ 灵活实现工艺逻辑
- ✅ 无需预定义配方

### 2. 连续性保证
- ✅ **速度连续性**: 完全保证，无跳变
- ✅ **压力平滑性**: 充分保证，斜坡+滤波
- ⚠️ **加速度连续性**: 段内连续，段间可控跳变

### 3. 工业标准兼容
- ✅ IEC61131-3标准
- ✅ PLCopen函数块风格
- ✅ 嵌入式友好设计

---

## 🔧 详细API说明

### 初始化与配置

```c
void HDY_MotionControlFB_Init(HDY_MotionControlFB* fb);
```
**功能**: 完全初始化功能块，清除所有状态
**调用时机**: 系统启动或复位后
**注意**: 会清除所有配置，需要重新设置参数

```c
HDY_BOOL HDY_MotionControlFB_LoadDirectSegment(
    HDY_MotionControlFB* fb, 
    const HDY_MotionSegment* segment
);
```
**功能**: 加载Direct模式的段参数
**返回值**: true=成功，false=参数验证失败
**调用时机**: 启动新段之前
**注意**: 不会自动启动，需要调用StartSegment

### 执行控制

```c
HDY_BOOL HDY_MotionControlFB_StartSegment(
    HDY_MotionControlFB* fb, 
    size_t segmentIndex,    // Direct模式下忽略
    HDY_TIME timestamp
);
```
**功能**: 启动段执行
**返回值**: true=成功，false=状态不允许
**调用时机**: 加载段参数后
**注意**: 实际状态转换在下一个Execute()周期生效

```c
void HDY_MotionControlFB_Execute(HDY_MotionControlFB* fb);
```
**功能**: 执行一个控制周期
**调用时机**: 每个PLC周期调用一次
**输入**: 通过AXIS_REF提供传感器反馈
**输出**: 通过PUMP_SPEED、STATE等输出控制结果

### 状态监控

```c
fb.ACTIVE              // bool: 段是否在执行中
fb.SEGMENT_COMPLETED   // bool: 段是否完成
fb.SEGMENT_CHANGED     // bool: 段刚切换（单周期脉冲）
fb.FINISHED            // bool: 所有段完成
fb.FAULT               // bool: 故障状态
fb.ERROR               // bool: 错误状态
fb.PUMP_SPEED          // real: 泵速命令输出（rpm）
fb.FB_STATE            // enum: 功能块状态机状态
```

### 详细状态信息

```c
fb.STATE.plannedVelocity          // real: 规划速度
fb.STATE.plannedFlow              // real: 规划流量
fb.STATE.commandedPumpSpeed       // real: 命令泵速
fb.STATE.currentSegmentTag        // uint8: 当前段标签（替代原currentSegmentName）
fb.STATE.pressureLoop             // struct: 压力循环遥测（需启用HDY_ENABLE_PRESSURE_LOOP_TELEMETRY）
fb.STATE.status                   // enum: 控制器状态
fb.STATE.plannedDirection         // enum: 规划方向
```

---

## 📊 段参数配置

### 速度斜坡模式

```c
HDY_MotionSegment segment;
memset(&segment, 0, sizeof(segment));

segment.segmentTag = 1;  // 段标签（替代原name字段）
segment.mode = HDY_MODE_SPEED_RAMP;
segment.endCondition = HDY_END_POSITION;  // 或 HDY_END_TIME
segment.direction = HDY_DIRECTION_EXTEND; // 或 RETRACT

segment.targetPosition = 100.0;    // mm (位置结束时使用)
segment.duration = 2.0;            // s (时间结束时使用)
segment.maxVelocity = 200.0;       // mm/s
segment.maxAcceleration = 500.0;    // mm/s²
segment.maxFlow = 50.0;            // L/min
segment.targetFlow = 40.0;         // L/min
segment.velocityToFlowGain = 0.2;   // L/min per mm/s

segment.positionTolerance = 0.1;   // mm
segment.timeoutLimit = 5.0;        // s
```

### 压力闭环模式

```c
HDY_MotionSegment segment;
memset(&segment, 0, sizeof(segment));

segment.segmentTag = 2;  // 段标签（替代原name字段）
segment.mode = HDY_MODE_PRESSURE_CLOSED_LOOP;
segment.endCondition = HDY_END_TIME;  // 或 HDY_END_PRESSURE
segment.direction = HDY_DIRECTION_HOLD;

segment.targetPressure = 80.0;    // MPa
segment.targetFlow = 5.0;         // L/min (前馈流量)
segment.maxFlow = 20.0;          // L/min
segment.duration = 2.0;          // s (时间结束时使用)

// 压力控制器参数
segment.pressureController = HDY_PRESSURE_CONTROLLER_PI;
segment.pressureKp = 0.5;         // L/min per MPa
segment.pressureKi = 0.1;         // L/min per (MPa·s)
segment.pressureKd = 0.0;         // L/min per (MPa/s)
segment.pressureIntegralLimit = 10.0;  // L/min
segment.pressureDeadband = 0.5;   // MPa

// 压力平滑参数
segment.pressureRampRate = 5.0;   // MPa/s
segment.pressureFilterAlpha = 0.8; // 测量滤波系数
segment.pressureDerivativeFilterAlpha = 0.8; // 微分滤波

segment.pressureTolerance = 0.5;  // MPa
segment.timeoutLimit = 5.0;       // s
```

---

## 🔄 典型调用链路

### 标准注射成型工艺流程

```
1. 合模阶段
   └─> HDY_MODE_SPEED_RAMP
       └─> HDY_END_POSITION

2. 注射阶段  
   └─> HDY_MODE_SPEED_RAMP
       └─> HDY_END_POSITION

3. 保压阶段
   └─> HDY_MODE_PRESSURE_CLOSED_LOOP
       └─> HDY_END_TIME

4. 冷却阶段
   └─> HDY_MODE_PRESSURE_CLOSED_LOOP
       └─> HDY_END_TIME

5. 开模阶段
   └─> HDY_MODE_SPEED_RAMP
       └─> HDY_END_POSITION

6. 顶出阶段
   └─> HDY_MODE_SPEED_RAMP
       └─> HDY_END_POSITION
```

### 工艺层控制流程

```c
// 全局变量
HDY_MotionControlFB fb;
HDY_TIME currentTime = 0.0;
int currentPhase = 0;

// 初始化
void InitSystem() {
    HDY_MotionControlFB_Init(&fb);
    fb.EN = true;
    fb.USE_RECIPE = false;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 1.2;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    
    StartPhase(0);  // 启动合模阶段
}

// 周期任务（1ms）
void CyclicTask() {
    // 1. 更新传感器反馈
    fb.AXIS_REF.position = GetEncoderPosition();
    fb.AXIS_REF.velocity = GetEncoderVelocity();
    fb.AXIS_REF.flow = GetFlowSensor();
    fb.AXIS_REF.pressure = GetPressureSensor();
    fb.AXIS_REF.timestamp = currentTime;
    
    // 2. 执行控制
    HDY_MotionControlFB_Execute(&fb);
    
    // 3. 输出控制命令
    SetPumpSpeed(fb.PUMP_SPEED);
    
    // 4. 段切换管理
    if (fb.SEGMENT_COMPLETED) {
        currentPhase++;
        StartPhase(currentPhase);
    }
    
    // 5. 故障处理
    if (fb.FAULT) {
        HandleFault(fb.DIAGNOSTIC);
    }
    
    currentTime += 0.001;  // 1ms周期
}

// 启动特定阶段
void StartPhase(int phase) {
    HDY_MotionSegment segment;
    
    switch (phase) {
        case 0:  // 合模
            segment = CreateClampingSegment();
            break;
        case 1:  // 注射
            segment = CreateInjectionSegment();
            break;
        case 2:  // 保压
            segment = CreateHoldingSegment();
            break;
        case 3:  // 冷却
            segment = CreateCoolingSegment();
            break;
        case 4:  // 开模
            segment = CreateOpeningSegment();
            break;
        case 5:  // 顶出
            segment = CreateEjectionSegment();
            break;
        default:
            // 完成一个循环，重新开始
            currentPhase = 0;
            segment = CreateClampingSegment();
            break;
    }
    
    HDY_MotionControlFB_LoadDirectSegment(&fb, &segment);
    HDY_MotionControlFB_StartSegment(&fb, 0, currentTime);
}
```

---

## ⚙️ 参数调优指南

### 运动参数调优

#### 1. 速度参数
```c
segment.maxVelocity = 200.0;  // mm/s
```
**调优原则**:
- 从低速开始，逐步提高
- 考虑机械限制和振动
- 监控加速度和压力波动

#### 2. 加速度参数
```c
segment.maxAcceleration = 500.0;  // mm/s²
```
**调优原则**:
- 避免过大的加速度跳变
- 考虑电机能力和液压响应
- 相邻段加速度差异<200 mm/s²

#### 3. 流量增益
```c
segment.velocityToFlowGain = 0.2;  // L/min per mm/s
```
**调优原则**:
- 基于液压缸面积计算
- 通过实际测试验证
- 考虑系统泄漏和压缩性

### 压力控制参数调优

#### 1. 比例增益 (Kp)
```c
segment.pressureKp = 0.5;  // L/min per MPa
```
**调优步骤**:
1. 从小值开始（0.1-0.3）
2. 观察压力响应速度
3. 逐步增加直到出现振荡
4. 回退到稳定值的70%

#### 2. 积分增益 (Ki)
```c
segment.pressureKi = 0.1;  // L/min per (MPa·s)
```
**调优步骤**:
1. Kp调整完成后开始
2. 从0开始，逐步增加
3. 消除稳态误差
4. 避免积分饱和

#### 3. 微分增益 (Kd)
```c
segment.pressureKd = 0.0;  // L/min per (MPa/s)
```
**调优步骤**:
1. 最后调整，通常为0
2. 改善动态响应
3. 可能引入噪声，需要配合滤波

#### 4. 压力斜坡率
```c
segment.pressureRampRate = 5.0;  // MPa/s
```
**调优原则**:
- 保压段: 5-10 MPa/s
- 快速响应: 10-20 MPa/s
- 精密控制: 2-5 MPa/s

#### 5. 测量滤波
```c
segment.pressureFilterAlpha = 0.8;  // 0<alpha<=1
```
**调优原则**:
- 压力噪声大: 降低alpha（0.5-0.7）
- 要求快速响应: 提高alpha（0.8-0.95）
- 1.0表示无滤波

---

## 🔍 故障诊断

### 常见故障代码

| 故障代码 | 含义 | 可能原因 | 处理方法 |
|---------|------|---------|---------|
| HDY_DIAG_CODE_TIMEOUT | 超时 | 段执行时间过长 | 检查maxVelocity、增加timeoutLimit |
| HDY_DIAG_CODE_OVER_PRESSURE | 超压 | 压力超过安全限值 | 检查压力传感器、调整控制参数 |
| HDY_DIAG_CODE_UNDER_PRESSURE | 欠压 | 压力无法达到目标 | 检查液压系统、增加Kp |
| HDY_DIAG_CODE_POSITION_DEVIATION | 位置偏差 | 无法到达目标位置 | 检查机械系统、增加maxVelocity |
| HDY_DIAG_CODE_SEGMENT_INVALID | 段参数无效 | 参数验证失败 | 检查段参数配置 |

### 诊断信息读取

```c
if (fb.FAULT || fb.ERROR) {
    HDY_DiagnosticCode code = fb.DIAGNOSTIC.code;
    HDY_DiagnosticSeverity severity = fb.DIAGNOSTIC.severity;
    HDY_DiagnosticSource source = fb.DIAGNOSTIC.source;
    
    printf("Diagnostic: Code=%d, Severity=%d, Source=%d\n", 
           code, severity, source);
    printf("Pressure Error: %.3f MPa\n", fb.DIAGNOSTIC.pressureError);
    printf("Flow Error: %.3f L/min\n", fb.DIAGNOSTIC.flowError);
    
    // 处理故障后确认并清除
    HDY_MotionControlFB_AcknowledgeDiagnostics(&fb);
}
```

### 性能监控

```c
#if HDY_ENABLE_PRESSURE_LOOP_TELEMETRY
HDY_PressureLoopState* loop = &fb.STATE.pressureLoop;

printf("Pressure Control Telemetry:\n");
printf("  Target Pressure: %.3f MPa\n", loop->targetPressure);
printf("  Filtered Pressure: %.3f MPa\n", loop->filteredPressure);
printf("  Control Error: %.3f MPa\n", loop->controlError);
printf("  Output Flow: %.3f L/min\n", loop->outputFlow);
printf("  Saturated: %s\n", loop->saturated ? "YES" : "NO");

if (fb.STATE.pressureControllerApplied == HDY_PRESSURE_CONTROLLER_RBF_PID) {
    printf("  Adaptive Kp: %.3f\n", loop->adaptiveKp);
    printf("  Adaptive Ki: %.3f\n", loop->adaptiveKi);
    printf("  Adaptive Kd: %.3f\n", loop->adaptiveKd);
}
#endif
```

---

## ✅ 验证测试

### 连续性验证

运行 `test_direct_mode_simple` 进行连续性验证：

```bash
./out/build/unixgcc/test_direct_mode_simple
```

**预期结果**:
```
✅ Velocity continuity: EXCELLENT
✅ Pressure smoothness: EXCELLENT
✅ The motion control library meets industrial requirements
```

### 性能基准测试

运行 `benchmark_performance` 进行性能测试：

```bash
./out/build/unixgcc/benchmark_performance
```

**关注指标**:
- 执行时间: < 100μs (在嵌入式平台)
- 内存占用: 符合预期
- CPU使用率: < 10% (在1ms周期)

---

## 📈 最佳实践

### 1. 参数配置
- ✅ 从保守参数开始
- ✅ 逐步调整优化
- ✅ 记录每次修改
- ✅ 保存最优参数

### 2. 段切换管理
- ✅ 在SEGMENT_COMPLETED后切换
- ✅ 避免过于频繁的段切换
- ✅ 考虑添加短暂稳定时间
- ✅ 监控段切换时的状态

### 3. 故障处理
- ✅ 及时响应FAULT信号
- ✅ 记录详细的诊断信息
- ✅ 分析根本原因
- ✅ 实施预防措施

### 4. 性能优化
- ✅ 监控CPU和内存使用
- ✅ 优化控制周期时间
- ✅ 合理配置诊断功能
- ✅ 启用/禁用遥测功能

---

## 🎓 学习资源

### 文档
- `Direct模式API使用说明.md` - 完整API参考
- `运动控制库连续性分析报告.md` - 技术深度分析
- `include/motion_control.h` - 头文件注释
- `src/motion_planner.c` - 运动规划实现
- `src/pressure_controller.c` - 压力控制实现

### 测试程序
- `tests/test_direct_mode_simple.c` - 连续性验证
- `tests/test_direct_mode.c` - 完整示例
- `tests/main.c` - 基本示例
- `tests/benchmark_performance.c` - 性能测试

### 标准
- IEC61131-3 可编程控制器标准
- PLCopen 运动控制规范

---

## 🔗 相关链接

- 项目根目录: `/home/dan/project/hdy-motion-light`
- API头文件: `include/motion_control.h`
- 类型定义: `include/common_types.h`
- 配置文件: `include/hdy_config.h`

---

## 💡 总结

Direct模式为工艺层提供了强大而灵活的运动控制能力：

**核心优势**:
- 工艺层完全控制段切换
- 速度和压力连续性得到保证
- 符合工业标准和实践

**使用要点**:
- 合理配置段参数
- 监控执行状态
- 及时处理故障
- 持续优化参数

**质量保证**:
- 完整的测试覆盖
- 详细的文档说明
- 标准化的接口设计
- 工业级的可靠性

通过本指南，工艺开发工程师可以快速掌握Direct模式的使用方法，实现高效、可靠的运动控制。
