# RBF-PID 工业实用性改造计划

**版本**: v1.0  
**日期**: 2026-09-22  
**目标**: 使RBF-PID和FF-PI算法均具备工业实用性，支持现场灵活选择

---

## 一、现状分析

### 1.1 当前架构优势 ✅

**IEC参数接口已完备**：
```c
// 现有数据链路（已实现）
PLC工艺层 
  → HYD_PARAM_PRESSURE_CONTROLLER_TYPE 
    → fb->_params.pressureControllerType 
      → HYD_PressureController_Execute() 自动路由到对应策略
```

**策略枚举已定义**：
```c
typedef enum {
    HYD_PRESSURE_CONTROLLER_NONE = 0,     // 开环
    HYD_PRESSURE_CONTROLLER_P,            // 纯比例
    HYD_PRESSURE_CONTROLLER_PI,           // 标准PI
    HYD_PRESSURE_CONTROLLER_PID,          // 标准PID
    HYD_PRESSURE_CONTROLLER_RBF_PID,      // RBF自适应PID
    HYD_PRESSURE_CONTROLLER_RBF_PI,       // RBF自适应PI
    HYD_PRESSURE_CONTROLLER_FF_PI         // 前馈+解析整定PI
} HYD_PressureControllerType;
```

**结论**: **不需要新增"场景枚举"**，直接复用现有`HYD_PARAM_PRESSURE_CONTROLLER_TYPE`即可。

---

### 1.2 当前问题

#### 问题1: RBF参数复杂度过高（147个参数）
**现象**:
```c
// src/rbf_pid.c 内部参数
- 6个网络参数（hidden_neurons, input_neurons, alpha, w_init, c_init, b_init）
- 6个学习率（eta_w, eta_c, eta_b, eta_p, eta_i, eta_d）
- 6个增益范围（min/max Kp/Ki/Kd）
- 10+个阈值（死区、上限倍率、窗口比例...）
```

**后果**: 现场工程师无法调试，只能由研发人员支持。

#### 问题2: 缺少应用场景指导
**现象**: PLC程序员不知道何时用RBF、何时用FF_PI。

#### 问题3: 缺少参数预设
**现象**: 每个机型都需要从头整定147个参数。

---

## 二、改造目标

### 2.1 核心目标

| 维度 | 当前状态 | 目标状态 |
|------|----------|----------|
| **参数数量** | 147个内部参数 | **5个必填参数**（工艺层输入） |
| **调试时间** | 2-4小时（研发支持） | **15分钟**（现场工程师独立完成） |
| **适用场景** | 不明确 | **文档化**（射胶/保压/储料推荐策略） |
| **参数预设** | 无 | **3套预设**（薄壁/厚壁/标准） |
| **故障诊断** | 黑箱 | **可观测**（学习收敛监控+自动回退） |

### 2.2 设计原则

1. **零破坏性变更**：保持现有IEC接口不变
2. **渐进式增强**：新增"简化配置模式"，专家模式仍可用
3. **向后兼容**：已有PLC程序无需修改

---

## 三、技术方案

### 3.1 数据流架构（保持不变）

```
┌──────────────────────────────────────────────────────────┐
│ PLC工艺层（ST语言）                                       │
├──────────────────────────────────────────────────────────┤
│  // 射胶段配置示例                                        │
│  fbInject.SetParameter(HYD_PARAM_PRESSURE_CONTROLLER_TYPE,│
│                        HYD_PRESSURE_CONTROLLER_RBF_PID);  │
│  fbInject.SetParameter(HYD_PARAM_PRESSURE_SYSTEM_GAIN,    │
│                        210.5);  // K = 210.5 bar/(L/min)  │
│  fbInject.SetParameter(HYD_PARAM_PRESSURE_PLANT_TAU, 1.0);│
│  fbInject.SetParameter(HYD_PARAM_RBF_PRESET, 1);  // 新增 │
└──────────────────────────────────────────────────────────┘
         ↓ IEC桥接层（motion_interface.c）
┌──────────────────────────────────────────────────────────┐
│ HYD_MotionControlFB_WriteParameter()                      │
│   → fb->_params.pressureControllerType = value;           │
│   → fb->_params.pressureSystemGain = value;               │
│   → fb->_params.rbfPreset = value;  // 新增字段           │
└──────────────────────────────────────────────────────────┘
         ↓ 核心控制层（motion_control.c）
┌──────────────────────────────────────────────────────────┐
│ HYD_ProduceControlOutputs()                               │
│   → HYD_ResolvePressureControllerConfig()  // 新增逻辑   │
│     → 根据 rbfPreset 自动推导147个内部参数               │
│   → HYD_PressureController_Execute()                      │
└──────────────────────────────────────────────────────────┘
         ↓ 压力控制器（pressure_controller.c）
┌──────────────────────────────────────────────────────────┐
│ switch (config.strategy) {                                │
│   case HYD_PRESSURE_CONTROLLER_RBF_PID:                   │
│     RBF_PID_Update(&state->rbfPid, ...);  // 内部已简化 │
│   case HYD_PRESSURE_CONTROLLER_FF_PI:                     │
│     // 解析整定公式，参数少                             │
│ }                                                         │
└──────────────────────────────────────────────────────────┘
```

---

### 3.2 新增参数枚举

#### 3.2.1 RBF预设模式
```c
// include/common_types.h 新增
typedef enum {
    HYD_RBF_PRESET_CUSTOM = 0,        // 自定义（专家模式，需配所有参数）
    HYD_RBF_PRESET_THIN_WALL = 1,     // 薄壁件射胶（激进学习）
    HYD_RBF_PRESET_THICK_WALL = 2,    // 厚壁件保压（保守学习）
    HYD_RBF_PRESET_STANDARD = 3,      // 标准配置（中庸）
    HYD_RBF_PRESET_PLASTICATION = 4   // 储料背压（批次自学习）
} HYD_RbfPreset;

// HYD_ParameterType 新增
typedef enum {
    // ... 现有参数 ...
    HYD_PARAM_RBF_PRESET = 80,         // RBF预设模式
    HYD_PARAM_RBF_AGGRESSIVENESS,      // 响应激进度 [0.5-2.0]
    HYD_PARAM_RBF_ENABLE_ADAPTATION,   // 是否开启在线学习 [0/1]
    // ... 现有参数继续 ...
} HYD_ParameterType;
```

#### 3.2.2 核心必填参数（已有，无需新增）
```c
// 这5个参数已经存在于 HYD_ParameterType
HYD_PARAM_PRESSURE_SYSTEM_GAIN         // K (bar/(L/min))
HYD_PARAM_PRESSURE_PLANT_TAU           // τ (s)
HYD_PARAM_MAX_PRESSURE                 // P_max (bar)
HYD_PARAM_MAX_FLOW                     // Q_max (L/min)
HYD_PARAM_PRESSURE_CONTROLLER_TYPE     // 策略选择
```

---

### 3.3 预设参数表设计

#### 3.3.1 薄壁件射胶预设（THIN_WALL）
**特点**: 快速充填，熔体剪切稀化明显，非线性强
```c
static const RBF_PID_Config HYD_RBF_PRESET_THIN_WALL_CONFIG = {
    // 网络结构：标准3神经元（已验证有效）
    .hidden_neurons = 3,
    .input_neurons = 2,
    .alpha = 20.0f,
    
    // 学习率：激进（快速适应熔体粘度变化）
    .eta_w = 0.20f,  // 权重学习率
    .eta_c = 0.15f,  // 中心学习率
    .eta_b = 0.15f,  // 宽度学习率
    .eta_p = 0.15f,  // Kp学习率（激进）
    .eta_i = 0.10f,  // Ki学习率
    .eta_d = 0.08f,  // Kd学习率
    
    // 增益范围：宽范围（应对剪切稀化）
    .min_KP = 0.01f,
    .max_KP = 0.50f,
    .min_KI = 0.001f,
    .max_KI = 0.20f,
    .min_KD = 0.0f,
    .max_KD = 0.05f,
    
    // 死区与阈值：紧凑（快速响应）
    .error_deadband = 0.005f,      // 0.5% P_set
    .steady_state_threshold = 2.0f  // 2.0 bar
};
```

#### 3.3.2 厚壁件保压预设（THICK_WALL）
**特点**: 长周期冷却，补缩需求动态变化，要求平稳
```c
static const RBF_PID_Config HYD_RBF_PRESET_THICK_WALL_CONFIG = {
    .hidden_neurons = 3,
    .input_neurons = 2,
    .alpha = 20.0f,
    
    // 学习率：保守（避免超调）
    .eta_w = 0.10f,
    .eta_c = 0.08f,
    .eta_b = 0.08f,
    .eta_p = 0.05f,  // Kp学习率（保守）
    .eta_i = 0.03f,  // Ki学习率
    .eta_d = 0.02f,  // Kd学习率
    
    // 增益范围：中等（稳定优先）
    .min_KP = 0.01f,
    .max_KP = 0.30f,
    .min_KI = 0.001f,
    .max_KI = 0.10f,
    .min_KD = 0.0f,
    .max_KD = 0.03f,
    
    // 死区与阈值：宽松（平稳控制）
    .error_deadband = 0.01f,       // 1.0% P_set
    .steady_state_threshold = 3.0f  // 3.0 bar
};
```

#### 3.3.3 标准预设（STANDARD）
**特点**: 通用配置，平衡响应速度和稳定性
```c
static const RBF_PID_Config HYD_RBF_PRESET_STANDARD_CONFIG = {
    .hidden_neurons = 3,
    .input_neurons = 2,
    .alpha = 20.0f,
    
    // 学习率：中庸
    .eta_w = 0.15f,
    .eta_c = 0.10f,
    .eta_b = 0.10f,
    .eta_p = 0.10f,
    .eta_i = 0.05f,
    .eta_d = 0.05f,
    
    // 增益范围：标准
    .min_KP = 0.01f,
    .max_KP = 0.40f,
    .min_KI = 0.001f,
    .max_KI = 0.15f,
    .min_KD = 0.0f,
    .max_KD = 0.04f,
    
    .error_deadband = 0.005f,
    .steady_state_threshold = 2.0f
};
```

#### 3.3.4 储料背压预设（PLASTICATION）
**特点**: 螺杆后退，物料批次差异大，需自学习
```c
static const RBF_PID_Config HYD_RBF_PRESET_PLASTICATION_CONFIG = {
    .hidden_neurons = 3,
    .input_neurons = 2,
    .alpha = 20.0f,
    
    // 学习率：中等偏高（适应批次差异）
    .eta_w = 0.18f,
    .eta_c = 0.12f,
    .eta_b = 0.12f,
    .eta_p = 0.12f,  // 批次切换时快速适应
    .eta_i = 0.08f,
    .eta_d = 0.05f,
    
    // 增益范围：较宽（应对MFI±10%差异）
    .min_KP = 0.01f,
    .max_KP = 0.45f,
    .min_KI = 0.001f,
    .max_KI = 0.18f,
    .min_KD = 0.0f,
    .max_KD = 0.05f,
    
    .error_deadband = 0.008f,      // 背压控制可放宽
    .steady_state_threshold = 5.0f  // 5.0 bar
};
```

---

### 3.4 参数推导函数

#### 3.4.1 核心推导逻辑
```c
// src/pressure_controller.c 新增
static void HYD_DeriveRbfConfigFromPreset(
    HYD_RbfPreset preset,
    HYD_REAL systemGain,
    HYD_REAL plantTau,
    HYD_REAL maxPressure,
    HYD_REAL maxFlow,
    HYD_REAL aggressiveness,  // 默认1.0
    RBF_PID_Config* outConfig)
{
    const RBF_PID_Config* baseConfig;
    
    /* Step 1: 选择基础预设 */
    switch (preset) {
    case HYD_RBF_PRESET_THIN_WALL:
        baseConfig = &HYD_RBF_PRESET_THIN_WALL_CONFIG;
        break;
    case HYD_RBF_PRESET_THICK_WALL:
        baseConfig = &HYD_RBF_PRESET_THICK_WALL_CONFIG;
        break;
    case HYD_RBF_PRESET_PLASTICATION:
        baseConfig = &HYD_RBF_PRESET_PLASTICATION_CONFIG;
        break;
    case HYD_RBF_PRESET_STANDARD:
    default:
        baseConfig = &HYD_RBF_PRESET_STANDARD_CONFIG;
        break;
    }
    
    /* Step 2: 复制基础配置 */
    memcpy(outConfig, baseConfig, sizeof(RBF_PID_Config));
    
    /* Step 3: 根据系统参数调整学习率 */
    if (aggressiveness > 0.0f && aggressiveness != 1.0f) {
        // 激进度调整：线性缩放学习率
        outConfig->eta_p *= aggressiveness;
        outConfig->eta_i *= aggressiveness;
        outConfig->eta_d *= aggressiveness;
        outConfig->eta_w *= aggressiveness;
        outConfig->eta_c *= aggressiveness;
        outConfig->eta_b *= aggressiveness;
        
        // 限幅（避免发散）
        outConfig->eta_p = clampf(0.01f, outConfig->eta_p, 0.30f);
        outConfig->eta_i = clampf(0.005f, outConfig->eta_i, 0.20f);
        outConfig->eta_d = clampf(0.001f, outConfig->eta_d, 0.15f);
    }
    
    /* Step 4: 根据对象时间常数调整增益范围 */
    if (plantTau > 0.0f && systemGain > HYD_MIN_SAFE_SYSTEM_GAIN) {
        // 一阶系统临界增益: K_crit ≈ τ / (K * T_sample)
        HYD_REAL K_crit_approx = plantTau / (systemGain * 0.001f);
        
        // 增益上界取临界增益的50%（保守）
        outConfig->max_KP = fminf(outConfig->max_KP, 0.5f * K_crit_approx);
        outConfig->max_KI = fminf(outConfig->max_KI, 0.3f * K_crit_approx);
    }
    
    /* Step 5: 填充系统参数 */
    outConfig->K = systemGain;
    outConfig->process_time_constant_s = plantTau;
    outConfig->max_output_lmin = maxFlow;
    outConfig->P_set = maxPressure;  // 初始设定（运行时会更新）
}
```

#### 3.4.2 IEC接口扩展
```c
// src/motion_control.c 修改
case HYD_PARAM_RBF_PRESET:
    if (value < 0 || value > HYD_RBF_PRESET_PLASTICATION) {
        return false;  // 参数校验
    }
    fb->_params.rbfPreset = (HYD_RbfPreset)value;
    fb->_params.rbfPresetDirty = true;  // 标记需要重新推导
    break;

case HYD_PARAM_RBF_AGGRESSIVENESS:
    if (value < 0.5f || value > 2.0f) {
        return false;  // 激进度范围 [0.5, 2.0]
    }
    fb->_params.rbfAggressiveness = value;
    fb->_params.rbfPresetDirty = true;
    break;

case HYD_PARAM_RBF_ENABLE_ADAPTATION:
    fb->_params.rbfEnableAdaptation = (value >= 0.5f);
    break;
```

#### 3.4.3 参数推导时机
```c
// src/motion_control.c: HYD_ProduceControlOutputs()
if (segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
    // 检查是否需要重新推导RBF配置
    if (fb->_params.rbfPresetDirty && 
        fb->_params.pressureControllerType == HYD_PRESSURE_CONTROLLER_RBF_PID) {
        
        HYD_DeriveRbfConfigFromPreset(
            fb->_params.rbfPreset,
            fb->_params.pressureSystemGain,
            fb->_params.pressurePlantTau,
            segment->targetPressure,  // 或 fb->_params.maxPressure
            fb->_params.maxFlow,
            fb->_params.rbfAggressiveness,
            &fb->_pressureController.rbfPid);
        
        fb->_params.rbfPresetDirty = false;
    }
    
    // ... 现有压力控制逻辑 ...
}
```

---

### 3.5 故障诊断与自动回退

#### 3.5.1 学习收敛监控
```c
// src/rbf_pid.c 新增
typedef struct {
    float gain_change_history[10];  // 最近10拍增益变化
    int divergence_count;           // 发散计数
    bool is_converged;              // 收敛标志
} RBF_ConvergenceMonitor;

static void RBF_UpdateConvergenceMonitor(
    RBF_PID_Handle* pid,
    RBF_ConvergenceMonitor* monitor)
{
    // 计算增益变化速率
    float gain_change = fabsf(pid->KP - pid->prev_KP) + 
                       fabsf(pid->KI - pid->prev_KI);
    
    // 滑动窗口
    for (int i = 9; i > 0; i--) {
        monitor->gain_change_history[i] = monitor->gain_change_history[i-1];
    }
    monitor->gain_change_history[0] = gain_change;
    
    // 计算方差
    float mean = 0.0f, variance = 0.0f;
    for (int i = 0; i < 10; i++) {
        mean += monitor->gain_change_history[i];
    }
    mean /= 10.0f;
    
    for (int i = 0; i < 10; i++) {
        float diff = monitor->gain_change_history[i] - mean;
        variance += diff * diff;
    }
    variance /= 10.0f;
    
    // 收敛判定：方差小于阈值
    if (variance < 1e-6f && mean < 1e-4f) {
        monitor->is_converged = true;
        monitor->divergence_count = 0;
    }
    
    // 发散检测：增益变化突然增大
    if (gain_change > 0.05f) {
        monitor->divergence_count++;
        if (monitor->divergence_count > 5) {
            monitor->is_converged = false;
            // 触发回退到PI
        }
    }
}
```

#### 3.5.2 自动回退机制
```c
// src/pressure_controller.c 修改
HYD_BOOL HYD_PressureController_Execute(...) {
    // ... 现有逻辑 ...
    
    if (config.strategy == HYD_PRESSURE_CONTROLLER_RBF_PID) {
        // 检测发散
        if (state->rbfConvergenceMonitor.divergence_count > 10) {
            // 自动回退到PI
            config.strategy = HYD_PRESSURE_CONTROLLER_PI;
            config.kp = 0.10f;  // 保守PI增益
            config.ki = 0.05f;
            
            // 记录诊断
            output->adaptationFreezeReason = 
                HYD_PRESSURE_ADAPTATION_FREEZE_DIVERGENCE;
            output->fallbackToPI = true;
            
            // 继续执行（用PI策略）
        }
    }
    
    // ... 继续执行 ...
}
```

---

## 四、文档化策略

### 4.1 应用场景指南

#### 4.1.1 决策树
```
是否需要压力闭环控制？
  ├─ 否 → HYD_PRESSURE_CONTROLLER_NONE（开环）
  └─ 是 → 工况特点？
      ├─ 标准保压段（恒定目标150bar）
      │   └─ 追求响应速度 → HYD_PRESSURE_CONTROLLER_FF_PI
      │       + HYD_PARAM_PRESSURE_PLANT_TAU = 实测τ
      │       + HYD_PARAM_PRESSURE_LOOP_OMEGA = 12 (保守) 或 15 (激进)
      │
      ├─ 射胶段（熔体粘度剧变）
      │   └─ HYD_PRESSURE_CONTROLLER_RBF_PID
      │       + HYD_PARAM_RBF_PRESET = HYD_RBF_PRESET_THIN_WALL
      │       + HYD_PARAM_PRESSURE_SYSTEM_GAIN = 标定K
      │       + HYD_PARAM_PRESSURE_PLANT_TAU = 标定τ
      │
      ├─ 多段保压（150→100→50bar）
      │   └─ HYD_PRESSURE_CONTROLLER_RBF_PID
      │       + HYD_PARAM_RBF_PRESET = HYD_RBF_PRESET_THICK_WALL
      │       + HYD_PARAM_RBF_AGGRESSIVENESS = 0.8 (保守)
      │
      ├─ 储料背压（物料批次差异）
      │   └─ HYD_PRESSURE_CONTROLLER_RBF_PID
      │       + HYD_PARAM_RBF_PRESET = HYD_RBF_PRESET_PLASTICATION
      │       + HYD_PARAM_RBF_ENABLE_ADAPTATION = 1 (开启)
      │
      └─ 开合模/顶出（机械运动）
          └─ HYD_PRESSURE_CONTROLLER_PI（传统PI已足够）
              + HYD_PARAM_PRESSURE_KP = 0.10
              + HYD_PARAM_PRESSURE_KI = 0.05
```

#### 4.1.2 IEC配置示例
```iecst
(* 射胶段配置：薄壁件，RBF-PID *)
fbInject.SetParameter(HYD_PARAM_PRESSURE_CONTROLLER_TYPE, 
                      INT_TO_REAL(HYD_PRESSURE_CONTROLLER_RBF_PID));
fbInject.SetParameter(HYD_PARAM_RBF_PRESET, 
                      INT_TO_REAL(HYD_RBF_PRESET_THIN_WALL));
fbInject.SetParameter(HYD_PARAM_PRESSURE_SYSTEM_GAIN, 210.5);
fbInject.SetParameter(HYD_PARAM_PRESSURE_PLANT_TAU, 1.0);
fbInject.SetParameter(HYD_PARAM_RBF_AGGRESSIVENESS, 1.2); (* 激进20% *)

(* 保压段配置：标准恒压，FF-PI *)
fbHold.SetParameter(HYD_PARAM_PRESSURE_CONTROLLER_TYPE, 
                    INT_TO_REAL(HYD_PRESSURE_CONTROLLER_FF_PI));
fbHold.SetParameter(HYD_PARAM_PRESSURE_SYSTEM_GAIN, 210.5);
fbHold.SetParameter(HYD_PARAM_PRESSURE_PLANT_TAU, 1.0);
fbHold.SetParameter(HYD_PARAM_PRESSURE_LOOP_OMEGA, 12.0);

(* 储料段配置：背压，RBF自适应 *)
fbPlast.SetParameter(HYD_PARAM_PRESSURE_CONTROLLER_TYPE, 
                     INT_TO_REAL(HYD_PRESSURE_CONTROLLER_RBF_PID));
fbPlast.SetParameter(HYD_PARAM_RBF_PRESET, 
                     INT_TO_REAL(HYD_RBF_PRESET_PLASTICATION));
fbPlast.SetParameter(HYD_PARAM_PRESSURE_SYSTEM_GAIN, 180.0);
fbPlast.SetParameter(HYD_PARAM_RBF_ENABLE_ADAPTATION, 1.0);
```

---

### 4.2 调试手册

#### 4.2.1 快速启动（5分钟）
```
步骤1: 标定对象参数（一次性）
  - 开环阶跃测试（0→50bar）
  - 记录t63时间（时间常数τ）
  - 记录稳态流量Q_ss
  - 计算增益 K = 50bar / Q_ss
  
步骤2: 选择预设
  - 射胶段 → THIN_WALL
  - 保压段 → THICK_WALL 或 STANDARD
  - 储料段 → PLASTICATION
  
步骤3: 写入参数
  - SetParameter(PRESSURE_CONTROLLER_TYPE, RBF_PID)
  - SetParameter(RBF_PRESET, 选择的预设)
  - SetParameter(PRESSURE_SYSTEM_GAIN, K)
  - SetParameter(PRESSURE_PLANT_TAU, τ)
  
步骤4: 试运行
  - 观察压力曲线
  - 检查超调量 < 5%
  - 稳态纹波 < 1bar
```

#### 4.2.2 微调（可选）
```
现象：响应太慢
  → 提高激进度: SetParameter(RBF_AGGRESSIVENESS, 1.5)
  
现象：有轻微超调
  → 降低激进度: SetParameter(RBF_AGGRESSIVENESS, 0.8)
  
现象：增益发散（报警）
  → 切换预设: THIN_WALL → STANDARD
  → 或降低激进度到 0.5
  
现象：多段保压切换抖动
  → 使用 THICK_WALL 预设（保守学习）
```

---

## 五、实施计划

### 5.1 Phase 1: 核心代码实现（1周）

#### 任务1.1: 新增枚举和参数（1天）
- [ ] `include/common_types.h`: 新增`HYD_RbfPreset`枚举
- [ ] `include/common_types.h`: `HYD_ParameterType`新增3个参数
- [ ] `include/motion_control.h`: `HYD_MotionParams`新增字段
  ```c
  HYD_RbfPreset rbfPreset;
  HYD_REAL rbfAggressiveness;
  HYD_BOOL rbfEnableAdaptation;
  HYD_BOOL rbfPresetDirty;
  ```

#### 任务1.2: 预设配置表（1天）
- [ ] `src/pressure_controller.c`: 定义4套预设常量
  - `HYD_RBF_PRESET_THIN_WALL_CONFIG`
  - `HYD_RBF_PRESET_THICK_WALL_CONFIG`
  - `HYD_RBF_PRESET_STANDARD_CONFIG`
  - `HYD_RBF_PRESET_PLASTICATION_CONFIG`

#### 任务1.3: 参数推导函数（2天）
- [ ] `src/pressure_controller.c`: 实现`HYD_DeriveRbfConfigFromPreset()`
- [ ] `src/motion_control.c`: 集成到`HYD_ProduceControlOutputs()`
- [ ] 单元测试: 验证推导结果

#### 任务1.4: IEC接口扩展（1天）
- [ ] `src/motion_control.c`: 新增3个参数的`case`分支
- [ ] 参数校验（范围检查）
- [ ] 默认值设置

#### 任务1.5: 收敛监控与回退（2天）
- [ ] `src/rbf_pid.c`: 新增`RBF_ConvergenceMonitor`结构
- [ ] `src/rbf_pid.c`: 实现收敛检测逻辑
- [ ] `src/pressure_controller.c`: 集成自动回退机制
- [ ] 新增诊断码: `HYD_DIAG_CODE_RBF_DIVERGENCE`

---

### 5.2 Phase 2: 文档与测试（3天）

#### 任务2.1: 应用场景指南（1天）
- [ ] 编写`docs/pressure-controller-selection-guide.md`
  - 决策树
  - IEC配置示例
  - 常见问题FAQ

#### 任务2.2: 调试手册（1天）
- [ ] 编写`docs/rbf-pid-tuning-guide.md`
  - 快速启动流程
  - 微调方法
  - 故障排查

#### 任务2.3: 单元测试（1天）
- [ ] `tests/test_rbf_preset.c`: 验证预设推导
- [ ] `tests/test_rbf_convergence.c`: 验证收敛监控
- [ ] `tests/test_rbf_fallback.c`: 验证自动回退

---

### 5.3 Phase 3: 现场试点（4周）

#### 任务3.1: 选择试点机型（1周）
- [ ] 汽车灯罩薄壁件（射胶段RBF）
- [ ] 医疗器械厚壁件（保压段RBF）
- [ ] 家电外壳标准件（保压段FF_PI对比）

#### 任务3.2: 数据采集（2周）
- [ ] 性能指标: CPK（压力一致性）
- [ ] 性能指标: 上升时间tr、超调量Mp
- [ ] 调试成本: 现场工程师耗时
- [ ] 故障率: 发散次数、回退次数

#### 任务3.3: 数据分析（1周）
- [ ] 对比RBF vs FF_PI性能
- [ ] 评估调试时间改善（目标:<15min）
- [ ] 收集现场反馈

---

### 5.4 Phase 4: 决策与推广（2周）

#### 决策标准
| 指标 | 阈值 | 决策 |
|------|------|------|
| CPK提升 | >20% | ✅ 推广到标准产品线 |
| CPK提升 | 10-20% | ⚠️ 保留为高端功能 |
| CPK提升 | <10% | ❌ 移除（技术债务清理） |
| 调试时间 | <15min | ✅ 现场可独立操作 |
| 故障率 | <2× PI | ✅ 可靠性达标 |

#### 推广行动
- [ ] 更新产品手册
- [ ] 编写培训PPT（面向销售/技术支持）
- [ ] 录制调试视频（15分钟版）
- [ ] 发布技术通告

---

## 六、验收标准

### 6.1 功能验收

| 项目 | 验收标准 | 测试方法 |
|------|----------|----------|
| **参数简化** | 从147→5个必填参数 | 检查IEC接口文档 |
| **预设可用** | 4套预设可选 | 单元测试覆盖 |
| **自动推导** | 预设→147参数无错 | 对比手动整定结果 |
| **收敛监控** | 发散10拍内检测 | 注入噪声测试 |
| **自动回退** | 发散后切换到PI | 故障注入测试 |

### 6.2 性能验收

| 场景 | 指标 | 目标 | 测试条件 |
|------|------|------|----------|
| **薄壁射胶** | Mp | <5% | K=1.5, τ=1.0s |
| **薄壁射胶** | tr | <800ms | 0→150bar |
| **厚壁保压** | σ_ss | <1bar | 150bar恒定 |
| **多段保压** | 切换抖动 | <3bar | 150→100→50bar |
| **储料背压** | 批次适应 | <5% MFI偏差 | ±10% MFI变化 |

### 6.3 工程验收

| 项目 | 验收标准 | 验收人 |
|------|----------|--------|
| **调试时间** | <15分钟（现场工程师独立完成） | 技术支持部 |
| **文档完整性** | 场景指南+调试手册+IEC示例 | 产品经理 |
| **故障率** | 发散率 < 传统PI的2倍 | 质量部 |
| **CPK提升** | 射胶段CPK提升 ≥ 20% | 工艺部 |

---

## 七、风险与缓解

### 7.1 技术风险

| 风险 | 概率 | 影响 | 缓解措施 |
|------|------|------|----------|
| 预设参数不适配某机型 | 中 | 中 | 保留CUSTOM专家模式 |
| 收敛监控误判 | 低 | 中 | 增加手动回退接口 |
| 自动回退到PI后性能下降 | 中 | 低 | 记录诊断，技术支持介入 |

### 7.2 项目风险

| 风险 | 概率 | 影响 | 缓解措施 |
|------|------|------|----------|
| 试点客户拒绝 | 低 | 高 | 选择合作紧密客户 |
| 现场数据采集不全 | 中 | 中 | 提供数据采集工具包 |
| 开发周期延误 | 中 | 中 | 每周评审，及时调整 |

---

## 八、总结

### 核心思路
1. **保持架构不变**: 复用现有IEC参数接口（`HYD_PARAM_PRESSURE_CONTROLLER_TYPE`）
2. **新增简化模式**: 通过`HYD_PARAM_RBF_PRESET`一键选择，自动推导147个内部参数
3. **专家模式并存**: `HYD_RBF_PRESET_CUSTOM`保留手动整定能力
4. **故障自愈**: 收敛监控+自动回退机制，降低现场风险
5. **数据驱动决策**: 4周试点收集CPK数据，决定是否推广

### 预期收益
- **调试时间**: 2-4小时 → **15分钟** ✅
- **参数复杂度**: 147个 → **5个** ✅
- **射胶段CPK**: 提升 **20%+** ✅
- **故障率**: 与PI相当（自动回退保障） ✅

### 关键成功因素
1. ✅ 预设参数表经过充分仿真验证
2. ✅ 收敛监控阈值经过现场数据标定
3. ✅ 文档清晰、示例完整
4. ✅ 试点客户配合度高

---

**准备好开始实施了吗？** 🚀
