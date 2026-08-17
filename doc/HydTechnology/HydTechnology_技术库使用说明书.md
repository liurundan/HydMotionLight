# HydTechnology 技术库使用说明书

**文档版本**：V0.1（对应 `plc.xml` 中的 `productVersion=0.1.0.0`）  
**适用对象**：注塑机液压系统、电动轴、传感器及工艺应用工程师  
**编程语言**：IEC 61131-3 Structured Text（ST）  
**源码依据**：项目根目录 `plc.xml`，生成日期 2026-08-14

---

## 1. 概述

HydTechnology 是面向液压注塑机技术应用库的 IEC 功能块集合，当前源码包含：

- 传感器信号处理：位置传感器、压力传感器、低通/中值/限幅/滑动平均滤波。
- 液压轴封装：把创建液压轴、位置连续运动、速度控制、压力控制、停止、急停、复位、反馈写入和反馈读取封装为一个状态机功能块。
- 电动轴/伺服泵控制：完成驱动器参数读取、编码器分辨率配置、仿真配置、自学习、速度跟随和减速停机。
- 液压轴最大速度计算：根据油缸有效面积和泵参数计算伸出/缩回方向的理论最大速度。

本库是“应用层封装”，并不替代底层运动库、PLCopen 运动控制库或现场总线驱动库。`plc.xml` 中引用的 `HYD_*`、`MC_*`、`SMC_*`、`PorgaSDOUpload`、`StartMotorSelfLearn` 等类型必须由目标 PLC 平台或 HydMotion/驱动器库提供。

### 1.1 工程结构

| 内容 | 数量 | 说明 |
|---|---:|---|
| 自定义数据类型 | 4 | `E_FilterType`、`ST_FilterConfig`、`ST_SensorConfig`、`ST_HydAxisRef` |
| 自定义功能块 | 9 | 见第 4～7 章 |


### 1.2 调用原则

1. 每个功能块实例必须在其所属任务中**每个扫描周期调用一次**，不要只在启动沿调用。
2. 配置类方法（例如传感器 `Init`、`SetFilter`）应在启动或参数修改时调用；周期运行逻辑仍需持续调用主功能块。
3. 运动命令使用上升沿触发。对同一实例重复执行同一个命令，必须先让 `EXECUTE` 回到 `FALSE`，再重新置 `TRUE`。
4. 所有 `REAL`、`UINT`、`UDINT` 参数的物理单位由应用工程统一约定；本库内部的换算约定见各功能块章节。
5. 运动控制和传感器采样必须使用稳定、已知的任务周期。当前代码中存在固定 `1ms` 假设，而示例任务配置为 `20ms`，见第 9.2 节。

### 1.3 依赖库

| 依赖类型 | 本库使用的符号 | 用途 |
|---|---|---|
| 液压 IEC 库 | `HYD_CreateMotion`、`HYD_MoveContinuousAbsolute`、`HYD_MoveVelocity`、`HYD_PressureHandle`、`HYD_Stop`、`HYD_Reset`、`HYD_ReadSimFeedback`、`HYD_WriteParameter`、`HYD_SetAxisFeedback` | 液压轴创建、运动、压力、停止、复位、反馈和参数写入 |
| PLCopen/平台运动库 | `AXIS_REF`、`MC_Power`、`MC_WriteParameter`、`MC_WriteBoolParameter`、`MC_Reset`、`MC_SetControllerMode`、`MC_ReadActualVelocity`、`MC_ReadActualPosition`、`MC_ReadActualTorque`、`SMC_FollowVelocity` | 电动轴/伺服泵控制 |
| 驱动器扩展库 | `PorgaSDOUpload`、`StartMotorSelfLearn`、`PAR_WORD_ARRAY` | 读取编码器和旋转方向、执行电机自学习、传递电机参数 |
| 标准库 | `R_TRIG`、`TON`、`LIMIT`、`MAX` 及类型转换函数 | 边沿检测、延时、限幅和数值转换 |

如果编译器提示上述类型不存在，应先安装并引用对应底层库；不要在应用工程中重新声明同名类型。

---

## 2. 数据类型

### 2.1 `E_FilterType`：滤波类型枚举

| 枚举值 | 含义 | 说明 |
|---|---|---|
| `FTNone` | 不滤波 | 直接输出换算后的原始值 |
| `FTLowPass` | 一阶低通滤波 | 使用 `ST_FilterConfig.rParam1` 作为 `alpha` |
| `FTMedian` | 三点中值滤波 | 固定 3 个采样点，抗脉冲毛刺 |
| `FTLimit` | 限幅滤波 | 单次变化超过阈值时保持上次输出 |
| `FTMovingAverage` | 滑动窗口均值 | 使用 `uiParam2` 个采样点，最大 100 |
| `FTLowPassAndMovingAverage` | 低通后滑动平均 | 先低通，再做滑动平均 |

### 2.2 `ST_FilterConfig`：滤波配置

```pascal
TYPE ST_FilterConfig : STRUCT
    eType    : E_FilterType;
    rParam1  : REAL := 0.1;
    uiParam2 : UINT := 3;
END_STRUCT;
END_TYPE
```

| 字段 | 类型 | 默认值 | 用途 |
|---|---|---:|---|
| `eType` | `E_FilterType` | `FTNone`（枚举默认值） | 选择滤波算法 |
| `rParam1` | `REAL` | `0.1` | 低通 `alpha` 或限幅最大允许变化量 |
| `uiParam2` | `UINT` | `3` | 滑动平均窗口大小；运行时被限制在 `1..100` |

参数建议：

- 低通滤波要求 `0.0 <= rParam1 <= 1.0`。代码未自动限幅，工程师必须自行保证范围。
- `rParam1=1.0` 基本等同于不滤波，`rParam1` 越小响应越慢。
- 中值滤波不使用两个参数，建议仍传入稳定的默认值。
- `uiParam2=0` 会被功能块改为 `1`；大于 `100` 会被改为 `100`。

### 2.3 `ST_SensorConfig`：传感器标定配置

| 字段 | 类型 | 含义 |
|---|---|---|
| `rZeroVoltage` | `REAL` | 零点标定值 |
| `rMaxVoltage` | `REAL` | 满量程标定值 |
| `rRange` | `REAL` | 工程量程，例如 `500.0 mm` 或 `10.0 MPa` |

当前实现的换算公式为：

```text
rRawValue = LIMIT(0, (uiRawADC - rZeroVoltage)
                      / (rMaxVoltage - rZeroVoltage) * rRange,
                   rRange)
```

注意：字段名带有 `Voltage`，但源码实际直接使用 `uiRawADC` 的数值，没有使用 `cADC_MAX=65535` 或 `cVOLTAGE_MAX=10.0` 做 ADC 到电压的换算。因此当前版本应将 `rZeroVoltage`、`rMaxVoltage` 按 **ADC 原始计数** 配置；如果上层传入的是 0～10 V 工程值，必须先在应用层换算，或修改库实现后再使用。

当 `rMaxVoltage <= rZeroVoltage` 时，功能块置 `bAlarm=TRUE`，原始值输出 0。

### 2.4 `ST_HydAxisRef`：液压轴双向数据通道

该结构通过 `FB_HydAxis` 的 `VAR_IN_OUT` 传递命令、配置和反馈。建议一个液压轴对应一个独立实例，不要多个轴共享同一个结构变量。

#### 命令与设定字段

| 字段 | 类型 | 方向 | 单位/约定 |
|---|---|---|---|
| `bFunBlockStart` | `BOOL` | 预留 | 当前 `FB_HydAxis` 未使用 |
| `bStart` | `BOOL` | 内部镜像 | `FB_HydAxis` 将输入 `bStart` 写入此字段 |
| `bStop` | `BOOL` | 内部镜像 | 将输入 `bStop` 写入此字段 |
| `uiDir` | `UINT` | 设定 | 方向：0 无效，1 正向，2 反向；底层库具体枚举需一致 |
| `udiPos` | `UDINT` | 设定 | 位置，代码按 `0.01` 缩放为 `REAL` |
| `uiSpd` | `UINT` | 设定 | 速度百分比，代码按 `uiSpd/1000 * rMaxSpeed` 换算 |
| `uiEndSpd` | `UINT` | 设定 | 终点速度百分比，同上；用于连续轨迹平滑过渡 |
| `uiPres` | `UINT` | 设定 | 压力设定，代码按 `uiPres * 0.1` 换算 |

#### 反馈字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `rPosition` | `REAL` | 从 `HYD_ReadSimFeedback` 读出的反馈位置 |
| `rVeloctity` | `REAL` | 反馈速度（字段名按源码拼写） |
| `rPerssure` | `REAL` | 反馈压力（字段名按源码拼写） |
| `rFlow` | `REAL` | 反馈流量 |
| `iStep` | `INT` | `FB_HydAxis` 当前状态步号 |
| `rActPressure` | `REAL` | 写入底层 `HYD_SetAxisFeedback` 的实际压力 |
| `rActPosition` | `REAL` | 写入底层的实际位置 |
| `rActVelocity` | `REAL` | 写入底层的实际速度 |

#### 液压轴配置字段

| 字段 | 类型 | 默认值 | 说明 |
|---|---|---:|---|
| `rMaxSpeed` | `REAL` | 0 | 当前轴最大速度，单位 mm/s；运动换算实际使用该字段 |
| `rMaxSpeedExtend` | `REAL` | 0 | 初始化时由 `FB_CalcAxisMaxSpeed` 计算并写入 |
| `rMaxSpeedRetract` | `REAL` | 0 | 初始化时由 `FB_CalcAxisMaxSpeed` 计算并写入 |
| `rVelToFlowGain` | `REAL` | 0.2 | 传给底层参数号 5 的速度/流量增益 |
| `rCylinderExtendArea` | `REAL` | 0 | 伸出腔有效面积，单位 mm²，传给参数号 31 |
| `rCylinderRetractArea` | `REAL` | 0 | 缩回腔有效面积，单位 mm²，传给参数号 32 |
| `iTypePressurePID` | `SINT` | 4 | 压力闭环类型，传给参数号 25；具体枚举由底层库定义 |
| `iAxisID` | `SINT` | -1 | 创建后写入底层轴索引 |
| `iMechanismType` | `SINT` | 0 | 机械运动学类型；值 1 会应用曲肘变速系数 |
| `udiPosToleranceValue` | `UDINT` | 1 | 位置容差预留字段，当前 `FB_HydAxis` 未直接使用 |

当前代码在运动换算时直接使用 `rMaxSpeed`；虽然初始化会计算 `rMaxSpeedExtend` 和 `rMaxSpeedRetract`，但按方向选择最大速度的代码被注释。因此应用工程应在启动前正确填写 `rMaxSpeed`，不能仅依赖两个方向计算结果。

---

## 3. 通用滤波功能块

### 3.1 调用约定

滤波功能块都是有状态实例。必须保留同一个实例连续调用，不能每个周期重新声明或复制实例，否则首周期状态、循环缓冲区和历史输出会丢失。

### 3.2 `FB_LimitFilter`：限幅滤波

**接口**

| 参数 | 类型 | 方向 | 含义 |
|---|---|---|---|
| `rInput` | `REAL` | IN | 当前输入 |
| `rMaxDiff` | `REAL` | IN | 允许相对上次输出的最大变化量 |
| `rOutput` | `REAL` | OUT | 滤波输出 |

算法：首周期直接输出输入；之后若 `ABS(rInput-rOutput) > rMaxDiff`，保持上次输出，否则接受新输入。它不是斜坡限制器，不会逐步逼近异常值。

```pascal
VAR
    fbLimit : FB_LimitFilter;
    rPosRaw : REAL;
    rPos    : REAL;
END_VAR

fbLimit(rInput := rPosRaw, rMaxDiff := 2.0);
rPos := fbLimit.rOutput;
```

注意：`rMaxDiff < 0` 会使所有正常变化都被保持，应在配置层禁止负值；传感器启动时的首个样本无异常判断。

### 3.3 `FB_LowPassFilter`：一阶低通滤波

| 参数 | 类型 | 方向 | 含义 |
|---|---|---|---|
| `rInput` | `REAL` | IN | 当前输入 |
| `rAlpha` | `REAL` | IN | 滤波系数，推荐 0～1 |
| `rOutput` | `REAL` | OUT | 滤波输出 |

算法：`y(k)=y(k-1)+alpha*(x(k)-y(k-1))`。首周期 `y=x`。

```pascal
fbLowPass(rInput := rPressureRaw, rAlpha := 0.15);
pressure := fbLowPass.rOutput;
```

`alpha=0` 会锁住首个值，`alpha>1` 会产生过冲，源码不主动校验，必须由应用层限制。

### 3.4 `FB_MedianFilter`：三点中值滤波

| 参数 | 类型 | 方向 | 含义 |
|---|---|---|---|
| `rInput` | `REAL` | IN | 当前输入 |
| `rOutput` | `REAL` | OUT | 三点排序后的中间值 |

内部固定 `ARRAY[1..3] OF REAL` 环形缓冲区，每周期写入一个点并对 3 点排序。适合去除单点尖峰，不适合替代低通滤波。

```pascal
fbMedian(rInput := pressureRaw);
pressure := fbMedian.rOutput;
```

上电初期缓冲区未填满，初始零值会参与排序；如需避免启动瞬态，应在工艺状态机中等待 3 个采样周期后再使用输出。

### 3.5 `FB_MovingAverageFilter`：滑动窗口均值

| 参数 | 类型 | 方向 | 含义 |
|---|---|---|---|
| `rInput` | `REAL` | IN | 当前输入 |
| `uiSize` | `UINT` | IN | 窗口大小，运行时限制为 1～100 |
| `rOutput` | `REAL` | OUT | 窗口平均值 |

内部缓冲区固定为 `ARRAY[1..100] OF REAL`，每周期对当前窗口求和。窗口扩展时未填充元素为 0，启动初期输出会被零值拉低；需要无启动偏差时，应在应用层预充缓冲区或使用较小窗口逐步增大。

```pascal
fbAverage(rInput := flowRaw, uiSize := 8);
flow := fbAverage.rOutput;
```

---

## 4. 传感器功能块

### 4.1 `FB_PositionSensor`：位置传感器

**接口**

| 参数 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `uiRawADC` | `UINT` | IN | ADC 原始计数 |
| `rActualValue` | `REAL` | OUT | 滤波后的最终位置值 |
| `rActualValueDeriv` | `REAL` | OUT | 一阶差分，单位为工程量/s |
| `rRawValue` | `REAL` | OUT | 未滤波的换算值，便于诊断 |
| `bAlarm` | `BOOL` | OUT | 标定参数无效时为 TRUE |

内部使用 `ST_SensorConfig` 和 `ST_FilterConfig`。位置导数固定按 `cTCycle=0.001` 计算：

```text
rActualValueDeriv = (rActualValue - rLastActualValue) / 0.001
```

**公共方法**

```pascal
Init(rZero, rMax, rRng) : BOOL
SetFilter(eType, rParam1, uiParam2) : BOOL
```

示例：

```pascal
VAR
    fbPos : FB_PositionSensor;
    bInit : BOOL;
    bCfg  : BOOL;
END_VAR

IF FirstScan THEN
    bInit := fbPos.Init(rZero := 820.0, rMax := 59300.0, rRng := 500.0);
    bCfg  := fbPos.SetFilter(
        eType := FTLowPassAndMovingAverage,
        rParam1 := 0.2,
        uiParam2 := 5);
END_IF;

fbPos(uiRawADC := aiPosition);
position_mm := fbPos.rActualValue;
position_speed_mm_s := fbPos.rActualValueDeriv;
IF fbPos.bAlarm THEN
    // 进入传感器故障处理
END_IF;
```

注意：

- `Init` 和 `SetFilter` 只写入内部结构，不会产生异步 Busy/Done，返回值当前恒为 TRUE。
- `bAlarm` 只反映 `rMaxVoltage-rZeroVoltage<=0`，不包含断线、超范围、通讯丢帧等硬件诊断。
- `bUpperLimit`、`bLowerLimit` 在当前实现中只是预留变量，尚未形成输出报警。
- 任务周期若不是 1 ms，`rActualValueDeriv` 的数值比例会错误，必须修改库常量或在应用层重新计算。

### 4.2 `FB_PressureSensor`：压力传感器

接口与位置传感器基本一致，但没有导数输出：

| 参数 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `uiRawADC` | `UINT` | IN | ADC 原始计数 |
| `rActualValue` | `REAL` | OUT | 滤波后的压力 |
| `rRawValue` | `REAL` | OUT | 未滤波压力 |
| `bAlarm` | `BOOL` | OUT | 标定参数无效报警 |

配置方法相同：

```pascal
fbPress.Init(rZero := 4000.0, rMax := 60000.0, rRng := 250.0);
fbPress.SetFilter(FTLowPass, 0.12, 1);
fbPress(uiRawADC := aiPressure);
pressure_mpa := fbPress.rActualValue;
```

`rRange` 的单位由工程定义；若液压系统内部使用 bar、MPa 或 kgf/cm²，必须在变量命名和 HMI 标定表中保持一致。

---

## 5. 电动轴/伺服泵功能块

### 5.1 `FB_ServoControl`：电机速度控制封装

**接口**

| 参数 | 类型 | 方向 | 含义 |
|---|---|---|---|
| `StartMotor` | `BOOL` | IN | TRUE 使能并运行；FALSE 进入减速停机 |
| `SetVelCmd` | `REAL` | IN | 速度指令，源码先限幅再换算为驱动器速度单位 |
| `FlowRateLimit` | `UINT` | IN | 流量比例限制，0～1000 对应 0～100% |
| `uiMaxMotorSpeed` | `UINT` | IN | 电机最大转速 rpm |
| `EnSimulation` | `BOOL` | IN | 写入底层参数 1000 的仿真开关 |
| `SelfLearn` | `BOOL` | IN | 是否执行驱动器电机自学习 |
| `UsePlanner` | `BOOL` | IN | 当前实现未使用，保留接口 |
| `tcycle` | `REAL` | IN | 周期参数，当前实现未用于运算，默认 0.001 |
| `ActSpeedRPM` | `UINT` | OUT | 实际速度，按 `dfActVel*60/360` 换算 |
| `ActPressBAR` | `UINT` | OUT | 当前实现未赋值，保持默认值 |
| `outErrorID` | `UINT` | OUT | 运动功能块错误号 |
| `Status` | `UINT` | OUT | 内部状态机编号 |
| `ActMotorPos` | `UINT` | OUT | 当前实现未赋值；接口注释标明单位为 360° |
| `ActTorquePercent` | `UINT` | OUT | 实际转矩，接口注释为 0.1% |

#### 状态机

| `Status` | 状态 | 说明 |
|---:|---|---|
| 0 | 初始化 | 读取编码器分辨率 SDO 19 |
| 1 | 等待 SDO1 | 计算编码器分辨率 |
| 2 | 等待 SDO2 | 读取旋转方向 SDO 106 |
| 10 | 写参数 | 写参数 1003（编码器分辨率）、1000（仿真） |
| 20 | 自学习 | `SelfLearn=TRUE` 时执行自学习 |
| 30 | 设置模式 | 设置 `mcFollowVelocity` |
| 40 | 待机 | 等待驱动器 Power 状态有效 |
| 50 | 速度运行 | 执行 `SMC_FollowVelocity` |
| 60 | 减速停机 | 目标速度置 0，低于 5 UU/s 持续 1 s 后退出速度控制 |
| 99 | 错误 | 停止速度命令，保留错误状态 |

速度换算核心逻辑：

```text
MaxVel           = uiMaxMotorSpeed * 6.0              // 360/60
fMotorSpeedLimit = uiMaxMotorSpeed * LIMIT(FlowRateLimit,0,1000) * 0.001
fSetVel          = LIMIT(-100.0, SetVelCmd, fMotorSpeedLimit)
fSetVel          = fSetVel * dir                         // 当前 dir=-1
dfSetVel         = fSetVel * 6.0                         // UU/s
```

**调用示例**

```pascal
VAR
    fbServo : FB_ServoControl;
END_VAR

fbServo(
    StartMotor       := pumpEnable,
    SetVelCmd        := pumpSpeedCmd,
    FlowRateLimit    := 850,
    uiMaxMotorSpeed  := 1800,
    EnSimulation     := FALSE,
    SelfLearn        := firstCommissioning,
    UsePlanner       := FALSE,
    tcycle           := 0.001);

pump_speed_rpm := fbServo.ActSpeedRPM;
drive_status := fbServo.Status;
IF fbServo.outErrorID <> 0 THEN
    // 记录驱动器错误并禁止继续启动
END_IF;
```

注意事项：

- `nAxisRef` 是功能块内部变量，当前初始化为 0；本版本没有公开轴引用输入。若目标系统不是固定轴 0，需要扩展源码或在平台中映射内部轴引用。
- `wDirRotate` 读取后没有参与 `dir` 计算，当前方向固定为 `-1.0`。若驱动器安装方向变化，不能仅依靠 SDO 106 自动修正。
- `UsePlanner`、`tcycle`、`ActPressBAR`、`ActMotorPos` 当前没有实际逻辑，应用工程不应把它们当作已实现功能。
- 失去 `StartMotor` 后会减速停机；重新置 TRUE 可在停机过程中恢复运行。
- 错误状态没有公开复位输入，若 `SMC_FollowVelocity.Error` 进入 99，需要由上层重新实例化、扩展复位逻辑或调用底层复位功能块。

---

## 6. 液压轴封装

### 6.1 `FB_CalcAxisMaxSpeed`：计算液压轴理论最大速度

**接口**

| 参数 | 类型 | 方向 | 默认值 | 单位/说明 |
|---|---|---|---:|---|
| `rAreaExtendSide` | `REAL` | IN | - | 伸出侧有效面积，mm² |
| `rAreaRetractSide` | `REAL` | IN | - | 缩回侧有效面积，mm² |
| `rPumpMaxRPM` | `REAL` | IN | 1800 | 泵最大转速，rpm |
| `rPumpDisplacement` | `REAL` | IN | 50 | 泵排量，cc/rev |
| `iMechanismType` | `SINT` | IN | 0 | 机械类型；1 应用曲肘变速比 |
| `rMaxSpeedExtend` | `REAL` | OUT | - | 伸出最大速度，mm/s |
| `rMaxSpeedRetract` | `REAL` | OUT | - | 缩回最大速度，mm/s |
| `bValid` | `BOOL` | OUT | - | 参数有效标志 |

计算公式：

```text
Qmax = rPumpDisplacement * rPumpMaxRPM             // cc/min
Vext = Qmax * 16.667 / rAreaExtendSide             // mm/s
Vret = Qmax * 16.667 / rAreaRetractSide             // mm/s
```

当 `iMechanismType=1` 时，继续乘以：伸出 `0.73`，缩回 `1.18`。面积、排量必须为正，最大转速不得小于 0；参数非法时两个速度为 0、`bValid=FALSE`。

```pascal
VAR
    fbMaxSpeed : FB_CalcAxisMaxSpeed;
END_VAR

fbMaxSpeed(
    rAreaExtendSide  := 7850.0,
    rAreaRetractSide := 5026.0,
    rPumpMaxRPM      := 1800.0,
    rPumpDisplacement := 50.0,
    iMechanismType   := 0);

IF fbMaxSpeed.bValid THEN
    axisRef.rMaxSpeedExtend := fbMaxSpeed.rMaxSpeedExtend;
    axisRef.rMaxSpeedRetract := fbMaxSpeed.rMaxSpeedRetract;
END_IF;
```

该功能块只计算理论上限，不包含阀口饱和、压力补偿、泄漏、温度、负载和安全限速。实际工艺速度还应乘以工艺限幅系数。

### 6.2 `FB_HydAxis`：液压轴统一状态机

#### 接口

| 参数 | 类型 | 方向 | 含义 |
|---|---|---|---|
| `stHydAxisRef` | `ST_HydAxisRef` | IN_OUT | 共享命令/配置/反馈结构 |
| `bStart` | `BOOL` | IN | 上升沿启动或触发下一段 |
| `bStop` | `BOOL` | IN | 上升沿正常减速停止 |
| `bEStop` | `BOOL` | IN | 上升沿急停，使用极大减速度 |
| `bReset` | `BOOL` | IN | 报警状态下上升沿复位 |
| `uiDir` | `UINT` | IN | 方向，0 无效，1 正向，2 反向 |
| `uiMode` | `UINT` | IN | 1 位置，2 速度，3 压力 |
| `uiPres` | `UINT` | IN | 压力设定，内部乘 0.1 |
| `uiSpd` | `UINT` | IN | 速度百分比，内部除以 1000 |
| `uiEndSpd` | `UINT` | IN | 终点速度百分比 |
| `udiPos` | `UDINT` | IN | 位置设定，内部乘 0.01 |
| `uiForce` | `UINT` | IN | 当前实现未使用，保留 |
| `uiAcc` | `UINT` | IN | 加速度参数；0 使用默认 200 |
| `uiDec` | `UINT` | IN | 减速度参数；0 使用默认 200 |
| `uiJerk` | `UINT` | IN | Jerk 参数；0 表示 0 |
| `kkk` | `UINT` | IN | 当前实现未使用，保留 |
| `bBusy` | `BOOL` | OUT | 轴仍有有效执行命令 |
| `bDone` | `BOOL` | OUT | 位置模式为到达终点速度；速度/压力模式为达到对应目标 |
| `bInVelocity` | `BOOL` | OUT | 速度功能块已达到目标速度 |
| `bInPressure` | `BOOL` | OUT | 压力功能块已达到目标压力 |
| `bAlarm` | `BOOL` | OUT | 当前有报警 |
| `dwAlarmID` | `DWORD` | OUT | 报警号或底层错误号 |

#### 内部状态

| 步号 | 名称 | 说明 |
|---:|---|---|
| 0 | 初始化 | 创建液压轴，写参数 5/25/31/32，并计算最大速度 |
| 10 | 空闲 | 等待启动、停止、急停或复位沿 |
| 100 | 运行 | 根据 `uiActiveMode` 执行位置/速度/压力命令 |
| 105 | 运行过渡 | 保持一个周期 `Execute=FALSE`，形成下一段上升沿 |
| 200 | 正常停止 | `HYD_Stop`，减速度为计算值 |
| 300 | 急停 | `HYD_Stop`，减速度固定为 `99999.0` |
| 400 | 报警复位 | `HYD_Reset` |
| 900 | 报警等待 | 等待 `bReset` 上升沿 |

#### 参数换算

```text
rPres   = uiPres * 0.1
rVel    = uiSpd    * 0.1 * 0.01 * rMaxSpeed = uiSpd    / 1000 * rMaxSpeed
rEndVel = uiEndSpd * 0.1 * 0.01 * rMaxSpeed = uiEndSpd / 1000 * rMaxSpeed
rPos    = udiPos * 0.01
rAcc    = (uiAcc=0) ? 200 : rMaxSpeed/uiAcc*1000
rDec    = (uiDec=0) ? 200 : rMaxSpeed/uiDec*1000
rJerk   = (uiJerk=0) ? 0 : rMaxSpeed/uiJerk
```

请在 HMI 和配方层明确这些缩放关系。例如 `uiSpd=500` 表示 `rMaxSpeed` 的 50%，而不是 500 mm/s。

#### 基本调用示例

```pascal
VAR
    fbAxis : FB_HydAxis;
    axisRef : ST_HydAxisRef;
END_VAR

// 启动前配置（一次性或配方切换时）
axisRef.rMaxSpeed := 120.0;          // mm/s
axisRef.rVelToFlowGain := 0.2;
axisRef.rCylinderExtendArea := 7850.0;
axisRef.rCylinderRetractArea := 5026.0;
axisRef.iTypePressurePID := 4;
axisRef.iMechanismType := 0;

// 每周期调用
fbAxis(
    stHydAxisRef := axisRef,
    bStart := cmdStart,
    bStop := cmdStop,
    bEStop := cmdEStop,
    bReset := cmdReset,
    uiDir := 1,
    uiMode := 1,
    uiPres := 800,
    uiSpd := 700,
    uiEndSpd := 700,
    udiPos := 12500,   // 125.00 mm
    uiForce := 0,
    uiAcc := 50,
    uiDec := 50,
    uiJerk := 0,
    kkk := 0);

IF fbAxis.bDone THEN
    // 进入下一工艺段
END_IF;
IF fbAxis.bAlarm THEN
    // 锁定动作并显示 DWORD_TO_WORD(fbAxis.dwAlarmID)
END_IF;
```

#### 三种模式

1. **位置模式 `uiMode=1`**：调用 `HYD_MoveContinuousAbsolute`，目标位置为 `rPos`，目标速度为 `rVel`，终点速度为 `rEndVel`。`bDone` 映射到底层 `InEndVelocity`，它代表到达段末并满足终点速度条件，不一定等同于“速度已经降为 0”。
2. **速度模式 `uiMode=2`**：调用 `HYD_MoveVelocity`，`bInVelocity` 映射到底层 `InVelocity`。这是持续运行模式，正常情况下不会自动产生位置意义上的完成。
3. **压力模式 `uiMode=3`**：调用 `HYD_PressureHandle`，压力斜坡率使用 `rAcc`，持续时间固定为 `0.0`，即持续到停止或被其他命令取代；`bInPressure` 映射到底层 `INPRESSURE`。

同一模式下再次产生 `bStart` 上升沿会进入 105 步，先清低 `Execute` 一个周期，再重新启动，以支持位置连续轨迹的分段切换。跨模式切换直接修改 `uiActiveMode`，是否允许无缝切换取决于底层 HYD 库实现；若底层不允许，应在应用层先执行停止。
- HYD_MoveContinuousAbsolute，更新目标设定参数，需要适用上升沿重新触发，让目标输入参数生效；
- HYD_MoveVelocity和HYD_PressureHandle功能块在使用时，若开启连续更新功能时，即continuousupdate=1，则输入Execute需要一直保持true；

#### 公共方法

| 方法 | 参数 | 底层参数号 | 说明 |
|---|---|---:|---|
| `SetVelocityToFlowGain` | `rVelToFlowGain : REAL` | 5 | 写入速度/流量增益；返回底层 `Done` |
| `SetPidType` | `rTypePID : REAL` | 25 | 写入压力 PID 类型；返回底层 `Done` |
| `SetCylinderConfig` | `rExtendArea`、`rRetractArea : REAL` | 31、32 | 依次写入伸出/缩回有效面积 |

这些方法内部复用同一个 `HYD_WriteParameter` 实例。调用方法时仍应保持 `FB_HydAxis` 每周期运行，并按返回值判断底层写入是否完成。`SetCylinderConfig` 是两次异步写入，第一项未完成时不会启动第二项。

#### 报警与安全

- 创建失败：`dwAlarmID := 16#00000001`。
- 急停完成或急停错误：`dwAlarmID := 16#0000FFFF`，并进入报警等待。
- 运动/停止/复位底层错误：直接转发底层 `ERRORID`。
- `bEStop` 仅是软件运动停止命令，不等同于符合安全标准的硬件 STO、安全继电器或液压卸荷回路。危险区域必须由独立安全回路处理。
- `bStart`、`bStop`、`bEStop`、`bReset` 在功能块内部使用 `R_TRIG`，输入信号必须至少保持一个完整任务周期可见。
- 当前 `FB_HydAxis` 初始化内部固定调用 `HYD_CreateMotion(USE_SIMULATION := 0)`，因此不能通过外部输入直接切换为仿真模式；需要仿真时应扩展接口或直接使用底层 HYD 功能块。

---

## 7. 推荐工程调用模板

### 7.1 启动、采样、工艺、输出顺序

建议主任务按以下顺序组织：

```pascal
// 1. 首周期/配置阶段
IF FirstScan THEN
    // 初始化传感器标定、滤波参数、液压轴配置
END_IF;

// 2. 先采集并滤波传感器
fbPosition(uiRawADC := aiPosition);
fbPressure(uiRawADC := aiPressure);

// 3. 将实际反馈写入液压底层接口
axisRef.rActPosition := fbPosition.rActualValue;
axisRef.rActPressure := fbPressure.rActualValue;
axisRef.rActVelocity := fbPosition.rActualValueDeriv;

// 4. 工艺状态机给出 bStart/uiMode/uiSpd/uiPres/udiPos 等命令
fbHydAxis(...);

// 5. 伺服泵/电动轴控制
fbServo(...);

// 6. 输出到底层驱动器、阀或模拟量
pumpSpeedCommand := fbServo.ActSpeedRPM;
```

传感器和运动 FB 必须保持稳定的实例生命周期。工艺状态机只改变输入命令，不要在状态机中条件性跳过功能块调用。

### 7.2 典型注塑机动作序列

```pascal
CASE step OF
    0: // 合模快速
        uiMode := 1; uiDir := 1; uiSpd := 800;
        udiPos := 20000;
        IF fbHydAxis.bDone THEN step := 10; END_IF;

    10: // 合模低速找零/接触
        uiMode := 2; uiDir := 1; uiSpd := 100;
        IF moldTouch THEN step := 20; END_IF;

    20: // 高压锁模
        uiMode := 3; uiPres := 1200;
        IF fbHydAxis.bInPressure THEN step := 30; END_IF;

    30: // 射胶位置段
        uiMode := 1; uiDir := 1; uiSpd := 900;
        udiPos := 45000;
        IF fbHydAxis.bDone THEN step := 40; END_IF;

    40: // 保压
        uiMode := 3; uiPres := 800;
        IF holdTimer.Q THEN step := 50; END_IF;

    50: // 开模
        uiMode := 1; uiDir := 2; uiSpd := 600;
        udiPos := 0;
        IF fbHydAxis.bDone THEN step := 0; END_IF;
END_CASE;
```

实际项目中应在每个步骤加入位置、压力、传感器、超时和安全互锁条件，并在 `bAlarm`、硬件急停、门锁、低油位等条件下统一进入安全停机流程。

---

## 8. 参数、单位和边界检查清单

在交付应用工程前，至少确认以下项目：

- 传感器的 `rZeroVoltage`/`rMaxVoltage` 是否按当前实现要求使用 ADC 计数，而不是直接填入电压值。
- `rMaxVoltage > rZeroVoltage`，`rRange > 0`。
- 低通 `rAlpha` 在 0～1，限幅阈值非负，移动平均窗口在 1～100。
- `rMaxSpeed` 已赋值且大于 0；不能只填写 `rMaxSpeedExtend`/`rMaxSpeedRetract`。
- 油缸面积单位为 mm²，泵排量为 cc/rev，泵转速为 rpm。
- `uiSpd`、`uiEndSpd` 是否按 0～1000 的千分比使用；`udiPos` 是否按 0.01 位置单位使用。
- 压力 `uiPres*0.1` 的最终单位与压力传感器、底层 HYD 库一致。
- 所有运动命令输入沿至少保持一个完整扫描周期。
- 任务周期与代码中的 `1ms` 假设一致，或已完成相应改造。
- `FB_ServoControl` 的固定轴引用、方向符号和未实现输出已经在平台层验证。
- 软件急停不能替代硬件安全回路。

---

## 9. 当前版本限制与后续维护建议

以下内容是从 `plc.xml` 当前实现直接观察到的限制，后续更新库代码时应同步更新本文档：

1. `FB_PositionSensor`、`FB_PressureSensor` 定义了 ADC/电压常量，但换算公式未使用它们；建议后续统一为“ADC 计数→电压→工程量”的明确两步模型。
2. 传感器导数固定按 1 ms 计算；建议增加 `tCycle` 输入或使用平台周期变量。
3. `FB_HydAxis` 已计算伸出/缩回最大速度，但按方向选择的代码处于注释状态；建议恢复方向相关限速并增加实际速度上限。
4. `FB_HydAxis.uiForce`、`kkk`、`bFunBlockStart`、`udiPosToleranceValue` 目前为预留接口，应在文档和实现中保持一致。
5. `FB_ServoControl` 的 `UsePlanner`、`tcycle`、`ActPressBAR`、`ActMotorPos` 目前未参与有效逻辑；若对外承诺这些功能，应补齐实现和测试。


### 9.1 建议的回归测试

- 每个滤波器：首周期、突变输入、边界参数、长时间运行和复位后的状态。
- 传感器：零点、满量程、反向输入、非法标定、滤波切换和任务周期变化。
- `FB_CalcAxisMaxSpeed`：非法面积、零排量、机械类型 0/1 和数值单位换算。
- `FB_HydAxis`：初始化、三种模式启动、同模式分段、停止、急停、复位、底层错误传播。
- `FB_ServoControl`：驱动器初始化、仿真、自学习、正负速度、流量限制、减速停机和错误状态。

### 9.2 版本更新规则

每次修改 `plc.xml` 后，应至少同步更新：

- 数据类型/功能块清单和接口表。
- 参数单位、缩放公式和默认值。
- 状态机步号、错误号和方法行为。
- 典型 ST 示例及外部依赖版本。
- 本章“当前版本限制”与回归测试清单。

---

### 9.3 HydTechnology OOP 单泵快速原型（V0.2 框架）

本原型在 `plc.xml` 中增加统一轴抽象和工艺层门面，作为后续完整实现的接口骨架：

| 类型 | 职责 |
|---|---|
| `IAxis` | 电动轴、液压轴共用的使能、速度/位置命令和状态契约 |
| `IHydraulicAxis` | `IAxis` 的液压扩展，提供压力命令和请求流量状态 |
| `FB_AxisBase` | 公共状态与急停/复位边界的抽象基类 |
| `FB_ElectricAxis` | 组合现有 `MC_Power`、`SMC_FollowVelocity` |
| `FB_HydraulicAxis` | 适配现有 `FB_HydAxis`、`HYD_ReadStatus`，保留旧输入引脚 |
| `FB_HydPumpManager` | 单泵请求入口；每个 PLC 周期只调用一次 `HYD_GetPumpRequest` 和 `FB_ServoControl` |
| `FB_HydTechnology` | 应用层唯一工艺入口，承载轴注册占位、泵管理器和五类工艺动作 |
| `FB_ClampProcess` / `FB_MoldOpenProcess` | 锁模、开模多段动作骨架 |
| `FB_InjectionProcess` / `FB_ChargeProcess` / `FB_EjectProcess` | 射胶、储料、顶针动作骨架 |

单泵约束：`HYD_GetPumpRequest` 的仲裁策略仍由 HydMotion 实现；HydTechnology 不复制该算法，也不允许应用层直接调用 `HYD_GetPumpRequest`、`FB_ServoControl`、泵速或流量分配变量。`FB_HydPumpManager` 是唯一调用点，并带有周期重入诊断位 `ST_HydPumpStatus.bCycleReentry`。

`program0` 仅展示适配器实例化、统一的 `bEnable`/`bMove`/`rVelocity` 命令名、接口注册边界和工艺门面调用。OOP 元数据（`Interface`、`Inheritance`、`Implements`）来自本地 CODESYS PLCopen 扩展命名空间，当前未在 Gm2xx 目标上完成编译验证；导入目标若不接受该元数据，应先用目标编辑器导出同等接口/继承结构再替换 XML 表示。

原型暂不承诺完整配方持久化、温控闭环、实际的多轴优先级分配、真实轴引用绑定和安全认证；这些功能应在目标平台编译通过后分阶段补齐。

---

## 10. 版本历史

| 版本 | 日期 | 说明 |
|---|---|---|
| V0.1 | 2026-08-14 | 基于项目 `plc.xml` 完成首版数据类型、功能块、方法、调用示例和限制说明 |
| V0.2 | 2026-08-17 | 增加 OOP 轴抽象、液压泵管理器和锁模/开模/射胶/储料/顶针快速原型框架 |
