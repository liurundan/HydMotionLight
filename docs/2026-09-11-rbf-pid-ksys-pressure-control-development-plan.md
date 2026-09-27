# 共享泵 Ksys 标定与压力环适配开发计划

**日期：** 2026-09-11  
**状态：** 计划已确认，本文档只记录计划，本轮不修改源码  
**适用范围：** 共享单泵、多缸液压注塑机压力闭环

## 1. 总体目标

将 `Ksys` 明确定义为：

```text
Ksys = DeltaPressure(bar) / DeltaPumpSpeed(rpm)    [bar/rpm]
```

压力控制器内部仍保持 `L/min` 输出，因此运行时统一换算为：

```text
flowToPumpSpeedGain = 1000 / (pumpDisplacement[mL/rev] * volumetricEfficiency)
                      [rpm/(L/min)]

Kflow = Ksys * flowToPumpSpeedGain
       [bar/(L/min)]
```

默认不改变位置环、速度环、泵转换器和现有无 Ksys 配方的行为。

## 2. 接口与数据结构

### 2.1 共享泵对象

扩展现有 `HydroPump`，由一个共享实例持有：

- 泵排量；
- 容积效率；
- 最大转速；
- `HYD_PumpFeedback` 实时反馈；
- 固定标定结果 `system_gain_bar_per_rpm`；
- `system_gain_valid` 标志。

不在每个轴复制泵属性。`HYD_MotionControlFB` 只增加一个 `HydroPump*` 指针，并提供：

```c
void HYD_MotionControlFB_SetSharedPump(
    HYD_MotionControlFB* fb,
    HydroPump* pump);
```

旧版 `pumpConfig` 字段保留用于 ABI 和旧调用兼容；绑定共享泵后，泵参数优先从共享对象读取。

软复位保留共享泵指针，完整 `Init()` 清除绑定关系。

### 2.2 压力控制器输入

给 `HYD_PressureControllerInput` 增加：

- `systemGainBarPerRpm`；
- `const HYD_PumpFeedback* pumpFeedback`；
- 反馈有效性由 `validFlags`、有限值和时间戳共同判断。

无有效实际 rpm 时：

- 不重新估算 Ksys；
- 继续使用已经标定的固定 Ksys；
- 若没有固定 Ksys，则保持现有无增益补偿逻辑；
- 不使用 `PUMP_SPEED` 指令冒充实际反馈 rpm。

### 2.3 压力段配置

新增字段，不复用旧 `systemGain`：

- `systemGainBarPerRpm`：段级 Ksys 覆盖值，单位 `bar/rpm`；
- `pressureFeedforwardGain`：范围 `0~1`，默认 `0` 表示关闭基于 Ksys 的前馈；
- `pressureFeedforwardBiasPressure`：压力偏置，默认 `0`；
- `pressureRbfConfig.referenceFlowProcessGain`：RBF 增益范围缩放参考值，单位 `bar/(L/min)`，默认 `0` 表示不自动缩放。

旧 `systemGain` 继续按原有流量域语义兼容处理，避免已有配方被静默改变。新设备使用共享泵的 `system_gain_bar_per_rpm` 或段级 `systemGainBarPerRpm`。

## 3. 标定逻辑

标定只在调试/参数写入流程中执行，不在闭环每周期更新。

使用两个稳定开环反馈样本：

```text
Ksys = (P1 - P0) / (rpm1 - rpm0)
```

约束：

- 两个 rpm 必须有效且有限；
- `abs(rpm1-rpm0)` 必须大于最小激励阈值；
- 结果必须为正且有限；
- 标定完成后写入共享 `HydroPump`；
- 闭环运行期间固定使用该值。

若现场确认零速压力偏置可忽略，可使用单点近似：

```text
Ksys = P / rpm
```

但默认仍采用双点差分，避免压力偏置和泄漏造成系统性误差。

## 4. PI 前馈逻辑

仅当 `pressureFeedforwardGain > 0` 且 Ksys、泵排量和容积效率均有效时启用：

```text
Qbias = segment.targetFlow

Qff = Qbias
    + pressureFeedforwardGain
    * (targetPressure - pressureFeedforwardBiasPressure)
    / Kflow
```

然后：

```text
outputFlow = Qff + PI_feedback
```

`Qff` 必须先经过输出上下限限制，PI 积分仍使用现有抗饱和逻辑。

因此：

- `pressureFeedforwardGain=0`：完全保持当前行为；
- `pressureFeedforwardGain=1`、`targetFlow=0`、偏置为 0：实现 `Qff=targetPressure/Kflow`；
- 前馈只负责静态工作点，动态建压仍由压力斜坡、PI 和流量限制负责。

## 5. RBF-PID 逻辑

### 5.1 统一前馈语义

当前 RBF 分支不真正叠加 `feedforwardFlow`，需要修正为与 PI 相同的总流量语义。

RBF 句柄增加当前/上一拍前馈状态，采用前馈变化量：

```text
Output(k) =
    Output(k-1)
    + feedback_delta(k)
    + Qff(k) - Qff(k-1)
```

这样既能加入前馈，又不会每周期重复累加固定流量，并且目标压力或前馈变化时保持无扰。

### 5.2 Ksys 软限幅

软限幅使用换算后的 `Kflow`：

```text
Qcap = 1.05 * targetPressure / Kflow
```

但只在接近目标或保压阶段启用；快速建压、误差较大时不使用该静态上限，避免低 Ksys 设备被静态上限限制。

### 5.3 RBF 增益范围缩放

当配置了 `referenceFlowProcessGain` 时：

```text
gainScale =
    clamp(referenceFlowProcessGain / Kflow, 0.25, 4.0)
```

再对 `Kp/Ki/Kd` 的上下限统一缩放。

当该参考值为 0 时，保持当前配方范围不变，避免已有设备出现隐式重新整定。

在泵换算参数相同的前提下：

```text
Ksys=0.66 与 Ksys=3.5 的比例约为 5.30
```

低增益设备的流量域 RBF 增益范围可约放大 5.3 倍，高增益设备则相应收窄，但最终受 `0.25~4.0` 安全倍率和输出饱和保护限制。

### 5.4 学习保护

以下情况冻结 RBF 参数学习：

- 实际输出饱和；
- Kflow 无效；
- 压力反馈或泵 rpm 非有限；
- 泵反馈时间戳过期；
- 快速建压阶段；
- 超压泄压阶段。

## 6. 参数接口路由

更新参数读写路径：

- 若 FB 已绑定共享泵，泵排量、容积效率、最高转速写入共享 `HydroPump`；
- 未绑定共享泵时，回退现有 FB 内的兼容字段；
- 不扩展 `HYD_AXISMOTION` 或现有 IEC 反馈结构，避免 PLC ABI 变化；
- 实际泵反馈由 HAL/仿真共享对象更新，压力环只读取指针，不复制反馈包。

## 7. 测试计划

新增或调整以下测试：

1. `Ksys(bar/rpm) * flowToPumpSpeedGain(rpm/(L/min))` 转换正确。
2. `Ksys=0.66` 和 `Ksys=3.5` 下，100 bar 的前馈流量分别符合：

   ```text
   Qff = 100 / Kflow
   ```

3. 无效排量、无效容积效率、零 Ksys、零 rpm 时安全回退。
4. 双点反馈标定正确拒绝零激励、负增益和非有限输入。
5. PI 前馈默认关闭时，已有输出和积分行为保持不变。
6. RBF 前馈加入后不重复累加，目标变化时无明显跳变。
7. RBF 软限幅只在接近目标阶段生效，快速建压不被静态 Ksys 限制。
8. 两个轴绑定同一个 `HydroPump` 时共享排量、反馈 rpm 和 Ksys，且不产生每轴泵属性副本。
9. 无共享泵绑定时，现有压力控制器、位置环和速度环回归测试全部保持通过。
10. 运行现有：

    ```bash
    cmake --build --preset unixgcc
    ctest --test-dir out/build/unixgcc --output-on-failure
    ```

## 8. 兼容与默认行为

- 新增字段全部默认关闭或无效；
- 旧配方不自动启用 Ksys 前馈；
- 旧 `systemGain` 保持流量域兼容语义；
- 新 Ksys 使用 `systemGainBarPerRpm` 或共享泵固定标定值；
- 不使用动态内存；
- 不改变非压力闭环动作；
- 不把实际泵 rpm 写入每个轴的状态结构，只保存共享泵指针。

## 9. 当前源码边界与后续执行顺序

当前源码已有统一的 `HYD_PumpFeedback`，包含 `rpm`、`angleDeg`、`torquePermille`、时间戳和有效标志，但主 `HYD_MotionControlFB` 尚未消费该反馈。`fb->PUMP_SPEED` 是指令值，不得当作实际反馈 rpm。

执行顺序：

1. 先补充失败测试，锁定 Ksys 单位转换、共享泵绑定和无反馈回退行为；
2. 修改公共头文件和共享泵/压力环接口；
3. 实现显式双点标定及固定增益消费；
4. 接入 PI Ksys 前馈；
5. 修正 RBF 前馈、Kflow 软限幅和增益范围调度；
6. 运行压力控制器、RBF、泵转换器及完整回归测试；
7. 对嵌入式结构体大小、PLC ABI 和多轴共享行为做静态检查。

本计划的控制原则是：实际 rpm 只来自共享泵反馈；无反馈时安全回退；Ksys 只在显式标定后固定使用；任何新增前馈和自适应行为默认关闭，避免影响已有设备。
