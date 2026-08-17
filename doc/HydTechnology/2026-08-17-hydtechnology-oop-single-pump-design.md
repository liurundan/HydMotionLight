# HydTechnology OOP 单泵多缸重构设计

日期：2026-08-17

状态：已确认设计，暂不实施代码

## 1. 目标与约束

HydTechnology 面向液压单泵多缸注塑机，提供工艺层统一轴操作和注塑动作封装。设计必须：

- 使用 IEC OOP ST 的接口、继承、多态、方法和属性；
- 让液压轴与电动轴共享 `IAxis`；
- 保留 `pousGm2.xml` 的 `MC_Power`、`SMC_FollowVelocity` 等基础块；
- 保留 `pousHydMotion.xml` 的液压基础块和现有 `FB_HydAxis` 状态机；
- 让应用层不接触泵请求、泵速仲裁或伺服泵输出；
- 保证单泵调度每 PLC 周期只执行一次；
- 支持锁模、开模、射胶、储料和顶针工艺动作。

目标编译器已确认支持 `INTERFACE`、`EXTENDS`、`METHOD`、`PROPERTY` 等 OOP 语义。

## 2. 关键边界：HydMotion 是唯一泵仲裁者

当前 `pousHydMotion.xml` 的 `HYD_GetPumpRequest` 输入为 `ENABLE`、`STRATEGY`、`ALLOW_NEGATIVE`，输出为单一 `PUMPSPEED`、`CONFLICT`、`BUSY`、`DONE` 和错误状态；其文档说明该块从运动系统获取一个泵速命令。

因此不得在 HydTechnology 中复制第二套每轴流量排序或分配算法。唯一职责边界为：

```text
FB_HydraulicAxis
    -> HydMotion 运动运行时内部请求汇总/仲裁
    -> HYD_GetPumpRequest（唯一仲裁出口）
    -> FB_HydPumpManager（封装调用时序）
    -> FB_ServoControl（唯一伺服泵输出）
```

`FB_HydPumpManager` 可以读取各轴请求用于诊断和调用前检查，但不能根据这些请求再次计算授权流量。轴优先级、泵容量、负向流量和内部限幅由 HydMotion 的配置及 `STRATEGY` 决定；当前 HydMotion 未公开每轴优先级输入，因此 HydTechnology 不伪造应用级 `uiPriority`。

## 3. 类型与接口

### 3.1 公共类型

- `E_AxisState`：初始化、就绪、空闲、忙、完成、停止、急停、错误。
- `E_AxisMode`：位置、速度、压力。
- `ST_AxisCommand`：统一使能、停止、急停、速度、位置和动态参数。
- `ST_AxisStatus`：统一状态、错误、实际位置、实际速度和压力。
- `ST_HydPumpPolicy`：HydMotion 的全局策略、负向流量许可和泵轴配置。
- `ST_HydPumpStatus`：泵速、冲突、忙、完成和错误状态。
- `ST_AxisSlot`：注册表中的接口引用、轴类型、HydMotion 轴 ID 和使用标志。

### 3.2 接口层

`IAxis` 提供：

- `Enable`、`MoveVelocity`、`MoveAbsolute`、`Jog`、`Stop`、`EStop`、`Reset`；
- `Cyclic`；
- `GetStatus`；
- `AxisName`、`State`、`Enabled`、`Busy`、`Done`、`Error` 属性。

`IHydraulicAxis EXTENDS IAxis` 增加：

- `MovePressure`；
- `GetRequestedFlow`；
- `GetHydMotionAxisID`；
- `SetPumpStatus`；
- `RequestedFlow`、`PumpConflict` 属性。

该接口不提供应用级流量分配方法。

## 4. 功能块层次

```text
FB_AxisBase (ABSTRACT)
    |- FB_ElectricAxis
    `- FB_HydraulicAxis

FB_HydTechnology
    |- FB_ClampProcess
    |- FB_MoldOpenProcess
    |- FB_InjectionProcess
    |- FB_ChargeProcess
    |- FB_EjectProcess
    `- FB_HydPumpManager
```

### 4.1 `FB_AxisBase`

抽象基类集中处理公共命令锁存、状态转换、停止/急停优先级、错误状态和周期入口。它不可直接实例化。

### 4.2 `FB_ElectricAxis`

继承 `FB_AxisBase` 并实现 `IAxis`。内部调用 `MC_Power`、`SMC_FollowVelocity`、位置/速度读取和复位块。它不访问液压泵接口。

### 4.3 `FB_HydraulicAxis`

继承 `FB_AxisBase` 并实现 `IHydraulicAxis`。内部组合现有 `FB_HydAxis`，复用现有位置、速度、压力模式；通过 `HYD_ReadStatus` 读取请求和限幅诊断；不直接调用 `HYD_GetPumpRequest` 或 `FB_ServoControl`。

### 4.4 `FB_HydPumpManager`

内部只拥有一个 `HYD_GetPumpRequest` 和一个 `FB_ServoControl`。每周期：

1. 检查已注册液压轴的请求状态；
2. 调用一次 `HYD_GetPumpRequest`；
3. 读取 `PUMPSPEED`、`CONFLICT` 和错误状态；
4. 将唯一泵速传给一次 `FB_ServoControl`；
5. 将系统级泵状态广播给液压轴。

不得实现第二套流量求和、排序或比例分配。

### 4.5 `FB_HydTechnology`

应用层唯一周期入口和工艺库门面。负责轴注册、动作状态机、轴周期调用、单泵管理器调用、统一报警和动作完成状态。

固定周期顺序：

```text
工艺动作状态机锁存命令
    -> 每个注册轴调用一次 IAxis.Cyclic()
    -> 调用一次 FB_HydPumpManager.Cyclic()
    -> 汇总状态/报警
```

## 5. 工艺动作

所有动作仅依赖 `IAxis` 或 `IHydraulicAxis`：

- `FB_ClampProcess`：快速合模、慢速合模、压力锁模；
- `FB_MoldOpenProcess`：脱模、快速开模、终点慢速；
- `FB_InjectionProcess`：快速射胶、慢速射胶、压力切换、保压；
- `FB_ChargeProcess`：储料到计量位置并减速停止；
- `FB_EjectProcess`：顶针前进、保持、后退。

工艺动作只生成轴命令。压力切换使用 `IHydraulicAxis.MovePressure`，泵速生成仍由 HydMotion 完成。

## 6. 应用层契约

应用层流程为：

1. 创建轴实例并绑定轴引用/`ST_HydAxisRef`；
2. 将具体 FB 赋给 `IAxis` 或 `IHydraulicAxis` 接口引用；
3. 调用 `FB_HydTechnology.RegisterAxis` 或 `RegisterHydraulicAxis` 一次；
4. 绑定锁模、射胶、储料、开模和顶针动作轴；
5. 通过统一接口调用使能、速度、位置和停止；
6. 通过 `StartClamp`、`StartMoldOpen`、`StartInjection`、`StartCharge`、`StartEject` 发出工艺命令；
7. 每 PLC 周期只调用 `FB_HydTechnology.Cyclic()`。

应用层不得出现：

- `HYD_GetPumpRequest`；
- `FB_ServoControl`；
- `PUMPSPEED`、`CONFLICT`；
- 流量数组、流量排序、流量分配或泵速限幅逻辑。

## 7. 安全与错误处理

- 急停优先于普通运动命令；
- 泵冲突/限幅作为系统级诊断，不自动伪装成轴硬故障；
- 轴底层错误转换到 `ST_AxisStatus`，由 `FB_HydTechnology` 汇总；
- 软件急停不能替代硬件安全回路；
- `FB_HydPumpManager` 或任一关键轴错误时，进入安全停泵策略；
- 所有运动和泵功能块必须保持稳定实例并连续周期调用。

## 8. 验证标准

设计实现后至少验证：

- 电动轴和液压轴均能通过 `IAxis` 完成使能、速度、位置、停止、急停和复位；
- 同一 PLC 周期内每个轴只执行一次 `Cyclic()`；
- 同一 PLC 周期内只调用一次 `HYD_GetPumpRequest` 和一次 `FB_ServoControl`；
- 应用层无底层泵函数调用；
- 锁模、开模、射胶、储料和顶针动作的状态转换、完成和错误传播正确；
- HydMotion 的 `CONFLICT`、泵速限幅和错误状态能够完整传递到工艺层；
- 不修改 `pousGm2.xml` 和 `pousHydMotion.xml` 的现有基础功能语义。

