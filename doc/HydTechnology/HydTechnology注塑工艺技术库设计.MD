<设计思路>

# HydTechnology 注塑工艺技术库重构设计

## 1. 范围和硬性约束

本文档定义液压单泵多缸注塑机的 HydTechnology 工艺库架构、接口、调用时序、PLCopen XML 组织方式和 IEC 61131-3 ST 示例。本文档不修改 pousGm2.xml、pousHydMotion.xml 或任务代码。

必须满足：

1. 液压轴和电动轴通过统一的 IAxis 接口供上层使用。
2. 具体轴采用 IEC OOP ST 的接口、继承、方法和多态机制。
3. 保留 pousGm2.xml 中的 MC_Power、SMC_FollowVelocity 等基础功能块。
4. 保留现有 FB_HydAxis 和 pousHydMotion.xml 中的 HYD_* 基础功能块。
5. 应用层不得调用 HYD_GetPumpRequest、FB_ServoControl 或流量分配逻辑。
6. 单泵调度在每一个 PLC 周期内只能执行一次。
7. 软件互锁不得替代硬件急停、安全继电器或安全 PLC。
8. 示例中的 POU 名称、变量名和参数名必须与当前 XML 中的真实接口一致。

## 2. 架构合理性评估

“应用层—工艺技术库—液压运动控制库”的三层划分可以作为逻辑视图，但没有表达电动轴 PLCopen 库的独立边界。实施视图必须明确为四层：

~~~text
应用层
  ↓ 只依赖 HydTechnology 公共门面和 IAxis
HydTechnology 工艺技术层
  ↓ 私有调用底层适配器
HydMotion 液压运动控制层
电动轴 PLCopen 控制层
~~~

HydMotion 和电动轴库属于底层技术执行层，但不可合并：

- HydMotion 负责液压轨迹、压力闭环、流量换算、液压状态和泵请求；
- 电动轴库负责 AXIS_REF 和 PLCopen 运动控制功能块；
- HydTechnology 负责工艺语义、统一接口和底层调用隔离；
- 应用层只负责机器流程和工艺动作组合。

## 3. 方案比较

### 3.1 方案 A：统一接口、抽象轴基类和单一泵门面（采用）

- 定义 IAxis；
- 定义抽象 FB_AxisBase；
- FB_ElectricAxis 组合 MC_Power、SMC_FollowVelocity 等已有 POU；
- FB_HydraulicAxis 组合现有 FB_HydAxis 和 HYD_ReadStatus；
- FB_HydTechnology 管理轴注册、工艺动作、温度、安全和周期；
- FB_HydPumpManager 在整个单泵系统内只有一个实例。

该方案兼容当前工程，并能让同一工艺动作绑定液压轴或电动轴。

### 3.2 方案 B：HydTechnology 自行实现逐轴流量授权（本版本不采用）

当前 HYD_GetPumpRequest 的真实接口为：

~~~text
输入：ENABLE、STRATEGY、ALLOW_NEGATIVE
输出：PUMPSPEED、CONFLICT、BUSY、DONE、ERROR、ERRORID
~~~

该接口没有逐轴授权流量输入，也没有逐轴分配结果输出。因此本版本不能伪造逐轴流量授权 API。若未来必须由 HydTechnology 向每个液压轴写入授权流量，必须先扩展 HydMotion，例如新增 HYD_SetAxisFlowPermit 和 HYD_ReadAxisFlowAllocation，然后重新评审接口版本。

### 3.3 方案 C：每个液压轴创建泵管理器（禁止）

每个液压轴独立调用泵管理器会造成泵速覆盖、调用顺序不确定和资源冲突，违反“每周期一次”的硬性约束。

## 4. 四层职责

### 4.1 应用层

负责：

- 自动循环、手动模式和配方选择；
- HMI 命令和操作权限；
- 启动、停止、复位和急停请求；
- 锁模、开模、射胶、储料和顶针动作组合；
- 工艺参数输入；
- 读取统一状态、报警和完成信号。

禁止：

- 直接调用 MC_Power、SMC_FollowVelocity、FB_HydAxis；
- 直接调用 HYD_GetPumpRequest、FB_ServoControl；
- 管理液压轴 ID、流量数组、泵优先级和流量分配。

### 4.2 HydTechnology 工艺技术层

负责：

- IAxis、IHydraulicAxis 和抽象轴基类；
- 轴注册、轴绑定和注册合法性检查；
- 多段位置/速度组合；
- 速度—压力切换；
- 锁模、开模、射胶、储料、顶针状态机；
- 工艺参数存储和范围检查；
- 温度采样、加热许可和温度报警；
- 工艺级安全互锁；
- 单泵管理器唯一实例；
- HYD_GetPumpRequest 和 FB_ServoControl 的私有调用；
- 统一诊断和错误传播。

HydTechnology 是泵调度的唯一公开所有者，但不复制 HydMotion 的液压闭环算法。

### 4.3 HydMotion 液压运动控制层

负责：

- 液压轴创建和轴 ID；
- 液压轨迹规划；
- 位置、速度、压力模式；
- 油缸面积和流量换算；
- 压力/流量闭环；
- 反馈更新；
- 液压请求汇总和系统泵速计算；
- HYD_GetPumpRequest 实现；
- 流量、泵速、压力和软限位状态。

不负责注塑机工艺语义。

### 4.4 电动轴 PLCopen 控制层

负责：

- AXIS_REF；
- MC_Power；
- MC_MoveAbsolute、MC_MoveVelocity、MC_Jog；
- SMC_FollowVelocity；
- MC_Stop、MC_Reset；
- 实际位置、速度、转矩和轴状态。

不负责注塑工艺、温控、液压泵和工艺互锁。

## 5. 统一轴抽象

IAxis 只包含两类轴都具备的能力：

- 使能；
- 点动；
- 速度运行；
- 位置运行；
- 停止；
- 急停；
- 复位；
- 周期执行；
- 状态读取。

压力控制不属于所有电动轴的公共能力，因此定义：

~~~text
IHydraulicAxis EXTENDS IAxis
~~~

IHydraulicAxis 增加压力控制、请求流量、HydMotion 轴 ID、液压限幅状态和内部泵状态回写能力。泵状态回写方法为内部方法，不能暴露给应用层。

## 6. 继承关系

~~~text
FB_AxisBase ABSTRACT IMPLEMENTS IAxis
    ├─ FB_ElectricAxis IMPLEMENTS IAxis
    └─ FB_HydraulicAxis IMPLEMENTS IHydraulicAxis
~~~

FB_AxisBase 只处理公共命令、状态、错误和周期入口。FB_ElectricAxis 和 FB_HydraulicAxis 只处理各自技术适配。

## 7. 单泵调度边界

当前版本的唯一调用链：

~~~text
FB_HydraulicAxis
    → HydMotion 运动运行时
    → HYD_GetPumpRequest
    → FB_HydPumpManager
    → FB_ServoControl
    → 液压泵
~~~

规定：

- 一个单泵组只能实例化一个 FB_HydPumpManager；
- 该实例只能由一个 FB_HydTechnology 持有；
- 液压轴不得创建泵管理器；
- 应用层不得访问泵管理器；
- HYD_GetPumpRequest 每周期调用一次；
- FB_ServoControl 每周期调用一次；
- 逐轴 REQUESTEDFLOW 只能用于诊断，不能作为第二套仲裁算法；
- 当前版本没有逐轴授权流量输出。

## 8. 周期唯一性

固定周期顺序：

~~~text
温度和安全采样
    → 工艺状态机生成命令
    → 每个注册轴执行一次 IAxis.Cyclic()
    → FB_HydPumpManager.ExecuteCycle()
    → 广播泵状态
    → 汇总诊断
~~~

应用层每个 PLC 周期只能调用一次 FB_HydTechnology.Cyclic()。周期方法接收任务维护的 udiCycleToken，泵管理器保存 udiLastCycleToken。同一 token 再次进入时设置 bCycleReentry 并拒绝第二次调度。

仅使用一个普通布尔变量不能检测所有连续重复调用，因此“单一实例、单一调用点、任务周期 token”是完整约束。

## 9. 安全和错误

- 硬件急停优先于所有软件运动命令；
- 软件急停不能替代硬件安全链；
- 温度、安全门、液位、油温、润滑和模具位置不满足时禁止相关工艺动作；
- 泵冲突、流量限幅和泵速限幅必须与轴本体错误区分；
- 底层错误转换为 ST_AxisStatus，再由 HydTechnology 汇总；
- 泵错误或关键轴急停时进入安全停机；
- 复位不能自动重启未确认的工艺动作。

</设计思路>

<库框架设计>

## 1. 文件结构

~~~text
doc/HydTechnology/
├─ plc.xml
├─ pousGm2.xml
├─ HydTechnology注塑工艺技术库设计.MD
├─ TF8560_Plastic_Technology_Functions_EN-1.5.1.md
└─ TF5810_TC3_Hydraulic_Positioning_EN-1.8.3.md

仓库根目录/
└─ pousHydMotion.xml
~~~

## 2. plc.xml 新增和整理对象

~~~text
dataTypes:
    E_AxisState
    E_AxisMode
    E_AxisKind
    E_PumpPolicy
    E_ProcessStep
    E_ProcessAxisRole
    ST_AxisCommand
    ST_AxisStatus
    ST_HydAxisStatus
    ST_HydPumpStatus
    ST_AxisSlot

interfaces:
    IAxis
    IHydraulicAxis
    IHydTechnology

axis adapters:
    FB_AxisBase
    FB_ElectricAxis
    FB_HydraulicAxis

technology services:
    FB_HydPumpManager
    FB_TemperatureManager
    FB_SafetyInterlock
    FB_ProcessParameterManager

process blocks:
    FB_ClampProcess
    FB_MoldOpenProcess
    FB_InjectionProcess
    FB_ChargeProcess
    FB_EjectProcess

facade:
    FB_HydTechnology

example:
    program0
~~~

原有 FB_ServoControl、FB_HydAxis、FB_EleAxis、FB_CalcAxisMaxSpeed 继续保留。新适配器通过继承和组合使用它们。

## 3. pousGm2.xml 和 pousHydMotion.xml

### 3.1 pousGm2.xml

保持基本不变，继续提供：

- AXIS_REF；
- MC_Power；
- MC_MoveAbsolute；
- MC_MoveVelocity；
- MC_Jog；
- MC_Stop；
- MC_Reset；
- MC_ReadActualPosition；
- MC_ReadActualVelocity；
- SMC_FollowVelocity。

### 3.2 pousHydMotion.xml

保持基本不变，继续提供：

- HYD_CreateMotion；
- HYD_MoveContinuousAbsolute；
- HYD_MoveVelocity；
- HYD_PressureHandle；
- HYD_Stop；
- HYD_Reset；
- HYD_ReadStatus；
- HYD_ReadSimFeedback；
- HYD_GetPumpRequest；
- 液压轨迹和闭环控制基础功能。

FB_HydraulicAxis 复用现有 FB_HydAxis，不重新实现液压状态机。

## 4. 公共类型

### 4.1 枚举

~~~iecst
TYPE E_AxisState :
(
    eInit := 0,
    eReady := 10,
    eIdle := 20,
    eBusy := 30,
    eDone := 40,
    eStopping := 50,
    eStopped := 60,
    eEmergencyStop := 70,
    eError := 99
);
END_TYPE

TYPE E_AxisMode :
(
    eNone := 0,
    ePosition := 1,
    eVelocity := 2,
    ePressure := 3
);
END_TYPE

TYPE E_AxisKind :
(
    eUnknown := 0,
    eElectric := 1,
    eHydraulic := 2
);
END_TYPE

TYPE E_PumpPolicy :
(
    eMotionDefault := 0,
    eProcessPriority := 1
);
END_TYPE

TYPE E_ProcessAxisRole :
(
    eRoleNone := 0,
    eRoleClamp := 1,
    eRoleMoldOpen := 2,
    eRoleInjection := 3,
    eRoleCharge := 4,
    eRoleEject := 5
);
END_TYPE
~~~

eProcessPriority 只映射到 HydMotion 的 STRATEGY，不代表当前 HydTechnology 已有逐轴优先级分配算法。

### 4.2 结构体

~~~iecst
TYPE ST_AxisCommand :
STRUCT
    bEnable         : BOOL;
    bStop           : BOOL;
    bEStop          : BOOL;
    bReset          : BOOL;
    bMoveVelocity   : BOOL;
    bMoveAbsolute   : BOOL;
    bJog            : BOOL;
    bJogPositive    : BOOL;
    rPosition       : LREAL;
    rVelocity       : LREAL;
    rAcceleration   : LREAL;
    rDeceleration   : LREAL;
    rJerk           : LREAL;
END_STRUCT
END_TYPE
~~~

~~~iecst
TYPE ST_AxisStatus :
STRUCT
    eState          : E_AxisState;
    bEnabled        : BOOL;
    bBusy           : BOOL;
    bDone           : BOOL;
    bError          : BOOL;
    udiErrorID      : UDINT;
    rActualPosition : LREAL;
    rActualVelocity : LREAL;
    rActualPressure : REAL;
END_STRUCT
END_TYPE
~~~

~~~iecst
TYPE ST_HydAxisStatus :
STRUCT
    iAxisID               : SINT;
    rRequestedFlow        : REAL;
    rMaxFlow              : REAL;
    bFlowLimitActive      : BOOL;
    bPumpSpeedLimitActive : BOOL;
    bPressureLimitActive  : BOOL;
    bSoftLimitActive      : BOOL;
    bDerated              : BOOL;
END_STRUCT
END_TYPE
~~~

~~~iecst
TYPE ST_HydPumpStatus :
STRUCT
    rPumpSpeed    : REAL;
    rTotalRequest : REAL;
    bConflict     : BOOL;
    bBusy         : BOOL;
    bDone         : BOOL;
    bError        : BOOL;
    wErrorID      : WORD;
    bCycleReentry : BOOL;
END_STRUCT
END_TYPE
~~~

~~~iecst
TYPE ST_AxisSlot :
STRUCT
    bUsed            : BOOL;
    sName            : STRING(80);
    eAxisKind        : E_AxisKind;
    iAxis            : IAxis;
    iHydraulicAxis   : IHydraulicAxis;
    iHydMotionAxisID : SINT;
    uiPumpGroup      : UINT;
END_STRUCT
END_TYPE
~~~

## 5. 功能块职责

| 对象 | 责任 | 禁止事项 |
|---|---|---|
| IAxis | 公共命令和状态 | 不包含泵管理 |
| IHydraulicAxis | 压力、请求和液压限幅状态 | 不开放逐轴流量分配 |
| FB_AxisBase | 公共命令、状态、错误和周期入口 | 不访问具体驱动 |
| FB_ElectricAxis | 组合 PLCopen 电动轴 POU | 不实现液压逻辑 |
| FB_HydraulicAxis | 组合 FB_HydAxis 和 HYD_ReadStatus | 不调用泵功能 |
| FB_HydPumpManager | 单泵唯一调用链 | 不复制 HydMotion 仲裁 |
| FB_TemperatureManager | 温度和加热许可 | 不执行运动 |
| FB_SafetyInterlock | 工艺安全许可 | 不替代硬件安全 |
| FB_*Process | 工艺状态机 | 不访问泵 POU |
| FB_HydTechnology | 公共门面和唯一周期入口 | 不允许应用绕过 |

## 6. 调用关系

~~~text
program0
    → FB_HydTechnology.Initialize()
    → RegisterAxis()
    → RegisterHydraulicAxis()
    → Bind*Axis()

每个 PLC 周期：
FB_HydTechnology.Cyclic()
    → FB_TemperatureManager.Cyclic()
    → FB_SafetyInterlock.Cyclic()
    → FB_ClampProcess.Cyclic()
    → FB_MoldOpenProcess.Cyclic()
    → FB_InjectionProcess.Cyclic()
    → FB_ChargeProcess.Cyclic()
    → FB_EjectProcess.Cyclic()
    → 每个 IAxis.Cyclic() 一次
    → FB_HydPumpManager.ExecuteCycle() 一次
    → 统一诊断
~~~

## 7. 注册约束

注册阶段必须检查：

1. 槽位未超过固定容量；
2. 同一接口对象不能重复注册；
3. 同一 HydMotion 轴 ID 不能重复注册；
4. 单泵架构下液压轴必须属于同一泵组；
5. 初始化完成后禁止修改轴类型和泵组；
6. 注册失败返回 FALSE 和明确诊断码；
7. 应用层不能直接写注册数组。

## 8. PLCopen XML 映射

- 数据类型使用 dataType 和 baseType；
- 接口使用当前工程已有 interface 扩展元数据；
- 接口继承使用目标编辑器导出的 pouinheritance 元数据；
- 方法使用独立 Method 节点；
- 访问权限使用 accessmodifiers；
- ST 正文放在 ST 内；
- POU 变量名必须与真实接口一致；
- 不能把不存在的 POU 输入输出写入 XML；
- 本次只整理 plc.xml，不复制或重写两个底层库文件。

## 9. 当前版本能力边界

当前版本是“HydTechnology 统一封装 + HydMotion 权威运行时仲裁”：

- 应用层看不到泵调用；
- HydTechnology 唯一负责泵调用时序；
- HydMotion 负责实际液压请求汇总和系统泵速；
- rTotalRequest 只能用于诊断；
- 当前没有逐轴流量授权结果；
- 新增逐轴授权必须扩展 HydMotion 后再升版。

</库框架设计>

<抽象设计示例>

代码为 IEC 61131-3 OOP ST 语义示例；写入 plc.xml 时，接口、方法、属性和继承通过目标工具对应的 PLCopen XML 扩展元数据表示。

## 1. IAxis

~~~iecst
INTERFACE IAxis

    METHOD Enable : BOOL
    VAR_INPUT
        bExecute : BOOL;
    END_VAR
    END_METHOD

    METHOD Jog : BOOL
    VAR_INPUT
        bExecute      : BOOL;
        bPositive     : BOOL;
        rVelocity     : LREAL;
        rAcceleration : LREAL;
        rDeceleration : LREAL;
    END_VAR
    END_METHOD

    METHOD MoveVelocity : BOOL
    VAR_INPUT
        bExecute      : BOOL;
        rVelocity     : LREAL;
        rAcceleration : LREAL;
        rDeceleration : LREAL;
    END_VAR
    END_METHOD

    METHOD MoveAbsolute : BOOL
    VAR_INPUT
        bExecute      : BOOL;
        rPosition     : LREAL;
        rVelocity     : LREAL;
        rAcceleration : LREAL;
        rDeceleration : LREAL;
    END_VAR
    END_METHOD

    METHOD Stop : BOOL
    VAR_INPUT
        bExecute : BOOL;
    END_VAR
    END_METHOD

    METHOD EStop : BOOL
    VAR_INPUT
        bExecute : BOOL;
    END_VAR
    END_METHOD

    METHOD Reset : BOOL
    VAR_INPUT
        bExecute : BOOL;
    END_VAR
    END_METHOD

    METHOD Cyclic : BOOL
    END_METHOD

    METHOD GetStatus : ST_AxisStatus
    END_METHOD

    PROPERTY Enabled : BOOL;
    PROPERTY Busy : BOOL;
    PROPERTY Done : BOOL;
    PROPERTY Error : BOOL;
    PROPERTY State : E_AxisState;

END_INTERFACE
~~~

## 2. IHydraulicAxis

~~~iecst
INTERFACE IHydraulicAxis EXTENDS IAxis

    METHOD MovePressure : BOOL
    VAR_INPUT
        bExecute  : BOOL;
        rPressure : REAL;
        rRampRate : REAL;
        tDuration : TIME;
    END_VAR
    END_METHOD

    METHOD GetRequestedFlow : REAL
    END_METHOD

    METHOD GetHydMotionAxisID : SINT
    END_METHOD

    METHOD GetHydraulicStatus : ST_HydAxisStatus
    END_METHOD

    // INTERNAL：只允许 HydTechnology 内部调用
    METHOD SetPumpStatus
    VAR_INPUT
        rPumpSpeed : REAL;
        bConflict  : BOOL;
        bPumpError : BOOL;
    END_VAR
    END_METHOD

END_INTERFACE
~~~

## 3. FB_AxisBase

~~~iecst
FUNCTION_BLOCK ABSTRACT FB_AxisBase IMPLEMENTS IAxis

VAR_PROTECTED
    stCommand : ST_AxisCommand;
    stStatus  : ST_AxisStatus;
    eState    : E_AxisState := eInit;
END_VAR
~~~

~~~iecst
METHOD PUBLIC Enable : BOOL
VAR_INPUT
    bExecute : BOOL;
END_VAR
stCommand.bEnable := bExecute;
Enable := TRUE;
END_METHOD
~~~

~~~iecst
METHOD PUBLIC Jog : BOOL
VAR_INPUT
    bExecute      : BOOL;
    bPositive     : BOOL;
    rVelocity     : LREAL;
    rAcceleration : LREAL;
    rDeceleration : LREAL;
END_VAR
stCommand.bJog := bExecute;
stCommand.bJogPositive := bPositive;
stCommand.rVelocity := rVelocity;
stCommand.rAcceleration := rAcceleration;
stCommand.rDeceleration := rDeceleration;
Jog := TRUE;
END_METHOD
~~~

~~~iecst
METHOD PUBLIC MoveVelocity : BOOL
VAR_INPUT
    bExecute      : BOOL;
    rVelocity     : LREAL;
    rAcceleration : LREAL;
    rDeceleration : LREAL;
END_VAR
stCommand.bMoveVelocity := bExecute;
stCommand.rVelocity := rVelocity;
stCommand.rAcceleration := rAcceleration;
stCommand.rDeceleration := rDeceleration;
MoveVelocity := TRUE;
END_METHOD
~~~

~~~iecst
METHOD PUBLIC MoveAbsolute : BOOL
VAR_INPUT
    bExecute      : BOOL;
    rPosition     : LREAL;
    rVelocity     : LREAL;
    rAcceleration : LREAL;
    rDeceleration : LREAL;
END_VAR
stCommand.bMoveAbsolute := bExecute;
stCommand.rPosition := rPosition;
stCommand.rVelocity := rVelocity;
stCommand.rAcceleration := rAcceleration;
stCommand.rDeceleration := rDeceleration;
MoveAbsolute := TRUE;
END_METHOD
~~~

~~~iecst
METHOD PUBLIC Stop : BOOL
VAR_INPUT
    bExecute : BOOL;
END_VAR
stCommand.bStop := bExecute;
Stop := TRUE;
END_METHOD
~~~

~~~iecst
METHOD PUBLIC EStop : BOOL
VAR_INPUT
    bExecute : BOOL;
END_VAR
stCommand.bEStop := bExecute;
EStop := TRUE;
END_METHOD
~~~

~~~iecst
METHOD PUBLIC Reset : BOOL
VAR_INPUT
    bExecute : BOOL;
END_VAR
stCommand.bReset := bExecute;
Reset := TRUE;
END_METHOD
~~~

~~~iecst
METHOD PUBLIC GetStatus : ST_AxisStatus
GetStatus := stStatus;
END_METHOD
~~~

~~~iecst
PROPERTY PUBLIC Enabled : BOOL
GET
    Enabled := stStatus.bEnabled;
END_GET
END_PROPERTY

PROPERTY PUBLIC Busy : BOOL
GET
    Busy := stStatus.bBusy;
END_GET
END_PROPERTY

PROPERTY PUBLIC Done : BOOL
GET
    Done := stStatus.bDone;
END_GET
END_PROPERTY

PROPERTY PUBLIC Error : BOOL
GET
    Error := stStatus.bError;
END_GET
END_PROPERTY

PROPERTY PUBLIC State : E_AxisState
GET
    State := stStatus.eState;
END_GET
END_PROPERTY
~~~

~~~iecst
METHOD PUBLIC Cyclic : BOOL
IF stCommand.bEStop THEN
    eState := eEmergencyStop;
END_IF;
IF (eState <> eEmergencyStop) OR stCommand.bReset THEN
    ExecuteTechnology();
END_IF;
stStatus.eState := eState;
Cyclic := TRUE;
END_METHOD
~~~

~~~iecst
METHOD PROTECTED ExecuteTechnology : BOOL
ExecuteTechnology := FALSE;
END_METHOD
~~~

## 4. FB_ElectricAxis

SMC_FollowVelocity 没有 InVelocity 输出。速度到达状态必须使用 MC_MoveVelocity.InVelocity 或实际速度误差带，不能访问不存在的成员。

~~~iecst
FUNCTION_BLOCK FB_ElectricAxis
EXTENDS FB_AxisBase
IMPLEMENTS IAxis

VAR
    AxisRef : AXIS_REF;
    fbPower        : MC_Power;
    fbFollowVel    : SMC_FollowVelocity;
    fbJog          : MC_Jog;
    fbMoveAbsolute : MC_MoveAbsolute;
    fbStop         : MC_Stop;
    fbReset        : MC_Reset;
    fbReadPosition : MC_ReadActualPosition;
    fbReadVelocity : MC_ReadActualVelocity;
END_VAR
~~~

绑定方法只在初始化阶段调用一次，用于完成 VAR_IN_OUT 轴引用绑定；绑定完成后轴的底层 POU 只能由 Cyclic 方法调用。

~~~iecst
METHOD PUBLIC Initialize : BOOL
VAR_INPUT
    AxisRefIn : AXIS_REF;
END_VAR

AxisRef := AxisRefIn;
Initialize := TRUE;
END_METHOD
~~~

~~~iecst
METHOD PROTECTED ExecuteTechnology : BOOL

fbPower(
    Axis           := AxisRef,
    Enable         := stCommand.bEnable AND NOT stCommand.bEStop,
    EnablePositive := TRUE,
    EnableNegative := TRUE
);

fbFollowVel(
    Axis     := AxisRef,
    Execute  := stCommand.bMoveVelocity
                AND NOT stCommand.bJog
                AND NOT stCommand.bStop
                AND NOT stCommand.bEStop,
    Velocity := stCommand.rVelocity
);

fbJog(
    Axis         := AxisRef,
    JogForward   := stCommand.bJog AND stCommand.bJogPositive,
    JogBackward  := stCommand.bJog AND NOT stCommand.bJogPositive,
    Velocity     := stCommand.rVelocity,
    Acceleration := stCommand.rAcceleration,
    Deceleration := stCommand.rDeceleration,
    Jerk         := stCommand.rJerk
);

fbMoveAbsolute(
    Axis             := AxisRef,
    Execute          := stCommand.bMoveAbsolute
                       AND NOT stCommand.bStop
                       AND NOT stCommand.bEStop,
    ContinuousUpdate := FALSE,
    Position         := stCommand.rPosition,
    Velocity         := stCommand.rVelocity,
    Acceleration     := stCommand.rAcceleration,
    Deceleration     := stCommand.rDeceleration,
    Jerk             := stCommand.rJerk,
    Direction        := mcAbsolutePosition,
    BufferMode       := mcAborting
);

fbStop(
    Axis         := AxisRef,
    Execute      := stCommand.bStop OR stCommand.bEStop,
    Deceleration := stCommand.rDeceleration
);

fbReset(
    Axis    := AxisRef,
    Execute := stCommand.bReset
);

fbReadPosition(
    Axis   := AxisRef,
    Enable := TRUE
);

fbReadVelocity(
    Axis   := AxisRef,
    Enable := TRUE
);

stStatus.bEnabled := fbPower.Status;
stStatus.bBusy := fbFollowVel.Busy
                  OR fbJog.Busy
                  OR fbMoveAbsolute.Busy
                  OR fbStop.Busy;
stStatus.bDone := fbMoveAbsolute.Done OR fbJog.Done OR fbStop.Done;
stStatus.bError := fbPower.Error
                   OR fbFollowVel.Error
                   OR fbJog.Error
                   OR fbMoveAbsolute.Error
                   OR fbStop.Error;
stStatus.rActualPosition := fbReadPosition.Position;
stStatus.rActualVelocity := fbReadVelocity.Velocity;

IF fbPower.Error THEN
    stStatus.udiErrorID := WORD_TO_UDINT(fbPower.ErrorID);
ELSIF fbFollowVel.Error THEN
    stStatus.udiErrorID := WORD_TO_UDINT(fbFollowVel.ErrorID);
ELSIF fbMoveAbsolute.Error THEN
    stStatus.udiErrorID := WORD_TO_UDINT(fbMoveAbsolute.ErrorID);
END_IF;

IF stStatus.bError THEN
    eState := eError;
ELSIF stCommand.bEStop THEN
    eState := eEmergencyStop;
ELSIF stStatus.bBusy THEN
    eState := eBusy;
ELSIF stStatus.bDone THEN
    eState := eDone;
ELSIF stStatus.bEnabled THEN
    eState := eReady;
ELSE
    eState := eIdle;
END_IF;

stStatus.eState := eState;
ExecuteTechnology := TRUE;
END_METHOD
~~~

## 5. FB_HydraulicAxis

现有 FB_HydAxis 的 ST_HydAxisRef 使用以下换算：

- 速度：uiSpd / 1000.0 * rMaxSpeed；
- 位置：udiPos / 100.0；
- 压力：uiPres * 0.1。

~~~iecst
FUNCTION_BLOCK FB_HydraulicAxis
EXTENDS FB_AxisBase
IMPLEMENTS IHydraulicAxis

VAR_IN_OUT
    stHydAxisRef : ST_HydAxisRef;
END_VAR

VAR
    fbLegacyHydAxis : FB_HydAxis;
    fbReadStatus    : HYD_ReadStatus;
    uiMode          : UINT;
    uiAcc           : UINT;
    uiDec           : UINT;
    uiJerk          : UINT;
    rCommandPressure : REAL;
    rRequestedFlow  : REAL;
    stHydStatus     : ST_HydAxisStatus;
    bPumpConflict   : BOOL;
    bPumpError      : BOOL;
END_VAR
~~~

由于现有 FB_HydAxis 使用 ST_HydAxisRef 的 VAR_IN_OUT 语义，stHydAxisRef 必须在目标 PLCopen XML 实例配置中绑定到稳定的结构体变量；初始化阶段只允许校验和写入初始参数，不能在应用层每周期直接调用该功能块。

~~~iecst
METHOD PUBLIC Initialize : BOOL
VAR_INPUT
    bConfigured : BOOL;
END_VAR

Initialize := bConfigured AND (stHydAxisRef.iAxisID >= -1);
END_METHOD
~~~

~~~iecst
METHOD PUBLIC MovePressure : BOOL
VAR_INPUT
    bExecute  : BOOL;
    rPressure : REAL;
    rRampRate : REAL;
    tDuration : TIME;
END_VAR

stCommand.bMoveAbsolute := FALSE;
stCommand.bMoveVelocity := FALSE;

// 当前 FB_HydAxis 的 HYD_PressureHandle 持续时间固定为 0.0。
// 未扩展底层接口前，非零持续时间必须拒绝，不能静默忽略。
IF tDuration <> T#0s THEN
    MovePressure := FALSE;
    RETURN;
END_IF;

rCommandPressure := rPressure;
uiMode := 3;
stHydAxisRef.uiPres := REAL_TO_UINT(
    LIMIT(0.0, rPressure / 0.1, 65535.0)
);

IF (rRampRate > 0.0) AND (stHydAxisRef.rMaxSpeed > 0.0) THEN
    uiAcc := REAL_TO_UINT(
        LIMIT(1.0, stHydAxisRef.rMaxSpeed * 1000.0 / rRampRate, 65535.0)
    );
ELSE
    uiAcc := 0;
END_IF;

IF NOT bExecute THEN
    rCommandPressure := 0.0;
END_IF;

MovePressure := TRUE;
END_METHOD
~~~

~~~iecst
METHOD PUBLIC GetRequestedFlow : REAL
GetRequestedFlow := rRequestedFlow;
END_METHOD

METHOD PUBLIC GetHydMotionAxisID : SINT
GetHydMotionAxisID := stHydAxisRef.iAxisID;
END_METHOD

METHOD PUBLIC GetHydraulicStatus : ST_HydAxisStatus
GetHydraulicStatus := stHydStatus;
END_METHOD
~~~

~~~iecst
METHOD INTERNAL SetPumpStatus
VAR_INPUT
    rPumpSpeed : REAL;
    bConflict  : BOOL;
    bPumpErrorIn : BOOL;
END_VAR

bPumpConflict := bConflict;
bPumpError := bPumpErrorIn;
END_METHOD
~~~

~~~iecst
METHOD PROTECTED ExecuteTechnology : BOOL
VAR
    uiHydSpeed : UINT;
    udiHydPos  : UDINT;
END_VAR

IF stHydAxisRef.rMaxSpeed > 0.0 THEN
    uiHydSpeed := REAL_TO_UINT(
        LIMIT(
            0.0,
            ABS(stCommand.rVelocity) / stHydAxisRef.rMaxSpeed * 1000.0,
            1000.0
        )
    );
ELSE
    uiHydSpeed := 0;
END_IF;

udiHydPos := REAL_TO_UDINT(MAX(stCommand.rPosition, 0.0) * 100.0);

IF stCommand.bMoveAbsolute THEN
    uiMode := 1;
    stHydAxisRef.udiPos := udiHydPos;
    stHydAxisRef.uiSpd := uiHydSpeed;
ELSIF stCommand.bMoveVelocity THEN
    uiMode := 2;
    stHydAxisRef.uiSpd := uiHydSpeed;
ELSIF rCommandPressure > 0.0 THEN
    uiMode := 3;
END_IF;

fbLegacyHydAxis(
    stHydAxisRef := stHydAxisRef,
    bStart       := stCommand.bMoveAbsolute
                    OR stCommand.bMoveVelocity
                    OR (rCommandPressure > 0.0),
    bStop        := stCommand.bStop,
    bEStop       := stCommand.bEStop,
    bReset       := stCommand.bReset,
    uiDir        := stHydAxisRef.uiDir,
    uiMode       := uiMode,
    uiPres       := stHydAxisRef.uiPres,
    uiSpd        := stHydAxisRef.uiSpd,
    uiEndSpd     := stHydAxisRef.uiEndSpd,
    udiPos       := stHydAxisRef.udiPos,
    uiAcc        := uiAcc,
    uiDec        := uiDec,
    uiJerk        := uiJerk
);

fbReadStatus(
    AXISID := stHydAxisRef.iAxisID,
    ENABLE := TRUE
);

rRequestedFlow := fbReadStatus.REQUESTEDFLOW;
bPumpConflict := fbReadStatus.PUMPSPEEDLIMITACTIVE
                 OR fbReadStatus.FLOWLIMITACTIVE;
bPumpError := fbReadStatus.ERROR;

stHydStatus.iAxisID := stHydAxisRef.iAxisID;
stHydStatus.rRequestedFlow := fbReadStatus.REQUESTEDFLOW;
stHydStatus.rMaxFlow := fbReadStatus.MAXFLOW;
stHydStatus.bFlowLimitActive := fbReadStatus.FLOWLIMITACTIVE;
stHydStatus.bPumpSpeedLimitActive := fbReadStatus.PUMPSPEEDLIMITACTIVE;
stHydStatus.bPressureLimitActive := fbReadStatus.PRESSURELIMITACTIVE;
stHydStatus.bSoftLimitActive := fbReadStatus.SOFTLIMITACTIVE;
stHydStatus.bDerated := fbReadStatus.DERATED;

stStatus.bEnabled := stHydAxisRef.iAxisID >= 0;
stStatus.bBusy := fbLegacyHydAxis.bBusy;
stStatus.bDone := fbLegacyHydAxis.bDone;
stStatus.bError := fbLegacyHydAxis.bAlarm OR bPumpError;
stStatus.udiErrorID := DWORD_TO_UDINT(fbLegacyHydAxis.dwAlarmID);
stStatus.rActualPosition := REAL_TO_LREAL(fbReadStatus.ACTUATORPOSITION);
stStatus.rActualVelocity := REAL_TO_LREAL(fbReadStatus.ACTUATORVELOCITYCOMMAND);
stStatus.rActualPressure := INT_TO_REAL(fbReadStatus.PRESSURECONTROLLERAPPLIED);

IF stStatus.bError THEN
    eState := eError;
ELSIF stCommand.bEStop THEN
    eState := eEmergencyStop;
ELSIF stStatus.bBusy THEN
    eState := eBusy;
ELSIF stStatus.bDone THEN
    eState := eDone;
ELSIF stStatus.bEnabled THEN
    eState := eReady;
ELSE
    eState := eIdle;
END_IF;

stStatus.eState := eState;
ExecuteTechnology := TRUE;
END_METHOD
~~~

FB_HydraulicAxis 只上报请求流量和限幅状态，不调用 HYD_GetPumpRequest，不调用 FB_ServoControl，不实现优先级排序。

## 6. FB_HydPumpManager

~~~iecst
FUNCTION_BLOCK FB_HydPumpManager

VAR_INPUT
    bEnable          : BOOL;
    ePolicy          : E_PumpPolicy;
    bAllowNegative   : BOOL;
    uiMaxMotorSpeed  : UINT;
    uiFlowRateLimit  : UINT;
    udiCycleToken    : UDINT;
END_VAR

VAR_IN_OUT
    aHydAxis     : ARRAY[0..15] OF IHydraulicAxis;
    aHydAxisUsed : ARRAY[0..15] OF BOOL;
END_VAR

VAR_OUTPUT
    stStatus : ST_HydPumpStatus;
END_VAR

VAR
    fbGetPumpRequest : HYD_GetPumpRequest;
    fbServoControl   : FB_ServoControl;
    siStrategy       : SINT;
    udiLastCycleToken: UDINT;
    bTokenValid      : BOOL;
    ui               : UINT;
END_VAR
~~~

~~~iecst
METHOD INTERNAL ExecuteCycle : BOOL
VAR
    rTotalRequest : REAL;
END_VAR

IF bTokenValid AND (udiLastCycleToken = udiCycleToken) THEN
    stStatus.bCycleReentry := TRUE;
    ExecuteCycle := FALSE;
    RETURN;
END_IF;

bTokenValid := TRUE;
udiLastCycleToken := udiCycleToken;
stStatus.bCycleReentry := FALSE;
rTotalRequest := 0.0;

FOR ui := 0 TO 15 DO
    IF aHydAxisUsed[ui] THEN
        rTotalRequest := rTotalRequest
                       + ABS(aHydAxis[ui].GetRequestedFlow());
    END_IF;
END_FOR;

CASE ePolicy OF
    eMotionDefault:
        siStrategy := 0;
    eProcessPriority:
        siStrategy := 1;
ELSE
    siStrategy := 0;
END_CASE;

fbGetPumpRequest(
    ENABLE         := bEnable,
    STRATEGY       := siStrategy,
    ALLOW_NEGATIVE := bAllowNegative
);

fbServoControl(
    StartMotor      := bEnable
                       AND NOT fbGetPumpRequest.ERROR
                       AND NOT fbGetPumpRequest.CONFLICT,
    SetVelCmd       := fbGetPumpRequest.PUMPSPEED,
    FlowRateLimit   := uiFlowRateLimit,
    uiMaxMotorSpeed := uiMaxMotorSpeed,
    EnSimulation    := FALSE,
    SelfLearn       := FALSE,
    UsePlanner      := FALSE
);

stStatus.rPumpSpeed := fbGetPumpRequest.PUMPSPEED;
stStatus.rTotalRequest := rTotalRequest;
stStatus.bConflict := fbGetPumpRequest.CONFLICT;
stStatus.bBusy := fbGetPumpRequest.BUSY;
stStatus.bDone := fbGetPumpRequest.DONE;
stStatus.bError := fbGetPumpRequest.ERROR
                   OR (fbServoControl.outErrorID <> 0);
stStatus.wErrorID := fbGetPumpRequest.ERRORID;

IF fbServoControl.outErrorID <> 0 THEN
    stStatus.wErrorID := UINT_TO_WORD(fbServoControl.outErrorID);
END_IF;

FOR ui := 0 TO 15 DO
    IF aHydAxisUsed[ui] THEN
        aHydAxis[ui].SetPumpStatus(
            rPumpSpeed := stStatus.rPumpSpeed,
            bConflict := stStatus.bConflict,
            bPumpErrorIn := stStatus.bError
        );
    END_IF;
END_FOR;

ExecuteCycle := TRUE;
END_METHOD
~~~

## 7. 工艺动作

| 动作 | 状态顺序 | 压力能力 |
|---|---|---|
| 锁模 | 快速合模 → 慢速合模 → 压力锁模 → 完成 | 压力锁模必须有 IHydraulicAxis |
| 开模 | 破闸/脱模 → 快速开模 → 末端低速 → 完成 | 可选 |
| 射胶 | 快速射胶 → 慢速射胶 → 速度/压力切换 → 保压 → 完成 | 必须有 IHydraulicAxis |
| 储料 | 储料速度 → 计量位置 → 减速停止 → 完成 | 可选 |
| 顶针 | 前进 → 保持 → 返回 → 完成 | 可选 |

速度—压力切换必须由工艺参数明确选择位置阈值、压力阈值或先到条件，不能隐式混用。

## 8. FB_HydTechnology

~~~iecst
FUNCTION_BLOCK FB_HydTechnology

VAR_INPUT
    bEnable          : BOOL;
    ePumpPolicy      : E_PumpPolicy;
    bAllowNegative   : BOOL;
    uiMaxMotorSpeed  : UINT;
    uiFlowRateLimit  : UINT;
    udiCycleToken    : UDINT;
END_VAR

VAR_OUTPUT
    bInitialized : BOOL;
    bBusy        : BOOL;
    bDone        : BOOL;
    bError       : BOOL;
    stPumpStatus : ST_HydPumpStatus;
END_VAR

VAR
    aAxis       : ARRAY[0..31] OF IAxis;
    aAxisUsed   : ARRAY[0..31] OF BOOL;
    uiAxisCount : UINT;

    aHydAxis       : ARRAY[0..15] OF IHydraulicAxis;
    aHydAxisUsed   : ARRAY[0..15] OF BOOL;
    uiHydAxisCount : UINT;

    iClampAxis     : IAxis;
    iClampHydAxis  : IHydraulicAxis;
    iMoldOpenAxis  : IAxis;
    iInjectionAxis : IHydraulicAxis;
    iChargeAxis    : IAxis;
    iEjectAxis     : IAxis;

    fbClamp       : FB_ClampProcess;
    fbMoldOpen    : FB_MoldOpenProcess;
    fbInjection   : FB_InjectionProcess;
    fbCharge      : FB_ChargeProcess;
    fbEject       : FB_EjectProcess;
    fbTemperature : FB_TemperatureManager;
    fbSafety      : FB_SafetyInterlock;
    fbPumpManager : FB_HydPumpManager;

    ui            : UINT;
    bCycleStarted : BOOL;
    uiConfiguredMaxMotorSpeed : UINT;
    uiConfiguredFlowRateLimit : UINT;
END_VAR
~~~

初始化方法：

~~~iecst
METHOD PUBLIC Initialize : BOOL
VAR_INPUT
    uiMaxMotorSpeed : UINT;
    uiFlowRateLimit : UINT;
END_VAR

IF uiMaxMotorSpeed = 0 THEN
    Initialize := FALSE;
    RETURN;
END_IF;

uiConfiguredMaxMotorSpeed := uiMaxMotorSpeed;
uiConfiguredFlowRateLimit := uiFlowRateLimit;
bInitialized := FALSE;
Initialize := TRUE;
END_METHOD
~~~

注册方法示例：

~~~iecst
METHOD PUBLIC RegisterAxis : BOOL
VAR_INPUT
    iAxis : IAxis;
END_VAR

IF uiAxisCount >= 32 THEN
    RegisterAxis := FALSE;
    RETURN;
END_IF;

IF aAxisUsed[uiAxisCount] THEN
    RegisterAxis := FALSE;
    RETURN;
END_IF;

aAxis[uiAxisCount] := iAxis;
aAxisUsed[uiAxisCount] := TRUE;
uiAxisCount := uiAxisCount + 1;
RegisterAxis := TRUE;
END_METHOD
~~~

~~~iecst
METHOD PUBLIC RegisterHydraulicAxis : BOOL
VAR_INPUT
    iAxis : IHydraulicAxis;
END_VAR

IF uiHydAxisCount >= 16 THEN
    RegisterHydraulicAxis := FALSE;
    RETURN;
END_IF;

aHydAxis[uiHydAxisCount] := iAxis;
aHydAxisUsed[uiHydAxisCount] := TRUE;
uiHydAxisCount := uiHydAxisCount + 1;

RegisterAxis(iAxis := iAxis);
RegisterHydraulicAxis := TRUE;
END_METHOD
~~~

公共方法必须提供：

~~~text
Initialize()
RegisterAxis()
RegisterHydraulicAxis()
BindClampAxis()
BindMoldOpenAxis()
BindInjectionAxis()
BindChargeAxis()
BindEjectAxis()
StartClamp()
StartMoldOpen()
StartInjection()
StartCharge()
StartEject()
Stop()
EStop()
Reset()
Cyclic()
~~~

周期方法：

~~~iecst
METHOD PUBLIC Cyclic : BOOL

IF bCycleStarted THEN
    bError := TRUE;
    Cyclic := FALSE;
    RETURN;
END_IF;

bCycleStarted := TRUE;

fbTemperature.Cyclic();
fbSafety.Cyclic();

IF fbSafety.bMotionPermit AND fbTemperature.bReady THEN
    fbClamp.Cyclic(
        iAxis := iClampAxis,
        iHydraulicAxis := iClampHydAxis
    );
    fbMoldOpen.Cyclic(iAxis := iMoldOpenAxis);
    fbInjection.Cyclic(iAxis := iInjectionAxis);
    fbCharge.Cyclic(iAxis := iChargeAxis);
    fbEject.Cyclic(iAxis := iEjectAxis);
END_IF;

FOR ui := 0 TO 31 DO
    IF aAxisUsed[ui] THEN
        aAxis[ui].Cyclic();
    END_IF;
END_FOR;

fbPumpManager.ExecuteCycle(
    bEnable          := bEnable AND fbSafety.bMotionPermit,
    ePolicy          := ePumpPolicy,
    bAllowNegative   := bAllowNegative,
    uiMaxMotorSpeed  := uiMaxMotorSpeed,
    uiFlowRateLimit  := uiFlowRateLimit,
    udiCycleToken    := udiCycleToken,
    aHydAxis         := aHydAxis,
    aHydAxisUsed     := aHydAxisUsed
);

stPumpStatus := fbPumpManager.stStatus;
bBusy := fbPumpManager.stStatus.bBusy
         OR fbClamp.bBusy
         OR fbMoldOpen.bBusy
         OR fbInjection.bBusy
         OR fbCharge.bBusy
         OR fbEject.bBusy;

bError := fbPumpManager.stStatus.bError
          OR fbClamp.bError
          OR fbMoldOpen.bError
          OR fbInjection.bError
          OR fbCharge.bError
          OR fbEject.bError;

bDone := NOT bBusy AND NOT bError;
bInitialized := TRUE;
bCycleStarted := FALSE;
Cyclic := TRUE;
END_METHOD
~~~

</抽象设计示例>

<应用层集成示例>

## 1. 变量声明

~~~iecst
PROGRAM program0

VAR
    stElectricAxisRef : AXIS_REF;
    stClampHydRef     : ST_HydAxisRef;
    stInjectionHydRef : ST_HydAxisRef;

    fbElectricAxis  : FB_ElectricAxis;
    fbClampHydAxis  : FB_HydraulicAxis;
    fbInjectionAxis : FB_HydraulicAxis;

    iElectricAxis  : IAxis;
    iClampAxis     : IAxis;
    iClampHydAxis  : IHydraulicAxis;
    iInjectionAxis : IHydraulicAxis;

    fbHydTechnology : FB_HydTechnology;

    bInitialized    : BOOL;
    bMachineEnable  : BOOL;
    bManualMode     : BOOL;
    bAutoMode       : BOOL;
    bManualRun      : BOOL;
    bManualPositive : BOOL;
    rManualVelocity : LREAL;

    bClampStart     : BOOL;
    bInjectionStart : BOOL;
    bChargeStart    : BOOL;
    bEjectStart     : BOOL;
    bResetCommand   : BOOL;

    stElectricStatus  : ST_AxisStatus;
    stClampStatus     : ST_AxisStatus;
    stInjectionStatus : ST_AxisStatus;
    bMachineReady     : BOOL;
    bCycleComplete    : BOOL;

    udiTaskCycleToken : UDINT;
END_VAR
~~~

## 2. 初始化和注册

~~~iecst
IF NOT bInitialized THEN

    stElectricAxisRef := 0;

    stClampHydRef.iAxisID := -1;
    stClampHydRef.iMechanismType := 0;
    stClampHydRef.rMaxSpeed := 500.0;
    stClampHydRef.rVelToFlowGain := 0.2;

    stInjectionHydRef.iAxisID := -1;
    stInjectionHydRef.iMechanismType := 0;
    stInjectionHydRef.rMaxSpeed := 300.0;
    stInjectionHydRef.rVelToFlowGain := 0.2;

    fbElectricAxis.Initialize(
        AxisRefIn := stElectricAxisRef
    );

    fbClampHydAxis.Initialize(
        bConfigured := TRUE
    );

    fbInjectionAxis.Initialize(
        bConfigured := TRUE
    );

    iElectricAxis := fbElectricAxis;
    iClampAxis := fbClampHydAxis;
    iClampHydAxis := fbClampHydAxis;
    iInjectionAxis := fbInjectionAxis;

    fbHydTechnology.Initialize(
        uiMaxMotorSpeed := 1800,
        uiFlowRateLimit := 1000
    );

    fbHydTechnology.RegisterAxis(
        iAxis := iElectricAxis
    );

    fbHydTechnology.RegisterHydraulicAxis(
        iAxis := iClampHydAxis
    );

    fbHydTechnology.RegisterHydraulicAxis(
        iAxis := iInjectionAxis
    );

    fbHydTechnology.BindClampAxis(
        iAxis := iClampAxis,
        iHydraulicAxis := iClampHydAxis
    );

    fbHydTechnology.BindInjectionAxis(
        iAxis := iInjectionAxis
    );

    fbHydTechnology.BindChargeAxis(
        iAxis := iElectricAxis
    );

    bInitialized := TRUE;
END_IF;
~~~

初始化后禁止重复注册同一对象、修改轴类型或修改泵组。

## 3. 统一手动操作

~~~iecst
IF bManualMode AND bMachineEnable THEN

    iElectricAxis.Enable(
        bExecute := TRUE
    );

    iElectricAxis.MoveVelocity(
        bExecute      := bManualRun,
        rVelocity     := rManualVelocity,
        rAcceleration := 1000.0,
        rDeceleration := 1000.0
    );

    iClampAxis.Enable(
        bExecute := TRUE
    );

    iClampAxis.MoveVelocity(
        bExecute      := bManualRun,
        rVelocity     := rManualVelocity,
        rAcceleration := 500.0,
        rDeceleration := 500.0
    );

    IF NOT bManualRun THEN
        iElectricAxis.Stop(bExecute := TRUE);
        iClampAxis.Stop(bExecute := TRUE);
    END_IF;
END_IF;
~~~

应用层使用相同的 IAxis 方法操作电动轴和液压轴，不判断具体轴类型。

## 4. 自动工艺命令

~~~iecst
IF bAutoMode AND bMachineEnable THEN

    IF bClampStart THEN
        fbHydTechnology.StartClamp(
            bExecute := TRUE
        );
    END_IF;

    IF bInjectionStart THEN
        fbHydTechnology.StartInjection(
            bExecute := TRUE
        );
    END_IF;

    IF bChargeStart THEN
        fbHydTechnology.StartCharge(
            bExecute := TRUE
        );
    END_IF;

    IF bEjectStart THEN
        fbHydTechnology.StartEject(
            bExecute := TRUE
        );
    END_IF;
END_IF;
~~~

应用层只发出工艺动作请求，多段运动、压力切换、保压和保持时间由 HydTechnology 内部处理。

## 5. 急停和复位

~~~iecst
IF gSafetyEmergencyStop THEN
    fbHydTechnology.EStop(
        bExecute := TRUE
    );
END_IF;

IF bResetCommand AND NOT gSafetyEmergencyStop THEN
    fbHydTechnology.Reset(
        bExecute := TRUE
    );
END_IF;
~~~

HydTechnology 内部将急停和复位传递给所有已注册轴，并控制单泵安全停机。

## 6. 唯一 PLC 周期入口

~~~iecst
udiTaskCycleToken := udiTaskCycleToken + 1;

fbHydTechnology.Cyclic(
    bEnable         := bMachineEnable,
    ePumpPolicy     := eMotionDefault,
    bAllowNegative  := FALSE,
    uiMaxMotorSpeed := 1800,
    uiFlowRateLimit := 1000,
    udiCycleToken   := udiTaskCycleToken
);
~~~

应用层不得在同一周期再次调用具体轴实例或泵管理器。应用层中禁止出现：

任何底层泵请求、泵伺服、泵速变量、流量数组或泵算法代码。

## 7. 状态读取

~~~iecst
stElectricStatus := iElectricAxis.GetStatus();
stClampStatus := iClampAxis.GetStatus();
stInjectionStatus := iInjectionAxis.GetStatus();

bMachineReady :=
    stElectricStatus.bEnabled
    AND stClampStatus.bEnabled
    AND stInjectionStatus.bEnabled
    AND NOT fbHydTechnology.bError;

bCycleComplete :=
    fbHydTechnology.bDone
    AND NOT fbHydTechnology.bBusy
    AND NOT fbHydTechnology.bError;
~~~

应用层只读取统一轴状态、工艺状态、温度许可、安全许可和汇总报警。

</应用层集成示例>
