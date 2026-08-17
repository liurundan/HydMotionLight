# HydTechnology OOP 单泵原型框架 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不破坏现有电动轴、液压轴基础封装的前提下，为 `plc.xml` 搭建可编译的 HydTechnology OOP 原型框架，包含统一轴接口、单泵管理器门面、五类工艺动作骨架和应用层调用样例。

**Architecture:** 以 `IAxis` 为统一多态入口，以抽象 `FB_AxisBase` 复用公共状态和命令锁存；`FB_ElectricAxis` 组合现有 `MC_Power`/`SMC_FollowVelocity`，`FB_HydraulicAxis` 组合现有 `FB_HydAxis`。`FB_HydPumpManager` 只封装一次 `HYD_GetPumpRequest` 和一次 `FB_ServoControl`，不复制 HydMotion 的仲裁算法；`FB_HydTechnology` 是应用层唯一周期入口并拥有工艺动作实例。

**Tech Stack:** PLCopen XML、IEC 61131-3 Structured Text、Beremiz/Luban OOP 编译器、现有 `pousGm2.xml` 和 `pousHydMotion.xml` 库。

---

## Scope and file map

本计划只实施快速原型框架，不实现完整参数管理、完整多段轨迹算法、实际温控闭环或新的底层液压驱动算法。

**Files to modify**

- `plc.xml: before ST_HydAxisRef`：新增公共枚举、状态结构和策略类型。
- `plc.xml: after existing type declarations`：新增 `IAxis`、`IHydraulicAxis` 和 OOP 元数据。
- `plc.xml: after FB_CalcAxisMaxSpeed`：新增 `FB_AxisBase`、`FB_ElectricAxis`、`FB_HydraulicAxis`、`FB_HydPumpManager`、工艺动作 FB 和 `FB_HydTechnology`。
- `plc.xml: program0`：替换占位计数代码为最小注册、周期和动作调用样例。
- `HydTechnology_技术库使用说明书.md: append prototype section`：记录新增类型、周期约束和原型限制。

**Files to keep unchanged**

- `pousGm2.xml`：不改 `MC_Power`、`SMC_FollowVelocity` 等基础块。
- `pousHydMotion.xml`：不改现有 `HYD_GetPumpRequest`、`HYD_ReadStatus` 和运动块语义。
- `build/`：不手工编辑。

当前工作区没有 `.git`，执行阶段不安排 Git 提交命令；每个任务通过 XML 解析、静态扫描和 Beremiz 构建日志形成验证证据。

## Task 1: Baseline and XML safety gate

**Files:**

- Read: `plc.xml`
- Read: `pousGm2.xml`
- Read: `pousHydMotion.xml`
- Read: `HydTechnology_技术库使用说明书.md`

- [ ] **Step 1: Capture the baseline XML and symbol list**

Run:

```powershell
Select-String -Path plc.xml -Pattern '<pou name="FB_ServoControl"','<pou name="FB_HydAxis"','<pou name="program0"','<pou name="FB_EleAxis"'
rg -n "MC_Power|SMC_FollowVelocity|HYD_GetPumpRequest|HYD_ReadStatus" pousGm2.xml pousHydMotion.xml
```

Expected: the existing anchors are reported near lines 1078, 1724, 2475 and 2648; all four required lower-level symbols are found in the two library files.

- [ ] **Step 2: Parse the unmodified PLCopen XML**

Run:

```powershell
[xml]$baseline = Get-Content -Raw plc.xml
if ($null -eq $baseline.project) { throw 'PLCopen project root is missing' }
Write-Output 'baseline plc.xml: XML OK'
```

Expected: `baseline plc.xml: XML OK`.

- [ ] **Step 3: Record the OOP XML encoding gate**

Use the target OOP-capable editor/compiler to export or inspect one minimal interface/function-block inheritance fixture. Preserve the existing `<Method>` and `accessmodifiers` pattern from `plc.xml` for methods. The implementation must use the target’s actual PLCopen XML metadata for `INTERFACE`, `EXTENDS`, `IMPLEMENTS` and `PROPERTY`; do not invent unsupported XML attributes. The fixture is a syntax reference only and is not added to the project.

## Task 2: Add prototype data types and interfaces

**Files:**

- Modify: `plc.xml` before `ST_HydAxisRef` (around line 99)

- [ ] **Step 1: Add the shared enumerations and structures**

Add the following IEC declarations to the type section, represented in PLCopen XML using the existing `dataType`/`baseType` structure:

```iecst
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

TYPE E_PumpPolicy :
(
    eMotionDefault := 0,
    eProcessPriority := 1
);
END_TYPE

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

TYPE ST_HydPumpStatus :
STRUCT
    rPumpSpeed : REAL;
    bConflict  : BOOL;
    bBusy      : BOOL;
    bDone      : BOOL;
    bError     : BOOL;
    wErrorID   : WORD;
END_STRUCT
END_TYPE
```

- [ ] **Step 2: Add `IAxis` using the validated target XML OOP encoding**

The interface must expose the following signatures and properties:

```iecst
INTERFACE IAxis
    METHOD Enable : BOOL
        VAR_INPUT bExecute : BOOL; END_VAR
    END_METHOD

    METHOD MoveVelocity : BOOL
        VAR_INPUT
            bExecute : BOOL;
            rVelocity : LREAL;
            rAcceleration : LREAL;
            rDeceleration : LREAL;
        END_VAR
    END_METHOD

    METHOD MoveAbsolute : BOOL
        VAR_INPUT
            bExecute : BOOL;
            rPosition : LREAL;
            rVelocity : LREAL;
            rAcceleration : LREAL;
            rDeceleration : LREAL;
        END_VAR
    END_METHOD

    METHOD Stop : BOOL
        VAR_INPUT bExecute : BOOL; END_VAR
    END_METHOD

    METHOD EStop : BOOL
        VAR_INPUT bExecute : BOOL; END_VAR
    END_METHOD

    METHOD Reset : BOOL
        VAR_INPUT bExecute : BOOL; END_VAR
    END_METHOD

    METHOD Cyclic : BOOL
    END_METHOD

    METHOD GetStatus : ST_AxisStatus
    END_METHOD

    PROPERTY AxisName : STRING;
    PROPERTY State : E_AxisState;
    PROPERTY Enabled : BOOL;
    PROPERTY Busy : BOOL;
    PROPERTY Done : BOOL;
    PROPERTY Error : BOOL;
END_INTERFACE
```

- [ ] **Step 3: Add `IHydraulicAxis EXTENDS IAxis`**

Add only the hydraulic-specific methods required by the prototype:

```iecst
INTERFACE IHydraulicAxis EXTENDS IAxis
    METHOD MovePressure : BOOL
        VAR_INPUT
            bExecute : BOOL;
            rPressure : REAL;
            rRampRate : REAL;
        END_VAR
    END_METHOD

    METHOD GetRequestedFlow : REAL
    END_METHOD

    METHOD GetHydMotionAxisID : SINT
    END_METHOD

    METHOD SetPumpStatus
        VAR_INPUT
            rPumpSpeed : REAL;
            bConflict : BOOL;
            bError : BOOL;
        END_VAR
    END_METHOD

    PROPERTY RequestedFlow : REAL;
    PROPERTY PumpConflict : BOOL;
END_INTERFACE
```

- [ ] **Step 4: Parse the project after type/interface insertion**

Run:

```powershell
[xml]$project = Get-Content -Raw plc.xml
$required = 'E_AxisState','E_AxisMode','E_PumpPolicy','ST_AxisStatus','ST_HydPumpStatus'
foreach ($name in $required) {
    if (-not ($project.project.types.dataTypes.dataType | Where-Object name -eq $name)) {
        throw "Missing data type $name"
    }
}
Write-Output 'shared types: present; XML OK'
```

Expected: all five types are present and the XML parses.

## Task 3: Add the axis abstraction and concrete adapters

**Files:**

- Modify: `plc.xml` after `FB_CalcAxisMaxSpeed`
- Preserve: existing `FB_HydAxis` around line 1724
- Preserve: placeholder `FB_EleAxis` until the compatibility alias is added

- [ ] **Step 1: Add the abstract `FB_AxisBase` skeleton**

Add an abstract FB that implements command latching and status properties. The prototype body may leave actual motion execution to `ExecuteTechnology`:

```iecst
FUNCTION_BLOCK ABSTRACT FB_AxisBase IMPLEMENTS IAxis
VAR_PROTECTED
    eAxisState : E_AxisState := eInit;
    stStatus : ST_AxisStatus;
    bCmdEnable : BOOL;
    bCmdStop : BOOL;
    bCmdEStop : BOOL;
    bCmdReset : BOOL;
    bCmdMoveVelocity : BOOL;
    rCmdVelocity : LREAL;
END_VAR

METHOD PUBLIC Enable : BOOL
VAR_INPUT bExecute : BOOL; END_VAR
bCmdEnable := bExecute;
Enable := TRUE;
END_METHOD

METHOD PUBLIC Stop : BOOL
VAR_INPUT bExecute : BOOL; END_VAR
bCmdStop := bExecute;
Stop := TRUE;
END_METHOD

METHOD PUBLIC EStop : BOOL
VAR_INPUT bExecute : BOOL; END_VAR
bCmdEStop := bExecute;
EStop := TRUE;
END_METHOD

METHOD PUBLIC Reset : BOOL
VAR_INPUT bExecute : BOOL; END_VAR
bCmdReset := bExecute;
Reset := TRUE;
END_METHOD
```

Add `MoveVelocity`, `MoveAbsolute`, `Cyclic`, `GetStatus` and the six read-only properties with the same names declared in `IAxis`. Add a protected `ExecuteTechnology : BOOL` method with a default `FALSE` result; `Cyclic` must be the only method that calls this technology-specific implementation.

- [ ] **Step 2: Add `FB_ElectricAxis` as a minimal adapter**

Declare `AxisRef : AXIS_REF`, `fbPower : MC_Power`, and `fbFollowVelocity : SMC_FollowVelocity`. The prototype cycle must call both existing FBs every time:

```iecst
fbPower(
    Axis := AxisRef,
    Enable := bCmdEnable AND NOT bCmdEStop,
    Enable_Positive := TRUE,
    Enable_Negative := TRUE
);

fbFollowVelocity(
    Axis := AxisRef,
    Execute := bCmdMoveVelocity AND NOT bCmdStop AND NOT bCmdEStop,
    Velocity := rCmdVelocity
);

stStatus.bEnabled := fbPower.Status;
stStatus.bBusy := fbFollowVelocity.Busy;
stStatus.bDone := fbFollowVelocity.InVelocity;
stStatus.bError := fbPower.Error OR fbFollowVelocity.Error;
```

Map `fbFollowVelocity.ErrorID` to `stStatus.udiErrorID`. Do not add a second servo implementation.

- [ ] **Step 3: Add `FB_HydraulicAxis` as a wrapper around `FB_HydAxis`**

Declare `stHydAxisRef : ST_HydAxisRef` as `VAR_IN_OUT`, one `FB_HydAxis` instance and one `HYD_ReadStatus` instance. Pass every required input from the existing `FB_HydAxis` interface, using the existing defaults for fields not yet modeled by the prototype. The adapter must:

```iecst
fbLegacyHydAxis(
    stHydAxisRef := stHydAxisRef,
    bStart := bCmdMoveVelocity,
    bStop := bCmdStop,
    bEStop := bCmdEStop,
    bReset := bCmdReset,
    uiMode := 2
);

fbReadStatus(
    AXISID := stHydAxisRef.iAxisID,
    ENABLE := TRUE
);

rRequestedFlow := fbReadStatus.REQUESTEDFLOW;
stStatus.bError := fbLegacyHydAxis.bAlarm OR fbReadStatus.ERROR;
```

Add `MovePressure`, `GetRequestedFlow`, `GetHydMotionAxisID` and `SetPumpStatus` methods. `SetPumpStatus` only stores the global pump diagnostic; it must not alter a per-axis allocation.

- [ ] **Step 4: Add project-structure entries and compile-name checks**

Add `FB_AxisBase`, `FB_ElectricAxis`, `FB_HydraulicAxis` and interface objects to the PLCopen project structure using the same `addData` convention as existing objects. Keep `FB_EleAxis` as a compatibility placeholder until the new adapter is proven to parse.

Run:

```powershell
[xml]$project = Get-Content -Raw plc.xml
$names = $project.project.types.pous.pou.name
foreach ($name in 'FB_AxisBase','FB_ElectricAxis','FB_HydraulicAxis') {
    if ($names -notcontains $name) { throw "Missing $name" }
}
Write-Output 'axis abstraction: present; XML OK'
```

## Task 4: Add the single-pump manager wrapper

**Files:**

- Modify: `plc.xml` after `FB_HydraulicAxis`
- Read-only dependency: `pousHydMotion.xml:1164` (`HYD_GetPumpRequest`)
- Read-only dependency: `plc.xml:1078` (`FB_ServoControl`)

- [ ] **Step 1: Declare the manager state**

Add a fixed array of `IHydraulicAxis`, used flags, one `HYD_GetPumpRequest` and one `FB_ServoControl`:

```iecst
FUNCTION_BLOCK FB_HydPumpManager
VAR_INPUT
    bEnable : BOOL;
    ePolicy : E_PumpPolicy;
    bAllowNegative : BOOL;
    uiMaxMotorSpeed : UINT;
END_VAR
VAR_OUTPUT
    stStatus : ST_HydPumpStatus;
END_VAR
VAR
    aHydAxis : ARRAY[0..15] OF IHydraulicAxis;
    aHydAxisUsed : ARRAY[0..15] OF BOOL;
    uiHydAxisCount : UINT;
    fbGetPumpRequest : HYD_GetPumpRequest;
    fbServoControl : FB_ServoControl;
END_VAR
```

- [ ] **Step 2: Add registration without priority sorting**

Implement `RegisterHydraulicAxis(iAxis : IHydraulicAxis) : BOOL`. It stores the interface in the first free slot and returns `FALSE` when all 16 slots are occupied. There is no `uiPriority` field because current HydMotion does not expose per-axis priority.

- [ ] **Step 3: Add the one-call cycle body**

Declare a local `siStrategy : SINT` and map the two prototype policies explicitly before calling the two lower-level blocks exactly once:

```iecst
CASE ePolicy OF
    eMotionDefault:
        siStrategy := 0;
    eProcessPriority:
        siStrategy := 1;
ELSE
    siStrategy := 0;
END_CASE;

fbGetPumpRequest(
    ENABLE := bEnable,
    STRATEGY := siStrategy,
    ALLOW_NEGATIVE := bAllowNegative
);

fbServoControl(
    StartMotor := bEnable
                  AND NOT fbGetPumpRequest.ERROR
                  AND NOT fbGetPumpRequest.CONFLICT,
    SetVelCmd := fbGetPumpRequest.PUMPSPEED,
    FlowRateLimit := 1000,
    uiMaxMotorSpeed := uiMaxMotorSpeed,
    EnSimulation := FALSE,
    SelfLearn := FALSE,
    UsePlanner := FALSE
);
```

Copy `PUMPSPEED`, `CONFLICT`, `BUSY`, `DONE`, `ERROR` and `ERRORID` into `stStatus`, then broadcast only the global status through `SetPumpStatus`. Do not calculate `rGrantedFlow`, sort requests, or call either lower-level FB inside the axis adapters.

- [ ] **Step 4: Add static duplicate-call guards**

Add `udiCycleSequence` and `bCycleExecuted` diagnostics. `Cyclic()` sets `bCycleExecuted := TRUE` at entry, clears it at exit, and exposes a diagnostic bit if reentered before completion. This is a prototype diagnostic, not a second scheduler.

- [ ] **Step 5: Verify manager isolation**

Run:

```powershell
rg -n "HYD_GetPumpRequest|FB_ServoControl|fbGetPumpRequest|fbServoControl" plc.xml
```

Expected: the new names occur in `FB_HydPumpManager` and the existing `FB_ServoControl` definition only; no process FB or `program0` contains these calls.

## Task 5: Add the five process-action skeletons

**Files:**

- Modify: `plc.xml` after `FB_HydPumpManager`

- [ ] **Step 1: Add shared process-step enumeration**

Add `E_ProcessStep` with `eIdle`, `eSegment1`, `eSegment2`, `ePressure`, `eDwell`, `eDone`, `eError`.

- [ ] **Step 2: Add `FB_ClampProcess`**

Inputs: `bExecute : BOOL`, `iAxis : IAxis`, `iHydAxis : IHydraulicAxis`, fast position/velocity, slow position/velocity and lock pressure. Add a public `Cyclic() : BOOL` method; its body contains the following prototype sequence:

```iecst
CASE eStep OF
    eIdle:
        IF bExecute THEN
            iAxis.Enable(TRUE);
            iAxis.MoveAbsolute(TRUE, rFastPosition, rFastVelocity, 0.0, 0.0);
            eStep := eSegment1;
        END_IF;
    eSegment1:
        IF iAxis.Done THEN
            iAxis.MoveAbsolute(TRUE, rSlowPosition, rSlowVelocity, 0.0, 0.0);
            eStep := eSegment2;
        END_IF;
    eSegment2:
        IF iAxis.Done THEN
            iHydAxis.MovePressure(TRUE, rLockPressure, rPressureRamp);
            eStep := ePressure;
        END_IF;
    ePressure:
        IF NOT bExecute THEN
            iHydAxis.Stop(TRUE);
            eStep := eDone;
        END_IF;
END_CASE;
```

Guard every active step with `iAxis.Error`/`iHydAxis.Error` and transition to `eError`.

- [ ] **Step 3: Add `FB_MoldOpenProcess`**

Add a public `Cyclic() : BOOL` method and implement three position segments: breakaway, fast open and end-position slow open. It accepts only `IAxis`, so it can be bound to either an electric or hydraulic axis.

- [ ] **Step 4: Add `FB_InjectionProcess`**

Add a public `Cyclic() : BOOL` method and implement fast injection, slow injection, pressure switch and holding pressure using `IHydraulicAxis.MovePressure`. The process FB does not inspect pump speed or call a pump block.

- [ ] **Step 5: Add `FB_ChargeProcess`**

Add a public `Cyclic() : BOOL` method and implement one `MoveAbsolute` to the metering position with the configured charging velocity. Preserve `Busy`, `Done` and `Error` outputs.

- [ ] **Step 6: Add `FB_EjectProcess`**

Add a public `Cyclic() : BOOL` method and implement forward position, `TON` dwell, return position and completion. The timer instance must be retained across cycles.

- [ ] **Step 7: Run process isolation scan**

Run:

```powershell
$processNames = 'FB_ClampProcess','FB_MoldOpenProcess','FB_InjectionProcess','FB_ChargeProcess','FB_EjectProcess'
foreach ($name in $processNames) {
    $hit = Select-String -Path plc.xml -Pattern "<pou name=\"$name\""
    if ($null -eq $hit) { throw "Missing process FB $name" }
}

$processBlock = Get-Content -Raw plc.xml
if ($processBlock -match 'FB_ClampProcess[\s\S]*?HYD_GetPumpRequest') {
    throw 'Process layer must not call HYD_GetPumpRequest'
}
Write-Output 'process skeletons: present and isolated'
```

## Task 6: Add `FB_HydTechnology` facade and program0 prototype

**Files:**

- Modify: `plc.xml` after the process FBs
- Modify: `plc.xml:2475` (`program0`)

- [ ] **Step 1: Add the facade registry**

Declare `aAxis : ARRAY[0..31] OF IAxis`, `aAxisUsed : ARRAY[0..31] OF BOOL`, `uiAxisCount`, one instance of every process FB and one `FB_HydPumpManager`. Add `RegisterAxis`, `RegisterHydraulicAxis`, `SetPumpPolicy`, `StartClamp`, `StartMoldOpen`, `StartInjection`, `StartCharge`, and `StartEject` methods.

- [ ] **Step 2: Implement the deterministic cycle order**

The only cyclic body is:

```iecst
fbClamp.Cyclic();
fbMoldOpen.Cyclic();
fbInjection.Cyclic();
fbCharge.Cyclic();
fbEject.Cyclic();

FOR i := 0 TO 31 DO
    IF aAxisUsed[i] THEN
        aAxis[i].Cyclic();
    END_IF;
END_FOR;

fbPumpManager.Cyclic();
```

The process FB calls must occur before axis cycles so commands are latched for the current PLC cycle. The pump manager call must occur once after all registered axes.

- [ ] **Step 3: Replace `program0` placeholder code with a smoke example**

Declare one hydraulic axis, one electric axis, their interface references, one `FB_HydTechnology`, an initialization latch and command booleans. The body must contain only:

```iecst
IF NOT bRegistered THEN
    iHydAxis := fbHydraulicAxis;
    iEleAxis := fbElectricAxis;
    fbTechnology.RegisterHydraulicAxis(iHydAxis);
    fbTechnology.RegisterAxis(iEleAxis);
    fbTechnology.SetPumpPolicy(eMotionDefault);
    bRegistered := TRUE;
END_IF;

iHydAxis.Enable(bMachineEnable);
iEleAxis.Enable(bMachineEnable);
iHydAxis.MoveVelocity(bRunHydraulic, 20.0, 100.0, 100.0);
iEleAxis.MoveVelocity(bRunElectric, 1200.0, 2000.0, 2000.0);
fbTechnology.StartClamp(bClampCommand);
fbTechnology.StartInjection(bInjectionCommand);
fbTechnology.Cyclic();
```

No `HYD_GetPumpRequest`, `FB_ServoControl`, `PUMPSPEED`, `CONFLICT`, flow arrays or allocation calculations may appear in `program0`.

- [ ] **Step 4: Update the project structure metadata**

Add all new FBs, types, interfaces and methods to the existing PLCopen project-structure `addData` section. Keep the existing `program0`, `FB_ServoControl`, `FB_HydAxis`, `FB_CalcAxisMaxSpeed` entries.

## Task 7: Document the prototype boundary

**Files:**

- Modify: `HydTechnology_技术库使用说明书.md` after the existing FB usage sections

- [ ] **Step 1: Add the prototype API table**

Document `IAxis`, `IHydraulicAxis`, `FB_AxisBase`, `FB_ElectricAxis`, `FB_HydraulicAxis`, `FB_HydPumpManager`, `FB_HydTechnology` and the five process FBs.

- [ ] **Step 2: Add the single-pump rule**

State that `HYD_GetPumpRequest` is HydMotion’s sole arbitration point, `FB_HydPumpManager` calls it once per PLC cycle, and application code must never call it directly.

- [ ] **Step 3: Add prototype limitations**

Record that per-axis priority and granted-flow telemetry are not exposed by the current HydMotion interface; the prototype forwards global pump status only. Record that full temperature control, recipe persistence and production-grade safety validation are outside this prototype.

## Task 8: Validation and handoff

**Files:**

- Read/validate: `plc.xml`, `HydTechnology_技术库使用说明书.md`

- [ ] **Step 1: Parse final XML**

Run:

```powershell
[xml]$final = Get-Content -Raw plc.xml
if ($null -eq $final.project) { throw 'PLCopen project root is missing' }
Write-Output 'final plc.xml: XML OK'
```

Expected: `final plc.xml: XML OK`.

- [ ] **Step 2: Verify required symbols**

Run:

```powershell
$required = @(
    'FB_AxisBase','FB_ElectricAxis','FB_HydraulicAxis',
    'FB_HydPumpManager','FB_HydTechnology',
    'FB_ClampProcess','FB_MoldOpenProcess',
    'FB_InjectionProcess','FB_ChargeProcess','FB_EjectProcess'
)
$names = $final.project.types.pous.pou.name
foreach ($name in $required) {
    if ($names -notcontains $name) { throw "Missing $name" }
}
Write-Output 'required prototype symbols: present'
```

Expected: `required prototype symbols: present`.

- [ ] **Step 3: Verify one-call isolation statically**

Run:

```powershell
$xmlText = Get-Content -Raw plc.xml
$managerBlock = ($xmlText -split '<pou name="FB_HydPumpManager"')[1] -split '</pou>' | Select-Object -First 1
if (($managerBlock | Select-String -AllMatches 'HYD_GetPumpRequest').Matches.Count -ne 1) {
    throw 'Expected one HYD_GetPumpRequest occurrence inside manager'
}
if (($managerBlock | Select-String -AllMatches 'FB_ServoControl').Matches.Count -lt 1) {
    throw 'Expected FB_ServoControl usage inside manager'
}
Write-Output 'single-pump static check: OK'
```

- [ ] **Step 4: Run Beremiz build/syntax check**

Open `beremiz.xml` in Beremiz, select the configured Gm2xx target and build. Expected result: no PLCopen XML import errors, no unknown OOP symbols, and generated output refreshed under `build/`. If the target rejects the OOP metadata, use the syntax captured in Task 1 Step 3, update only the XML representation, and rerun Tasks 8.1–8.3.

- [ ] **Step 5: Report prototype evidence and remaining risks**

Report the final XML parse result, symbol scan result, Beremiz build result, and the explicit prototype limitations. Do not claim motion behavior or pump arbitration correctness without simulation/target evidence.

## Self-review checklist

- Spec coverage: unified interfaces, inheritance, existing FB reuse, single HydMotion arbitration, one-cycle call order, five process actions, facade registration, application isolation and verification are all assigned to Tasks 2–8.
- Placeholder scan: no unresolved marker or unspecified implementation step is used; unsupported full features are explicitly excluded from this rapid prototype.
- Type consistency: `IAxis`, `IHydraulicAxis`, `E_AxisState`, `E_AxisMode`, `E_PumpPolicy`, `ST_AxisStatus`, `ST_HydPumpStatus`, `FB_HydPumpManager` and `FB_HydTechnology` names are used consistently across tasks.
- Compatibility: `pousGm2.xml`, `pousHydMotion.xml`, existing `FB_ServoControl`, `FB_HydAxis`, `FB_CalcAxisMaxSpeed` and the legacy `FB_EleAxis` entry are preserved.
