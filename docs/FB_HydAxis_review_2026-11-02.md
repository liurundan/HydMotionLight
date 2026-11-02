# `FB_HydAxis` 工艺层接口功能块评审报告

- **评审日期：** 2026-11-02
- **评审对象：** [`iec/FB_HydAxis.st`](../iec/FB_HydAxis.st)
- **评审范围：** 工艺段速度/压力斜率、模式切换、启动停止、库状态读取、配置失败处理，以及与 HydroMotionLib 运行时接口的交互
- **总体结论：** 存在多个高优先级控制风险；建议修正关键项后再进行设备调试与投产评估
- **架构状态：** WATCH
- **审查建议：** REQUEST CHANGES

## 结论摘要

该功能块已具备位置、速度和压力三种模式的基本调用与切换框架，且位置段通过 105 步重触发适配了底层 FB 没有在线更新能力的接口约束。主要风险集中在时间基准、压力 ramp 上限、方向切换末速度、未支持的 jerk 参数，以及急停错误处理。状态读取和配置失败路径也需要补齐。

本次评审为只读评审，未修改功能块代码。

## 高优先级问题

### 1. 斜率计时固定为 1 ms，未跟随实际 PLC 扫描周期

`rCycleTime` 固定为 `0.001` 秒，段间过渡计时每次调用只增加 1 ms；`rCycle` 也以固定 1 ms 累加。【[`FB_HydAxis.st:163`](../iec/FB_HydAxis.st#L163)】【[`FB_HydAxis.st:645`](../iec/FB_HydAxis.st#L645)】【[`FB_HydAxis.st:838`](../iec/FB_HydAxis.st#L838)】

虽然 ST 将 `TIMESTAMP := rCycle` 传给 `HYD_SetAxisFeedback`，当前适配器没有读取该输入；运行库通过静态的 `dfCycleTime = 0.001` 生成轴时间戳。【[`FB_HydAxis.st:840`](../iec/FB_HydAxis.st#L840)】【[`motion_interface.c:11`](../src/motion_interface.c#L11)】【[`motion_interface.c:2714`](../src/motion_interface.c#L2714)】【[`motion_interface.c:852`](../src/motion_interface.c#L852)】

若 PLC 周期为 4 ms，按调用次数计时的斜率和超时会相对实际时间偏慢约 4 倍；若周期短于 1 ms，则偏快。位置重触发和运行库规划都会受到影响。

**建议：** 使用 PLC 任务的实际周期或统一的单调时间戳，传入并贯穿工艺层与运行库；如果现场只允许 1 ms 周期，应把它明确为部署约束并在启动时校验。

### 2. 压力 ramp 的零时间被静默改成 1 ms

压力模式调用 `CalcPresRampRate` 时先执行 `MAX(uiPressureAccTime, 1)`，因此方法内部的 `uiTimeMs = 0` 保护不可达。【[`FB_HydAxis.st:824`](../iec/FB_HydAxis.st#L824)】【[`FB_HydAxis.st:243`](../iec/FB_HydAxis.st#L243)】

例如 `uiPres` 从 0 变为 1000（代码换算为 100 bar），且 `uiPressureAccTime = 0` 时，目标变化被当成 1 ms 完成，计算结果为 100,000 bar/s。库内 ramp controller 直接按 `rampRate × deltaTime` 更新，没有额外的最大压力斜率限制。【[`ramp_controller.c:24`](../src/ramp_controller.c#L24)】这会让压力设定值近似一个周期内跳变。

**建议：** 明确零值语义：拒绝无效输入，或使用经过工艺验证的默认时间；为压力斜率设置机器可配置的上限，并在超过上限时报警或限制。

### 3. 方向切换时位置段末速度可能沿用上一方向的速度能力

`rEndVel` 在当前方向的最大速度计算之前生成，随后 `rMaxSpeed` 才按 `uiDir` 更新。【[`FB_HydAxis.st:569`](../iec/FB_HydAxis.st#L569)】【[`FB_HydAxis.st:576`](../iec/FB_HydAxis.st#L576)】

若新段方向与上一段相反，`rVel` 使用新方向的能力，`rEndVel` 仍可能使用上一方向的能力。`HYD_MoveContinuousAbsolute` 在 `EXECUTE` 上升沿锁存参数，因此不会在下一周期自动修正。

**建议：** 先选择本周期的方向最大速度，再用该值统一计算 `rVel` 与 `rEndVel`。

### 4. 非零 `uiJerk` 与当前库接口不兼容

`uiJerk > 0` 时功能块会计算非零 `rJerk`。【[`FB_HydAxis.st:506`](../iec/FB_HydAxis.st#L506)】但当前 IEC 适配器将 `JERK` 声明为预留参数，并拒绝超过容差的非零值；常规 jerk 设置会导致启动命令返回 `COMMAND_NOT_ALLOWED`。【[`motion_interface.h:218`](../include/motion_interface.h#L218)】【[`motion_interface.c:542`](../src/motion_interface.c#L542)】

**建议：** 在运行库支持 jerk 之前，明确要求该输入为 0 并在输入无效时给出可诊断错误；支持实现完成后再开放参数。

### 5. 工艺层急停使用极大减速度，并将停止错误当作离开急停状态的条件

`fbEStop` 与普通 `fbStop` 同为 `HYD_Stop`，急停时传入 `DECELERATION := 99999.0`。【[`FB_HydAxis.st:113`](../iec/FB_HydAxis.st#L113)】【[`FB_HydAxis.st:894`](../iec/FB_HydAxis.st#L894)】运行库会将正减速度值直接作为停止参数保存，没有在该入口处限制到经过设备验证的范围。【[`motion_control.c:3464`](../src/motion_control.c#L3464)】极大的减速度可能形成很陡的速度/流量变化。

状态机在 `DONE OR ERROR` 时都会退出急停步骤，并关闭运动 FB 的执行信号；此后没有确认轴已静止，也没有保留底层 `ERRORID`。【[`FB_HydAxis.st:781`](../iec/FB_HydAxis.st#L781)】因此 `ERROR` 不应被当作停止完成。

**建议：** 将 `bEStop` 定义为工艺层受控快停，并使用经液压系统验证的停止斜率；错误时保持明确的未确认停止/故障状态，保留底层错误码并验证静止状态。机器急停应由独立安全链实现并确认。

## 中优先级问题

### 6. 初始化配置失败没有完整的故障出口

轴创建成功后，速度增益、PID 类型和油缸参数写入的返回值未检查，配置失败时仍继续进入运行流程。【[`FB_HydAxis.st:680`](../iec/FB_HydAxis.st#L680)】肘杆配置仅处理类型 3 和 105；若其他奇数机构类型被映射为肘杆，配置方法不会调用底层 FB，状态机可能一直停留在 Step 1，且不报警。【[`FB_HydAxis.st:267`](../iec/FB_HydAxis.st#L267)】【[`FB_HydAxis.st:307`](../iec/FB_HydAxis.st#L307)】【[`FB_HydAxis.st:712`](../iec/FB_HydAxis.st#L712)】

**建议：** 创建轴前校验机构类型；检查每个 setter、`FB_CalcAxisMaxSpeed.bValid` 与肘杆配置的 `ERROR/DONE`。任何初始化失败都应进入可诊断的启动故障状态。

### 7. 库状态未对外提供，停止/报警时带内状态可能残留

`HYD_ReadStatus` 的结果写入私有变量 `iAxisState` 后没有继续使用；`stHydAxisRef.iStep` 输出的是工艺层状态步骤，不是库运行状态。【[`FB_HydAxis.st:918`](../iec/FB_HydAxis.st#L918)】在停止、急停、复位和报警状态，代码只更新 `bBusy`、`bDone`，未清除 `bInVelocity` 与 `bInPressure`，这两个输出可能保留停止前的 `TRUE`。【[`FB_HydAxis.st:943`](../iec/FB_HydAxis.st#L943)】

此外，速度模式和压力模式的 `bDone` 直接映射到 `InVelocity` 与 `InPressure`，即目标带内状态，不一定代表工艺段终止。库的运行时契约明确区分带内信号和终端完成信号。【[`motion-runtime-contract.md:225`](../docs/architecture/motion-runtime-contract.md#L225)】

**建议：** 定义并统一 `bDone` 的语义；停止、报警、切换模式时清除不再有效的带内输出；如上层需要库状态，将 `iAxisState` 映射到公开输出或共享结构。

### 8. 未处理 FB 的 `COMMANDABORTED`

运行状态仅检查当前模式 FB 的 `ERROR`，没有处理 `COMMANDABORTED`。【[`FB_HydAxis.st:737`](../iec/FB_HydAxis.st#L737)】运行库在同轴命令被其他 FB 抢占时会输出 `COMMANDABORTED`；如果现场允许其他调用方使用同一个 `AXISID`，本功能块可能继续保持运行状态和 `bBusy`。【[`motion_interface.c:2437`](../src/motion_interface.c#L2437)】

**建议：** 若轴由本功能块独占，应在系统接口中明确并落实这一约束；若允许共享，则在被抢占时清理本地执行状态并输出可区分的中止状态。

## 工业接口对照

- PLCopen Part 6 的 `MC_LoadControl` 将 `LoadRamp`、`InLoad`、`Busy`、`Active`、`CommandAborted` 和 `Error` 分别表达；其中 `InLoad` 表示达到压力/载荷设定值。该模式适合参考，用来区分“目标到达”与“命令终止”。([PLCopen Motion Control Part 6 — Fluid Power Extensions](https://plcopen.org/system/files/downloads/plcopen_motion_control_part_6_version_2.0.pdf))
- 倍福液压库提供独立的 `MC_EmergencyStop_BkPlcMc`，用 `RampTime` 表达停止时间，并分别报告 `Done`、`Error`、`ErrorID` 和 `CommandAborted`。([Beckhoff MC_EmergencyStop_BkPlcMc](https://infosys.beckhoff.com/content/1033/tcplclibhydraulics30/1599694603.html))
- 西门子 S7-1500/S7-1500T 文档将 SS1、STO 作为驱动安全功能描述，可作为工艺层停机与安全停车职责分离的参考。([Siemens S7-1500/S7-1500T Motion Control Overview V9.0](https://support.industry.siemens.com/cs/attachments/109974347/s71500_s71500t_motion_control_overview_function_manual_en-US_en-US.pdf))

## 验证记录

以下 11 项相关 C 库测试通过：模式切换、压力控制、速度停止/反向、位置连续运动、故障恢复、反向交接、肘杆集成、接口抢占、状态输出和停止完成等。

```bash
ctest --test-dir out/build/unixgcc -R '^(test_mode_switch_bumpless|test_movecontinuousabsolute_integration|test_movevelocity_stop_reverse_restart|test_fault_recovery|test_vp_bumpless_reverse|test_toggle_motion_integration|test_pressure_controller|test_motion_interface_arbitration|test_motion_interface_done_signals|test_stop_immediate_done|test_toggle_motion_interface)$' --output-on-failure
```

**验证限制：** 当前环境没有可用的 IEC ST 编译器，以上测试覆盖运行库 C 逻辑，没有直接编译或执行 `FB_HydAxis.st`。报告结论基于 ST 源码、运行库接口实现、项目文档和 PLCopen/厂商接口文档的交叉检查。
