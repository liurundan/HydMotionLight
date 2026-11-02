FB_HydAxis 工艺层功能块代码评审报告
〇、总体评价
该功能块整体架构思路清晰：状态机 + PRIVATE METHOD 拆分算法 + 命令侧速度链（V0 锁存），这部分设计明显借鉴了成熟运动控制层的做法（如 SEG 段管理、CONTINUOUSUPDATE 语义区分、105 重触发），注释详尽，版本管理意识好。但在 FB 握手时序、多 FB 并发 EXECUTE 冲突、参数初始化依赖、急停语义 等方面存在会导致现场故障甚至设备损坏的严重缺陷。以下按严重程度分级评审。
一、严重缺陷（P0 —— 会导致功能失效或安全风险）
1. WriteParameter 的"同周期 Execute/Done 握手"是伪握手（最严重）
所有 SetXxx 方法都是同一模式：
fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := FALSE);
fbHYD_WriteParameter( AXISID := siAxisId, EXECUTE := TRUE, ...);
SetCylinderConfig := fbHYD_WriteParameter.Done;   // ← 同一扫描周期读取
在 IEC 61131-3 的 FB 语义下，EXECUTE 上升沿后 Done 至少要到下一个任务周期（通常是 Busy→Done 数个周期）才置位。同一个扫描周期内先置 FALSE 再置 TRUE 再读 Done，Done 几乎恒为 FALSE。后果：
SetCylinderConfig 第一次写（伸出面积）后判断 IF NOT ... THEN RETURN —— 缩回面积（参数 32）永远不会被写入，液压缸两腔面积配置不完整；
SetPumpConfig 排量（28）写完后 Done=FALSE 提前 RETURN —— 泵最大转速（30）、效率（29）永远写不进去，而这两个参数直接决定 rMaxSpeed 能力边界；
SetPidKp/Ki/Kd、SetVelocityToFlowGain、SetPidType 返回值恒为 FALSE，上层若据此判断会误报失败。
修正方案：这是典型的"边沿驱动的服务 FB"用法错误。正确做法是：
方案 A（推荐）：方法只做请求置位（如 bParamWriteReq := TRUE; rParamValue := ...），实际的 WriteParameter 编排放在 CFB 主体中用一个小状态机（REQ→BUSY→DONE/ERROR）完成，方法返回 VOID；
方案 B：方法内部自带完整握手，但必须跨周期运行——方法本身退化为一个内嵌 FB 实例（把 fbHYD_WriteParameter 提升为 CFB 成员变量），方法调用只是触发。
2. 停止/急停期间运动 FB 仍在执行 —— 多 FB 并发驱动同一轴
进入 Step 200（正常停止）或 300（急停）时：
200: // bCmdExecute 仍为 TRUE
    IF fbStop.DONE THEN bCmdExecute := FALSE; ...
此时 fbMoveContAbs / fbMoveVel / fbPresHdl 的 EXECUTE := (uiActiveMode = X) AND bCmdExecute 仍然为 TRUE，同时 fbStop/fbEStop 也 EXECUTE=TRUE。PLCopen 规范明确：Stop/E-Stop 必须中止（Abort）当前运动指令，同一轴上两个运动控制 FB 同时持有 Execute 是未定义行为——多数实现会报 AxisBusy/CommandConflict 错误，或者后发的 Stop 被库拒绝，导致：
停止指令可能根本没下发，轴继续按原轨迹走完；
急停场景下这是安全功能失效，属于不可接受的缺陷。
修正：进入 200/300 的同一拍立即 bCmdExecute := FALSE;，确认运动 FB 的 Execute 下降后（或按库语义当拍）再使能 fbStop/fbEStop。急停建议直接调用库的 HYD_Halt/QuickStop 类接口而非普通 Stop 换一个减速度。
3. 急停 DONE 被当作报警处理
300: IF fbEStop.DONE OR fbEStop.ERROR THEN
        bAlarm := TRUE; dwAlarmID := 16#0000FFFF; ...
急停成功停稳也置 bAlarm，导致：
急停后必须人工复位才能继续（某些工艺这合理，但应由上层决定，不应在工艺层强制）；
dwAlarmID := 16#0000FFFF 不是有意义的错误码，诊断时无法区分"急停正常完成"和"急停过程故障"；
若急停过程中出现 ERROR，混合在一个分支里，掩盖了真实的停止失败（如减速距离不足撞硬限位前的最后机会告警）。
修正：DONE → 进入 Step 10 + 置一个 bEStopActive 状态位（非 Alarm）；ERROR → 单独报警码。
4. 最大速度回退链路失效 + 加速度钳位参数未初始化
IF stHydAxisRef.rMaxSpeedExtend > 0.0 THEN ...
ELSIF stHydAxisRef.rMaxSpeed > 0.0 THEN ...
// 两者都 <=0 时：rMaxSpeed = 0，且 rAccMin/rDecMin/rAccMax/rDecMax 保持初始值 0
后备 stHydAxisRef.rMaxSpeed 的回退分支没有给 rAccMax/rDecMin 等赋值，后续 LIMIT(rAccMin, x, rAccMax) = LIMIT(0, x, 0) = 0，运动 FB 收到 ACCELERATION=0 的行为完全依赖库的容错（多数库会报参数错或拒绝启动）；
而 Step 0 中 fbCalcMaxSpeed 又被硬编码调用：fbCalcMaxSpeed(面积, 面积, 1800, 50, ...) —— 泵排量/转速写死 50cc/1800rpm，与 SetPumpConfig(50, 1700)（还是 1700rpm！两处转速不一致）以及方法中可配置的泵参数完全脱节。fbCalcMaxSpeed.bValid 输出被完全忽略。
修正：泵参数只保留一个数据源（建议 stHydAxisRef 中的 ST_PumpConfig 结构体），Step 0 统一写入底层 + 计算能力边界；bValid=FALSE 时置 dwAlarmID 并走 Step 900，而不是静默让 rMaxSpeed=0。
5. Step 105 一拍过渡步期间会丢失 Start/Stop/Mode 边沿
bNewSeg := rTrigStart.Q AND (iStep = 10 OR iStep = 100) —— 若 R_TRIG 的 Q 恰好落在 iStep=105 的那一个周期（V2.4 的 105 重触发 + 同模式新段切换共用此步），该 Start 边沿被永久丢弃；同理 Step 105 中不检测 bStop/bEStop。虽然 105 只存在一个周期，概率低，但：
这类"偶发丢段"在现场极难复现和排查；
根因是 105 被赋予了双重职责（段切换 + rDec 重锁存），两个语义共用一个状态，违背单一职责。
修正：边沿检测放在 CASE 之前统一捕获进"命令队列/挂起标志"（如 bStartPend），CASE 内各状态消费挂起标志，而不是直接消费单拍 Q 信号。或者把 V2.4 的 rDec 重锁存改用独立机制（如直接调用库的重规划接口）。
二、重要缺陷（P1 —— 影响可靠性、可移植性、可维护性）
6. 任务周期时间硬编码
rCycleTime := 0.001、rCycle := rCycle + 0.001、所有 UINT × 0.001 的时间换算都假设 1ms 任务。若部署到 2ms/4ms 任务：
§3.4 的过渡计时器、105 兜底时间全部偏差 2~4 倍；
uiAcc=100（声称 100ms）实际变 200ms。
修正：周期时间作为 VAR_INPUT 由调用方传入（或用 _TaskInfo 读取），所有换算统一使用该变量。
7. 运行中输入参数未锁存，位置模式下尤甚
rPos/rVel/rPres/uiDir/iDirSign 每个周期都从输入刷新，但位置段 FB 无 CONTINUOUSUPDATE，运行中操作员改变 uiDir 或 udiPos 时：
底层库仍执行旧指令（EXECUTE 未重新锁存），而工艺层的 iDirSign、rVelSigned、CalcStopDeceleration、§3.4 减速区判断却用新值计算——内部状态与轴实际运动脱钩，减速区距离判断会提前/延后触发 105 重触发，可能造成非预期的反向重规划。
修正：段触发拍（bNewSeg）将所有工艺参数锁存到 stActiveSeg 结构体，运行期一律使用锁存值；输入变更只在 Step 10 或经由 105 显式重触发生效。这也是 PLCopen 分段运动控制（Segment sequencing）的标准做法（参考 Siemens Motion Control V90 的 Job 管理、Beckhoff TF-52xx 的 MC_CommandStack 模式）。
8. Step 1（肘杆配置）无超时/失败保护
1: bConfig := SetToggleMechanism( siAxisId, stHydAxisRef.iMechanismType );
   IF bConfig THEN iStep := 10; END_IF
SetToggleMechanism 每周期 EXECUTE:=TRUE 反复触发（应为边沿），且若配置失败/不返回 DONE，状态机永久卡死在 Step 1，无报警、无诊断信息。iMechanismType 既不是 3 也不是 105 时 bConfig 永远 FALSE（方法中两个 IF 都不执行，Done 不会被置位）——这是必现死锁。
9. 输出语义交叉污染
bInVelocity := fbMoveVel.InVelocity;   // 位置模式下也有值
bDone 的三种来源混在 CASE 里
bInVelocity 应只在 uiActiveMode=2 有效，否则位置段运行中速度 FB 残留的 InVelocity 会误导上层；
fbPresHdl.INPRESSURE 在 105 步（bCmdExecute=FALSE 一拍）时库可能短暂丢压状态，bDone 闪烁。
修正：输出严格按 uiActiveMode 门控；bDone 上升沿用 R_TRIG 锁存（rTrigDone 已声明却从未使用——死代码）。
10. 压力模式斜率计算的边界问题
§6 只在 rTrigStart.Q 或压差>0.1 时重算，但 rPresLastCycle 在模式切换/新段后未复位：上一段压力 150bar，新段目标 30bar，斜率按 120bar/时间算出来可能极大；
CalcPresRampRate 里 IF uiTimeMs=0 THEN RETURN（返回 800）与 IF rDeltaPres<0.1 THEN RETURN（同返回默认值）混淆了"异常"和"已到位"两种语义——压差为 0 时应返回 0 或保持当前斜率让 FB 维持，而不是给最大斜率 800 bar/s 冲击系统；
压力模式下 rFlowPercent 强制下限 5%，但速度模式下 uiSpd=0 时 rVel=0 仍 EXECUTE——两种模式的最小流量语义不一致，建议统一为"低于最小流量报参数错误"。
11. fbCreate 的重复执行风险
Step 0 每周期调用 fbCreate(...)，若 stHydAxisRef.bSimulation 或机构类型在创建过程中变化，部分库会重复创建轴实例导致轴资源泄漏。应使用 fbCreate.EXECUTE 边沿语义（若库支持）或在 DONE/ERROR 后本周期内立即停止调用。此外 Step 0 无超时保护——创建 FB 卡 Busy 时状态机死等。
三、一般问题（P2 —— 工程质量）
#	问题	说明	
12	魔法数字泛滥	0.73/1.18（变速比）、16.667、肘杆机构的 LR=190, LF=260...、iParamterIndex=5/25/31/32/28/29/30/11/14/15 全部硬编码。参数号应定义为 GVL 常量或 ENUM（eHydParam.Kp := 11），变速比/肘杆几何参数应来自配置结构体，否则换机型必须改代码	
13	状态机用 INT 魔法数	0/1/10/100/105/200/300/400/900 应定义为 ENUM E_HydAxisStep（IDLE/RUN/RUN_TRANS/STOPPING/ESTOP/RESET/ERROR），可读性和调试监视效率大幅提升	
14	错误码无体系	dwAlarmID 混用底层 ERRORID、自定义 1、16#FFFF，建议自定义码段（如 16#10xx = 工艺层错误，16#20xx = 底层透传）	
15	命名拼写错误	rPerssure、rVeloctiy、iParamterIndex——虽不影响运行，但反映评审流程缺失	
16	rStopDec=5000 默认值	停止减速兜底值写死，大质量惯量轴可能造成压力冲击；rDeltaVDeadZone=1.0 mm/s 也是拍脑袋值，应做成可配置参数并注明单位	
17	监视数据回写职责混杂	§0 每周期向 stHydAxisRef 回写输入镜像、§8 读取仿真反馈、§9 汇总输出——诊断数据建议独立到 GetDiagnostics METHOD 或独立监视 CFB，避免 CFB 主体膨胀（目前主体已 ~400 行）	
18	无软限位/位置窗口保护	工艺层完全依赖底层库的限位，作为注塑机合模机构（肘杆类型暗示的工况），工艺层至少应做合模保护区间/低压护模的独立判断，这是安全相关的行业惯例	
19	uiJerk=0 表示不限 Jerk 的约定	建议用 UINT#FFFF 之类显式"无效值"而非 0（0 是合法的物理值），并与库文档对齐	
20	缺少版本/校验接口	建议增加 METHOD GetVersion : STRING 与 stHydAxisRef.uiFbVersion 回写，现场多版本并存时诊断必需	
四、值得肯定的设计（保留并强化）
命令侧速度链（V0 锁存，CalcStartVelocity/ClearSegLatch）：不依赖传感器反馈推算起始速度，规避了反馈噪声和延迟，这是正确的工程取舍；ClearSegLatch 与状态机回闲的联动（§5）设计严谨。
rDec 双角色分离 + 段末值暂存（V2.2/V2.4）：过渡斜率与段末斜率解耦，105 重触发复用已验证语义，避免了为位置段引入新机制——注释把这个设计决策写得非常清楚，值得推广为团队的"设计决策记录"模板。
除零保护意识：CalcAxisMaxSpeed、CalcPresRampRate、MAX(uiTimeMs,1) 等处均有防护。
速度段/位置段 CONTINUOUSUPDATE 差异的显式处理：多数自研工艺层都会在这里踩坑，此处区分明确。
五、重构优先级建议
P0（下次代码合入前必须修复）
  ① WriteParameter 握手改为跨周期状态机（影响：油缸面积/泵参数写不进去）
  ② 进入 Stop/EStop 即拉低运动 FB 的 Execute（影响：停止可能不执行）
  ③ EStop DONE 与 ERROR 分离处理
  ④ 泵参数单一数据源，fbCalcMaxSpeed 用真实泵参数 + 校验 bValid
P1（版本 2.5）
  ⑤ 工艺参数段内锁存（stActiveSeg）
  ⑥ 任务周期时间参数化
  ⑦ Step 1 配置失败超时报警；边沿统一捕获为挂起命令
  ⑧ bDone/bInVelocity 按模式门控
P2（持续重构）
  ⑨ ENUM 化状态机与参数号、配置结构体外移魔法数字、
     拆分诊断监视、补充限位保护
总体而言，该 FB 的算法层（斜坡、段链、过渡处理）已经达到较高水准，但接口握手层和状态机编排层存在会直接导致功能失效的 P0 缺陷，尤其建议在样机台架上专门验证：① 油缸面积参数是否真正写入底层（读回参数 31/32 确认）；② 运行中触发 Stop，用示波器/Trace 确认速度曲线确实按 rStopDec 减速到零——这两项测试大概率会立即暴露问题。