# 曲肘轴开合模“泵流量百分比”速度语义 — 设计文档

- 日期：2026-11-18
- 分支：feature/rbf-pid-industrialization
- 状态：已确认（2026-11-18）；实施计划 `docs/superpowers/plans/2026-11-18-clamp-flow-percent.md`
- 修订（写计划时按代码与探针复核）：§4.2 接口同步文件、§4.3.3 零上限保护、§4.3.6 尺寸、§4.4 零速保护、§5 合模数据、§6 T3~T5 判据、§7 合模初值
- 修订（2026-11-19 终审）：§4.2 ABI 3、§4.3.2 近似相等按相等处理、§4.3.3 过渡完成后的恢复速率、§4.4 终点速度钳位、§7 低百分比零终速落地验收、§8 速度上限 700 的刹车包络风险
- 关联：`docs/2026-11-18-clamp-speed-fix.md`（问题分析）、`docs/2026-11-16-clamp-zero-end-decel-analysis.md`（105 几何、面积、段标尺、参数 35/36）
- 实验数据：`docs/data_deg/vofa_waveform_20261117_161526_748-开合模.csv`

## 1. 背景与根因

现状：曲肘轴（105 双曲肘，开模侧面积 3350mm²、合模侧 4800mm²，泵 50cc/rev、1700rpm、容积效率 0.95）
开模 2.02s、合模 1.50s，比原直压式算法各慢约 0.5s，泵转速大部分时间远低于 1700rpm。

- 面积与几何正确：实测速度 ÷ 规划速度 k 的中位数为合模 1.001、开模 0.999（x≥12mm 各区间 0.95~1.04）。
- 根因是段速度标尺：FB_HydAxis 在段启动沿按区间内**最大速比**锁存 100% 标尺，段内动模板速度恒定，
  所以只有最坏点泵满转，其余位置吃不满。开模 20→300 段 100% 只用到泵的 34%~42%。
- 合模 x=50 换段时标尺从 282 跳到 106mm/s，实测泵转速 75ms 内从 1700 掉到 930rpm，形成中段顿挫。
- 旧直压式波形（11-13）各段泵转速恒为 12.6rpm/%、与位置无关，即“百分比 = 泵流量”，
  曲肘机构自然形成“中间快、两头慢”的动模板速度。

## 2. 决策（2026-11-18 用户确认）

| 项 | 决策 |
|---|---|
| 方向 | 方案 B：MoveContinuousAbsolute 增加泵流量百分比语义，巡航按泵流量，停车仍按动模板位置规划 |
| 接口 | 新增 FB 输入（不走参数） |
| 百分比来源 | 直接使用工艺层速度命令 uiSpd（FB 内 `uiSpd × 0.1` = %），不新增工艺参数 |
| 新旧切换 | FB_HydAxis 保留新旧模式开关，便于上机 A/B 对比与回退 |
| 开模停位 | 终点误差 ±1.5mm 可接受 |

## 3. 与业界做法的对照（检索于 2026-11-18）

| 证据 | 来源 | 要点 | 支持的部分 |
|---|---|---|---|
| KEBA KePlast 控制器手册（科仕特机型） | renrendoc.com/paper/146607652 | 合模/开模每段设定“压力、流量%、结束位置”；OEM 参数含“最大斜率 / 斜率调整” | 百分比 = 流量；段间斜坡 |
| 海天 HTF450B 操作规程（弘讯电脑） | renrendoc.com/paper/209280322 | “低压关模……双曲臂移动有增速作用，移模速度逐步加快，所以用低压慢流量” | 按流量设定、利用曲肘自然速度特性 |
| 盛雄注塑机 | czsxsj.com/?Company/357.html | 开模设定“压力[bar]和流量[%]”；“每个阶段的斜率都能设置……斜坡持续时间能被计算并显示” | 流量% + 每段斜坡 |
| 曲肘速度特性 | ginlai.com/mobile/contents/161/2273.html | 合模时动模板“速度由零很快升到最大值，以后又逐渐减慢” | 恒流量下中间快、两头慢 |
| 海天合著论文（宁波大学学报） | fabiao.net/content-627-1499-1.html | 优化速比曲线，合模周期减少 6.6% | 厂家把速比曲线作为缩短周期的手段 |
| 汇川伺服驱动资料 | bbs.gongkong.com/d/202108/867215 | 流量控制靠电机转速；0~最大流量响应最短 30ms | 泵指令即流量，与实测约 20ms 纯滞后吻合 |
| 麦格米特专利 CN122378985A | zhuanli.book118.com | 按历史开模终点修正“动模板位置与速度关系的控制参数” | 停位精度在流量分段之外单独处理 |

结论：
- 巡航“百分比 = 泵流量 + 段间斜坡”与主流液压曲肘机一致（直接证据）；
- 终点按动模板位置刹车加纯滞后补偿，是本库对停位精度的做法，业界常见做法是“慢速收尾段 + 经验/学习修正”，
  属同一问题的不同解法（间接证据），操作工仍可按习惯加慢速段；
- 未取得的资料：KEBA/B&R 是否有“曲肘线性化”功能、全电动曲肘机的 mm/s 换算方式、伺服驱动流量斜坡参数名。

## 4. 设计

### 4.1 语义

`HYD_MoveContinuousAbsolute` 新增输入 `PUMPFLOWPERCENT`（REAL，%）：

- `0`：关闭，行为与现有完全一致；
- `(0, 100]`：泵流量百分比模式，100% 对应泵流量上限 `Q_base`；
- `Q_base = HYD_GetPumpFlowLimit(fb)`（泵转速上限 ÷ 流量-转速增益，本机 80.75 L/min）；不大于 0 时取 `_params.maxFlow`；
- `VELOCITY` 只作为动模板速度安全上限，`ENDVELOCITY` 仍为动模板 mm/s；
- 不与 PressureHandle 的 `FLOWLIMITPERCENT`（基准 20 L/min）同名，避免混淆。

### 4.2 IEC 接口

- `PUMPFLOWPERCENT : REAL` 追加在 `PRESSURELIMIT` 之后（输入区末尾），同步修改：
  `include/motion_interface.h`、`pousHydMotion.xml`；
  `tests/plcdemo/POUS.h/.c` 未收录此 FB，不改；`scripts/check_interface_layout_consistency.py` 也未覆盖此 FB，
  在 `tests/test_interface_layout_consistency.py` 中增加 XML 与 C 头文件字段顺序的专门核对；
- 执行上升沿校验：非有限值、小于 0 或大于 100 时 `ERROR = TRUE`，`ERRORID = COMMAND_NOT_ALLOWED`；
- 结构体布局改变，**PLC 工程需用新的 pousHydMotion.xml 重新生成并编译**；`HYD_ABI_VERSION` 2 → 3。

### 4.3 库实现

只改 MoveContinuousAbsolute 的执行路径，规划器、运动学、压力环、Stop 不改。

1. **上下文**：`HYD_ContinuousAbsoluteContext` 增加两个 REAL：
   - `flowPercent`：输入值，0 = 关闭；
   - `flowRampRate`：流量斜坡速率（L/min/s），段启动时锚定。
   `buildContinuousAbsoluteContext` 增加 `flowPercent` 形参。
2. **斜坡锚定**（与 crossingVelocity 同处计算，即 `motion_control.c` 中三处 `HYD_ResolveContinuousAbsoluteCrossingVelocity` 调用之后，读取续接恢复后的规划状态）：
   - `q0 = |_plannerState.lastTargetFlow|`（从静止启动为 0，链式接管时为上一拍指令流量）；
   - `q_target = flowPercent/100 × Q_base`；
   - `flowRampRate = |q_target − q0| / t`，上升取 `t = maxAcceleration`，下降取 `t = maxDeceleration`（ms→s），
     与 2026-11-10 确定的“任意过渡固定耗时”语义一致；`q_target == q0` 时取 `Q_base / t`；
   - 近似相等按相等处理（2026-11-19 终审）：`|q_target − q0| ≤ HYD_FLOW_PERCENT_RAMP_EQUAL_RATIO × Q_base`
     （`hyd_config.h` §14B，取 0.02）时取 `Q_base / t_acc`。同百分比链式续接的 Δq 只有 0.003~0.66 L/min，
     按 Δq/t 锚定的速率只有 0.01~2 L/min/s，上限被规划器短暂下探带下来后爬不回去（棘轮）。
3. **每周期流量上限**（`HYD_ExecuteActiveSegmentControl` 机构映射前，仅当 owner 为 MoveContinuousAbsolute 且 `flowPercent > 0`）：
   - `q_prev = |_plannerState.lastTargetFlow|`，**须在 `HYD_MotionPlanner_Execute` 之前读取**：规划器内部会用不含速比的增益改写该字段
     （干跑验证：执行后再读，起步斜坡起不来）；
   - `q_cap = clamp(q_target, q_prev − rate·dt, q_prev + rate·dt)`；
   - 过渡完成（`|q_prev − q_target| ≤ rate·dt`，上限已到目标）后，本段余下的 `rate` 改为 `Q_base / t_acc`（复用 `flowRampRate` 字段）。
     只换速率、不跳变：上限仍以 `q_prev` 为中心限幅、不超过 `q_target`。大 Δ 过渡中途规划器下探时仍按锚定速率恢复，直到上限到达目标；
   - `velocityFlowLimit = min(现有上限, q_cap)`；
   - `q_cap ≤ 0`（静止起步、本周期 dt = 0）时取 `HYD_REAL_EPSILON`：映射层把 `maxFlow = 0` 当作“不限流”，不能传 0。
4. **回算**：映射结果 `flowLimitActive` 为真时，把规划器状态 `lastTargetVelocity`、`plannerOutput->targetVelocity`
   以及由它派生的 `STATE.plannedVelocity`、执行参考速度改写为 `方向 × mapped.maxTemplateVelocity`。
   这样规划速度始终等于可实现速度，刹车触发距离按真实速度计算，速度偏差诊断不误报。
5. **限幅标志**：百分比模式下，百分比上限生效属于正常巡航，不置 `HYD_LIMIT_FLAG_FLOW`；
   只有当 `min(maxFlow, 泵流量上限)` 小于本周期 `q_cap` 时才置 FLOW。泵转速限幅标志仍由 pump_converter 判定。
6. **FB 尺寸**：上下文在 FB 内有两份（当前段与待接管段），FB 从 3312 增至 3328 字节（已实测）；
   `test_mode_switch_bumpless` 预算 3312 → 3328；`test_toggle_mechanism_pool` 预算为 3280 + 48 = 3328，不用改。

### 4.4 FB_HydAxis

- 新增 `VAR_INPUT bFlowPctMode : BOOL := TRUE;`，在段启动沿锁存为本段模式；
- 曲肘轴（`iMechBaseClass = 1`）、`uiMode = 1`、本段为百分比模式时：
  - `PUMPFLOWPERCENT := LIMIT(0.0, UINT_TO_REAL(uiSpd) × 0.1, 100.0)`
    （单独计算，不复用 `rFlowPercent`：后者给 PressureHandle 用，下限钳在 5%）；
  - 段启动沿调用 `fbMaxVel(XFROM := rPos, XTO := rPos, DIRECTION := iDir)`，锁存目标点的泵满流量动模板速度为 `rMaxSpeedSeg`，
    于是 `rEndVel = uiEndSpd% × rMaxSpeedSeg` 即“目标点处 uiEndSpd% 流量对应的动模板速度”；查询失败按现有逻辑报警进 900；
  - 终点速度钳位（2026-11-19 终审）：`rEndVel` 用 `MIN(uiEndSpd, uiSpd)` 计算，终点流量不超过本段流量。
    百分比上限在 sustain 阶段同样生效，`uiEndSpd > uiSpd`（如 30%/60%）时 INENDVELOCITY 永不置位、bDone 不来；旧模式不变；
  - `VELOCITY := rToggleVelCap`（FB 局部常量 700 mm/s，大于泵满流量在最小速比处的开模速度 687 mm/s；
    不用 `stHydAxisRef.rMaxSpeed`，因为该字段在本机仍是直压时代的 100 mm/s，会把速度压死）；
  - `uiSpd = 0` 时 `VELOCITY := 0`：保持库端拒绝零速命令的现有行为，避免 PUMPFLOWPERCENT = 0 关闭限流后按 700mm/s 运行。
- 其余情况（直压轴、`uiMode = 2` MoveVelocity、`bFlowPctMode = FALSE`）：保持现有段标尺逻辑，`PUMPFLOWPERCENT := 0`。

### 4.5 不变的部分

运动规划器（`motion_planner.c`）、零终速停车与参数 35/36、105 运动学与段标尺查询、Stop/EStop、
MoveProfile/配方路径、MoveVelocity、PressureHandle（护模、高压锁模）。

## 5. 可行性证据（仿真，不是上机结果）

一次性探针（未纳入仓库）链接 `libHydroMotionLib.a`，调用真实的 `HYD_MotionPlanner_Execute` 和 105 运动学；
被控对象为 20ms 纯滞后、8ms 泵一阶惯性、1700rpm 限幅。模型复现现状：合模 1475ms（实测 1497）、开模 2001ms（实测 2020）。

| 场景 | 合模 299.4→10 | 开模 1→300 |
|---|---|---|
| 现状实测 | 1.50s | 2.02s |
| 只限流不回算 | 1.01s | 1.36s（规划器按 VELOCITY 上限算刹车距离，提前刹车） |
| 限流 + 回算，第 7 节初值 | 1.27s（含 20% 起步段）/ 1.11s（不含） | 1.12 / 1.07 / 1.02s（减速 700 / 600 / 500ms） |

- 斜坡按本 spec 公式复核（固定耗时 Δq/t、每周期以上一拍指令流量为起点）：首版合模初值“95%→30、45% 段减速 250ms”
  到 10mm 时仍有 74mm/s（目标 47.7），刹车不及；改为“95%→40、45% 段减速 150ms”后，纯滞后 20/40ms、
  k = 0.95~1.05 四种对象下到 10mm 均为 40.6~44.5mm/s，合模 1.22~1.34s（含起步段）；
- 合模泵转速变化率不超过约 5700 rpm/s（即 45% 段 150ms 下降斜坡本身），x=50 处的顿挫消失；到 x=10 进入护模时 44.5mm/s，与现状实测 43.5mm/s 相当；
- 开模终点误差：纯滞后 20ms 时 +1.0~1.4mm；纯滞后 40ms 时 +3.9~5.3mm（超出 ±1.5mm，需上机实测纯滞后后调参数 35）；
- 实际速度比规划低 5%（k=0.95）且参数 35 = 15ms 时会停短爬行（开模 1.26s），故参数 35 先保持 10ms，面积继续“宁小勿大”；
- 零终速段长度须满足 `≥ 0.5·v·t_dec + v·τ`（v 为进入该段时的速度，τ 为参数 35）。开模在 250mm 切 30% 慢速段只剩 50mm、需约 100mm，
  刹车包络塌缩，减速度冲到 22~43 m/s²。开模因此推荐单段到终点；
- 以上耗时未计起步建压（实测约 0.1~0.15s）。

## 6. 测试（TDD，先写失败测试）

新增 `tests/test_toggle_flow_percent_integration.c`（105 几何、泵 50/1700/0.95、面积 3350/4800、1ms 周期）：

| # | 测试 | 判据 |
|---|---|---|
| T1 | 开模巡航 | 加速完成后、刹车前（x∈[20,100]mm）：泵转速 = 95% × 1700（±1%）；规划速度 = 当前位置 95% 流量对应速度（±0.5%）；无 FLOW、无 PUMP_SPEED 标志 |
| T2 | 流量斜坡 | 加速段每周期泵转速增量 ≤ 1615/t_acc × 1.05；T5 链式降百分比时每周期降幅 ≤ Δ/t_dec × 1.05 |
| T3 | 回算 | 理想对象下，T1 窗口内规划速度 = 实际可实现速度（即上条，±0.5%）；**变异检查**：注释掉回算后 T4 必须失败 |
| T4 | 零终速停车 | 20ms 纯滞后对象、参数 35 = 20：开模 1→300 总时长 < 1300ms（不回算约 1700ms）；终点误差 ≤ 1.5mm；InEndVelocity 置位；峰值到零的刹车时长 600~800ms（t_dec 700）；无 BRAKING_DISTANCE、无 VELOCITY_DEVIATION |
| T5 | 合模两段链式 | 299.4→40（95%，终点 = 40mm 处 95% 速度）→ 10（45%，减速 150ms）：第 2 段每周期泵转速降幅 ≤ T2 上限；到 10mm 时速度 = 45% × v_ceil(10)（±10%）；全程无 BRAKING_DISTANCE、无 VELOCITY_DEVIATION |
| T6 | 关闭与校验 | `PUMPFLOWPERCENT = 0` 时现有全部测试不变；−1、101、NaN 被拒绝 |

同步更新：`test_interface_layout_consistency`（XML 与 C 头字段顺序）、`test_mode_switch_bumpless` 尺寸预算。
FB_HydAxis 在仓库内无 ST 编译环境，用 `tests/test_hydaxis_p0_contract.py` 做文本契约检查，在 PLC 工程中编译后上机验证。
T4 的参数 35 取 20（= 对象纯滞后），用来隔离“回算”这一个变量；参数 35 = 10 对 20ms 纯滞后的停位误差已在 §5 探针中给出（+0.4~1.4mm）。

## 7. 上机验收与工艺初值

工艺初值（参数 35 = 10ms，参数 36 = 3mm/s）：

| 动作 | 段 | uiSpd | 加速 / 减速 ms | uiEndSpd |
|---|---|---|---|---|
| 合模 | 299.4 → 295（可选起步段） | 20% | 300 / 200 | 20% |
| 合模 | → 40 | 95% | 300 / 200 | 95%（保持流量，不刹车） |
| 合模 | 40 → 10 | 45% | 200 / 150 | 45% |
| 合模 | 10 → 0 护模、高压锁模 | 不变 | — | — |
| 开模 | 1 → 300（单段） | 95% | 200 / 700，逐步试 600、500 | 0 |

- 合模第 2 段终点 uiEndSpd 与本段 uiSpd 相同：FB 用目标点标尺，`rEndVel` 正好是 40mm 处 95% 流量的速度，换段时没有动模板刹车，
  降速全部由第 3 段的流量斜坡（150ms）完成；探针中换段处泵转速每周期变化 ≤ 5.7rpm，无台阶；
- 新语义 100% = 1700rpm；旧直压时代 100% ≈ 1263rpm，沿用旧百分比时约 ×0.74 折算；
- 开模不必再设慢速起步段：x=1 处速比 7.4，95% 流量下动模板只有约 51mm/s。

验收步骤：

1. `bFlowPctMode = FALSE` 跑一遍，确认与现状一致，作为基线；
2. `TRUE`，两个动作先用 70%：巡航泵转速 ≈ uiSpd% × 1700（±2%），无 FLOW 标志，k≈1；
3. 升到 95%，开模减速 700→600→500ms：终点误差 ≤ ±1.5mm，InEndVelocity 正常置位；
   落地段峰值减速度（与现状同一差分窗口）≤ 现状实测 1.85 m/s² 的 1.5 倍（约 2.8 m/s²），无反向回弹；
   误差超差时按 5ms 一档调参数 35，并确认 k≥1；
   30%~45% 的零终速段（若使用）落地峰值减速度（同一差分窗口）≤ 2.8 m/s²；超出时加大该段 t_dec，
   或让零终速段保持高百分比。开模仍为单段 95%；
4. 合模两段：到 10mm 速度 ≤ 现状 43.5mm/s 的 1.1 倍，护模动作与报警不受影响；全程泵转速无台阶式突降；
5. 周期目标：合模 ≤ 1.30s（含起步段），开模 ≤ 1.25s。

## 8. 风险与范围外

- 开模高速停车（约 620mm/s 起刹）对机身与模具的冲击需现场确认，先用 700ms 减速；
- 加速结束后已有约 ±8% 的速度波动，提速后可能放大；本次只观察，不加滤波；
- 压力限幅（output_limiter）降流量时不回算规划速度，维持现状；
- 回算改写规划器状态后，`motion_control.c` 的 D2 刹距诊断（BRAKING_DISTANCE）读取的斜坡锚定可能变化，
  探针未覆盖这部分，由 T4/T5 断言无误报；若误报，在百分比模式下改用包络速率判定；
- 刹车包络按 `VELOCITY = 700` 计算（2026-11-19 终审）：百分比模式下规划器刹车加速度为 `maxVelocity / t_dec`，
  低百分比零终速段落地比实际速度应有的更硬（20ms 纯滞后、20ms 窗口探针：30%/t_dec 300 为 2.58 m/s²，理想 1.22；
  45%/400 为 2.74，理想 1.37；70%/700 为 1.70，理想 1.20；95%/700 为 1.03，理想 0.98；95%/500、纯滞后 40ms 为 3.78）。
  同一 700 也使 BRAKING_DISTANCE 判定偏宽松。不改为 `700 × 百分比`：95%→30% 链式换段时规划速度单拍跌 336mm/s。
  本次只在 §7 加验收项，不改代码；
- 范围外：MoveVelocity、配方路径、仿真器改为消耗泵转速闭环。

## 9. 落地顺序

1. 库：上下文字段、斜坡锚定、流量上限、回算、标志（T1~T6）；
2. IEC 接口三处同步、尺寸预算；
3. FB_HydAxis：`bFlowPctMode`、PUMPFLOWPERCENT、目标点标尺、速度安全上限；
4. PLC 工程重新生成接口并编译，按第 7 节上机验收。
