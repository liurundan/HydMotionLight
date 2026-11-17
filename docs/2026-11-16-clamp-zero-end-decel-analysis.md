# 计划：把合模轴零终速停位减速分析保存为 docs 文档

## Context

用户要求把本次会话对 `docs/data_deg/vofa_waveform_20261113_142045_306.csv` 的分析结论保存到 docs 文件夹，格式为 md。内容包括：MoveContinuousAbsolute 在目标 300mm、终点速度 0 时减速时间不足的根因，油缸有效面积的推算，以及启用 105 机构后的操作步骤。

这次只新建一个文档，不改任何源码。

**更新记录（2026-11-17）**：105 连杆参数上机修正为 LF=323（提交 1a00e3f），机构校验通过；泵转速单位已澄清（库内为 rpm，CSV 曲线显示值 = rpm × 0.1）；§6、§8、§10、§11 按新几何重写，库与 FB_HydAxis 的修复已落地，见 §12。

## 1. 问题描述

开模最后一段 MoveContinuousAbsolute 的参数为：目标位置 300mm，ENDVELOCITY=0，减速时间 uiDec=500ms。上机实测：

- 规划减速只持续了约 52ms，泵指令从 25.26 降到约 2.47；
- 随后又经过约 500ms 才降到 0；
- 动模板最终停在约 304.8mm，过冲约 4.8mm。

需求：位置控制段的终点速度为 0 时，应按给定的减速时间减速，并尽量准确停在目标位置。

## 2. 测试条件与数据单位

| 项 | 值 |
|---|---|
| 机构类型 | type=0，HYD_MECHANISM_DIRECT（未启用肘杆运动学） |
| 油缸面积（库参数 31/32） | 两侧均为 10000mm²（不是真实值） |
| FB_HydAxis 最大线速度 rMaxSpeed | 100mm/s |
| 油泵 | 1700rpm，50cc/rev（库内按 0.95 容积效率计算） |
| 采样/控制周期 | 1ms |
| uiSpdCmd | 百分比，乘以 rMaxSpeed 得到 mm/s |
| uiAxisSpdAcc / uiAxisSpdDec | ms |
| 位置 | mm |
| PUMPSPEED | 库内单位 rpm；CSV 曲线显示值 = rpm × 0.1（仅为缩放观察） |

开模最后一段：uiSpd=20%，目标 300mm，t_acc=t_dec=500ms，END=0，规划速度 20mm/s。

## 3. 波形证据（CSV 行号 = 数据行序号）

| 证据 | 数值 | 说明 |
|---|---|---|
| 泵转速 ÷ 规划速度 | 各段恒为 1.263 | 25.26/20 = 37.89/30，规划速度确实是 20mm/s |
| 刹车触发点（第 18909 行） | 剩余 4.97mm | 与现有公式 0.5·20·0.5 − 0.5·20·0.001 = 4.99mm 一致 |
| 刹车期间规划速度 | 52 行内与 √(2·40·s) 吻合，误差约 0.01 | a = 20/0.5 = 40，s = 300 − 实测位置 |
| 刹车前实测动模板速度 | 约 100mm/s | 由位置斜率求得，是规划速度的 k≈5 倍 |
| 液压纯滞后 | 约 40–45ms | 泵指令开始下降后，动模板速度仍保持 95–100mm/s |
| 第 18962 行 | bStop=1，位置 300.02 | 工艺层在到位时发出停止 |
| 最终位置 | 约 304.8mm | 刹车开始后约 140ms 停稳 |

## 4. 根本原因

### 4.1 规划速度与实际速度失配（k ≈ 5）

规划器在模板坐标下按 20mm/s 规划，刹车距离为 0.5·v·t_dec = 5mm。动模板实际速度是 100mm/s，按 500ms 减速需要约 25mm。

- 刹车触发晚了约 20mm；
- 触发之后，剩余距离按 5 倍速度缩短，终速 0 的 blend 路径使用零延迟包络 √(2as)（`motion_planner.c:600–605`），包络在 52ms 内被压到 0，这就是实测的 52ms；
- 再加上 40–45ms 纯滞后期间动模板仍以约 100mm/s 运动，最终过冲约 4.8mm。

### 4.2 500ms 拖尾不是 MoveContinuousAbsolute 的减速

- 第 18962 行工艺层发出 bStop，FB_HydAxis 进入 iStep=200，调用 `fbStop(DECELERATION := uiDec)`（`iec/FB_HydAxis.st:477–481`）；
- HYD_Stop 按固定时间语义（`motion_control.c:3397–3408`）把剩余约 1.9mm/s 的规划速度用满 500ms 降到 0；
- 这时动模板早已停住。

工艺层发 bStop 的原因多半是 InEndVelocity 一直不置位：终速 0 时静止判据默认是 0.01mm/s（`motion_interface.c:2253–2257`），而停稳后速度信号噪声约 ±5mm/s。

### 4.3 上一轮修复（5ad5e60）为何无效

5ad5e60 只把触发距离调整了 ±0.5·v·dt，并修改了越过目标后的斜坡保持逻辑，没有触及 5 倍的速度失配和液压纯滞后，所以对本问题没有效果。

## 5. k 的构成：面积误差 × 肘杆速比

取泵转速恒定 ≥100ms 的稳态数据，按位置分段统计 k = 实测速度 / 规划速度：

| x (mm) | k_开模 | k_合模 | 开/合 |
|---|---|---|---|
| 20 | 1.67 | 1.16 | 1.43 |
| 60 | 3.29 | 2.29 | 1.44 |
| 100 | 4.19 | 2.95 | 1.42 |
| 160 | 4.90 | 3.41 | 1.44 |
| 240 | 5.04 | 3.53 | 1.43 |

- 开/合比在全行程恒为约 1.43，与位置无关，对应两侧面积不同（面积误差）；
- k 本身沿行程变化 3 倍以上，且开、合形状相同，这是肘杆速比 R(x) 的特征，不是面积问题。

direct 模式下：

    k(x) = 10000 / (A_真实 × R(x)) × (η_真实 / 0.95)

结论：**单靠这份 CSV 算不出油缸真实面积**，只能得到乘积 A×R(x)。面积、速比、实际容积效率三者耦合在一起。早先估算的 2000/2900mm² 是开模端（x≈200–300mm）的 A×R，不是油缸面积。

## 6. 结合 105 几何反算有效面积

105 几何（`FB_HydAxis.st` SetToggleMechanism iType=105）：LR=260，**LF=323**（上机实测修正，原 320），LPF=190，LPK=100，LD=125，HF=200，HM=170，DC=580，SM=450，XHANDOFF=0，分支参数均为 −1。

R(x) 由库函数 `HYD_ToggleKinematics_SolveOnline` 计算（已用库源码编译验证）：

| x (mm) | R(x) | A_开模 = 10000/(k·R) | A_合模 |
|---|---|---|---|
| 20 | 1.728 | 3466 | 4990 |
| 60 | 0.901 | 3375 | 4849 |
| 100 | 0.705 | 3386 | 4809 |
| 160 | 0.606 | 3370 | 4842 |
| 240 | 0.587 | 3382 | 4829 |
| 300 | 0.614 | — | — |

- 其他特征点：R(0)=10.02，R(3)=5.03，R(10)=2.646，R(450)=0.245；区间内极值 x≈218.5 处极小 0.584，x≈386.5 处极大 0.673；
- 细分位置区间拟合（泵转速恒定 ≥100ms 的稳态点）：x≥12mm 时 LF=323 的面积比 x≥60 均值最多高 3.3%，LF=320 则系统偏低 4.5%~12%，CSV 独立印证了 LF≈323；
- x<12mm 的点不能用于判断：合模 2~12mm 是压力段（泵 84rpm），开模 x<8mm 是起步瞬态；
- **有效面积：开模侧（参数 31）3350mm²，合模侧（参数 32）4800mm²**。拟合值 3385/4842，取略小约 1%，符合 §9 "面积宁小勿大"；
- 这两个值是相对"容积效率 0.95"标定的，实际容积效率、驱动器转速误差等恒定比例误差已包含在内。约束：**面积 ÷ 参数 29 必须保持不变**，以后改容积效率时，参数 31/32 要乘以（新值 ÷ 0.95）；
- 锁模端速比对连杆尺寸非常敏感：LF=322/323/324 时 R(0)=16.8/10.0/7.4，开模 0→10mm 段需上机确认（§8 第 6 步）；
- 校验裕度：LF 321 或 DC 582 即校验失败，裕度约 1.5mm。

## 7. 只改 direct 模式的面积/最大线速度，为什么不够

- 面积决定 k，rMaxSpeed 只是百分比的标尺：k = 库内面积 / (A×R)，与 rMaxSpeed 无关；
- 以 2000/2900 + rMaxSpeed 500 为例：开模端 k≈1，泵转速与现状相同，工艺百分比不用重调；
- 但 A×R 随位置变化：x≈20mm 处开模约 5800、合模约 8300。常数面积会在靠近模具的区段把速度高估约 3 倍，导致过早减速、爬行进位，或终速 0 时停短；
- 两个方向共用一个 rMaxSpeed（`FB_HydAxis.st:309`）时，合模百分比对应的泵转速会变；
- 结论：direct 模式只能修好开模端的这一个停位，要在全行程都准确，需要启用 105 机构。

## 8. 启用 105、按动模板速度规划的步骤

### 第 0 步：机构校验（已解决）

- LF=320 时 x=0 处主连杆不可达（错误码 6），校验失败；上机实测修正为 LF=323 后校验通过；
- 通过 IEC 流程（CreateMotion → ConfigureToggleMechanism → ReadToggleMechanism）验证：DONE，ERRORID=0，X_GEOMETRY_MIN=0，X_HANDOFF_EFFECTIVE=0，K=[0.245, 10.02]；CreateMotion 与 Configure 各 65 个扫描周期（约 65ms）；
- 0~450mm 按 0.01mm 逐点在线求解全部成功，无需修改校验逻辑；
- FB_HydAxis 第 1 步已加报警分支（§12），以后重测参数超出约 1.5mm 裕度时直接报警，不再卡在第 1 步。

### 第 1 步：xHandoff（已解决）

有效在线区间已覆盖 [0, 450]，**XHANDOFF 必须保持 0**。设成 3 会让开模从 x=0 起步时报 KINEMATICS_POSITION_OUT_OF_RANGE。前提：工艺层对位置做零点偏移时，结果必须 ≥0。

### 第 2 步：面积参数

- 库按 σC 修正后的执行器速度方向选面积（`actuation_mapper.c` resolve_cylinder_gain）：开模（位置增大）用参数 31，合模用参数 32；
- 即 **31 = 3350，32 = 4800**；
- 上机用 HYD_ReadStatus 的 ACTUATORDIRECTION 确认，开模时应为 1。

### 第 3 步：段速度标尺（已实现，§12 B1/C1）

启用 105 后 VELOCITY 就是动模板速度，且动模板的最大线速度随位置变化。HYD_ReadStatus 的 VELOCITYRATIO 只是上一拍、当前位置的速比，合模段的最坏点在目标端，运动开始前读不到，所以**不能用作标尺**，仅用于验收和 HMI 显示。

做法：段启动沿调用 `HYD_ReadMaxTemplateVelocity`，取本段区间 [起点, 目标] 内的最大速比 Rmax，标尺 = Q_lim ÷ (A × 6e-5 × Rmax)，Q_lim = min(maxFlow, 1700/21.05 = 80.75 L/min)。百分比 100% 正好让泵在本段最坏点达到 1700rpm。

泵满转时动模板速度上限：

| x (mm) | 3 | 20 | 100 | 300 |
|---|---|---|---|---|
| 开模 (mm/s) | 79.8 | 232.5 | 569.9 | 654.4 |
| 合模 (mm/s) | 55.7 | 162.3 | 397.8 | 456.7 |

各段标尺（沿用 CSV 百分比；泵转速单位 rpm）：

| 段 | Rmax（位置） | 标尺 mm/s | 动模板速度 mm/s | 泵 起点→终点 rpm |
|---|---|---|---|---|
| 合 300→150，30% | 0.615（150） | 456.1 | 136.8 | 509→510 |
| 合 150→10，30% | 2.646（10） | 106.0 | 31.8 | 118→510 |
| 　拆成 150→60 | 0.901 | 311.3 | 93.4 | 348→510 |
| 　拆成 60→10 | 2.646 | 106.0 | 31.8 | 174→510 |
| 开 0→10，20% | 10.02（0） | 40.1 | 8.0 | 340→90 |
| 开 10→200，20% | 2.646（10） | 151.8 | 30.4 | 340→75 |
| 　拆成 10→60 | 2.646 | 151.8 | 30.4 | 340→116 |
| 　拆成 60→200 | 0.901 | 446.1 | 89.2 | 221→340 |
| 开 200→300，20% | 0.614（300） | 654.4 | 130.9 | 325→340 |

- 段标尺由最坏点决定，跨越速比变化大的段会变慢、低速端泵转速偏低，建议每段 Rmax/Rmin ≤ 约 3，在 x≈60mm 处拆段；
- 合模 10→0 与锁模是压力段，不经过机构映射，不需要标尺；
- 出现 FLOW / PUMP_SPEED 限幅标志时实际速度低于规划，k≈1 的前提不成立。验收时百分比 ≤95%。

### 第 4 步：液压纯滞后补偿（已实现，§12 B2）

参数 35 HYDRAULIC_RESPONSE_DELAY（ms，库默认 0，上限 200）。只作用于 MoveContinuousAbsolute 终速 0 的 blend 停车：刹车触发距离加 v·τ，包络预留同样的 τ，两者重合，不再出现 52ms 塌缩。合模轴写 40ms（补偿宁小勿大：补偿过度会停短后爬行）。

### 第 5 步：工艺层停车逻辑

- 终速为 0 时等待 InEndVelocity（bDone）完成，不要在 PositionReached 时发 bStop；
- 停稳阈值用新参数 36 ZERO_END_SETTLE_VELOCITY（mm/s，合模轴写 3），**不要改参数 1 velocityTolerance**：参数 1 同时打开速度偏差诊断（100ms 去抖，2s 升级 FAULT，MoveContinuousAbsolute 报 ERROR）；
- 合模工艺层的速度反馈已加一阶滤波（停稳时原始噪声标准差 3.3mm/s，滤波后约 0.5mm/s）。

### 第 6 步：上机验收

1. ReadToggleMechanism 读回 LF=323、X_GEOMETRY_MIN=0、K_MAX≈10.02，第 1 步约 65ms 内完成；
2. 开模时 ACTUATORDIRECTION=1；VELOCITYRATIO 在 x=100 时约 0.705，x=20 时约 1.728；
3. GetPumpRequest.PUMPSPEED 在各段最坏点 ≈ 百分比 × 1700 rpm（±1%），段内分布与第 3 步表一致（VOFA 曲线显示 ×0.1）；
4. x≥12mm 范围内实测速度 ÷ 规划速度 ≈ 1（±5%），无 FLOW/PUMP_SPEED 标志；
5. 300mm 终速 0：减速时长 ≈ uiDec（±10%），终点误差 <0.5mm，InEndVelocity 正常置位，无 500ms 拖尾，全循环无 VELOCITY_DEVIATION；
6. 开模 0→10mm：对比实测与规划速度，确认锁模端速比。

## 9. 简化模型仿真对比（不是上机结果）

模型按本波形拟合：45ms 纯滞后，减速能力约 1000mm/s²。复刻了规划器的固定时间触发、Δv/T 斜坡、√(2as) 包络，以及工艺层到位发 bStop。

| 方案 | 规划减速到位耗时 | 终点误差 |
|---|---|---|
| 现状（k=5） | 约 50ms | +5.4mm（实测 +4.8mm） |
| 面积修正 k=1，库不改 | 约 370ms，到位时残速 2.7%，又触发 bStop 拖尾 | +1.35mm |
| k=1 + τ=40ms | 约 480ms | +0.25mm |
| k=1 + τ=45ms | 约 500ms | +0.05mm |
| k=1.1（面积偏小 10%）+ τ=40ms | 约 425ms | +0.34mm |
| k=0.9（面积偏大 10%）+ τ=40ms | 未到达目标 | −2.4mm（实际会再逼近、爬行） |

结论：面积/机构标定解决 k，τ 补偿解决纯滞后，两者都要做。面积宁可略偏小（k 略大于 1），不要偏大。

库内实现后的回归测试（`test_movecontinuousabsolute_integration`，45ms 纯滞后、k=1、目标 300mm、20mm/s、减速 500ms、终速 0）：

| τ | 规划减速时长 | 终点误差 |
|---|---|---|
| 0（对照） | 381ms | +0.13mm |
| 40ms | 486ms | +0.05mm |
| 45ms | 594ms（略过补偿，斜坡被拉长） | +0.004mm |
| 40ms，k=1.1 | 429ms | +0.06mm |

3mm 短行程（峰值 6mm/s，未到巡航）在 τ=40ms 下规划减速 488ms，终点误差 +0.016mm。

## 10. 落地顺序

1. 配置：保持 SetPumpConfig(50, 1700)、容积效率 0.95；面积 31=3350、32=4800；LF=323、XHANDOFF=0；
2. 库：B1 段标尺查询、B2 纯滞后补偿、B3 独立停稳阈值（§12，已完成）；
3. FB_HydAxis：C1 段标尺、C2 第 1 步报警与复位重试、C3 参数 35/36（§12，已完成）；stHydAxisRef 增加两个字段（§12）；
4. 工艺层：等 bDone 再进下一步；在 x≈60mm 拆段并重整百分比；开模 0→10 先 ≤50%；
5. 按 §8 第 6 步上机验收。

## 11. 待确认事项

1. ~~105 校验~~：LF=323 后通过（§8 第 0 步）；
2. ~~R(x) 数值~~：已用库源码编译验证（§6）；
3. ~~泵转速单位~~：库内为 rpm，CSV 显示值 = rpm × 0.1。CSV 稳态值 25.26/37.89/8.42/18.53 全部符合增益 21.05 rpm/(L/min)；
4. **铭牌参数**：油缸缸径、杆径用于交叉核对 §6 的有效面积和实际容积效率；
5. **锁模端速比**：R(0) 对 LF ±1mm 敏感（7.4~16.8），按 §8 第 6 步第 6 项上机确认；
6. **FB_HydAxis 未经 IEC 编译**：仓库内没有 ST 编译/仿真环境，§12 C 部分的修改需在 PLC 工程中编译并上机验证。

## 12. 已落地的修改

### 液压库

| 项 | 内容 | 文件 |
|---|---|---|
| B1 段标尺查询 | `HYD_ActuationMapper_MaxTemplateVelocityOverRange`：区间裁剪到 [xHandoffEffective, SM]，两端点 + 16 个内点求最大速比（`HYD_TOGGLE_RANGE_RATIO_SAMPLES`），按执行器方向选面积；direct 机构速比 1；方向不定取两向较小值 | `actuation_mapper.c/.h`、`toggle_kinematics.h` |
| | `HYD_MotionControlFB_ResolveMaxTemplateVelocity`：Q_lim = min(maxFlow, 泵转速上限 ÷ 增益) | `motion_control.c/.h` |
| | IEC FB `HYD_ReadMaxTemplateVelocity`：输入 AXISID/ENABLE/XFROM/XTO/DIRECTION（0 = 按 XTO−XFROM 符号），输出 VALID/ERROR/ERRORID/MAXVELOCITY/VELOCITYRATIOMAX/FLOWLIMIT | `motion_interface.c/.h`、`pousHydMotion.xml`、`tests/plcdemo/POUS.c/.h` |
| B2 纯滞后补偿 | 参数 35 `HYDRAULIC_RESPONSE_DELAY`（ms，默认 0，上限 200）；终速 0 blend 停车的触发距离加 v·τ，包络预留 τ | `common_types.h`、`hyd_config.h`、`motion_planner.c/.h`、`motion_control.c` |
| B3 独立停稳阈值 | 参数 36 `ZERO_END_SETTLE_VELOCITY`（mm/s，默认 0 = 原行为）；终速 0 的停稳判据优先用它，不打开速度偏差诊断 | `common_types.h`、`motion_interface.c`、`motion_control.c` |

FB 结构体增加 8 字节（3304 → 3312），`test_mode_switch_bumpless` 的尺寸预算同步更新。

### FB_HydAxis

- **C1**：段启动沿（rTrigStart）对曲肘轴的位置/速度段调用 fbMaxVel，锁存 rMaxSpeedSeg 作为本段 100% 标尺；位置段区间 = [实际位置, 目标]，速度段取当前点；直压轴仍用 stHydAxisRef.rMaxSpeed。标尺查询失败时报警进 900，不启动运动；
- **C2**：第 1 步 ConfigureToggleMechanism 报错时报警进 900；复位后如果机构未配置成功，撤销 EXECUTE 回第 1 步重试；
- **C3**：新增 SetParameter 方法；第 0 步在 stHydAxisRef.rResponseDelayMs、rZeroEndSettleVelocity 大于 0 时写入参数 35/36。

**需要在 PLC 工程的 ST_HydAxisRef 中新增两个字段**（仓库内没有该结构体定义）：

```iecst
rResponseDelayMs       : REAL;   // 合模轴 40，其他轴 0
rZeroEndSettleVelocity : REAL;   // 合模轴 3，其他轴 0
```

### 测试

| 测试 | 守护的行为 |
|---|---|
| `test_parameter_access` | 参数 35/36 默认值、读写、范围校验；写参数 36 不打开速度偏差诊断 |
| `test_movecontinuousabsolute_integration` | 1.5mm/s 停稳噪声在参数 36=3 时 InEndVelocity 置位；45ms 纯滞后下 τ=40 规划减速 450~550ms、终点 <0.3mm、斜坡不塌缩；3mm 短行程同样满足 |
| `test_actuation_mapper` | 105 几何 150→10 Rmax=2.646 / 标尺 106.0，200→300 开模 654.4 / 合模 456.7，300→450 捕获内部极值 0.673，越界裁剪与报错，direct 速比 1 |
| `test_toggle_motion_interface` | IEC FB 读取、泵配置前后 Q_lim 90 → 80.75，maxFlow 更小时取 maxFlow，越界/无效轴/ENABLE=FALSE 输出 |
| `test_toggle_motion_integration` | 105 合模 150→10：0.99 倍标尺泵峰值 ≈1683rpm 无限幅；1.05 倍出现限幅、泵钳在 1700rpm |
| `test_interface_layout_consistency` | 头文件、XML、plcdemo 三处字段一致 |

## 附录：分析方法

- 稳态筛选：泵转速连续 100ms 不变的行，排除液压滞后段；
- 实测速度：由位置差分求得（10–100ms 窗口），未使用噪声较大的 rActualValueDeriv；
- 规划速度：泵转速 ÷ 1.263，并用刹车触发点和 √(2as) 包络两处与代码公式交叉验证；
- R(x) 与机构校验：用库源码编译的临时探针调用 `HYD_ToggleKinematics_ValidateBlocking` / `SolveOnline` 计算；
- 分析脚本为临时脚本，未纳入仓库。
```
