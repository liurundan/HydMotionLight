# FB_HydAxis 段间速度斜率与液压换向保护设计

Date: 2026-11-05

Related files:

- `iec/FB_HydAxis.st`
- `tests/test_hydaxis_p0_contract.py`
- `src/motion_planner.c`
- `src/motion_interface.c`

## 1. 目标与范围

在 `FB_HydAxis` 内完成工艺层 `uiAcc`、`uiDec`（毫秒）到液压库加速度、减速度（速度单位/秒）的换算，并根据工艺段之间的目标速度规划连续的速度斜率。

本设计覆盖：

- 位置段和速度段的段间速度衔接；
- 位置段 `Vend` 作为下一段 `V0` 的命令侧速度链；
- 速度幅值与运动方向的分离；
- 反向时先减到零再切换方向；
- `uiAcc=0`、`uiDec=0` 或过小时间的 100 ms 最小时间保护；
- 方向相关最大速度选择和速度幅值边界保护；
- 使用功能块私有 method 封装斜率计算。

本设计不修改底层 C 库的运动学、压力控制器或泵控模型，只在 IEC 工艺层补齐段间命令规划和必要的换向编排。

## 2. 液压运动语义

液压泵控轴使用“正速度幅值 + 独立方向”语义：

```text
速度幅值：V0、Vset、Vend >= 0
方向：Extend 或 Retract
```

因此，`0 -> 300` 和 `300 -> 0` 都使用正速度幅值。位置方向由 `DIRECTION` 和 `ENDVELOCITYDIRECTION` 传递给底层库，不能通过把 `rVel` 或 `rEndVel` 设为负数来表示缩回。

液压系统没有电机式反向制动能力。正在伸出时直接切换为缩回会造成阀控方向突变和液压冲击，所以换向必须分成两个阶段：

1. 保持原方向，按减速度将速度幅值降到零；
2. 方向切换为目标方向，从零按加速度建立目标速度。

底层库已有位置规划器的方向变化安全分支；IEC 层必须保持方向和速度幅值的独立语义，不能破坏该分支，也不能在速度段在线更新时直接生成反向速度跳变。

## 3. 现状与问题

当前 `FB_HydAxis.st` 的速度换算为：

```iecst
rVel := UINT_TO_REAL(uiSpd) * 0.1 * 0.01 * rMaxSpeed;
rEndVel := UINT_TO_REAL(uiEndSpd) * 0.1 * 0.01 * rMaxSpeed;
rAcc := rMaxSpeed / UINT_TO_REAL(uiAcc) * 1000;
rDec := rMaxSpeed / UINT_TO_REAL(uiDec) * 1000;
```

存在以下缺口：

1. `rAcc` 和 `rDec` 使用整个最大速度，未使用当前段真实的 `|Vset - V0|`；
2. 上一段的 `Vend`、速度段的 `Vset` 没有锁存为下一段 `V0`；
3. `rMaxSpeedExtend`、`rMaxSpeedRetract` 已计算，但当前换算仍直接取通用 `rMaxSpeed`；
4. 没有明确区分同向速度调整和反向换向；
5. `uiAcc=0` 或 `uiDec=0` 时没有统一的 100 ms 时间下限；
6. 直接比较带方向的速度会把方向变化误判为普通加速或减速。

## 4. 设计方案

### 4.1 方向相关最大速度

在换算 `uiSpd` 和 `uiEndSpd` 之前，根据当前工艺方向选择能力边界：

```text
uiDir = Extend  -> rMaxSpeedExtend
uiDir = Retract -> rMaxSpeedRetract
其它             -> 已有通用 rMaxSpeed 作为兼容回退
```

选出的 `rMaxSpeedDir` 用于 `Vset`、`Vend` 换算、速度幅值限制和斜率上限保护。

上一段锁存的 `V0` 使用上一段实际换算出的物理速度幅值保存，不重新使用下一段的最大速度归一化。这样即使伸出和缩回最大速度不同，段间比较仍在同一物理速度单位内完成。

### 4.2 时间保护与基本换算

定义固定最小时间：

```text
MIN_RAMP_TIME_MS = 100
tAcc = MAX(uiAcc, 100) * 0.001 s
tDec = MAX(uiDec, 100) * 0.001 s
```

当 `uiAcc=0` 或 `uiDec=0` 时直接使用 100 ms，不产生除零或无限斜率。

同向段间过渡使用速度幅值差：

```text
Vset > V0:  rAcc = (Vset - V0) / tAcc
Vset < V0:  rDec = (V0 - Vset) / tDec
Vset = V0:  不产生段间过渡
```

所有结果保持为正的加速度或减速度。`V0`、`Vset`、`Vend` 的比较只比较幅值，不把方向编码进数值符号。

### 4.3 位置段和速度段的速度链

首段或速度链断开后：

```text
V0 = 0
```

下一段 `V0` 来源：

| 上一段模式 | 下一段 `V0` |
| --- | --- |
| 位置段 | 上一段 `Vend` |
| 速度段 | 上一段 `Vset` |
| 压力段 | 速度链断开，取 `0` |

位置段的 `Vend` 是正的末速度幅值，既用于当前段终点速度下发，也用于下一段起始速度锁存。速度段下一段使用上一段目标速度幅值作为 `V0`，`uiEndSpd` 在速度段不参与速度链。

### 4.4 `Vend` 与当前段末端减速

位置段在底层表现为：

```text
V0 -> Vset -> Vend
```

段间过渡斜率和当前段末端减速是两个不同阶段：

- `V0 -> Vset`：决定新段开始时使用 `rAcc` 或过渡用 `rDec`；
- `Vset -> Vend`：决定接近位置终点时使用的末端 `rDec`。

当两者都需要减速而底层 FB 只有一个 `DECELERATION` 输入时，IEC 层在段切换/位置段重触发时更新该值：

1. 新段开始阶段使用 `|Vset - V0| / tDec`；
2. 进入末端减速区后切换为 `|Vset - Vend| / tDec`；
3. 通过已有的 `105` 重触发机制让 `HYD_MoveContinuousAbsolute` 重新锁存位置段参数。

速度段继续使用 `CONTINUOUSUPDATE=TRUE` 在线更新斜率。

### 4.5 反向保护

方向判断先于 `Vset/V0` 比较：

```text
当前方向 == 上一段方向：同向过渡
当前方向 != 上一段方向：反向过渡
```

反向过渡不计算 `Vset - V0`，也不把正反方向速度相加。规划器保存：

```text
iDirActive       当前实际下发方向
iDirTarget       当前工艺目标方向
bDirectionChange 是否发生方向变化
bDecelToZero     是否处于换向前减速阶段
```

反向阶段保持活动方向为上一段方向，并按 `rDec = V0 / tDec` 将目标幅值降到零。确认过渡完成后，才执行以下切换：

```text
iDirActive := iDirTarget
V0 := 0
rAcc = Vset / tAcc
```

对于位置段，优先复用底层位置规划器现有的“方向变化时保持旧方向并减速到零”保护；IEC 层锁存方向，避免新段参数更新时提前把方向切换到目标方向。对于速度段，`CONTINUOUSUPDATE` 更新必须经过同样的零速保护，不能只按幅值限速后直接切换方向。

### 4.6 Method 边界

在 `FB_HydAxis` 内新增或恢复私有 method：

```iecst
METHOD PRIVATE CalcSegmentRamp : BOOL
```

method 负责：

- 归一化 `uiAcc/uiDec` 时间；
- 选择方向最大速度并限制速度幅值；
- 读取上一段锁存速度和方向；
- 计算同向段 `rAcc/rDec`；
- 识别反向并设置零速过渡状态；
- 计算位置段末端减速值；
- 锁存当前段的 `Vset`、`Vend`、模式和方向。

主循环只负责检测 `bStart` 上升沿或允许的速度在线更新、调用 method、调用现有运动 FB，并在停止、急停、复位或报警回到空闲时清空速度链。

不新增外部输入，不改变 `uiAcc/uiDec`、`uiSpd/uiEndSpd` 的单位和 PLC 工艺接口。

## 5. 工程保护

实现必须包含以下保护：

- `uiAcc` 和 `uiDec` 的 100 ms 最小时间；
- `rMaxSpeedDir <= 0` 时输出零速度和零斜率，不进行除零；
- `uiSpd`、`uiEndSpd` 换算后的速度幅值限制在方向最大速度内；
- `V0`、`Vset`、`Vend` 使用 `MAX(value, 0)`，不允许出现负幅值；
- 无效方向不启动运动段，并沿用现有报警/状态路径；
- 反向段必须经过零速阶段；
- 停止、急停、复位、报警流程清空锁存的速度链和方向过渡状态；
- `rStopDec` 继续使用有效的减速度，不能因段间斜率计算失败而变成零。

实际速度反馈不直接替代命令侧 `V0`：液压压缩、摩擦、负载和压力限幅会使反馈速度滞后或抖动。命令侧锁存用于保证段间斜率确定性，底层实际反馈继续负责安全减速和运行状态判定。

## 6. 数据流

```text
uiDir ───────────────┐
                     ├─> 方向最大速度选择 ─> Vset/Vend 幅值换算
uiSpd/uiEndSpd ──────┘                         │
                                               v
上一段 mode/Vset/Vend/dir ─> V0 与方向判断 ─> CalcSegmentRamp
                                               │
                             ┌─────────────────┴─────────────────┐
                             v                                   v
                      同向加速/减速                       反向先减到零
                             │                                   │
                             └───────────────┬───────────────────┘
                                             v
                              rAcc/rDec + iDirActive
                                             │
                  HYD_MoveContinuousAbsolute / HYD_MoveVelocity
```

## 7. 测试设计

### 7.1 Method/source contract

扩展 `tests/test_hydaxis_p0_contract.py`，检查：

- 存在 `CalcSegmentRamp`；
- `V0/Vset/Vend` 使用非负速度幅值；
- `MAX(uiAcc, 100)` 和 `MAX(uiDec, 100)` 保护存在；
- 方向最大速度在速度换算前选择；
- 位置段 `Vend` 锁存为下一段 `V0`；
- 速度段 `Vset` 锁存为下一段 `V0`；
- 压力段断开速度链；
- 反向分支不使用带符号速度差直接换向；
- 位置 FB 使用活动方向和活动速度参数，而不是未锁存的下一段输入。

### 7.2 数值场景

至少覆盖：

| 场景 | 期望结果 |
| --- | --- |
| 首段 `V0=0`，`Vset>0` | 使用 `uiAcc` 加速 |
| 同向 `Vset>V0` | `rAcc=(Vset-V0)/tAcc` |
| 同向 `Vset<V0` | `rDec=(V0-Vset)/tDec` |
| 位置段 `Vend != Vset` | 下一段 `V0=Vend` |
| 速度段后续 | 下一段 `V0=上一段 Vset` |
| 压力段切回运动段 | `V0=0` |
| `uiAcc=0` | 使用 100 ms |
| `uiDec=0` | 使用 100 ms |
| `0→300` Extend | 正速度幅值，Extend 方向，按距离正向规划 |
| `300→0` Retract | 正速度幅值，Retract 方向，按距离反向规划 |
| Extend 运行中切换 Retract | 先保持 Extend 减到零，再 Retract 加速 |
| Retract 运行中切换 Extend | 先保持 Retract 减到零，再 Extend 加速 |
| 伸缩方向最大速度不同 | 分别使用对应方向的物理最大速度 |
| `rMaxSpeedDir<=0` | 零斜率、零速度且不除零 |

### 7.3 回归验证

实现后按仓库标准运行：

```text
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_hydaxis_p0_contract.py
cmake --build --preset unixgcc
ctest --test-dir out/build/unixgcc --output-on-failure
```

若 IEC 源码没有独立编译目标，则以 source-contract 测试和现有运动规划、停止、方向回归测试作为验证证据。

## 8. 非目标

本轮不做：

- 修改 C 侧 `HYD_MotionPlanner` 的单位或公共结构；
- 修改压力闭环的压力斜率算法；
- 新增可调最小减速时间输入；
- 新增完整 S 曲线或 jerk 规划器；
- 使用实际速度反馈重建工艺段 `V0`；
- 重构整个 `FB_HydAxis` 状态机；
- 修改 PLC 工艺层已有方向枚举和速度百分比接口。

## 9. 风险与控制

| 风险 | 控制措施 |
| --- | --- |
| 方向切换过早导致液压冲击 | 锁存活动方向，零速后才切换目标方向 |
| 缩回方向最大速度不同导致速度换算错误 | 先按方向选择 `rMaxSpeedDir`，再换算幅值 |
| `Vend` 被误当成带符号速度 | 统一使用非负幅值，方向单独锁存 |
| 位置段参数未被底层重新锁存 | 保留 `105` 重触发路径 |
| 实际反馈滞后造成命令斜率抖动 | 斜率使用命令侧锁存，反馈只做底层安全判定 |
| 100 ms 下限仍不适合个别机构 | 本轮固定为工程默认值，现场验证后再评估是否开放配置 |

## 10. 验收标准

完成后应满足：

1. `uiAcc/uiDec` 始终按毫秒换算为底层正的加速度/减速度；
2. `uiAcc=0` 和 `uiDec=0` 均按 100 ms 处理；
3. `0→300` 与 `300→0` 使用相同的正速度幅值语义，仅方向参数不同；
4. 同向段按 `V0` 与 `Vset` 的幅值差选择加速或减速时间；
5. 位置段 `Vend` 和速度段 `Vset` 正确形成下一段速度链；
6. 反向时没有带符号速度跳变，必须先减到零；
7. 方向最大速度选择正确，且无效能力边界不会除零；
8. 现有位置、速度、压力、停止、急停和报警生命周期保持可用；
9. 相关源契约测试和完整 CTest 回归通过。
