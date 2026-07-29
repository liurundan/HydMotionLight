# 五铰点曲肘锁模轴运动学软件架构设计

## 1. 文档状态

- 状态：已批准设计，待实施计划。
- 日期：2026-07-29。
- 适用对象：立式注塑机五铰点斜排列双曲肘内翻式锁模机构。
- 首版计算模式：每个控制周期在线解析计算。
- 公共轴坐标：动模板工程位置 `Xm`，机械闭模位置为 `0`，开模方向为正。

本文只定义运动控制软件如何接入已经推导完成的机构公式。公式、符号、默认参数和数值保护的数学依据见：

- [五点斜排双曲肘内翻锁模机构运动学与 Kv 查表设计](./2026-07-28-five-point-inclined-toggle-kinematics-design.md)
- [五点斜排锁模机构 PC 在线逐周期计算模型](./2026-07-28-five-point-inclined-toggle-online-cycle-model.md)

## 2. 决策摘要

1. PLCopen 命令、位置反馈、软限位、到位判断和诊断统一使用模板坐标 `Xm/Vm`。
2. 新增独立 `toggle_kinematics` 模块，不把五铰点公式写入运动规划器或 IEC 适配层。
3. 运动学位于模板运动规划和液压执行转换之间：`Xm/VmCmd -> Xs/VsCmd -> flow -> pump speed`。
4. `CreateMotion` 新增机构类型，零值继续表示直压式；曲肘轴必须显式声明。
5. 每个通用轴对象只保存机构类型和配置槽索引。曲肘几何数据放入独立静态配置池。
6. 几何参数通过专用 IEC FB 整组暂存、跨扫描校验并原子提交，禁止运行中修改。
7. 实时路径使用 `Xm -> Xs`、`Vm -> Vs`；`Xs -> Xm` 反解用于标定和诊断，不进入当前 1 ms 热路径。
8. 直压轴保持原有行为和数据流，除一个机构类型分支外不承担运动学计算。
9. 本阶段只做 PC 基准和 Cortex-M7F 运算量估算。没有 ARM GCC 和目标板 DWT 数据时，不宣称 STM32 1 ms WCET 已通过。
10. 有效在线区间为 `[xHandoffEffective, Sm]`。几何模块自动推导 `xGeometryMin`；`xHandoff=0` 使用自动值，显式值不得低于它。

## 3. 当前实现约束

当前代码具有以下事实：

- `src/motion_interface.c` 使用静态 `HYD_MotionControlFB_inst[HYD_MAX_AXIS_MOTION]` 分配轴实例。
- `__mcl_cmd_CreateMotion()` 当前在一次调用内完成轴分配和初始化。
- `HYD_AxisRef.position/velocity` 是运动规划、诊断和完成判定共同使用的轴反馈。
- `motion_planner.c` 同时生成目标模板速度和按固定 `velocityToFlowGain` 换算的目标流量。
- `motion_control.c` 在段开始时把 `cylinderConfig` 或段参数解析成单个固定 `velocityToFlowGain`。
- Stop、P 到 V 无扰切换和部分 blending 路径也直接使用固定增益换算速度与流量。
- `HYD_CylinderConfig.strokeMm` 当前按 `AXIS_REF.position` 执行软限位，因此它实际上是公共轴工程坐标的行程上限。

五铰点机构的 `dXs/dXm` 随位置变化，且通常为负。只在主运行路径替换一次固定增益是不完整的，所有速度与执行器流量互换路径必须通过统一机构适配边界。

## 4. 目标与非目标

### 4.1 目标

- 兼容直压式和五铰点曲肘式锁模轴。
- 根据模板位置和模板速度请求在线计算活塞位置、速度比和活塞速度命令。
- 提供有界的活塞位置到模板位置反解 API。
- 为不同机械尺寸提供 PLC 可配置的 IEC 接口。
- 对几何常识、参数间矛盾、全行程可达性和奇异裕量进行校验。
- 保持静态内存、无堆分配、确定性控制周期和可裁剪资源上限。
- 给出 PC 性能基准和后续 STM32 实测方法。

### 4.2 非目标

首版不包含：

- LUT 控制模式或在线/LUT 自动切换。
- 杆件弹性、模板变形、拉杆伸长和销轴间隙模型。
- 油液压缩、泄漏、摩擦和压力动力学模型。
- 使用活塞电子尺作为当前实时闭环位置源。
- 运行中的机械几何参数热更新。
- 通用函数指针式机构插件框架。

## 5. 坐标与方向契约

### 5.1 公共轴坐标

对直压轴和曲肘轴，以下量始终表示模板工程坐标：

- `HYD_AxisRef.position`：`Xm`。
- `HYD_AxisRef.velocity`：`Vm`。
- PLCopen `Position/Velocity` 输入。
- `HYD_MotionSegment.targetPosition/maxVelocity`。
- 软限位、到位判断、位置误差和速度误差。
- `HYD_MotionState.plannedVelocity`。

PLC/HMI 不应因机构类型而改变位置语义。未来若传感器安装在活塞侧，应先由反馈适配层执行 `Xs -> Xm`，再写入 `HYD_AxisRef`。

### 5.2 执行器坐标

以下量只在机构适配和液压执行边界中存在：

- `Xs`：合模油缸活塞或十字头位置。
- `Vs`：合模油缸活塞速度。
- `k=dXs/dXm`：有符号速度雅可比。
- `actuatorDirection`：由 `Vs` 的符号确定的执行器方向。

`templateDirection` 与 `actuatorDirection` 必须分开。默认曲肘参数下 `k` 通常为负，模板合模方向与活塞合模方向不能共用同一个方向枚举来选择油缸面积或驱动阀组。

## 6. 模块边界

### 6.1 新增模块

建议新增：

```text
include/toggle_kinematics.h
src/toggle_kinematics.c
```

该模块只负责：

- 原始参数的静态校验和预计算。
- 单点在线位置、雅可比和速度求解。
- 有界位置反解。
- 全行程验证过程所需的单步扫描。
- 运动学错误分类。

该模块不依赖：

- PLC IEC 宏和 FB 数据结构。
- `HYD_MotionControlFB`。
- 泵、阀、压力控制器和输出限制器。
- 动态内存。

### 6.2 机构适配器

建议在通用控制层增加一个轻量机构适配边界，按枚举分派：

```text
DIRECT:
    Xs = Xm
    k = 1
    Vs = Vm

FIVE_POINT_TOGGLE:
    result = ToggleKinematics_SolveOnline(config, Xm, Vm)
```

首版使用 `switch`，不使用函数指针。这样便于编译器优化、最坏执行时间分析和静态链接裁剪。

### 6.3 运行数据流

位置和速度模式：

```text
Axis feedback Xm/Vm
    -> motion planner
    -> template velocity VmCmd
    -> mechanism adapter
    -> actuator velocity VsCmd and actuatorDirection
    -> cylinder velocity-to-flow conversion
    -> optional velocity-loop flow correction
    -> flow/pressure/soft-limit protection
    -> pump converter
    -> pump-speed request
```

压力闭环模式：

```text
pressure controller
    -> actuator-side hydraulic flow
    -> flow/pressure protection
    -> pump converter
```

压力控制器输出已经是执行器侧流量，禁止再次乘以 `k`。

## 7. 数据模型与资源分配

### 7.1 机构类型

```c
typedef enum {
    HYD_MECHANISM_DIRECT = 0,
    HYD_MECHANISM_FIVE_POINT_TOGGLE = 1
} HYD_MechanismType;
```

数值 `0` 必须保持直压式，以兼容旧 PLC 程序和零初始化结构。

### 7.2 通用轴对象

每个 `HYD_MotionControlFB` 只增加紧凑绑定信息：

```c
HYD_UINT8 mechanismType;
HYD_UINT8 mechanismSlot;
```

`mechanismSlot=0xFF` 表示没有扩展机构配置。具体字段位置应结合结构对齐测试确定，必要时与现有 `HYD_UINT8` 字段相邻，避免额外填充。

### 7.3 曲肘原始配置

```c
typedef struct {
    HYD_REAL lr;
    HYD_REAL lf;
    HYD_REAL lpf;
    HYD_REAL lpk;
    HYD_REAL ld;
    HYD_REAL hf;
    HYD_REAL hm;
    HYD_REAL dc;
    HYD_REAL sm;
    HYD_REAL xHandoff; /* 0 = use derived xGeometryMin */
    int8_t sigmaK;
    int8_t signB;
    int8_t tauS;
} HYD_ToggleGeometryConfig;
```

默认值来自现有运动学规格：

```text
Lr=150 mm, Lf=230 mm, LPF=135 mm, LPK=75 mm, Ld=60 mm
HF=130 mm, HM=100 mm, dc=378 mm, Sm=202 mm, xHandoff=0 (auto)
sigmaK=-1, signB=-1, tauS=-1
```

### 7.4 预计算配置

预计算对象至少包含：

- `deltaH`、`aP`、`bP`。
- `Lr2`、`Lf2`、`Ld2`、`invLr`。
- 全行程 `Xs` 范围和 `k` 范围。
- 最小根式余量。
- 最小归一化主雅可比。
- 自动推导的 `xGeometryMin` 和最终 `xHandoffEffective`。
- 配置版本和有效标志。

只保存能够减少实时运算或支持故障诊断的数据，不缓存每个采样点，不在首版引入 LUT。

### 7.5 静态配置池

```c
typedef struct {
    HYD_BOOL used;
    HYD_BOOL valid;
    HYD_BOOL usingDefaults;
    HYD_UINT8 ownerAxis;
    HYD_UINT16 configVersion;
    HYD_ToggleGeometryConfig raw;
    HYD_TogglePreparedConfig prepared;
} HYD_ToggleMechanismSlot;
```

配置池容量：

```c
#ifndef HYD_MAX_TOGGLE_MECHANISMS
#define HYD_MAX_TOGGLE_MECHANISMS HYD_MAX_AXIS_MOTION
#endif
```

本项目默认允许 20 个曲肘轴，因此仍需为 20 个槽预留静态 RAM。独立池的价值是职责隔离、可单独裁剪和避免继续膨胀通用参数对象，不是在默认 20 槽配置下消除这部分内存。

预计每槽约 `64-80 B`，20 槽约 `1.3-1.6 KiB`。实施后必须通过 `sizeof` 测试给出真实结果，并检查编译器对齐。

### 7.6 数据对象归属

曲肘几何参数不放入以下对象：

- `HYD_AxisRef`：只保存逐周期动态反馈。
- `HYD_MotionSegment`：几何不随动作段改变。
- `HYD_MotionFBParams`：应保存所有轴普遍需要的控制参数。
- `HYD_CylinderConfig`：油缸物理参数与连杆机构几何属于不同层。

`HYD_CylinderConfig.strokeMm` 当前按公共轴位置执行软限位。首版保持 ABI，但文档必须明确它是公共轴工程行程。曲肘轴配置提交时：

- 若 `strokeMm=0`，允许由 `Sm` 初始化公共轴行程。
- 若 `strokeMm>0`，必须与 `Sm` 在配置容差内一致。
- 不得再把该字段解释为活塞 `Xs` 的物理行程。

## 8. CreateMotion 生命周期

`HYD_CREATEMOTION` 追加输入：

```text
MECHANISM_TYPE : SINT
```

兼容规则：

- `0` 或旧程序未设置：`HYD_MECHANISM_DIRECT`。
- `1`：`HYD_MECHANISM_FIVE_POINT_TOGGLE`。
- 其他值：创建失败并返回机构类型非法错误。

直压轴继续在现有路径创建，不分配机构槽。

曲肘轴创建采用事务式流程：

1. 预留轴槽和机构槽，但不增加已提交轴计数。
2. 装入默认几何候选配置。
3. 执行静态校验和跨扫描全行程校验。
4. 校验期间 `BUSY=TRUE`、`DONE=FALSE`。
5. 全部通过后原子提交两个槽并输出 `AXISID`。
6. 任一步失败都释放预留资源，轴池和机构池不能泄漏计数。

现有单调递增 `nextAllocatedFB` 需要重构为 reserve/commit 语义，或等价的已使用位图。不能先永久消耗轴槽，再发现机构槽不足。

## 9. IEC 机械参数接口

几何参数不加入通用 `HYD_WriteParameter`。逐项写入无法保证参数组的一致性，也无法表达跨扫描验证状态。

### 9.1 配置 FB

```text
HYD_ConfigureToggleMechanism

Inputs:
  AXISID, EXECUTE
  LR, LF, LPF, LPK, LD, HF, HM, DC, SM, XHANDOFF
  SIGMA_K, SIGN_B, TAU_S

Outputs:
  DONE, BUSY, ERROR, ERRORID, CONFIG_VERSION
```

行为：

- 在 `EXECUTE` 上升沿复制全部输入到 FB 私有候选对象。
- 只允许轴处于 `IDLE/READY/DONE/ABORTED`。
- 校验工作跨多个 PLC 扫描分片执行，每周期处理固定数量的行程采样点。
- 运行轴配置请求返回 `MECHANISM_CONFIG_BUSY`。
- 校验失败时生效配置和 `configVersion` 完全不变。
- 校验通过后在单个扫描边界原子替换，版本递增。

### 9.2 读取 FB

```text
HYD_ReadToggleMechanism

Inputs:
  AXISID, ENABLE

Outputs:
  VALID, USING_DEFAULTS
  LR, LF, LPF, LPK, LD, HF, HM, DC, SM, XHANDOFF
  SIGMA_K, SIGN_B, TAU_S
  CONFIG_VERSION
  X_GEOMETRY_MIN, X_HANDOFF_EFFECTIVE
  XS_MIN, XS_MAX, K_MIN, K_MAX
  ERROR, ERRORID
```

读取接口返回当前已提交配置，不暴露正在校验的候选配置。

## 10. 运动学 API

```c
HYD_BOOL HYD_ToggleKinematics_Prepare(
    const HYD_ToggleGeometryConfig *config,
    HYD_TogglePreparedConfig *prepared,
    HYD_ToggleError *error);

HYD_BOOL HYD_ToggleKinematics_SolveOnline(
    const HYD_TogglePreparedConfig *prepared,
    HYD_REAL xm,
    HYD_REAL vm,
    HYD_ToggleSolution *solution,
    HYD_ToggleError *error);

HYD_BOOL HYD_ToggleKinematics_InversePosition(
    const HYD_TogglePreparedConfig *prepared,
    HYD_REAL xs,
    HYD_REAL *xm,
    HYD_ToggleError *error);
```

`SolveOnline` 一次返回：

- `xs`。
- `velocityRatio=k`。
- `vs=k*vm`。
- `actuatorDirection`。
- 必要的根式余量和条件指标。

位置和雅可比不能由两个独立热路径 API 分别计算，否则会重复计算几何和开方。

`InversePosition` 只在已经校验为严格单调的 `[xGeometryMin, Sm]` 区间工作，使用固定上限次数的二分法。首版不使用 Newton 法，以避免初值依赖和近奇异区发散。失败时不返回最近一次迭代值作为有效结果。

## 11. 配置校验

### 11.1 基础校验

- 全部实数参数有限。
- 长度和 `Sm` 为正。
- `sigmaK/signB/tauS` 严格等于 `+1` 或 `-1`。
- 单位和公共位置约定固定，不接受运行时猜测方向。

### 11.2 几何常识与参数冲突

- 主曲肘满足 `|Lr-Lf| <= D <= Lr+Lf`。
- 固定三角形满足 `|LPF-LPK| <= Lr <= LPF+LPK`。
- `LPF^2-aP^2` 有足够正余量，拒绝退化或近共线装配。
- `|py| <= Ld` 并保留根式安全裕量。
- `Sm` 与公共轴行程、软限位和可配置目标范围不冲突。
- `xHandoff=0` 表示自动；显式值必须有限并位于 `[xGeometryMin, Sm]`。
- 如果油缸面积已配置，则相关面积必须有限且为正。
- 如果没有面积配置，仍可使用兼容的活塞速度到流量增益；但段开始前必须证明执行器侧速度能够转换为有效流量。

### 11.3 全行程验证

几何扫描先覆盖 `[0, Sm]`，但允许靠近锁紧区的一段不满足普通速度控制裕量。校验器从 `Sm` 向 `0` 搜索连续安全后缀，并自动推导最小在线位置 `xGeometryMin`：

1. 每个采样点按根式余量、归一化雅可比、驱动杆投影、`|k|` 最小值和最大值判定是否适合普通速度控制。
2. 从 `Sm` 向闭模方向扫描，找到最后一个安全到不安全的边界。
3. 使用固定次数二分法在安全侧细化边界，并加入固定几何安全裕量。
4. `[xGeometryMin, Sm]` 必须连续安全、严格单调且支路连续；安全区内部再次出现不安全点时拒绝整组配置。
5. `xHandoff=0` 时令 `xHandoffEffective=xGeometryMin`；否则要求显式值不小于 `xGeometryMin`。

每次配置提交至少检查：

- `rK` 和 `rS` 根式余量。
- `deltaJ` 和归一化条件指标。
- `px-Xs` 驱动杆水平投影。
- `k` 的有限性、最小绝对值和符号。
- `Xs(Xm)` 严格单调。
- 装配支路连续，不能在行程中切换平方根分支。
- 全部中间量和输出有限。

数学上可达但安全裕量低于配置阈值时仍拒绝提交。浮点根式只允许吸收舍入级微小负数，禁止用无条件 `max(0, radicand)` 掩盖真实不可达几何。

默认 `dc=378 mm` 时，`Xm=0` 的主曲肘距离为 `D=379.188607 mm < Lr+Lf=380 mm`，主根式余量 `rK=147.145273 mm^2`，满足主曲肘可达条件。是否允许在 `Xm=0` 执行普通速度控制仍由上述安全阈值和自动 `xGeometryMin` 决定，不能仅凭可达条件判定。

### 11.4 跨扫描校验

全行程扫描不得在一个 PLC 周期中一次完成。配置状态机每周期处理固定点数，并保存候选配置、当前索引和最差包络。这样即使被配置的轴处于空闲状态，其他运行轴的 1 ms 扫描也不会被长校验阻塞。

## 12. 控制路径改造

### 12.1 规划器职责

规划器的权威输出是模板速度 `VmCmd`。它不应知道五铰点几何。

当前规划器内部按固定 `velocityToFlowGain` 生成 `targetFlow`。实施时应提取统一的执行器映射步骤：

- 直压轴继续产生与当前实现相同的流量结果。
- 曲肘轴先把 `VmCmd` 变换为 `VsCmd`，再使用油缸面积或兼容增益换算流量。
- `maxFlow` 继续表示泵侧流量上限，在映射后执行。

### 12.2 速度闭环

模板速度误差仍使用 `VmCmd` 和 `VmActual`。首版保留现有速度控制器输出流量修正的单位契约，动态运动学流量作为 feedforward。该控制增益需要按曲肘轴重新整定并记录单位。

后续若要把速度控制器改为输出模板速度修正，必须作为独立行为变更设计，不能在本任务中暗中改变所有轴的闭环增益语义。

### 12.3 特殊路径

以下路径必须调用同一执行器映射函数，禁止继续直接使用固定 `segment->velocityToFlowGain`：

- 正常位置/速度执行。
- Stop 减速。
- P 到 V 无扰切换中的流量到模板速度回算。
- 速度段之间的 carryover。
- MoveContinuousAbsolute 的 approach/sustain 切换。
- blending 过渡。
- 仿真目标流量生成。

P 到 V 回算使用当前位置的有效动态增益：

```text
dynamicGain = cylinderVelocityToFlowGain * abs(k(Xm))
Vm = flow / dynamicGain
```

当动态增益低于安全阈值时不得执行无扰回算，应拒绝切换或安全停止。

### 12.4 执行器方向

曲肘轴应把 `actuatorDirection` 作为控制状态的一部分提供给进程层，用于阀组和油缸面积选择。模板段方向继续用于 PLCopen 位置语义，不能被覆盖。

最小必要输出包括：

- 当前 `actuatorDirection`。
- 当前 `velocityRatio`。
- 当前 `actuatorVelocityCommand`。

其余几何诊断遥测可由编译开关裁剪。

## 13. 错误处理与安全输出

新增可辨识错误类别：

- 机构类型非法。
- 曲肘配置池耗尽。
- 参数非有限或支路符号非法。
- 主曲肘三角形矛盾。
- 固定三角形矛盾或退化。
- 驱动杆不可达。
- 行程或软限位冲突。
- 全行程支路不连续或非单调。
- 奇异裕量不足。
- 运行中禁止配置。
- 实时模板位置越界。
- 实时运动学求解失败。

实时求解失败时：

1. 本周期流量和泵速安全置零。
2. 不沿用上一周期的 `k`、`Vs` 或 `Q`。
3. 轴进入 `FAULT`。
4. 诊断快照记录 `Xm`、配置版本和具体运动学错误。
5. 复位前禁止重新输出运动命令。

配置校验失败不是运行时轴故障。它只使配置 FB 返回错误，原生效配置继续保持。

## 14. 兼容性

- `MECHANISM_TYPE=0` 保持旧 `CreateMotion` 行为。
- 机构枚举只能追加，不能改变已有数值。
- 新 IEC 字段追加到结构末尾，并更新接口布局一致性测试。
- 直压轴不得分配机构槽，不执行 `sqrtf` 或运动学除法。
- 直压轴的规划速度、流量、泵速、Stop、blending 和诊断输出必须由逐周期回归测试证明未改变。
- `HYD_WriteParameter` 的现有参数编号不得重排。
- PLC demo 生成接口和测试夹具需要同步更新。

## 15. 性能策略

### 15.1 当前阶段结论

本次环境没有 STM32 ARM GCC 交叉编译工具链，也没有目标板 DWT 周期数据。因此当前阶段只能回答“在线计算在运算量上高度可行”，不能回答“STM32H743/H750 最坏情况已经实测通过 1 ms”。

按预计算 `aP/bP/invLr` 后的单周期公式，预计热路径包含：

- 约 3 次单精度平方根：`D`、`hK`、`g`。
- 约 3 到 4 次单精度除法，可通过复用倒数减少重复除法。
- 数十次乘加、绝对值、比较和有限值检查。

相对于 480 MHz 的 Cortex-M7F，这一运算量通常远低于 1 ms 的 `480,000 cycles`。即使对开方和除法使用很保守的数百周期估算，运动学本身仍预期处于数千周期量级。该推断具有较高可行性置信度，但不是编译后 WCET 证据，因为 `-Os`、libm 调用、浮点 ABI、Cache/TCM 和编译器是否生成硬件 `VSQRT/VDIV` 都会影响结果。

### 15.2 本次可执行验证

实施后扩展 PC `benchmark_performance`：

- 单点在线运动学，覆盖全行程和最差条件位置。
- 单曲肘轴完整控制扫描。
- 与直压轴完整控制扫描比较增量。
- 预热后记录 `min/mean/max`，迭代次数足以消除计时分辨率影响。
- 在 Linux 使用单调高分辨率时钟，不沿用低分辨率 `clock()` 作为唯一计时依据。
- 累积输出校验和或使用等价屏障，防止优化器删除被测计算。
- 使用发布优化配置运行，并在报告中记录编译器版本和实际编译选项。

PC 结果只用于：

- 发现数量级错误和重复计算。
- 建立持续集成性能回归基线。
- 比较优化前后相对变化。

不得按 PC 主频线性缩放后作为 STM32 WCET 结论。

### 15.3 暂定分析预算

- 在线运动学估算目标：明显低于 `48,000 cycles`，即目标 MCU 的 `100 us`。
- 单曲肘轴完整扫描的未来实测目标：低于 `240,000 cycles`，即 `500 us`。
- 整机调度的未来集成目标：低于 `384,000 cycles`，即 `800 us`，为 1 ms 保留至少 20% 裕量。

当前阶段只检查算法结构是否有能力满足第一目标，不把这些阈值标记为已通过。

### 15.4 后续目标板验证

具备交叉工具链和硬件后，必须使用真实生产配置：

```text
STM32H743/H750
Cortex-M7 480 MHz
硬件单精度 FPU
-Os
生产浮点 ABI、链接脚本和 Cache/TCM 配置
```

用 `DWT->CYCCNT` 测量全行程输入，重点覆盖最小根式余量和最小雅可比位置，并分别记录热 Cache 与冷启动样本。只有该验证通过后，才能正式宣称在线解析模式满足目标板 1 ms 周期。

## 16. 测试设计

### 16.1 数学单元测试

- 默认参数的 `Xm -> Xs` 全行程黄金数据。
- 默认参数的 `Xm -> Xs -> Xm` 往返误差。
- 解析 `k` 与中心差分结果比较。
- 开模/合模下 `Vm`、`Vs` 和方向关系。
- 端点、根式舍入容差和非有限输入。
- 每一种参数矛盾对应稳定错误码。
- 单调性、支路连续性和奇异裕量。
- `float` 目标实现与 PC `double` 参考结果比较。

精度按量纲分别定义：

- `Xs` 使用绝对误差。
- `k` 和流量使用相对误差，并设置近零分母下限。
- `Xs -> Xm` 误差必须小于轴位置容差。

具体阈值由默认参数高密度 `double` 扫描后固化，不能用一个笼统百分比覆盖所有输出。

### 16.2 配置测试

- 默认配置有效。
- 三组支路符号非法。
- 两组三角形不等式冲突。
- 驱动杆不可达。
- 行程/软限位冲突。
- 数学可达但奇异裕量不足。
- 自动 `xGeometryMin`、显式 `xHandoff` 下界和安全区连续性。
- 跨扫描 `BUSY` 和固定工作量。
- 失败提交不改变旧配置和版本。
- 运行轴拒绝配置。

### 16.3 资源与创建测试

- 直压轴不分配机构槽。
- 曲肘轴加载默认值并完成跨扫描创建。
- 机构池耗尽返回明确错误。
- 创建失败不泄漏轴槽或机构槽。
- `sizeof(HYD_MotionControlFB)` 和每槽字节数形成资源报告。
- `HYD_MAX_TOGGLE_MECHANISMS` 裁剪配置可编译并正确拒绝超额创建。

### 16.4 控制集成测试

- `MoveAbsolute`、`MoveVelocity`、`MoveContinuousAbsolute`。
- Stop、Hold/Resume、blending 和 P 到 V 切换。
- 压力闭环流量不重复应用雅可比。
- 模板位置反馈、软限位、到位和诊断保持 `Xm` 语义。
- 执行器方向由 `Vs` 而不是模板方向确定。
- 实时运动学失败同周期输出安全零并进入 `FAULT`。
- 仿真反馈仍以模板坐标积分，不把 `Vs` 误写为模板速度。

### 16.5 兼容性测试

- 旧 `CreateMotion` 零初始化路径逐周期输出不变。
- 现有直压式全部测试继续通过。
- 参数编号不变。
- IEC 结构布局检查通过。
- PLC demo 编译通过。
- 全套 `ctest` 通过。

## 17. 建议实施顺序

1. 先用现有公式建立独立 `double` 参考测试数据。
2. 实现无控制依赖的 `toggle_kinematics` 模块及单元测试。
3. 实现配置池、校验状态机和默认配置。
4. 扩展 `CreateMotion`，完成事务式资源分配。
5. 增加 IEC 配置/读取 FB 和布局测试。
6. 提取统一执行器映射边界，先锁定直压轴回归。
7. 接入曲肘位置/速度、Stop、切换和 blending 路径。
8. 增加错误码、安全输出和必要执行器方向状态。
9. 运行 PC 精度、完整测试和性能回归。
10. 后续具备工具链后执行 STM32 DWT 验证。

## 18. 验收条件

本设计对应的实现只有在以下条件全部满足时才可结束：

- 默认五铰点参数使用 `dc=378 mm`，可创建、配置、读取，并在自动推导的有效在线区间内运行。
- 曲肘轴按当前位置动态完成 `Vm -> Vs -> flow`。
- `Xm -> Xs`、`Xs -> Xm` 和解析雅可比通过参考测试。
- 所有几何矛盾和奇异风险在配置期或运行期被明确拒绝。
- 直压轴逐周期回归无行为变化。
- 运行时求解失败同周期安全置零并形成诊断。
- 静态 RAM 增量有 `sizeof` 证据。
- PC 性能基准无数量级回归。
- 文档明确 STM32 1 ms 目标板验证仍是后续未完成项，不伪造实测结论。
