# 五点斜排锁模机构 PC 在线逐周期计算模型

## 1. 目的

本文给出五点斜排双曲肘内翻锁模机构的在线计算公式，用于 PC 仿真中每个采样周期直接求解几何、速度比、`Kv`、油缸流量和伺服电机转速，并与离线 `Xm→Kv` 查表结果逐点比较。

本文使用的实机拓扑为：

```text
主曲肘：F-K-M
三铰点刚性构件：F-K-P
十字头驱动杆：S-P
M 与 S 的运动轴线平行
P 偏离 F-K 中心线并靠近 S 运动轴线
```

完整参数定义、测量方法和 LUT 生成方法见：

[五点斜排双曲肘内翻锁模机构运动学与 Kv 查表设计](./2026-07-28-five-point-inclined-toggle-kinematics-design.md)

在线模型有两种用途：

1. **反算模式**：给定当前 `Xm` 和模板速度 `Vm`，在线计算 `Xs、Vs、Kv、Q、np、nm`。
2. **正向仿真模式**：给定泵或电机实际转速，计算油缸速度、模板速度，并积分得到下一周期 `Xm`。

第一种模式适合生成真值并验证 LUT；第二种模式适合验证伺服泵速度指令、泵响应和位置轨迹。

## 2. 符号与周期状态

### 2.1 方向约定

- `x` 轴沿十字头 `S` 运动轴线，合模方向为正。
- 动模板机械闭模位置为 `Xm=0`。
- `Xm` 沿开模方向增大。
- `Xs` 是十字头铰点 `S` 的几何横坐标。
- 合模时通常 `Vm<0、Vs>0`，因此 `k=dXs/dXm` 通常为负。
- 有符号泵流量 `Q>0` 定义为使 `Xs` 增大的流量。

若实际泵或电机正方向与上述定义相反，在驱动接口增加固定方向因子 `sigmaPump`，不要修改机构方程。

### 2.2 每周期输入

```text
Ts              采样周期，s
Xm[k]           当前模板位置，mm
VmCmd[k]        模板速度命令，mm/s，开模为正
npActual[k]     泵实际有符号转速，rpm
pressure[k]     当前压力，可选
temperature[k]  油温，可选
```

### 2.3 常量参数

```text
Lr, Lf, LPF, LPK, Ld
HF, HM, dc
sigmaK, signB, tauS
AcClose, AcOpen
Dp, etaV, G
motorMaxRpm, pumpTimeConstant
xMin, xMax, xHandoff
```
锁模结构的默认曲肘机械参数是Lr=150mm,Lf=230mm,LPF=135mm,LPK=75mm,Ld=60mm,HF=130mm,HM=100mm,dc=378mm,Sm=202mm,sigmaK=-1,signB=-1,tauS=-1。

`xGeometryMin` 由根式余量、归一化雅可比、驱动杆投影和最大速度比阈值自动推导。`xHandoff=0` 表示使用该自动下界；显式配置 `xHandoff` 时必须满足 `xHandoff>=xGeometryMin`。普通位置/速度控制只允许在 `[xHandoff,Sm]` 内运行，靠近锁紧区后交给独立压力阶段。

其中 `G=np/nm`，直联时 `G=1`。

## 3. 每周期在线位置求解

下面所有式子均在当前周期 `k`、当前位置 `Xm=Xm[k]` 上计算。PC 使用双精度浮点。

### 3.1 动模板铰 M

\[
d=d_c-X_m,
\tag{1}
\]

\[
F=(0,H_F),\qquad M=(d,H_M).
\tag{2}
\]

定义：

\[
\Delta H=H_M-H_F,
\tag{3}
\]

\[
D=\sqrt{d^2+\Delta H^2}.
\tag{4}
\]

### 3.2 肘节 K

\[
a_K=\frac{L_r^2-L_f^2+D^2}{2D},
\tag{5}
\]

\[
r_K=L_r^2-a_K^2,
\tag{6}
\]

\[
h_K=\sqrt{r_K}.
\tag{7}
\]

`rK` 是第一个必须检查的根式余量。随后：

\[
u=\frac{a_Kd-\sigma_Kh_K\Delta H}{D},
\tag{8}
\]

\[
v=H_F+\frac{a_K\Delta H+\sigma_Kh_Kd}{D}.
\tag{9}
\]

### 3.3 偏置铰点 P

`F-K-P` 是刚性三角形，因此以下量可在初始化时预计算：

\[
a_P=\frac{L_{PF}^2+L_r^2-L_{PK}^2}{2L_r},
\tag{10}
\]

\[
b_P=\operatorname{signB}
\sqrt{L_{PF}^2-a_P^2}.
\tag{11}
\]

每个周期只计算：

\[
p_x=\frac{a_Pu-b_P(v-H_F)}{L_r},
\tag{12}
\]

\[
p_y=H_F+\frac{a_P(v-H_F)+b_Pu}{L_r}.
\tag{13}
\]

### 3.4 十字头 S

\[
r_S=L_d^2-p_y^2,
\tag{14}
\]

\[
g=\sqrt{r_S},
\tag{15}
\]

\[
X_s=p_x+\tau_Sg.
\tag{16}
\]

图示装配中 `S` 位于 `P` 左侧时，`tauS=-1`。

## 4. 每周期在线速度雅可比

### 4.1 主曲肘雅可比

定义：

\[
\Delta_J=u(v-H_M)-(u-d)(v-H_F).
\tag{17}
\]

在线解析导数为：

\[
u'=\frac{\mathrm du}{\mathrm dX_m}
=\frac{(v-H_F)(u-d)}{\Delta_J},
\tag{18}
\]

\[
v'=\frac{\mathrm dv}{\mathrm dX_m}
=-\frac{u(u-d)}{\Delta_J}.
\tag{19}
\]

### 4.2 偏置点速度导数

\[
p_x'=\frac{a_Pu'-b_Pv'}{L_r},
\tag{20}
\]

\[
p_y'=\frac{a_Pv'+b_Pu'}{L_r}.
\tag{21}
\]

### 4.3 十字头速度导数

\[
k(X_m)=\frac{\mathrm dX_s}{\mathrm dX_m}
=p_x'+\frac{p_yp_y'}{p_x-X_s},
\tag{22}
\]

也可写成：

\[
k(X_m)=p_x'-\tau_S\frac{p_yp_y'}{g}.
\tag{23}
\]

两式在线结果应一致。然后：

\[
V_s=k(X_m)V_m.
\tag{24}
\]

定义速度比幅值：

\[
R(X_m)=|k(X_m)|.
\tag{25}
\]

## 5. 实时 Kv、流量与转速反算

### 5.1 方向相关有效面积

根据 `Vs` 的方向和实际油路选择进油腔面积：

\[
A_c=
\begin{cases}
A_{close}, & V_s\text{ 为合模方向},\\
A_{open}, & V_s\text{ 为开模方向}.
\end{cases}
\tag{26}
\]

若只验证合模过程，始终使用已核实的 `AcClose`。

### 5.2 在线 Kv

\[
K_{v,RT}(X_m)=A_c|k(X_m)|.
\tag{27}
\]

该值是 PC 每周期在线解析真值，可直接与 LUT 插值值比较。

### 5.3 有符号运动学流量

若 `Q>0` 定义为使 `Xs` 增大，则不需要另设方向判断：

\[
Q_{kin}[L/min]
=0.00006A_ck(X_m)V_m.
\tag{28}
\]

幅值形式为：

\[
|Q_{kin}|=0.00006K_{v,RT}|V_m|.
\tag{29}
\]

式 (28) 用于 PC 有符号仿真，式 (29) 用于核对 `Kv` 定义。

### 5.4 泵和电机转速命令

有符号泵转速命令为：

\[
n_{p,cmd}
=\frac{1000Q_{cmd}}{D_p\eta_v},
\tag{30}
\]

\[
n_{m,cmd}=\frac{n_{p,cmd}}{G}.
\tag{31}
\]

纯运动学验证时：

\[
Q_{cmd}=Q_{kin}.
\]

需要液压补偿时：

\[
Q_{cmd}=Q_{kin}+Q_{leak}+Q_{comp}.
\tag{32}
\]

泄漏和压缩补偿不进入 `Kv`。

## 6. LUT 与在线解析结果逐周期比较

设定步长 LUT 范围为 `[xMin,xMax]`，节点间距为 `deltaX`。

`KvLUT` 与 `KvRT` 必须采用相同方向的油缸有效面积。若固件只存 `KvClose`，则只在合模周期直接比较；开模周期应先按 `AcOpen/AcClose` 缩放，或改用单独的开模表。

\[
z=\frac{X_m-x_{min}}{\Delta x},
\tag{33}
\]

\[
i=\lfloor z\rfloor,\qquad f=z-i.
\tag{34}
\]

\[
K_{v,LUT}=K_v[i]+f(K_v[i+1]-K_v[i]).
\tag{35}
\]

每周期记录：

\[
e_{Kv}=K_{v,LUT}-K_{v,RT},
\tag{36}
\]

\[
e_{Kv,rel}
=\frac{|e_{Kv}|}{\max(K_{v,RT},K_{v,floor})}.
\tag{37}
\]

同一周期还可比较流量和转速：

\[
e_Q=Q_{LUT}-Q_{RT},
\qquad
e_n=n_{LUT}-n_{RT}.
\tag{38}
\]

仿真结束输出：

```text
maxAbsKvError
maxRelativeKvError
rmsRelativeKvError
positionAtWorstKvError
maxFlowError
maxMotorSpeedError
```

## 7. 正向泵控运动仿真

反算模式验证“目标模板速度需要多少泵速”。正向模式验证“实际泵速最终产生多少模板速度和位移”。

### 7.1 泵实际转速动态

最简一阶泵/电机速度响应：

\[
\dot n_p=\frac{n_{p,cmd}-n_p}{\tau_p}.
\tag{39}
\]

推荐使用精确离散形式：

\[
n_p[k+1]
=n_{p,cmd}[k]
+(n_p[k]-n_{p,cmd}[k])e^{-T_s/\tau_p}.
\tag{40}
\]

再施加每周期加速度约束：

\[
|n_p[k+1]-n_p[k]|
\le a_{p,max}T_s.
\tag{41}
\]

若有实测电机转速反馈，可直接使用实测 `npActual=G*nmActual`，跳过式 (39)～(41)。

### 7.2 泵流量

\[
Q_p[k]=\frac{D_p\eta_v[k]n_p[k]}{1000}.
\tag{42}
\]

`etaV[k]` 可取常数，也可由压力、转速和油温二维或三维表得到：

\[
\eta_v[k]=f_{\eta}(p[k],|n_p[k]|,T[k]).
\tag{43}
\]

### 7.3 油缸速度

忽略油液压缩和外泄漏时：

\[
V_s[k]
=\frac{Q_p[k]\times10^6}{60A_c[k]}.
\tag{44}
\]

若使用显式泄漏流量：

\[
V_s[k]
=\frac{(Q_p[k]-Q_{leak}[k])\times10^6}
{60A_c[k]}.
\tag{45}
\]

### 7.4 模板速度

由在线几何雅可比：

\[
V_m[k]=\frac{V_s[k]}{k(X_m[k])}.
\tag{46}
\]

当 `|k|` 小于安全阈值，或主雅可比进入奇异区时，式 (46) 不得继续使用，应停止普通运动仿真并进入保护或压力阶段。

### 7.5 位置积分

显式 Euler：

\[
X_m[k+1]=X_m[k]+T_sV_m[k].
\tag{47}
\]

推荐 PC 验证使用梯形积分：

\[
X_m[k+1]
=X_m[k]+\frac{T_s}{2}(V_m[k]+V_m[k+1]).
\tag{48}
\]

式 (48) 可用预测-校正实现：

1. 用式 (47) 得到预测位置 `XmPred`。
2. 在 `XmPred` 重新计算在线几何和 `kPred`。
3. 计算预测模板速度 `VmPred=VsNext/kPred`。
4. 用 `XmNext=Xm+0.5*Ts*(Vm+VmPred)` 修正。

减半 `Ts` 后轨迹应收敛；若差异明显，说明采样周期过大或接近强非线性区。

## 8. 实测 Xm 的在线速度估计

若 PC 仿真读取实机电子尺采样，而不是内部积分位置，可用二阶后向差分：

\[
V_{m,raw}[k]
=\frac{3X_m[k]-4X_m[k-1]+X_m[k-2]}{2T_s}.
\tag{49}
\]

电子尺噪声会被微分放大，建议再使用低通滤波：

\[
V_{m,f}[k]
=\alpha V_{m,f}[k-1]+(1-\alpha)V_{m,raw}[k],
\tag{50}
\]

\[
\alpha=e^{-2\pi f_cT_s}.
\tag{51}
\]

截止频率 `fc` 应高于模板速度轨迹主要频率，同时低于电子尺噪声主频。在线解析 `Kv` 本身只依赖位置，不应使用带噪速度参与几何求解。

## 9. 可选液压压力动态

只验证运动学和 LUT 时，不需要本节。若要观察压力建立和锁模阶段，可增加腔压连续性方程。

以进油腔为例：

\[
\dot p_A
=\frac{\beta_e}{V_A(X_s)}
\left(Q_A-A_AV_s-C_t(p_A-p_B)\right).
\tag{52}
\]

回油腔为：

\[
\dot p_B
=\frac{\beta_e}{V_B(X_s)}
\left(A_BV_s-Q_B+C_t(p_A-p_B)\right).
\tag{53}

油缸净推力：

\[
F_s=A_Ap_A-A_Bp_B-F_{friction}.
\tag{54}
\]

按虚功关系，机构传到模板方向的广义力为：

\[
F_m=F_s\frac{\mathrm dX_s}{\mathrm dX_m}
=F_sk(X_m).
\tag{55}
\]

式 (52)～(55) 需要油腔初始容积、等效体积弹性模量、泄漏系数、摩擦和模板等效质量等附加参数。没有这些参数时，不应把压力动态仿真结果当成实机定量结论。

## 10. 在线数值保护

### 10.1 初始化校验

初始化时检查：

```text
all parameters finite
all lengths and areas positive
abs(Lr-Lf) <= D <= Lr+Lf over valid stroke
abs(LPF-LPK) <= Lr <= LPF+LPK
branch signs are exactly +1 or -1
sensor and model position ranges agree
```

### 10.2 根式处理

对于 `rK` 和 `rS`：

```text
if radicand < -radicandTolerance:
    fault(GEOMETRY_UNREACHABLE)
else:
    radicand = max(0, radicand)
```

只允许吸收浮点舍入导致的微小负数，不能用 `max(0,...)` 掩盖真实不可达几何。

### 10.3 奇异性保护

每周期检查：

```text
abs(deltaJ) > mainJacobianMinimum
abs(px-Xs) > driveProjectionMinimum
abs(k) > velocityRatioMinimum
all outputs finite
```

还应记录无量纲条件指标，避免阈值随机器尺寸失真。例如：

\[
c_J=\frac{|\Delta_J|}{L_rL_f}.
\tag{56}
\]

保护阈值通过设计裕量和数值试验确定。

### 10.4 周期越界

若积分预测跨越 `[xMin,xMax]`：

- 先计算到边界的剩余运动时间。
- 把位置钳位到边界。
- 将继续越界方向的速度和泵流量置零。
- 不允许使用表外或模型外位置继续计算。

## 11. 每周期在线算法

### 11.1 反算验证模式

```text
initialize(config):
    validateStaticParameters(config)
    precompute aP, bP
    loadAndValidateLut()

for each cycle k:
    validate Xm[k] and Ts

    geom = solveGeometryOnline(Xm[k])
    # geom: d, u, v, px, py, Xs, deltaJ

    jac = solveJacobianOnline(geom)
    # jac: uPrime, vPrime, pxPrime, pyPrime, k

    Ac = selectCylinderArea(sign(jac.k * VmCmd[k]))
    KvRT = Ac * abs(jac.k)
    QRT = 0.00006 * Ac * jac.k * VmCmd[k]
    npRT = 1000 * QRT / (Dp * etaV)
    nmRT = npRT / G

    KvLut = lookupKv(Xm[k])
    QLut = direction(QRT) * 0.00006 * KvLut * abs(VmCmd[k])
    nmLut = 1000 * QLut / (Dp * etaV * G)

    record geometry, KvRT, KvLut, errors, Q, rpm
```

### 11.2 正向泵控模式

```text
initialize state Xm, npActual

for each cycle k:
    geom = solveGeometryOnline(Xm[k])
    jac = solveJacobianOnline(geom)

    KvRT = selectedArea * abs(jac.k)
    VmMax = pumpFlowMax / (0.00006 * KvRT)
    VmCmd = clampRequestedTemplateSpeed(VmRequest, VmMax)

    QCmd = 0.00006 * selectedArea * jac.k * VmCmd
    npCmd = 1000 * QCmd / (Dp * etaV)
    npActual = updatePumpSpeed(npActual, npCmd, Ts)

    QActual = Dp * etaV * npActual / 1000
    VsActual = QActual * 1e6 / (60 * selectedArea)
    VmActual = VsActual / jac.k

    XmNext = predictorCorrectorIntegrate(Xm, VmActual, Ts)
    enforceTravelAndHandoff(XmNext)

    KvLut = lookupKv(Xm)
    record online-vs-LUT and trajectory data
```

## 12. PC 仿真输出曲线

至少输出以下时间曲线：

```text
Xm, VmCmd, VmActual
Xs, Vs
u, v, px, py
k=dXs/dXm
KvRT, KvLut, relativeKvError
QCmd, QActual
npCmd, npActual, nmCmd
deltaJ, driveProjection=px-Xs
stage, saturation, fault flags
```

还应输出位置域曲线：

```text
Xs(Xm)
k(Xm)
KvRT(Xm)
KvLut(Xm)
relativeKvError(Xm)
VmMax(Xm)
```

位置域曲线最容易发现支路切换、符号错误、奇异点和 LUT 局部欠采样。

## 13. 验证判据

在线计算实现至少满足：

1. 五条长度残差 `FK、KM、FP、KP、PS` 接近机器精度。
2. 式 (22) 与式 (23) 的 `k` 一致。
3. 解析 `k` 与中心差分 `dXs/dXm` 一致。
4. 反算得到的泵速再经正向公式计算，应恢复原模板速度。
5. `Ts` 减半后，积分位置轨迹按数值方法阶次收敛。
6. 全有效区间 `KvLut` 相对 `KvRT` 不超过 LUT 误差预算。
7. 越界、不可达根式、雅可比奇异和非有限数均产生明确故障，不产生静默钳位。

反算与正向回代的一致性可写成：

\[
V_m
\xrightarrow{k}
V_s
\xrightarrow{A_c}
Q
\xrightarrow{D_p,\eta_v}
n_p
\xrightarrow{D_p,\eta_v}
Q
\xrightarrow{A_c}
V_s
\xrightarrow{1/k}
\hat V_m.
\tag{57}
\]

在无饱和、无补偿和相同效率条件下，应有：

\[
\hat V_m=V_m
\]

至浮点舍入精度。

## 14. 实施建议

- PC 在线真值计算和离线 LUT 生成应调用同一套纯几何函数，避免公式出现两份不同实现。
- LUT 验证必须使用独立路径，例如在线解析结果对 LUT 插值，而不是用 LUT 生成节点自身验证。
- PC 默认使用双精度；固件定点或单精度结果作为第三条路径比较。
- 先完成无压力的运动学闭环验证，再增加泵动态和压力动态，便于隔离错误来源。
- 在线公式适合 PC 仿真和离线验证；量产嵌入式仍推荐使用一维 LUT，以减少周期时间、分支和异常处理复杂度。
