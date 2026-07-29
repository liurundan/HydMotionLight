# 五点斜排双曲肘内翻锁模机构运动学与 Kv 查表设计

> PC 每周期在线解析计算、正向泵控积分和 LUT 逐点对比公式见：
> [五点斜排锁模机构 PC 在线逐周期计算模型](./2026-07-28-five-point-inclined-toggle-online-cycle-model.md)

## 1. 结论与适用范围

本文面向立式注塑机的五点斜排双曲肘内翻锁模机构，建立动模板位置 `Xm` 到合模油缸十字头位置 `Xs` 的单侧刚体运动学关系，并给出速度比、速度-流量综合系数 `Kv`、泵流量和伺服电机转速的工程换算方法。

实机拓扑按以下事实建立：

- 固定铰 `F` 与动模板铰 `M` 不在同一水平线上。
- `M` 的运动轴线与十字头 `S` 的运动轴线平行。
- `M` 的运动轴线比 `F` 更靠近 `S` 的运动轴线。
- 十字头 `S` 不直接连接肘节 `K`。
- 十字头通过驱动杆 `S-P` 连接到后曲肘刚性构件 `F-K-P` 上的偏置铰点 `P`。
- `P` 不在 `F-K` 中心线上，且偏向 `S` 的运动轴线。
- 合模油缸与十字头沿同一运动轴线直接连接，中间没有额外摇臂、齿轮或绳轮传动。

因此，`F-K-P` 必须视为一个三铰点刚性构件。此前把 `S` 直接连接到 `K`，或假设 `F、M` 共线的模型均不适用于该实机。

本文只处理无载或低载运动阶段的平面刚体运动学。销轴间隙、杆件弹性、模板变形、拉杆伸长、油液压缩和泄漏不进入几何 `Kv`；这些因素在控制和验证章节中单独处理。

## 2. 专业名称

按本文单侧机构拓扑，五个独立转动铰点为：

```text
F：固定铰
K：主曲肘肘节铰
M：动模板铰
P：后曲肘构件上的偏置驱动铰
S：十字头铰
```

因此，该机构仍应称为**五点式（五铰点）斜排双曲肘内翻锁模机构**。`P` 偏离 `F-K` 中心线并不会自动增加第六个铰点；`P` 本身就是五个铰点之一。

只有在 `P` 与 `S` 之间还存在另一个独立转动副，例如增加一根中间连杆或摇臂，才适合称为六铰点结构。动模板和十字头相对机架的直线导向属于移动副，行业“五点式”命名通常不把这些移动副计作铰点。若按严格机构学对整机上下两组杆件和全部移动副计数，运动副总数会不同，但这不是注塑机行业“五点式”的命名口径。

## 3. 坐标、正方向与符号

### 3.1 坐标系

在机构运动平面内建立右手直角坐标系：

- `x` 轴沿十字头 `S` 的运动轴线，向合模方向为正。
- `y=0` 为十字头运动轴线。
- 动模板铰 `M` 沿与 `x` 轴平行的直线运动，其高度为 `HM`。
- 固定铰 `F` 的高度为 `HF`。
- 取 `F` 在 `x` 方向的投影为零点。

实机描述“`M` 比 `F` 更靠近 `S` 轴线”对应：

\[
|H_M|<|H_F|.
\]

各点坐标定义为：

\[
F=(0,H_F),\quad M=(d,H_M),\quad K=(u,v),
\]

\[
P=(p_x,p_y),\quad S=(X_s,0).
\]

动模板工程位置采用以下约定：

- 机械闭模基准为 `Xm=0`。
- 开模方向 `Xm` 增大。
- 低载闭模基准时 `F` 到 `M` 的水平投影距离为 `dc`。

所以：

\[
d=d_c-X_m,\qquad \frac{\mathrm d d}{\mathrm d X_m}=-1.
\]

若实际电子尺方向相反，只在传感器标定层改变符号，不改变机构方程。

### 3.2 机械长度参数

| 符号 | 定义 | 单位 |
| --- | --- | --- |
| `Lr` | `F-K` 中心距，后曲肘基准边 | mm |
| `Lf` | `K-M` 中心距，前曲肘 | mm |
| `LPF` | `P-F` 中心距 | mm |
| `LPK` | `P-K` 中心距 | mm |
| `Ld` | `P-S` 中心距，十字头驱动杆 | mm |
| `HF` | `F` 到 `S` 运动轴线的有符号垂距 | mm |
| `HM` | `M` 运动轴线到 `S` 运动轴线的有符号垂距 | mm |
| `dc` | `Xm=0` 时 `F-M` 的水平投影距离 | mm |
| `Sm` | 动模板有效开模行程 | mm |

还必须记录三个装配支路符号：

- `sigmaK`：`K` 取 `F、M` 两圆交点的哪一支。
- `signB`：`P` 位于有向直线 `F→K` 的哪一侧。
- `tauS`：`S` 位于 `P` 的左侧还是右侧；图示结构通常为左侧，即 `tauS=-1`。

这些符号必须通过一组已知装配姿态确定，并随参数一起固化，不能由运行时算法临时猜测。

## 4. 位置变换推导

### 4.1 主曲肘 `F-K-M` 的闭环约束

后曲肘长度和前曲肘长度分别给出两个圆约束：

\[
u^2+(v-H_F)^2=L_r^2,
\tag{1}
\]

\[
(u-d)^2+(v-H_M)^2=L_f^2.
\tag{2}
\]

定义 `F→M` 的高度差和距离：

\[
\Delta H=H_M-H_F,
\]

\[
D=\sqrt{d^2+\Delta H^2}.
\tag{3}
\]

沿 `F→M` 的单位向量和其逆时针法向量为：

\[
\mathbf e=\left(\frac dD,\frac{\Delta H}D\right),
\]

\[
\mathbf n=\left(-\frac{\Delta H}D,\frac dD\right).
\tag{4}
\]

由两圆交点公式，`F` 到两圆公共弦垂足的距离为：

\[
a_K=\frac{L_r^2-L_f^2+D^2}{2D},
\tag{5}
\]

肘节 `K` 到 `F-M` 连线的垂距为：

\[
h_K=\sqrt{L_r^2-a_K^2}.
\tag{6}
\]

因此：

\[
K=F+a_K\mathbf e+\sigma_K h_K\mathbf n,
\tag{7}
\]

其中 `sigmaK` 为 `+1` 或 `-1`。展开后：

\[
u=\frac{a_Kd-\sigma_Kh_K\Delta H}{D},
\tag{8}
\]

\[
v=H_F+\frac{a_K\Delta H+\sigma_Kh_Kd}{D}.
\tag{9}
\]

可达条件为：

\[
|L_r-L_f|\le D\le L_r+L_f.
\tag{10}
\]

在全行程内必须保持同一装配支路。离线制表时，先在一组已测量姿态确定 `sigmaK`，随后每一点选择与前一点连续的解；禁止在行程中切换平方根符号。

### 4.2 偏置铰点 `P` 的重建

`F、K、P` 固定在同一个刚性构件上。现场直接测量三条中心距 `FK=Lr`、`FP=LPF`、`KP=LPK`，即可重建 `P`，不需要直接测偏置角。

定义沿 `F→K` 的单位向量：

\[
\mathbf e_r=\frac{K-F}{L_r}
=\left(\frac u{L_r},\frac{v-H_F}{L_r}\right)
=(e_x,e_y),
\tag{11}
\]

其逆时针法向量为：

\[
\mathbf n_r=(-e_y,e_x).
\tag{12}
\]

`F→P` 在 `F→K` 方向上的投影为：

\[
a_P=\frac{L_{PF}^2+L_r^2-L_{PK}^2}{2L_r}.
\tag{13}
\]

`P` 到 `F-K` 中心线的有符号垂距为：

\[
b_P=\operatorname{signB}\sqrt{L_{PF}^2-a_P^2}.
\tag{14}
\]

所以：

\[
P=F+a_P\mathbf e_r+b_P\mathbf n_r.
\tag{15}
\]

展开得到：

\[
p_x=\frac{a_Pu-b_P(v-H_F)}{L_r},
\tag{16}
\]

\[
p_y=H_F+\frac{a_P(v-H_F)+b_Pu}{L_r}.
\tag{17}
\]

固定三角形 `F-K-P` 的可构造条件为：

\[
|L_{PF}-L_{PK}|\le L_r\le L_{PF}+L_{PK}.
\tag{18}
\]

实机 `P` 不在 `F-K` 中心线上，因此正常情况下 `|bP|` 应明显大于测量不确定度。

### 4.3 十字头位置 `Xs`

驱动杆 `P-S` 长度为 `Ld`，且 `S` 只能沿 `y=0` 运动：

\[
(p_x-X_s)^2+p_y^2=L_d^2.
\tag{19}
\]

所以：

\[
g=\sqrt{L_d^2-p_y^2},
\tag{20}
\]

\[
X_s=p_x+\tau_Sg.
\tag{21}
\]

若 `S` 位于 `P` 左侧，则 `tauS=-1`。驱动杆可达条件为：

\[
|p_y|\le L_d.
\tag{22}
\]

联立式 (3)～(9)、(11)～(17) 和 (20)～(21)，即可得到显式位置变换：

\[
X_s=f(X_m;L_r,L_f,L_{PF},L_{PK},L_d,H_F,H_M,d_c,
\sigma_K,\operatorname{signB},\tau_S).
\tag{23}
\]

若需要与油缸安装尺寸中的“伸出量”保持一致，可增加常数偏置和方向：

\[
X_{s,\mathrm{phys}}=C_s+\sigma_cX_s.
\tag{24}
\]

`Cs` 只改变绝对位置零点，`sigmaC` 只改变方向。两者都不改变速度比的幅值和 `Kv`。

## 5. 速度关系推导

### 5.1 肘节 `K` 对 `Xm` 的导数

对式 (1) 关于 `Xm` 求导，并除以 2：

\[
u u'+(v-H_F)v'=0,
\tag{25}
\]

其中：

\[
u'=\frac{\mathrm du}{\mathrm dX_m},\qquad
v'=\frac{\mathrm dv}{\mathrm dX_m}.
\]

对式 (2) 求导。因为 `d'=dd/dXm=-1`：

\[
(u-d)(u'-d')+(v-H_M)v'=0,
\]

即：

\[
(u-d)u'+(v-H_M)v'=-(u-d).
\tag{26}
\]

写成矩阵：

\[
\begin{bmatrix}
u & v-H_F\\
u-d & v-H_M
\end{bmatrix}
\begin{bmatrix}
u'\\v'
\end{bmatrix}
=
\begin{bmatrix}
0\\-(u-d)
\end{bmatrix}.
\tag{27}
\]

定义主曲肘雅可比行列式：

\[
\Delta_J=u(v-H_M)-(u-d)(v-H_F).
\tag{28}
\]

当 `DeltaJ` 非零时，可显式求得：

\[
u'=\frac{(v-H_F)(u-d)}{\Delta_J},
\tag{29}
\]

\[
v'=-\frac{u(u-d)}{\Delta_J}.
\tag{30}
\]

当 `|DeltaJ|` 趋近零时，`F-K-M` 接近共线，机构进入速度或力传递奇异区。普通速度控制不得跨越该区域。

### 5.2 偏置铰点 `P` 的导数

`aP、bP、Lr、HF` 均为常数。对式 (16)～(17) 求导：

\[
p_x'=\frac{a_Pu'-b_Pv'}{L_r},
\tag{31}
\]

\[
p_y'=\frac{a_Pv'+b_Pu'}{L_r}.
\tag{32}
\]

偏置 `bP` 同时进入 `px'` 和 `py'`。因此，即使 `F-K-M` 主曲肘几何不变，把驱动点错误地放在 `K` 上也会得到错误的十字头速度比。

### 5.3 十字头速度导数

对式 (19) 求导：

\[
(p_x-X_s)(p_x'-X_s')+p_yp_y'=0.
\tag{33}
\]

所以：

\[
X_s'=\frac{\mathrm dX_s}{\mathrm dX_m}
=p_x'+\frac{p_yp_y'}{p_x-X_s}.
\tag{34}
\]

利用式 (20)～(21)，也可写成：

\[
X_s'=p_x'-\tau_S\frac{p_yp_y'}{g}.
\tag{35}
\]

定义动模板速度和十字头速度：

\[
V_m=\dot X_m,\qquad V_s=\dot X_s.
\]

则速度关系为：

\[
V_s=X_s'V_m,
\tag{36}
\]

\[
V_m=\frac{V_s}{X_s'}.
\tag{37}
\]

定义速度比幅值：

\[
R(X_m)=\left|\frac{\mathrm dX_s}{\mathrm dX_m}\right|
=\frac{|V_s|}{|V_m|}.
\tag{38}
\]

在本文方向约定下，开模时 `Vm>0`，十字头通常向负 `x` 方向运动，所以 `Xs'` 往往为负。控制器应把方向状态与速度比幅值分开处理，不能把 `abs()` 后的结果同时用作方向判断。

### 5.4 退化校验

该一般模型应通过两个退化测试：

1. 当 `HF=HM` 时，`F、M` 共线，式 (8)～(9) 应退化为共线两圆交点公式。
2. 当 `P` 退化到 `K`，即 `LPF=Lr、LPK=0、bP=0` 时，`px=u、py=v`，式 (34) 应退化为十字头直接连接 `K` 的旧模型。

这两个退化测试只用于验证公式和程序，不能代替实机拓扑。

## 6. 速度-流量综合系数 Kv

### 6.1 油缸有效面积

设油缸缸径为 `D`，活塞杆直径为 `dr`：

\[
A_{cap}=\frac{\pi D^2}{4},
\tag{39}
\]

\[
A_{ann}=\frac{\pi(D^2-d_r^2)}{4}.
\tag{40}
\]

必须依据实际油路确认合模时哪个腔进油。若合模使用无杆腔，则 `Ac=A_cap`；若合模使用有杆腔，则 `Ac=A_ann`。

若多只油缸机械同步且液压并联，`Ac` 应取所有参与运动油腔面积之和。若快速合模使用差动或再生回路，泵口流量与缸腔流量的关系会改变，必须根据实际回路重新推导泵流量系数；此时不能直接把单腔面积代入式 (43)。

### 6.2 Kv 定义

油缸运动学流量为：

\[
q=A_c|V_s|.
\tag{41}
\]

由式 (36) 和 (38)：

\[
q=A_cR(X_m)|V_m|.
\tag{42}
\]

定义速度-流量综合系数：

\[
K_v(X_m)=A_cR(X_m)
=A_c\left|\frac{\mathrm dX_s}{\mathrm dX_m}\right|.
\tag{43}
\]

`Ac` 的单位为 `mm^2`，速度比无量纲，所以 `Kv` 的单位也是 `mm^2`。它可解释为从动模板速度折算到油缸流量的“位置相关等效面积”。

### 6.3 流量换算

因为：

```text
1 L = 1,000,000 mm^3
1 min = 60 s
```

所以：

\[
Q_{kin}[L/min]
=0.00006K_v[mm^2]|V_m[mm/s]|.
\tag{44}
\]

如果开模与合模使用不同油缸腔，应分别定义：

\[
K_{v,close}=A_{close}R,
\qquad
K_{v,open}=A_{open}R.
\tag{45}
\]

若嵌入式只控制合模速度，存储 `Kv_close` 一张表即可。开模也需要同等精度时，可存储几何比 `R` 再乘方向面积，或利用固定面积比从 `Kv_close` 换算，不必重复存储两张相同形状的表。

## 7. 从流量到泵和电机转速

设：

- 泵排量 `Dp`，单位 `cm^3/rev`。
- 泵容积效率 `etaV`。
- 泵转速 `np`，单位 `rpm`。
- 电机转速 `nm`，单位 `rpm`。
- 传动比 `G=np/nm`；直联时 `G=1`。

泵的有效输出流量为：

\[
Q=\frac{D_pn_p\eta_v}{1000}.
\tag{46}
\]

所以：

\[
n_p=\frac{1000Q}{D_p\eta_v},
\tag{47}
\]

\[
n_m=\frac{n_p}{G}.
\tag{48}
\]

代入式 (44)：

\[
n_m[rpm]
=\frac{0.06K_v(X_m)|V_m|}{D_p\eta_vG}.
\tag{49}
\]

若电机最高转速为 `nmMax`，最大有效流量为：

\[
Q_{max}=\frac{D_p\eta_vG n_{m,max}}{1000}.
\tag{50}
\]

当前位置可实现的动模板速度上限为：

\[
V_{m,max}(X_m)=\frac{Q_{max}}{0.00006K_v(X_m)}.
\tag{51}
\]

控制器必须先按式 (51) 限制模板速度，再生成泵速，不能只把泵速硬截断而继续积累速度环积分。

泵轴扭矩也要校核。使用 `bar` 和 `cm^3/rev` 时，近似为：

\[
T_p[N\cdot m]
=\frac{\Delta p[bar]D_p[cm^3/rev]}{20\pi\eta_m}.
\tag{52}
\]

其中 `etaM` 为泵机械效率。电机额定和峰值转矩必须覆盖速度轨迹对应的压力需求。

## 8. 几何参数的现场测量

### 8.1 安全前提

测量前必须：

- 停机、泄压并执行锁定挂牌。
- 用机械支撑可靠固定动模板和十字头。
- 在低载静止状态测量，不使用高压锁模状态作为刚体尺寸基准。
- 拆除防护罩时遵守设备维护规程，测量后恢复全部防护和联锁。

### 8.2 推荐仪器

优先顺序如下：

1. 激光跟踪仪或便携式关节臂三坐标。
2. 经过校准的激光位移计、磁栅、电子水平仪和中心塞组合。
3. 现场替代：内外径千分尺、伸缩规、深度尺、百分表和专用中心距量具。

最终精度不能仅按仪器标称精度判断，应通过参数扰动计算 `Kv` 灵敏度后分配测量公差。

### 8.3 建立共同坐标基准

1. 在十字头多个静止位置采集导轨、导向套或十字头基准面的中心点，拟合 `S` 运动轴线，定义为 `y=0`。
2. 在动模板多个位置采集 `M` 销轴中心，拟合 `M` 运动轴线。
3. 校验两条轴线平行度。若不平行误差超出机械和控制误差预算，本二维平行轴模型不准入。
4. 取沿 `S` 轴线合模方向为 `+x`，其法向为 `+y`。
5. 将固定铰 `F` 投影到 `x` 方向并定义其横坐标为零。

由拟合坐标直接得到：

\[
H_F=y_F,
\qquad
H_M=\operatorname{mean}(y_{M,i}).
\]

`HF、HM` 必须是相对 `S` 轴线的有符号垂距，不能用机架外表面、油缸外圆或未校准的水平尺读数代替。

### 8.4 销轴中心的确定

对可见销轴、轴承座或同轴加工圆：

- 在圆周上采集不少于 6～12 个点拟合圆心。
- 从机构两侧分别测量，并投影到机构中平面。
- 两侧拟合中心的差异可用于发现销轴倾斜或轴承座不同轴。

没有三坐标时，可沿两圆中心连线测量：

\[
L_{center}=L_{outside}-\frac{D_1+D_2}{2},
\tag{53}
\]

或：

\[
L_{center}=L_{inside}+\frac{D_1+D_2}{2}.
\tag{54}
\]

隐藏销轴应测同轴加工的轴承座圆、销轴端面中心孔或安装图中的加工基准。不得用连杆铸造边缘反推铰点中心。

### 8.5 五条中心距

在机构低载中行程位置测量：

```text
Lr  = distance(F,K)
Lf  = distance(K,M)
LPF = distance(P,F)
LPK = distance(P,K)
Ld  = distance(P,S)
```

每个中心距至少测量三次，并从开模方向和合模方向分别接近测量位置，以暴露间隙导致的滞差。

`P` 的偏置方向由以下有向面积确定：

\[
\operatorname{signB}
=\operatorname{sign}((K-F)\times(P-F)).
\tag{55}
\]

在二维坐标中：

\[
(K-F)\times(P-F)
=u(p_y-H_F)-(v-H_F)p_x.
\]

该符号必须和长度参数一起记录。

### 8.6 `dc`、行程和电子尺标定

1. 以规定的低速低压闭模参考位置建立 `Xm=0`，禁止在高压锁模拉伸状态置零。
2. 测量该位置 `F→M` 在 `x` 轴上的投影，得到 `dc`，不是 `F-M` 的斜向欧氏距离。
3. 用外部位移基准在全行程取不少于 10 个点，拟合电子尺原始计数到 `Xm` 的比例和偏置。
4. 电子尺映射为：

```text
Xm = direction * (raw - raw0) * mmPerCount
```

5. 检查线性残差、重复性、回程滞差、温漂和安装松动。

### 8.7 无 Xs 传感器时的处理

`Kv` 只需要 `dXs/dXm`，不需要十字头绝对零点，所以没有 `Xs` 传感器不妨碍离线制表。

若需要报告油缸绝对伸出量，应使用油缸安装图、收缩长度和活塞杆安装方向确定式 (24) 中的 `Cs、sigmaC`。不得利用一个任意的泵转数点强行修改几何零点。

## 9. 液压与伺服泵参数

制表和转速换算至少需要：

| 参数 | 获取方法 | 备注 |
| --- | --- | --- |
| 缸径 `D` | 油缸制造图、活塞实测 | 不用缸筒外径代替 |
| 杆径 `dr` | 制造图或活塞杆实测 | 决定有杆腔面积 |
| 合模进油腔 | 查液压原理图并现场核对管路 | 决定 `Ac` |
| 泵排量 `Dp` | 泵铭牌和制造商数据 | 变量泵还需当前排量指令 |
| 容积效率 `etaV` | 制造商压力-转速-温度曲线或台架测试 | 不宜全工况使用单一常数 |
| 机械效率 `etaM` | 制造商曲线或台架测试 | 用于扭矩校核 |
| 传动比 `G` | 联轴器或齿轮参数 | 直联为 1 |
| 最高转速 `nmMax` | 电机和泵允许值的较小者 | 还要考虑超速时间 |
| 最大压力 | 系统设定和元件额定值 | 用于扭矩和安全校核 |
| 油温范围 | 实机记录 | 影响黏度和效率 |

### 9.1 不应塞入几何 Kv 的补偿

运动学 `Kv` 应保持纯几何和有效面积定义。总命令流量可写为：

\[
Q_{cmd}=Q_{kin}+Q_{leak}+Q_{comp}.
\tag{56}
\]

其中泄漏补偿可近似为：

\[
Q_{leak}\approx C_t\Delta p,
\tag{57}
\]

压力建立阶段的压缩流量可近似为：

\[
Q_{comp}\approx\frac{V_t}{\beta_e}\frac{\mathrm d p}{\mathrm d t}.
\tag{58}
\]

`Vt` 为等效受压容积，`betaE` 为等效体积弹性模量。式 (57)～(58) 应由压力和液压状态管理器处理；若把它们固化进 `Kv(Xm)`，同一张表将无法适应压力、油温和泄漏变化。

## 10. 离线 Kv 表生成

### 10.1 表的有效区间

设普通速度控制区间为：

\[
X_m\in[X_{table,min},X_{table,max}].
\]

通常 `XtableMax` 为最大开模位置，`XtableMin` 为进入锁模压力阶段前的交接位置 `Xhandoff`。若数学死点位于或接近 `Xm=0`，表不得外推到死点。

### 10.2 静态参数校验

生成表前逐项检查：

1. 所有长度、面积、排量和效率为有限正数。
2. 全表范围满足式 (10) 的主曲肘可达条件。
3. 固定三角形 `F-K-P` 满足式 (18)。
4. 全表范围满足式 (22) 的驱动杆可达条件。
5. `sigmaK、signB、tauS` 与已测装配姿态一致。
6. `Xs(Xm)` 连续；若设计要求单调，则全区间导数不能换号。
7. `|DeltaJ|`、`|px-Xs|` 与所有根式余量高于安全阈值。
8. 预测的十字头总行程不超过油缸机械有效行程并保留端部缓冲余量。

任何一点失败，应判整张表无效，不输出部分有效表。

### 10.3 真值点计算

对给定 `Xm`：

1. 由式 (3)～(9) 求 `K`。
2. 由式 (11)～(17) 求 `P`。
3. 由式 (20)～(21) 求 `Xs`。
4. 解式 (27)，或使用式 (29)～(30) 求 `u'、v'`。
5. 由式 (31)～(32) 求 `px'、py'`。
6. 由式 (34) 或 (35) 求 `Xs'`。
7. 由式 (38) 和 (43) 求 `R、Kv`。
8. 由式 (44)、(49)、(51) 计算参考流量、转速和当前位置速度上限。

内部计算使用双精度浮点。每个点至少保存以下离线诊断量：

```text
Xm, d, u, v, px, py, Xs,
deltaJ, dXs_dXm, R, Kv,
Q_at_reference_speed, motor_rpm_at_reference_speed,
max_template_speed
```

固件不需要保存全部诊断量。

### 10.4 解析导数的独立校验

用中心差分独立验证：

\[
X_{s,fd}'(X_m)
\approx\frac{X_s(X_m+h)-X_s(X_m-h)}{2h}.
\tag{59}
\]

端点使用二阶单边差分。可用 `h` 和 `h/2` 做 Richardson 校正：

\[
D_R=\frac{4D(h/2)-D(h)}{3}.
\tag{60}
\]

若解析导数和独立差分在非奇异区不一致，禁止发布 LUT。

### 10.5 定步长表与误差收敛

推荐使用定步长节点：

\[
X_{m,i}=X_{table,min}+i\Delta x,
\qquad i=0,1,\ldots,N-1,
\tag{61}
\]

\[
\Delta x=\frac{X_{table,max}-X_{table,min}}{N-1}.
\tag{62}
\]

从较小表开始，例如 `N=65`。对每个区间使用高密度真值点扫描线性插值误差：

\[
\varepsilon_{Kv}
=\frac{|K_{v,LUT}-K_{v,true}|}
{\max(K_{v,true},K_{v,floor})}.
\tag{63}
\]

若最大误差超过分配预算，则令：

```text
N_new = 2*N_old - 1
```

依次得到 `65、129、257、513...` 点，直到通过或超过固件资源上限。推荐把 LUT 插值误差分配为总速度误差预算的约三分之一。例如总目标为 `1.5%` 时，可先以 `0.5%` 作为 LUT 误差上限；最终阈值应由整机性能要求确定。

定步长表的优点是运行时索引为常数时间，不需要二分搜索，也不必存储每个位置节点。

### 10.6 定点量化

表生成器必须同时验证浮点插值误差和固件定点量化误差。

- `Kv` 可按项目范围缩放到 `uint32` 或 Q 格式。
- 插值差值乘法使用 64 位中间量，避免溢出。
- 量化误差应单独分配预算，例如不超过 `0.05%`。
- 不得使用会在节点处产生不连续跳变的截断策略。

### 10.7 表元数据

固件表至少包含：

```text
schemaVersion
generatorVersion
parameterHash
calibrationVersion
branchSigns
positionDirection
xMinMm
xMaxMm
deltaX
pointCount
kvScale
crc32
```

`parameterHash` 必须覆盖全部机械参数、油缸有效面积、表范围、误差预算和量化规则。CRC 覆盖元数据和全部表点。参数、电子尺标定版本或表版本任一不匹配时，控制器不得进入自动合模。

## 11. 嵌入式查表和控制流程

### 11.1 O(1) 线性插值

伪代码：

```text
lookupKv(Xm):
    if table CRC/version/hash invalid:
        fault(TABLE_INVALID)

    if Xm < xMin or Xm > xMax:
        fault(POSITION_OUT_OF_TABLE)

    z = (Xm - xMin) * invDeltaX
    i = floor(z)

    if i >= pointCount - 1:
        return Kv[pointCount - 1]

    f = z - i
    return Kv[i] + f * (Kv[i + 1] - Kv[i])
```

定点实现中，`f` 用固定小数位表示，乘加使用 64 位中间值。

### 11.2 每周期流量和转速命令

```text
controlTick(rawPosition, VmRequest, pressure, motorFeedback):
    Xm = calibrateAndValidate(rawPosition)
    Kv = lookupKv(Xm)

    Qmax = Dp * etaV * G * motorMaxRpm / 1000
    VmMax = Qmax / (0.00006 * Kv)
    VmCmd = signedMinMagnitude(VmRequest, VmMax, stageSpeedLimit)

    Qkin = 0.00006 * Kv * abs(VmCmd)
    Qcmd = boundedHydraulicCompensation(Qkin, pressure, temperature)
    motorRpm = 1000 * Qcmd / (Dp * etaV * G)

    applyDirectionStateMachine(sign(VmCmd))
    applyAccelerationLimit(motorRpm)
    applyAntiWindupAfterSaturation()
```

方向状态必须和转速幅值分开。换向时先把旧方向转速斜坡降到零，确认零速后再改变泵方向。

### 11.3 锁模压力阶段交接

当满足位置、曲肘角或雅可比阈值中的任一交接条件时：

1. 停止更新普通速度轨迹。
2. 将模板速度、流量和泵速命令按安全斜率降到零。
3. 确认泵速和模板速度进入零速窗口。
4. 再进入独立的锁模压力控制阶段。

不得在同一周期从高速位置控制直接跳到高压命令，也不得使用 LUT 外推穿越奇异区。

## 12. 验证与验收

### 12.1 数学和离线工具验证

必须包括：

- 三条主闭环残差：`FK、KM、PS`。
- 固定三角形残差：`FP、KP`。
- 解析位置与直接约束方程的一致性。
- 解析导数与中心差分、Richardson 结果的一致性。
- `HF=HM` 和 `P=K` 两个退化测试。
- 全行程支路连续、根式可达、`Xs` 连续。
- 奇异区阈值和表截止位置验证。
- 高密度真值扫描下的 LUT 最大相对误差。
- 浮点参考与固件定点插值的逐点比较。
- CRC、版本、参数哈希和越界故障注入。

### 12.2 参数不确定度传播

对每个测量参数按其不确定度做正负扰动，重新生成全行程 `Kv`：

```text
Lr, Lf, LPF, LPK, Ld,
HF, HM, dc, Xm calibration scale and zero
```

计算每个位置的 `Kv` 包络和最坏相对变化。若误差超预算，应优先提高最敏感参数的测量精度，而不是盲目增加 LUT 点数。

### 12.3 无 Xs 传感器的实机间接验证

在低压、低速、远离奇异区的条件下，可用电机编码器和泵排量间接校核几何模型：

1. 选取多个 `Xm1→Xm2` 区间。
2. 由几何模型计算：

\[
\Delta X_{s,model}=X_s(X_{m,2})-X_s(X_{m,1}).
\]

3. 计算预测油液体积：

\[
\Delta V_{model}=A_c|\Delta X_{s,model}|.
\]

4. 由电机编码器累计泵转数，估算有效泵出体积：

\[
\Delta V_{pump}\approx D_p\eta_v\Delta N_p.
\]

5. 比较各区间体积误差及其随位置的趋势。

由于 `etaV`、泄漏和油液压缩存在误差，该方法适合作为趋势和一致性校验，不应单独用于反向修改杆长。若误差只随压力或油温变化，优先检查液压效率；若误差随 `Xm` 呈稳定的位置相关形状，优先检查几何参数、支路符号和电子尺比例。

### 12.4 实机控制验收

- 低速空载验证 `Xm` 方向、泵方向和模板运动方向一致。
- 分段验证目标模板速度、实际模板速度、命令流量和电机转速。
- 验证泵速饱和时模板速度指令按式 (51) 降低，速度环无积分累积。
- 验证接近 `Xhandoff` 时平滑减速并进入压力阶段。
- 验证传感器断线、表损坏、参数版本不匹配和越界时进入安全状态。
- 在最低和最高工作油温下重复关键速度点。

## 13. 制表记录模板

### 13.1 机械测量记录

```text
Machine ID:
Measurement date / temperature:
Instrument / calibration date:

Lr = FK:
Lf = KM:
LPF = PF:
LPK = PK:
Ld = PS:
HF:
HM:
dc:
Sm:

sigmaK:
signB:
tauS:

Xm raw0:
Xm mmPerCount:
Xm direction:
Xm linearity / hysteresis:
```

### 13.2 液压记录

```text
Cylinder bore D:
Rod diameter dr:
Closing inlet chamber:
Closing effective area Ac:

Pump displacement Dp:
Volumetric efficiency source/map:
Mechanical efficiency source/map:
Gear ratio G:
Motor/pump maximum speed:
Maximum working pressure:
Oil temperature range:
```

### 13.3 LUT 发布记录

```text
xMin / xMax / deltaX / pointCount:
Maximum floating interpolation error:
Maximum fixed-point error:
Analytic-vs-numerical derivative error:
Minimum main Jacobian margin:
Minimum drive-link tangent margin:
Maximum predicted motor rpm:
Parameter hash:
Calibration version:
CRC32:
Generator version:
Reviewer / approval date:
```

## 14. 对当前仿真代码的影响

当前工作区的 `toggle_clamp_sim_core.js` 使用旧的 `S-K` 直连和单一铰高模型。它不能作为本文实机的运动学真值，也不能生成可下装到实机的 `Kv` 表。

若后续实施，需要至少修改：

- 机械配置：增加 `LPF、LPK、HF、HM` 和支路符号。
- 位置求解：由 `S-K` 约束改为 `F-K-P` 重建加 `S-P` 约束。
- 速度比：改为本文式 (27)、(31)、(32)、(34)。
- 参数校验、哈希、表生成、渲染和测试基准。

在这些修改完成并通过本文验证项前，现有页面只能作为旧简化模型演示，不能用于本实机伺服泵参数标定。

## 15. 最终工程建议

1. 用便携式三坐标或激光跟踪仪建立 `S` 和 `M` 两条平行轴线及五个铰点坐标。
2. 以中心距 `FK、KM、FP、KP、PS` 和高度 `HF、HM` 建立一般五点斜排模型。
3. 用两圆交点求 `K`，用固定三角形求 `P`，用驱动杆与 `S` 轴线交点求 `Xs`。
4. 用闭环雅可比解析求 `dXs/dXm`，并以数值差分独立验证。
5. 定义 `Kv=Ac*|dXs/dXm|`，离线生成定步长一维表。
6. 从 65 点开始逐次加密，通过高密度扫描确定实际点数。
7. 嵌入式用 `Xm` 做 O(1) 索引和线性插值，再计算流量和伺服电机转速。
8. 在几何奇异区前结束速度表并平滑交接到锁模压力控制。
9. 用参数不确定度传播和泵转数体积积分完成无 `Xs` 传感器条件下的间接校核。

在实机几何参数尚未测量前，不应输出看似精确的数值 `Kv` 表。数值制表的有效性取决于铰点中心测量、装配支路符号、电子尺标定和液压有效面积是否真实可靠。
