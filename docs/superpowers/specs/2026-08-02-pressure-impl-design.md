# 压力闭环控制工程实施方案（PI+FF 基线 · RBF 监督 · 电机位置脉动在线补偿）

> 目标平台：**STM32H7**（Cortex-M7，480 MHz，带单精度 FPU + I/D-Cache，片上 SRAM 约 1 MB 级但资源有限）
> 适用约束：纯 C99、`HYD_REAL = float`（单精度）、**禁止动态分配**、热路径不引入 `double`/`sinf`/`sqrtf`
> 关联：`docs/pressure_control_analysis.md`（RBF vs PI 数据分析）、`tests/sim_pressure_control.c`（算法仿真）

---

## 0. STM32H7 优化总纲（贯穿全部设计）

| 维度 | 约束 / 做法 |
|---|---|
| 数值类型 | 全程 `float`（`HYD_REAL`）。所有常量加 `f` 后缀（如 `60.0f`）；禁止 `double` 字面量以免触发软浮点。 |
| 动态内存 | **零 `malloc`/`calloc`**。所有新状态放进预分配结构体（`HYD_PressureControllerState` / 新增 `HYD_PressureRippleCompState`），随 FB 静态存在。 |
| 热路径（每控制周期） | 只允许加/减/乘/查表/整型索引；**禁用 `sinf`/`cosf`/`sqrtf`/`expf`/`logf`**。三角函数仅在「稳态期低频刷新」时调用。 |
| 脉动补偿热路径 | 相位分箱 → `int` 索引 → `LUT[bin]` 读出，**O(1)、无循环、无分支（除总开关）**。 |
| 相位累加 | `theta` 保持在 `[0,1)`（整圈归一化），用 `theta -= (float)(int)theta` 包裹，避免大数精度丢失。 |
| 除法 | 仅出现在**段切换/初始化**（如 `targetPressure/systemGain`），每周期零除法。 |
| 缓存友好 | LUT 与累加器为连续 `float[NBINS]` 小数组，命中 D-Cache；结构体字段按访问频率排列（可选 `__attribute__((aligned))`）。 |
| 编译器 | 目标用 arm-none-eabi-gcc，建议 `-O2 -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard`；新代码须 `-O2 -Werror` 干净。mingw 预设仅用于 PC 仿真与 CI。 |
| 控制周期假设 | 1–10 kHz（0.1–1 ms）。480 MHz 下每周期约 4.8 万–48 万周期，本方案热路径增量 < 50 周期，**余量极大**。 |

---

## 1. 总体架构（三层解耦，每层可独立旁路）

```
段配方(segment)
   │  targetPressure, systemGain, targetFlow, pressureController, rippleCompEnable
   ▼
[① PI+FF 基线]  u_PI = FF + Kp·e + Ki·∫e            ← 确定性跟踪，含前馈
   │  （RBF 监督模式时，Kp/Ki 由 ② 在线修正）
   ▼
[② RBF 监督整定层]  仅更新/输出 adaptiveKp/Ki/Jacobian   ← 复用现有 RBF_PID_Update
   │  输出 = ① 的 PI 律（监督模式）或 纯 RBF 律（原模式）
   ▼
[③ 脉动补偿器]  rippleFF(θ)  ──► 叠加进 feedforwardFlow   ← 电机/泵相位 → LUT
   ▼
PumpConverter → 伺服泵+油缸 → 数字压力传感器(油泵出口)
   ▲                                              │
   └──────────── measuredPressure / motorAngleRev ┘
```

- **③ 关闭**（全局总闸或段级 `rippleCompEnable` 任一关）→ `rippleFF = 0`，零计算。
- **② 关闭**（段选 `PI` 或 `RBF_PI/RBF_PID` 原模式）→ 走既有路径，不引入新逻辑。

---

## 2. 需求①：PI+FF 基线 + FF 在线标定（消除实机稳态误差）

### 2.1 根因（来自代码）
- `motion_control.c:2124`：`pressureInput.feedforwardFlow = segment->targetFlow`。`targetFlow` 是配方人工值。
- 实机稳态保持流量 `flow* = targetPressure / systemGain`，其中 `systemGain = K_bar_per_rpm · flow2speed`（即 bar/(L/min)）。配方 `targetFlow` 与 `flow*` 偏差 → 只能靠 PI 积分慢慢补 → 表现为稳态误差/收敛慢。
- 注意：纯 `RBF_PI/RBF_PID` 路径（`pressure_controller.c` 的 RBF 分支）**丢弃 `feedforwardFlow`**（靠自适应补），所以纯 RBF 看似没问题；但 PI 基线会暴露该缺陷。新「PI+RBF 监督」模式必须把 FF 加回去。

### 2.2 做法（每周期成本：1 次除法*仅段切换* + 稳态期极轻量更新）
1. **物理推导初值**（段启动 / `systemGain` 变化时，仅算一次）：
   ```
   ffBase = (segment->systemGain > HYD_REAL_EPSILON) ? targetPressure / segment->systemGain : segment->targetFlow;
   ```
2. **在线微调偏置 `ffTrim`**（稳态保压闸门内缓更新，学出 `systemGain` 估算之外的残差，如油温/泄漏漂移）：
   ```
   gate = 稳态闸门(见 §4.3) 且 |eP| 方差小;
   if (gate) ffTrim += clamp(K_trim * eP, -ffTrimRate, +ffTrimRate);   // 指数缓动，K_trim 小、限幅
   effectiveFF = ffBase + ffTrim;
   ```
   - `ffTrim` 与 `ffBase` 存于 `HYD_PressureControllerState`（新增 2×`float` = 8 B）。
   - 稳态闸门外 `ffTrim` **冻结**，避免瞬态污染。
3. **向后兼容**：`systemGain == 0` 时回退到现有 `targetFlow` 行为，旧配方不受影响。

### 2.3 落点
- `motion_control.c` 压力分支：算 `effectiveFF`，写入 `pressureInput.feedforwardFlow`。
- `pressure_controller.c` / `common_types.h`：新增 `ffBase`、`ffTrim` 状态字段；稳态闸门状态位。

---

## 3. 需求②：PI+FF 与 RBF 融合（RBF 监督整定 PI）

### 3.1 新增控制器类型
`HYD_PressureControllerType` 增加 `HYD_PRESSURE_CONTROLLER_PI_RBF`（PI + RBF 监督）。

### 3.2 融合机制（复用现有 RBF，零新增学习算法）
每周期仍调用 `RBF_PID_Update(&rbf, target, measured)` 让其**自适应出 `adaptiveKp/Ki/Jacobian`**（现有机制），但**输出改用 PI 律并叠加 FF**：
```
// 监督模式
Kp_eff = clamp(rbf.adaptiveKp,  segment->minKp, segment->maxKp);
Ki_eff = clamp(rbf.adaptiveKi,  segment->minKi, segment->maxKi);
u = effectiveFF + Kp_eff*eP + Ki_eff*∫eP;     // ← 补回 FF（纯 RBF 模式丢弃）
```
- **保留 PI 的确定性、抗积分饱和、流量滤波**；获得 RBF 的在线增益整定与 `systemGain` 补偿。
- 既有 `PI` / `RBF_PI` / `RBF_PID` 仍可选，语义不变。
- 成本：RBF 更新本身（现有，~5–9 个神经元、少量 `expf`，仅初始化/刷新期）→ PI 律（加乘）→ 与纯 RBF 相比热路径增量极小。

### 3.3 三类动作映射（结合立式注塑机）
| 动作 | 推荐控制器 | 脉动补偿 | 说明 |
|---|---|---|---|
| 合模（高压保压） | `PI_RBF` + FF | 开 | 高压稳态，RBF 自适应抗油温漂移，脉动补偿降纹波 |
| 射胶（保压段） | `PI_RBF` + FF | 开（低速段更重要） | 低速时脉动周期长、振幅占比高，补偿收益最大 |
| 顶针 / 低速辅助 | 轻量 `PI` + FF | 关（或按需） | 动作快、压力低，关补偿省算力、避免误补偿 |

---

## 4. 需求③：电机位置前馈在线标定（脉动抵消）—— STM32 优化核心

### 4.1 新增模块（独立、可单测）
- `include/pressure_ripple_comp.h` + `src/pressure_ripple_comp.c`（新增，约 200 行）
- 状态 `HYD_PressureRippleCompState`（全部静态预分配）：

```c
#define HYD_RIPPLE_NBINS 26u   /* 齿相位分箱数，= 2×齿数(13) 以保留基波+2次谐波信息 */
typedef struct {
    float theta;                       /* 泵轴归一化转角 [0,1)，每周期更新 */
    float lut[HYD_RIPPLE_NBINS];       /* 脉动前馈 LUT（单位：L/min），热路径只读 */
    float binSum[HYD_RIPPLE_NBINS];    /* 相位平均累加器（刷新期用） */
    uint16_t binCount[HYD_RIPPLE_NBINS];/* 每箱样本数 */
    float ffGain;                      /* = 1/systemGain 或可调 K_comp，压力→流量换算 */
    uint8_t refreshTick;               /* 刷新节拍计数 */
    uint8_t calibSamples;              /* 已积累样本（达到阈值才刷新） */
    uint8_t enabled;                   /* 段级开关镜像 */
} HYD_PressureRippleCompState;
/* RAM 占用 ≈ 26*4*2 + 26*2 + 4*3 + 3 ≈ 270 B，极小 */
```

### 4.2 相位确定（数据源，优先编码器）
- **优先**：`HYD_AxisRef.motorAngleRev`（新增 `float`，单位：泵轴整圈数，调用方每周期喂入真实编码器角度÷减速比；默认 0 向后兼容）。
- **回退**（无编码器）：本控转速相位累加 `theta += Z · rpm · dt / 60.0f`，用上周期 `STATE.commandedPumpSpeed`（滞后 1 拍，稳态保压下可忽略）。
- 统一包裹到 `[0,1)`：`theta -= (float)(int)theta`。
- **注意**：齿轮泵齿数 `Z` 来自 `PressureModel` 既有约定 `Z=13`；作为 `segment` 或全局常量（`HYD_PUMP_TEETH`）。

### 4.3 脉动信号提取（数据分析流水线，阶梯化、热路径无三角）
1. **同步**：每周期取 `eP = measuredPressure - targetPressure`（AC 分量，含脉动）。
2. **相位分箱**（热路径，O(1)）：
   ```
   bin = (uint8_t)(theta * (float)HYD_RIPPLE_NBINS);   /* theta∈[0,1) → 0..NBINS-1 */
   ```
3. **稳态闸门**（决定何时学习）：压力已稳定（|eP| 小且方差小、速度已稳定）才累加，避免瞬态污染 LUT。闸门逻辑复用 §2.2 的稳态判定。
4. **增量分箱平均**（替代环形缓冲，省 RAM）：稳态期内 `binSum[bin] += eP; binCount[bin]++;`。
5. **低频刷新 LUT**（仅刷新节拍，如每 256 周期或样本足够）：
   ```
   for b in 0..NBINS-1:
       if binCount[b] > MIN_CNT:  lut[b] = -ffGain * (binSum[b] / binCount[b]);  /* 反相抵消 */
       binSum[b]=0; binCount[b]=0;
   ```
   - **热路径完全不含 `sinf`**；仅刷新时跑 `NBINS` 次除法（≈26 次/刷新，刷新频率极低）。
6. **（可选）幅相提取**：若需 HMI 显示振幅/相位，对 `lut[]` 做 Goertzel（齿频基波，可选 2 次谐波）得 `A, φ`——**仅在刷新时跑一次**，不进热路径。默认可关，以省算力。

### 4.4 热路径补偿输出（每周期，O(1)）
```
rippleFF = enabled ? lut[bin] : 0.0f;     /* 单查表 + 单分支 */
pressureInput.feedforwardFlow += rippleFF;
```
- 经现有 `PumpConverter` 自然限幅（`pumpSpeedLimit`），溢出安全，**无需额外钳位**。
- `rippleFF` 单位 L/min，与 `feedforwardFlow` 同量纲，直接相加。

### 4.5 关闭（满足「有些现场不适用」）
三级任一关即旁路（输出 0）：
- **全局总闸**：`fb->ENABLE_RIPPLE_COMP`（整线禁用，HMI/参数可设）。
- **段级**：`segment->rippleCompEnable`（逐动作/逐段关，如高速射胶关、顶针关）。
- **HMI 参数**：`HYD_PARAM_RIPPLE_COMP_ENABLE`。
- 关闭时 `lut` 不更新、热路径返回 0，**零计算、零 RAM 写入**。

### 4.6 落点（与代码结构契合）
- `motion_control.c` 压力分支，在 `measuredPressure` 就绪、`motorAngleRev` 可取之后、`HYD_PressureController_Execute` 之前：
  - 更新 `rippleComp.theta`（编码器或转速累加）；
  - 调 `HYD_PressureRippleComp_Update(&rippleComp, eP, steadyGate, dt)`（稳态累加 + 低频刷新）；
  - 把 `HYD_PressureRippleComp_GetFF(&rippleComp, theta)` 加入 `pressureInput.feedforwardFlow`。

---

## 5. 数据结构改动清单（精确字段 + 字节数）

`include/common_types.h`：
- `HYD_AxisRef`：+`float motorAngleRev;` （4 B，默认 0）
- `HYD_MotionSegment`：+`HYD_BOOL rippleCompEnable;` （1 B，默认 1）
- `HYD_MotionSegment`：+`HYD_REAL systemGain;`（已存在 line 365，复用即可）
- `HYD_PressureControllerInput`：+`float pumpAngleRev;` （4 B，默认 0，调用方喂入）
- `HYD_PressureControllerType` 枚举：+`HYD_PRESSURE_CONTROLLER_PI_RBF`
- 脉动补偿相关参数：`HYD_PARAM_RIPPLE_COMP_ENABLE`、可选 `HYD_PARAM_RIPPLE_FFGAIN`

`include/pressure_controller.h` / `src/pressure_controller.c`：
- `HYD_PressureControllerState`：+`float ffBase; float ffTrim;`（8 B）；稳态闸门状态位
- 新增 `HYD_PRESSURE_CONTROLLER_PI_RBF` 分支（复用 `RBF_PID` 句柄，输出 PI 律 + FF）

`src/motion_control.c`：
- 压力分支：算 `effectiveFF`（systemGain 推导 + ffTrim）；喂 `motorAngleRev` 与 `rippleFF`；调用脉动补偿

`include/pressure_ripple_comp.h` + `src/pressure_ripple_comp.c`（**新增**）：
- `HYD_PressureRippleCompState`、`Update`、`GetFF`、`Reset`、`SetEnabled`

---

## 6. 文件改动总表
| 文件 | 改动 |
|---|---|
| `include/common_types.h` | 新增 `motorAngleRev` / `rippleCompEnable` / `pumpAngleRev` / 枚举 `PI_RBF` / 参数 |
| `include/pressure_ripple_comp.h` | **新增** 脉动补偿模块头 |
| `src/pressure_ripple_comp.c` | **新增** 相位分箱 LUT 实现（STM32 优化） |
| `src/pressure_controller.c` | PI+RBF 监督分支、FF 自动推导钩子、稳态闸门状态 |
| `src/motion_control.c` | 算 effectiveFF、喂角度与 rippleFF、调用补偿 |
| `CMakeLists.txt` | 新增 `test_pressure_ripple_comp` 目标；扩展 `sim_pressure_control` |
| `tests/test_pressure_ripple_comp.c` | **新增** 单元测试（合成纹波：编码器路径 + 转速累加回退 + 关闭生效 + 无 sinf 热路径） |
| `tests/sim_pressure_control.c` | 扩展验证三项指标（FF 标定消稳态误差 / PI+RBF 融合 / 脉动补偿 RMS 降幅） |

---

## 7. 验证计划
1. **脉动补偿单元测试**（headless，mingw 预设编译）：
   - 合成已知 `A·sin(2π·Z·θ+φ)` 纹波，验证 LUT 收敛后 `rippleFF ≈ -A·sin(...)`，RMS 降幅 > 90%。
   - 编码器路径（`motorAngleRev` 喂入）与转速累加回退路径对比相位误差。
   - 关闭开关（`enabled=0`/全局闸）时输出恒 0、RAM 无写入。
   - 断言热路径函数不含 `sinf`/`sqrtf`（用编译产物/静态检查或注释约束）。
2. **仿真 harness 扩展**：在 `sim_pressure_control.c` 中验证
   - FF 标定消除 PI 稳态误差（对比未标定）；
   - PI+RBF 融合在增益失配时优于 PI+FF；
   - 脉动补偿在低速段 RMS 显著下降。
3. **构建与回归**：`cmake --build --preset mingw` + `ctest` 全绿。
4. **目标部署提示**：最终用 arm-none-eabi 工具链以 `-O2 -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard` 编译，确认 `.text`/`.bss` 增量在 H7 SRAM/Flash 预算内（本方案新增 RAM < 300 B，Flash < 数 KB）。

---

## 8. 性能预算（每控制周期，STM32H7 @480MHz）
| 模块 | 热路径操作 | 估算周期 |
|---|---|---|
| PI+RBF 监督 | RBF 更新（既有，~7 神经元 `expf`）+ PI 律 | 现有量级（百级） |
| 脉动补偿热路径 | 1 次乘（分箱）+ 1 次数组读 + 1 分支 | **< 10** |
| 脉动补偿刷新 | 每 256 周期：26 次除法 + 可选 Goertzel | 摊薄 ~0.1/周期 |
| 相位累加 | 1 次乘加 + 1 次包裹 | < 5 |
| **合计新增** | —— | **< 50 周期 / 周期**（< 0.1 µs @480MHz） |

---

## 9. 风险与权衡
- **`HYD_AxisRef` 加 `motorAngleRev`** 是公开结构体改动，但默认 0、完全向后兼容。
- **新模式语义**：纯 RBF 丢 FF、PI_RBF 加 FF——务必在文档/注释分清，避免重复叠加。
- **相位累加 vs 编码器**：累加路径在转速剧烈波动/负载扰动下相位会漂，故优先编码器；两者并存、可配置。
- **LUT 分辨率**：NBINS=26 足以描述齿频基波+2 次谐波；更高分辨率更准但占 RAM，可按需调（仍极小）。
- **`ffGain` 来源**：优先 `1/systemGain`；无 systemGain 时用可调 `K_comp`，由现场标定。
- **实时性**：所有循环有界、无 `malloc`、无阻塞，满足硬实时控制环。
