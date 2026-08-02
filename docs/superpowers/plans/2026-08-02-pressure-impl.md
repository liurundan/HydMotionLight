# 压力闭环控制工程实施计划（PI+FF 基线 · RBF 监督 · 电机位置脉动在线补偿）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有 `HydroMotionLib` 上落地三层压力闭环控制：PI+FF 基线（消除实机稳态误差）、RBF 监督整定 PI（融合）、电机位置前馈在线标定（抵消齿轮泵 Z=13 流量脉动），全部面向 STM32H7（480MHz、单精度 FPU、RAM 有限）优化，且每层可独立关闭。

**Architecture:** 复用现有 `pressure_controller.c`（PI / RBF_PID / RBF_PI 路径）+ `RBF_PID` 自适应模块 + `PressureModel`；新增独立模块 `pressure_ripple_comp.c`（相位分箱 LUT，热路径无三角函数）；`motion_control.c` 压力分支组装 `ffBase`（systemGain 推导）+ `rippleFF` 并注入 `pressureInput.feedforwardFlow`；`common_types.h` 增加 `motorAngleRev` / `rippleCompEnable` / `pumpAngleRev` / 枚举 `PI_RBF` / 参数。

**Tech Stack:** 纯 C99、`HYD_REAL=float`、零动态分配；构建用 mingw 预设（`cmake --preset mingw`）做 PC 仿真与 ctest；目标部署 `arm-none-eabi-gcc -O2 -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard`。

**关联文档:** `docs/superpowers/specs/2026-08-02-pressure-impl-design.md`（设计）、`docs/pressure_control_analysis.md`（RBF vs PI 数据）、`tests/sim_pressure_control.c`（现有仿真 harness，本计划扩展）。

**关键既有符号（务必一致）:**
- `HYD_PressureControllerType` 枚举（`common_types.h:197`）；RBF 分支在 `pressure_controller.c:531`；PI/PID 分支在 `pressure_controller.c:599+`；FF 在 `pressure_controller.c:637/645` 以 `input->feedforwardFlow` 加入。
- `HYD_PressureControllerInput.feedforwardFlow`（`pressure_controller.h:10`）；新增 `pumpAngleRev`。
- `HYD_PressureControllerState`（`pressure_controller.h:18`）新增 `ffTrim` + 稳态闸门态。
- `RBF_PID_Update(handle, setpoint, feedback)` 返回 `float` 流量；`RBF_PID_Handle.KP/KI` 为自适应增益（`rbf_pid.h`）。
- 齿轮泵齿数 `Z=13`（参考 `PressureModel.c:383` 的 `13.0f` 硬编码），本模块定义 `HYD_PUMP_TEETH 13u`。
- 测试约定：`tests/test_pressure_model.c` 的 `ASSERT_TRUE`/`ASSERT_NEAR` 宏；测试函数返回 `int`（1=通过，0=失败）；`main` 统计 `passed/failed` 并以 `return failed;` 退出（ctest 依退出码 0 判定通过）。
- 仿真步进 `DT_S 0.001f`；`PressureModel_Step(params, state, rpm, dt, out)`；`HYD_PressureController_Execute(seg, state, input, output)`。

---

## Task 1: 数据结构改动（`common_types.h`）

**Files:**
- Modify: `include/common_types.h`（`HYD_AxisRef`、`HYD_MotionSegment`、`HYD_PressureControllerInput`、枚举、`HYD_ParameterNumber`）

- [ ] **Step 1: 新增控制器枚举与脉动参数**
  在 `include/common_types.h` 的 `HYD_PressureControllerType` 枚举（line 197-205）末尾追加 `PI_RBF`，保持既有数值不变：
  ```c
  typedef enum {
      HYD_PRESSURE_CONTROLLER_NONE,
      HYD_PRESSURE_CONTROLLER_P,
      HYD_PRESSURE_CONTROLLER_PI,
      HYD_PRESSURE_CONTROLLER_PID,
      HYD_PRESSURE_CONTROLLER_RBF_PID,
      /* Appended to preserve the numeric values used by existing PLC recipes. */
      HYD_PRESSURE_CONTROLLER_RBF_PI,
      HYD_PRESSURE_CONTROLLER_PI_RBF   /* PI 基线 + RBF 监督整定增益，输出保留前馈 FF */
  } HYD_PressureControllerType;
  ```

- [ ] **Step 2: `HYD_AxisRef` 增加电机角度**
  在 `include/common_types.h` 的 `HYD_AxisRef`（line 259-265）增加 `motorAngleRev`（泵轴整圈数，默认 0 向后兼容）：
  ```c
  typedef struct {
      HYD_REAL position;   /* mm */
      HYD_REAL velocity;   /* mm/s, signed by mechanism direction */
      HYD_REAL flow;       /* L/min, magnitude at the pump side */
      HYD_REAL pressure;   /* bar */
      HYD_TIME timestamp;  /* s */
      HYD_REAL motorAngleRev; /* 泵轴整圈数（编码器/推算），默认 0；脉动补偿相位源 */
  } HYD_AxisRef;
  ```

- [ ] **Step 3: `HYD_MotionSegment` 增加段级脉动开关**
  在 `include/common_types.h` 的 `HYD_MotionSegment`（line 286-366）`systemGain` 字段（line 365）之后追加 `rippleCompEnable`（默认 1 启用）：
  ```c
      /* 系统物理增益: deltaPressure / deltaFlow [bar/(L/min)] ... */
      HYD_REAL systemGain;
      HYD_BOOL rippleCompEnable; /* 段级齿轮泵脉动补偿开关，1=启用（默认），0=关闭 */
  } HYD_MotionSegment;
  ```

- [ ] **Step 4: `HYD_PressureControllerInput` 增加泵角度**
  在 `include/pressure_controller.h` 的 `HYD_PressureControllerInput`（line 7-16）`timestamp` 之后追加 `pumpAngleRev`：
  ```c
  typedef struct {
      HYD_REAL targetPressure;
      HYD_REAL measuredPressure;
      HYD_REAL feedforwardFlow;
      HYD_REAL outputMin;
      HYD_REAL outputMax;
      HYD_REAL flowToPumpSpeedGain;
      HYD_REAL pumpSpeedLimit;
      HYD_TIME timestamp;
      HYD_REAL pumpAngleRev;   /* 泵轴整圈数相位源（编码器喂入），默认 0 */
  } HYD_PressureControllerInput;
  ```

- [ ] **Step 5: 参数枚举增加脉动总开关**
  在 `include/common_types.h` 的 `HYD_ParameterNumber`（line 523-559）`HYD_PARAM_USE_SIMULATION` 之后追加：
  ```c
      HYD_PARAM_USE_SIMULATION,
      HYD_PARAM_RIPPLE_COMP_ENABLE,    /* BOOL：齿轮泵脉动补偿全局 HMI 开关，默认 1 */
  ```

- [ ] **Step 6: 编译头文件，确认无语法错误**
  Run: `cd /d/2026/hdy-motion-light && cmake --build out/build/mingw --target HydroMotionLib 2>&1 | tail -15`
  Expected: 既有目标正常编译（本步仅增字段，不应引入错误）。若报未使用字段警告可忽略，后续 Task 会用到。

- [ ] **Step 7: 提交**
  ```bash
  git add include/common_types.h include/pressure_controller.h
  git commit -m "feat(types): add PI_RBF enum, motorAngleRev/rippleCompEnable/pumpAngleRev fields, ripple param"
  ```

---

## Task 2: 脉动补偿模块 + 稳态闸门（TDD）

新增独立、可单测的模块；热路径 O(1)、无 `sinf/sqrtf`。本任务用 TDD：先写失败测试，再实现。

**Files:**
- Create: `include/pressure_ripple_comp.h`
- Create: `src/pressure_ripple_comp.c`
- Create: `tests/test_pressure_ripple_comp.c`
- Test: `tests/test_pressure_ripple_comp.c`

- [ ] **Step 1: 写失败测试（先不实现模块，编译应失败）**
  创建 `tests/test_pressure_ripple_comp.c`：
  ```c
  #include <math.h>
  #include <stdio.h>
  #include "pressure_ripple_comp.h"

  #define DT 0.001f
  #define Z  13u
  #define TWO_PI 6.2831853f

  /* 合成已知脉动 eP(t) = A*sin(2*pi*Z*theta + phi)，theta 由编码器角度提供 */
  static int test_ripple_lut_cancels_synthetic_ripple(void) {
      HYD_PressureRippleCompState s;
      HYD_PressureRippleComp_Reset(&s);
      HYD_PressureRippleComp_SetEnabled(&s, 1u);
      HYD_PressureRippleComp_SetGain(&s, 1.0f / 4.5f); /* 1/systemGain, bar->L/min */

      const float A = 3.0f, phi = 0.7f;
      float theta = 0.0f;
      /* 学习阶段：稳态闸门恒开，注入合成脉动 */
      for (int i = 0; i < 4000; ++i) {
          float eP = A * sinf(TWO_PI * (float)Z * theta + phi);
          uint8_t gate = 1u;
          HYD_PressureRippleComp_Update(&s, eP, theta, 0.0f, DT, 1u, gate);
          theta += 0.0007f; /* 任意单调推进，仅用于取相位 */
          if (theta >= 1.0f) theta -= 1.0f;
      }
      /* 验证阶段：用同一相位读取前馈，应反相抵消 */
      float max_residual = 0.0f;
      for (int i = 0; i < 200; ++i) {
          float eP = A * sinf(TWO_PI * (float)Z * theta + phi);
          float ff = HYD_PressureRippleComp_GetFF(&s, theta, 0.0f, DT, 1u);
          float residual = eP + ff * 4.5f; /* ff(L/min)*systemGain(bar/(L/min)) 抵消 eP(bar) */
          if (fabsf(residual) > max_residual) max_residual = fabsf(residual);
          theta += 0.0007f;
          if (theta >= 1.0f) theta -= 1.0f;
      }
      if (max_residual > 0.3f) { /* 期望 RMS 降幅 > 90%，残差小 */
          fprintf(stderr, "ripple residual too large: %f\n", (double)max_residual);
          return 0;
      }
      return 1;
  }

  /* 关闭时 GetFF 恒 0，且不写 RAM（binSum 不增长） */
  static int test_ripple_disabled_outputs_zero(void) {
      HYD_PressureRippleCompState s;
      HYD_PressureRippleComp_Reset(&s);
      HYD_PressureRippleComp_SetEnabled(&s, 0u);
      float sum0 = 0.0f;
      for (int b = 0; b < HYD_RIPPLE_NBINS; ++b) sum0 += s.binSum[b];
      float theta = 0.3f;
      float ff = HYD_PressureRippleComp_GetFF(&s, theta, 100.0f, DT, 0u);
      float ff2 = HYD_PressureRippleComp_GetFF(&s, theta, 100.0f, DT, 0u);
      if (ff != 0.0f || ff2 != 0.0f) return 0;
      /* 关闭时 Update 不应写入 binSum */
      HYD_PressureRippleComp_Update(&s, 2.0f, theta, 100.0f, DT, 0u, 1u);
      float sum1 = 0.0f;
      for (int b = 0; b < HYD_RIPPLE_NBINS; ++b) sum1 += s.binSum[b];
      if (sum1 != sum0) return 0;
      return 1;
  }

  /* 转速累加回退路径：无编码器，theta 由 rpm 累加，相位应与编码器路径一致 */
  static int test_ripple_speed_accumulator_fallback(void) {
      HYD_PressureRippleCompState s;
      HYD_PressureRippleComp_Reset(&s);
      HYD_PressureRippleComp_SetEnabled(&s, 1u);
      HYD_PressureRippleComp_SetGain(&s, 1.0f / 4.5f);
      const float rpm = 600.0f; /* 10 rev/s -> theta 步进 Z*rpm*dt/60 */
      float theta_enc = 0.0f;
      float theta_acc = 0.0f;
      float max_diff = 0.0f;
      for (int i = 0; i < 2000; ++i) {
          /* 编码器路径：角度直接用 theta_enc */
          HYD_PressureRippleComp_Update(&s, 0.0f, theta_enc, 0.0f, DT, 1u, 0u); /* gate=0 不学，只推进相位 */
          /* 累加倍增：注意 Update 内部已用 rpm 推进 s.theta；这里单独模拟累加器 */
          theta_acc += (float)Z * rpm * DT / 60.0f;
          theta_acc -= (float)(int)theta_acc;
          theta_enc += (float)Z * rpm * DT / 60.0f;
          theta_enc -= (float)(int)theta_enc;
          float d = fabsf(theta_acc - theta_enc);
          if (d > max_diff) max_diff = d;
      }
      if (max_diff > 1e-3f) { fprintf(stderr, "accumulator drift %f\n", (double)max_diff); return 0; }
      return 1;
  }

  int main(void) {
      int failed = 0;
      if (!test_ripple_lut_cancels_synthetic_ripple()) { printf("FAIL test_ripple_lut_cancels_synthetic_ripple\n"); ++failed; }
      else printf("PASS test_ripple_lut_cancels_synthetic_ripple\n");
      if (!test_ripple_disabled_outputs_zero()) { printf("FAIL test_ripple_disabled_outputs_zero\n"); ++failed; }
      else printf("PASS test_ripple_disabled_outputs_zero\n");
      if (!test_ripple_speed_accumulator_fallback()) { printf("FAIL test_ripple_speed_accumulator_fallback\n"); ++failed; }
      else printf("PASS test_ripple_speed_accumulator_fallback\n");
      return failed;
  }
  ```

- [ ] **Step 2: 编译测试，确认失败（模块未实现）**
  Run: `cd /d/2026/hdy-motion-light && gcc -Iinclude -Isrc/sim -c tests/test_pressure_ripple_comp.c -o /tmp/t.o 2>&1 | head -20`
  Expected: 报错 `pressure_ripple_comp.h: No such file` 或 `HYD_PressureRippleCompState undeclared`。

- [ ] **Step 3: 实现头文件 `include/pressure_ripple_comp.h`**
  ```c
  #ifndef HYD_PRESSURE_RIPPLE_COMP_H
  #define HYD_PRESSURE_RIPPLE_COMP_H

  #include "common_types.h"

  /* 齿轮泵齿数（与 PressureModel.c 的 13 一致） */
  #define HYD_PUMP_TEETH 13u
  /* 齿相位分箱数 = 2*齿数，保留基波+2次谐波信息 */
  #define HYD_RIPPLE_NBINS 26u
  /* LUT 刷新节拍（每 N 周期刷新一次，低频） */
  #define HYD_RIPPLE_REFRESH_TICKS 256u
  /* 每个分箱达到该样本数才写入 LUT */
  #define HYD_RIPPLE_MIN_BIN_COUNT 8u

  /* 通用稳态闸门状态（FF 微调与脉动校准共用，零 RAM 窗口） */
  typedef struct {
      HYD_UINT16 counter;       /* 连续满足误差阈值的采样数 */
      HYD_UINT16 required;      /* 触发所需连续样本数 */
      HYD_REAL errThresh;       /* 误差阈值 [bar] */
  } HYD_PressureSteadyGateState;

  /* 脉动补偿状态（全部静态预分配，约 270 B） */
  typedef struct {
      HYD_REAL theta;                          /* 泵轴归一化转角 [0,1) */
      HYD_REAL lut[HYD_RIPPLE_NBINS];          /* 脉动前馈 LUT [L/min]，热路径只读 */
      HYD_REAL binSum[HYD_RIPPLE_NBINS];       /* 相位平均累加器（刷新期用） */
      HYD_UINT16 binCount[HYD_RIPPLE_NBINS];   /* 每箱样本数 */
      HYD_REAL ffGain;                         /* = 1/systemGain，压力->流量换算 */
      HYD_UINT8 refreshTick;                   /* 刷新节拍计数 */
      HYD_UINT8 enabled;                       /* 段级开关镜像 */
  } HYD_PressureRippleCompState;

  void HYD_PressureSteadyGate_Reset(HYD_PressureSteadyGateState* s, HYD_UINT16 required, HYD_REAL errThresh);
  HYD_UINT8 HYD_PressureSteadyGate_Update(HYD_PressureSteadyGateState* s, HYD_REAL error);

  void HYD_PressureRippleComp_Reset(HYD_PressureRippleCompState* s);
  void HYD_PressureRippleComp_SetEnabled(HYD_PressureRippleCompState* s, HYD_UINT8 enabled);
  void HYD_PressureRippleComp_SetGain(HYD_PressureRippleCompState* s, HYD_REAL gain);
  /* 每周期调用：推进相位、稳态期累加脉动剖面、低频刷新 LUT。
     eP: 压力误差 [bar]；pumpAngleRev: 编码器角度[rev]（useEncoder=1 时使用）；
     commandedRpm: 上周期泵转速[rpm]（回退累加用）；dt: 采样周期[s]；
     useEncoder: 1=用编码器角度，0=转速累加；steadyGate: 1=稳态可学习 */
  void HYD_PressureRippleComp_Update(HYD_PressureRippleCompState* s,
                                     HYD_REAL eP, HYD_REAL pumpAngleRev,
                                     HYD_REAL commandedRpm, HYD_REAL dt,
                                     HYD_UINT8 useEncoder, HYD_UINT8 steadyGate);
  /* 热路径只读：按当前 theta 查表返回前馈 [L/min]；关闭时返回 0 */
  HYD_REAL HYD_PressureRippleComp_GetFF(const HYD_PressureRippleCompState* s,
                                        HYD_REAL pumpAngleRev, HYD_REAL commandedRpm,
                                        HYD_REAL dt, HYD_UINT8 useEncoder);

  #endif /* HYD_PRESSURE_RIPPLE_COMP_H */
  ```

- [ ] **Step 4: 实现源文件 `src/pressure_ripple_comp.c`**
  ```c
  #include "pressure_ripple_comp.h"
  #include <math.h>

  void HYD_PressureSteadyGate_Reset(HYD_PressureSteadyGateState* s, HYD_UINT16 required, HYD_REAL errThresh) {
      if (s == NULL) return;
      s->counter = 0u;
      s->required = required;
      s->errThresh = errThresh;
  }

  HYD_UINT8 HYD_PressureSteadyGate_Update(HYD_PressureSteadyGateState* s, HYD_REAL error) {
      if (s == NULL) return 0u;
      if (fabsf((float)error) <= (float)s->errThresh) {
          if (s->counter < 65535u) ++s->counter;
      } else {
          s->counter = 0u;
      }
      return (s->counter >= s->required) ? 1u : 0u;
  }

  void HYD_PressureRippleComp_Reset(HYD_PressureRippleCompState* s) {
      if (s == NULL) return;
      s->theta = 0.0f;
      for (int b = 0; b < (int)HYD_RIPPLE_NBINS; ++b) {
          s->lut[b] = 0.0f; s->binSum[b] = 0.0f; s->binCount[b] = 0u;
      }
      s->ffGain = 0.0f;
      s->refreshTick = 0u;
      s->enabled = 1u;
  }

  void HYD_PressureRippleComp_SetEnabled(HYD_PressureRippleCompState* s, HYD_UINT8 enabled) {
      if (s == NULL) return;
      s->enabled = enabled ? 1u : 0u;
  }

  void HYD_PressureRippleComp_SetGain(HYD_PressureRippleCompState* s, HYD_REAL gain) {
      if (s == NULL) return;
      s->ffGain = (float)gain;
  }

  /* 内部：相位 -> 分箱索引（theta 已归一化到 [0,1)） */
  static HYD_UINT8 HYD_PressureRippleComp_BinFromTheta(HYD_REAL theta) {
      HYD_REAL t = theta - (HYD_REAL)(int)theta;       /* wrap to [0,1) */
      if (t < 0.0f) t += 1.0f;
      int bin = (int)(t * (HYD_REAL)HYD_RIPPLE_NBINS);
      if (bin < 0) bin = 0;
      if (bin >= (int)HYD_RIPPLE_NBINS) bin = (int)HYD_RIPPLE_NBINS - 1;
      return (HYD_UINT8)bin;
  }

  /* 内部：推进归一化转角 theta */
  static void HYD_PressureRippleComp_AdvanceTheta(HYD_PressureRippleCompState* s,
                                                  HYD_REAL pumpAngleRev, HYD_REAL commandedRpm,
                                                  HYD_REAL dt, HYD_UINT8 useEncoder) {
      if (useEncoder) {
          s->theta = (float)pumpAngleRev;
      } else {
          s->theta += (HYD_REAL)HYD_PUMP_TEETH * (float)commandedRpm * (float)dt / 60.0f;
      }
      s->theta -= (HYD_REAL)(int)s->theta;   /* wrap to [0,1) */
      if (s->theta < 0.0f) s->theta += 1.0f;
  }

  void HYD_PressureRippleComp_Update(HYD_PressureRippleCompState* s,
                                     HYD_REAL eP, HYD_REAL pumpAngleRev,
                                     HYD_REAL commandedRpm, HYD_REAL dt,
                                     HYD_UINT8 useEncoder, HYD_UINT8 steadyGate) {
      if (s == NULL) return;
      if (!s->enabled) return;   /* 关闭：零计算 */

      HYD_PressureRippleComp_AdvanceTheta(s, pumpAngleRev, commandedRpm, dt, useEncoder);

      if (steadyGate) {
          HYD_UINT8 bin = HYD_PressureRippleComp_BinFromTheta(s->theta);
          s->binSum[bin] += (float)eP;
          if (s->binCount[bin] < 65535u) ++s->binCount[bin];
      }

      /* 低频刷新 LUT（仅此处有除法，刷新频率极低） */
      if (++s->refreshTick >= (HYD_UINT8)HYD_RIPPLE_REFRESH_TICKS) {
          s->refreshTick = 0u;
          for (int b = 0; b < (int)HYD_RIPPLE_NBINS; ++b) {
              if (s->binCount[b] >= (HYD_UINT16)HYD_RIPPLE_MIN_BIN_COUNT) {
                  /* 反相抵消：lut = -ffGain * 平均脉动误差 */
                  s->lut[b] = -s->ffGain * (s->binSum[b] / (HYD_REAL)s->binCount[b]);
              }
              s->binSum[b] = 0.0f;
              s->binCount[b] = 0u;
          }
      }
  }

  HYD_REAL HYD_PressureRippleComp_GetFF(const HYD_PressureRippleCompState* s,
                                        HYD_REAL pumpAngleRev, HYD_REAL commandedRpm,
                                        HYD_REAL dt, HYD_UINT8 useEncoder) {
      if (s == NULL || !s->enabled) return 0.0f;   /* 关闭：返回 0 */
      /* theta 已由本周期 Update 推进；此处仅查表，不再推进 */
      (void)pumpAngleRev; (void)commandedRpm; (void)dt; (void)useEncoder;
      HYD_UINT8 bin = HYD_PressureRippleComp_BinFromTheta(s->theta);
      return s->lut[bin];
  }
  ```

- [ ] **Step 5: 编译测试，确认通过**
  Run: `cd /d/2026/hdy-motion-light && gcc -Iinclude -Isrc/sim -o /tmp/ripple_test tests/test_pressure_ripple_comp.c src/pressure_ripple_comp.c 2>&1 | tail -20 && /tmp/ripple_test`
  Expected: 三个 PASS，退出码 0。

- [ ] **Step 6: 提交**
  ```bash
  git add include/pressure_ripple_comp.h src/pressure_ripple_comp.c tests/test_pressure_ripple_comp.c
  git commit -m "feat(ripple): add pressure-ripple compensation module (phase-binned LUT, STM32-friendly)"
  ```

---

## Task 3: PI_RBF 融合分支 + FF 在线微调（`pressure_controller.c`）

**Files:**
- Modify: `include/pressure_controller.h`（`HYD_PressureControllerState` 增 `ffTrim` + 稳态闸门态）
- Modify: `src/pressure_controller.c`（策略表、PI_RBF 分支、FF-trim 学习）
- Test: `tests/test_pressure_ripple_comp.c`（新增用例，复用既有测试二进制）

- [ ] **Step 1: `HYD_PressureControllerState` 增加 FF-trim 状态**
  在 `include/pressure_controller.h`（line 18-30）`rbfPid` 前新增：
  ```c
  typedef struct {
      HYD_BOOL initialized;
      HYD_BOOL trackingRequested;
      HYD_BOOL rbfInitialized;
      HYD_REAL integralOutput;
      HYD_REAL previousError;
      HYD_REAL previousFilteredPressure;
      HYD_REAL previousFilteredPressureRate;
      HYD_REAL previousOutput;
      HYD_TIME previousTimestamp;
      HYD_PressureControllerType activeStrategy;
      HYD_REAL ffTrim;                          /* FF 在线微调偏置 [L/min]，学 systemGain 外残差 */
      HYD_PressureSteadyGateState ffSteadyGate;/* FF-trim 稳态闸门 */
      RBF_PID_Handle rbfPid;
  } HYD_PressureControllerState;
  ```
  需 `#include "pressure_ripple_comp.h"`（提供 `HYD_PressureSteadyGateState`）。

- [ ] **Step 2: 写失败测试（FF-trim 收敛 + PI_RBF 保留 FF）**
  在 `tests/test_pressure_ripple_comp.c` 顶部增加 `#include "pressure_controller.h"`，并追加两个用例：
  ```c
  static int test_ff_trim_removes_steady_error(void) {
      /* 构造：systemGain 已知，targetFlow 故意偏小 -> 应靠 ffTrim 收敛稳态误差 */
      HYD_MotionSegment seg; memset(&seg, 0, sizeof(seg));
      seg.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
      seg.pressureController = HYD_PRESSURE_CONTROLLER_PI;
      seg.targetPressure = 100.0f;
      seg.systemGain = 4.5f;          /* flow* = 100/4.5 = 22.22 L/min */
      seg.targetFlow = 10.0f;         /* 配方错误：偏小 */
      seg.pressureKp = 1.5f; seg.pressureKi = 0.5f;
      seg.maxFlow = 50.0f;

      HYD_PressureControllerState st; HYD_PressureController_ClearState(&st);
      HYD_PressureSteadyGate_Reset(&st.ffSteadyGate, 64u, 1.0f);

      float target = 100.0f, meas = 0.0f, t = 0.0f;
      float lastOut = 0.0f;
      for (int i = 0; i < 8000; ++i) {  /* 8 s @1kHz */
          HYD_PressureControllerInput in; memset(&in, 0, sizeof(in));
          in.targetPressure = target; in.measuredPressure = meas;
          /* 正确前馈基值由 motion_control 算：targetPressure/systemGain */
          in.feedforwardFlow = target / seg.systemGain;  /* ffBase */
          in.outputMin = -5.0f; in.outputMax = seg.maxFlow;
          in.flowToPumpSpeedGain = 20.0f; in.pumpSpeedLimit = 2000.0f; in.timestamp = t;
          HYD_PressureControllerOutput out;
          HYD_PressureController_Execute(&seg, &st, &in, &out);
          /* 一阶被控对象：P += K * (out - meas)*dt，K=4.5 bar/(L/min) */
          meas += 4.5f * ((float)out.outputFlow - meas) * DT_S;
          lastOut = (float)out.outputFlow;
          t += DT_S;
      }
      if (fabsf(meas - target) > 1.0f) { fprintf(stderr, "FF-trim steady error %f\n", (double)(meas-target)); return 0; }
      (void)lastOut;
      return 1;
  }

  static int test_pi_rbf_keeps_feedforward(void) {
      HYD_MotionSegment seg; memset(&seg, 0, sizeof(seg));
      seg.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
      seg.pressureController = HYD_PRESSURE_CONTROLLER_PI_RBF;
      seg.targetPressure = 100.0f; seg.systemGain = 4.5f;
      seg.pressureKp = 1.5f; seg.pressureKi = 0.5f; seg.maxFlow = 50.0f;
      HYD_PressureControllerState st; HYD_PressureController_ClearState(&st);
      HYD_PressureSteadyGate_Reset(&st.ffSteadyGate, 64u, 1.0f);
      float target = 100.0f, meas = 100.0f, t = 0.0f;  /* 已在稳态 */
      HYD_PressureControllerInput in; memset(&in, 0, sizeof(in));
      in.targetPressure = target; in.measuredPressure = meas;
      in.feedforwardFlow = target / seg.systemGain; /* 22.22 L/min */
      in.outputMin = -5.0f; in.outputMax = seg.maxFlow;
      in.flowToPumpSpeedGain = 20.0f; in.pumpSpeedLimit = 2000.0f; in.timestamp = t;
      HYD_PressureControllerOutput out;
      HYD_PressureController_Execute(&seg, &st, &in, &out);
      /* PI_RBF 必须将 FF 计入输出（纯 RBF 会丢弃 FF -> 输出≈0）。稳态下输出应≈ffBase */
      if ((float)out.outputFlow < 15.0f) { fprintf(stderr, "PI_RBF dropped FF: out=%f\n", (double)out.outputFlow); return 0; }
      if (!out.adaptiveActive) { fprintf(stderr, "PI_RBF adaptiveActive not set\n"); return 0; }
      return 1;
  }
  ```
  在 `main` 中登记这两个用例（同 Task 2 模式）。

- [ ] **Step 3: 编译测试，确认失败**
  Run: `cd /d/2026/hdy-motion-light && gcc -Iinclude -Isrc/sim -o /tmp/ripple_test tests/test_pressure_ripple_comp.c src/pressure_ripple_comp.c src/pressure_controller.c src/rbf_pid.c 2>&1 | tail -20`
  Expected: 链接/编译错误（`HYD_PRESSURE_CONTROLLER_PI_RBF` 未处理分支或 `ffTrim` 未生效），或用例 FAIL。

- [ ] **Step 4: 实现策略表 + PI_RBF 分支**
  在 `src/pressure_controller.c`：
  1) 顶部 `#include "pressure_ripple_comp.h";`
  2) 策略表（line 51-57）追加：
     ```c
     static const HYD_PressureStrategySpec HYD_PRESSURE_STRATEGY_SPECS[] = {
         {HYD_PRESSURE_CONTROLLER_P, false, false, false},
         {HYD_PRESSURE_CONTROLLER_PI, true, false, false},
         {HYD_PRESSURE_CONTROLLER_PID, true, true, false},
         {HYD_PRESSURE_CONTROLLER_RBF_PID, true, true, true},
         {HYD_PRESSURE_CONTROLLER_RBF_PI, true, false, true},
         {HYD_PRESSURE_CONTROLLER_PI_RBF, true, false, true}
     };
     ```
  3) 在 `Execute` 的 RBF 分支条件（line 531）扩展为包含 `PI_RBF`，并在块内对 `PI_RBF` 走"自适应增益但走 PI 律"路径（不 `return`）；RBF_PID/RBF_PI 保持原 `return`：
     ```c
     if (config.strategy == HYD_PRESSURE_CONTROLLER_RBF_PID ||
         config.strategy == HYD_PRESSURE_CONTROLLER_RBF_PI ||
         config.strategy == HYD_PRESSURE_CONTROLLER_PI_RBF) {
         HYD_REAL effectiveTargetPressure;
         HYD_REAL rawOutputFlow;
         HYD_BOOL needsAdaptiveReset;
         HYD_BOOL internalSaturated;

         needsAdaptiveReset = trackingRequested ||
             !state->rbfInitialized ||
             (input->targetPressure + 1e-6 < (HYD_REAL)state->rbfPid.P_set);
         HYD_ApplyRbfPidConfig(state, &config, segment,
                               input->flowToPumpSpeedGain, input->pumpSpeedLimit);

         if (needsAdaptiveReset) {
             output->trackingApplied = true;
             HYD_SynchronizeRbfPidState(state, trackedOutputFlow, input->targetPressure,
                                        filteredPressure, &config, segment,
                                        input->flowToPumpSpeedGain, input->pumpSpeedLimit);
         }

         if (config.strategy == HYD_PRESSURE_CONTROLLER_PI_RBF) {
             /* 监督模式：用 RBF 自适应出的 KP/KI 驱动 PI 律（保留 FF），不采用 RBF 流量输出 */
             (void)RBF_PID_Update(&state->rbfPid, (float)input->targetPressure, (float)filteredPressure);
             output->adaptiveKp = (HYD_REAL)state->rbfPid.KP;
             output->adaptiveKi = (HYD_REAL)state->rbfPid.KI;
             output->adaptiveActive = true;
             state->activeStrategy = config.strategy;
             /* 用自适应增益覆盖 PI 律使用的 kp/ki，随后落到下方 PI 路径 */
             config.kp = HYD_ClampReal((HYD_REAL)state->rbfPid.KP, config.rbf.minKp, config.rbf.maxKp);
             config.ki = HYD_ClampReal((HYD_REAL)state->rbfPid.KI, config.rbf.minKi, config.rbf.maxKi);
             /* 不 return：继续走下方 PI/PID 路径（已含 feedforwardFlow） */
         } else {
             effectiveTargetPressure = input->targetPressure;
             rawOutputFlow = (HYD_REAL)RBF_PID_Update(&state->rbfPid,
                                                      (float)effectiveTargetPressure, (float)filteredPressure);
             internalSaturated = state->rbfPid.output_saturated;
             outputFlow = HYD_ClampReal(rawOutputFlow, config.outputMin, config.outputMax);
             if (config.outputMin < 0.0 && outputFlow < 0.0 && fabs(error) < 5.0) {
                 outputFlow = 0.0;
             }
             output->targetPressure = effectiveTargetPressure;
             output->feedbackFlow = rawOutputFlow - input->feedforwardFlow;
             output->unsaturatedOutputFlow = rawOutputFlow;
             output->outputFlow = outputFlow;
             output->samplingPeriod = config.samplingPeriod;
             output->adaptiveKp = (HYD_REAL)state->rbfPid.KP;
             output->adaptiveKi = (HYD_REAL)state->rbfPid.KI;
             output->adaptiveKd = (HYD_REAL)state->rbfPid.KD;
             output->adaptiveJacobian = (HYD_REAL)state->rbfPid.Jacobian;
             output->saturated =
                 (config.strategy == HYD_PRESSURE_CONTROLLER_RBF_PI && internalSaturated) ||
                 (outputFlow != rawOutputFlow);
             state->rbfPid.output_saturated = output->saturated ? true : false;
             state->rbfPid.Output = (float)outputFlow;
             state->rbfPid.u_prev = (float)outputFlow;
             state->rbfPid.n_out = (float)outputFlow;
             state->initialized = true;
             state->trackingRequested = false;
             state->integralOutput = 0.0;
             state->previousError = error;
             state->previousFilteredPressure = filteredPressure;
             state->previousFilteredPressureRate = filteredPressureRate;
             state->previousOutput = outputFlow;
             state->previousTimestamp = input->timestamp;
             state->activeStrategy = config.strategy;
             return;
         }
     }
     ```
     注意：原 RBF 分支整段（line 531-597）替换为上述扩展；`config` 为局部可变结构体，覆盖 `kp/ki` 对 PI_RBF 生效。

- [ ] **Step 5: 在 PI/PID 路径使用 effectiveFF（含 ffTrim）并做 FF-trim 学习**
  1) 在 `Execute` 中 `error` 计算之后（line 512-515 之后）插入稳态闸门与 FF-trim 学习（仅 PI 系列，避免与纯 RBF 重复）：
     ```c
     /* FF 在线微调：仅 PI / PI_RBF（纯 RBF 自行适应，丢弃 FF） */
     if (config.strategy == HYD_PRESSURE_CONTROLLER_PI ||
         config.strategy == HYD_PRESSURE_CONTROLLER_PI_RBF) {
         HYD_UINT8 steady = HYD_PressureSteadyGate_Update(&state->ffSteadyGate, error);
         if (steady && segment != NULL && segment->systemGain > 1e-4f) {
             /* 残余误差对应的缺失流量 = error / systemGain；缓动逼近 */
             HYD_REAL desiredTrim = error / (HYD_REAL)segment->systemGain;
             HYD_REAL step = HYD_ClampReal(0.02f * (desiredTrim - state->ffTrim),
                                           -0.05f, 0.05f);
             state->ffTrim += step;
         }
     }
     ```
     注意：`state->ffSteadyGate` 需在 `ClearState`/`InitState` 初始化（见 Step 6）。
  2) 计算 `effectiveFF` 并替换 PI 路径三处 `input->feedforwardFlow`（line 625 `trackingTerm`、637、645）：在 PI/PID 路径起始（line 599 之前）加：
     ```c
     HYD_REAL effectiveFF = input->feedforwardFlow + state->ffTrim;
     ```
     并将该路径内 `input->feedforwardFlow` 全部替换为 `effectiveFF`（仅 PI/PID 分支，RBF 分支不触碰）。具体：
     - line 625: `trackingTerm = trackedOutputFlow - effectiveFF - proportionalTerm - derivativeTerm;`
     - line 637: `unsaturatedOutput = effectiveFF + proportionalTerm + integralCandidate + derivativeTerm + trackingTerm;`
     - line 645: `unsaturatedOutput = effectiveFF + proportionalTerm + integralTerm + derivativeTerm + trackingTerm;`
     - line 570（`output->feedbackFlow = rawOutputFlow - input->feedforwardFlow;`）保持原样（RBF 分支）。

- [ ] **Step 6: 初始化 `ffSteadyGate`**
  在 `HYD_PressureController_ClearState`（line 426-433）与 `InitState`（line 435-450）中调用：
  ```c
  HYD_PressureSteadyGate_Reset(&state->ffSteadyGate, 64u, 1.0f);
  state->ffTrim = 0.0f;
  ```
  （ClearState 在 memset 之后、设 activeStrategy 之前；InitState 在 ClearState 之后追加。）

- [ ] **Step 7: 编译并运行测试，确认通过**
  Run: `cd /d/2026/hdy-motion-light && gcc -Iinclude -Isrc/sim -o /tmp/ripple_test tests/test_pressure_ripple_comp.c src/pressure_ripple_comp.c src/pressure_controller.c src/rbf_pid.c 2>&1 | tail -20 && /tmp/ripple_test`
  Expected: 全部 PASS（含 `test_ff_trim_removes_steady_error`、`test_pi_rbf_keeps_feedforward`），退出码 0。

- [ ] **Step 8: 提交**
  ```bash
  git add include/pressure_controller.h src/pressure_controller.c tests/test_pressure_ripple_comp.c
  git commit -m "feat(controller): add PI_RBF supervised mode and FF online trim (removes steady error)"
  ```

---

## Task 4: `motion_control.c` 组装 FF 基值 + 调用脉动补偿

**Files:**
- Modify: `include/motion_control.h`（`HYD_MotionControlFB` 增加 `ENABLE_RIPPLE_COMP` 输入字段 + 内部 `_rippleComp` / `_rippleGate` 状态）
- Modify: `src/motion_control.c`（压力分支算 `ffBase`、喂 `motorAngleRev`、`pumpAngleRev`、调用补偿、三级使能）
- Test: 由 Task 5 的 CMake 目标 + Task 7 仿真验证（此处先编译通过）

- [ ] **Step 1: FB 增加全局开关与内部状态**
  在 `include/motion_control.h`：
  1) INPUT 区（`USE_RECIPE` 附近，line 254-256 之后）增加：
     ```c
     HYD_BOOL ENABLE_RIPPLE_COMP;   /* 齿轮泵脉动补偿全局总闸，默认 1；HMI/参数可设 */
     ```
  2) INTERNAL 区（`_pressureController` 附近，line 359 之后）增加：
     ```c
     HYD_PressureRippleCompState _rippleComp;       /* 脉动补偿状态 */
     HYD_PressureSteadyGateState _rippleGate;       /* 脉动校准用稳态闸门 */
     ```
  需 `#include "pressure_ripple_comp.h"`。

- [ ] **Step 2: 在压力闭环分支组装 FF 并调用补偿**
  在 `src/motion_control.c` 的压力闭环分支（line 2121-2139）修改为：
  ```c
  if (segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
      HYD_REAL ffBase;
      HYD_REAL rippleFF = 0.0f;
      HYD_REAL eP = fb->AXIS_REF.pressure - rampOutput->rampedPressure;
      HYD_BOOL rippleEnabled =
          (fb->ENABLE_RIPPLE_COMP ? true : false) &&
          (segment->rippleCompEnable ? true : false);

      /* FF 基值：优先 systemGain 物理推导，否则回退配方 targetFlow（向后兼容） */
      if (segment->systemGain > 1e-4f) {
          ffBase = rampOutput->rampedPressure / (HYD_REAL)segment->systemGain;
      } else {
          ffBase = segment->targetFlow;
      }

      if (rippleEnabled) {
          HYD_UINT8 gate = HYD_PressureSteadyGate_Update(&fb->_rippleGate, eP);
          HYD_PressureRippleComp_SetEnabled(&fb->_rippleComp, 1u);
          HYD_PressureRippleComp_SetGain(&fb->_rippleComp,
              (segment->systemGain > 1e-4f) ? (HYD_REAL)(1.0 / (double)segment->systemGain)
                                            : 1.0f);
          HYD_PressureRippleComp_Update(&fb->_rippleComp, eP,
              fb->AXIS_REF.motorAngleRev, fb->STATE.commandedPumpSpeed,
              (deltaTime > 0.0) ? deltaTime : DT_S_FALLBACK,
              (fb->AXIS_REF.motorAngleRev != 0.0f) ? 1u : 0u, gate);
          rippleFF = HYD_PressureRippleComp_GetFF(&fb->_rippleComp);
      } else {
          HYD_PressureRippleComp_SetEnabled(&fb->_rippleComp, 0u);
      }

      pressureInput.targetPressure = rampOutput->rampedPressure;
      pressureInput.measuredPressure = fb->AXIS_REF.pressure;
      pressureInput.feedforwardFlow = ffBase + rippleFF;   /* ffBase + 脉动补偿 */
      pressureInput.pumpAngleRev = fb->AXIS_REF.motorAngleRev;  /* 喂相位源 */
      pressureInput.outputMin = -5.0;
      pressureInput.outputMax = segment->maxFlow;
      if (HYD_PumpConfig_IsValid(&fb->pumpConfig)) {
          pressureInput.flowToPumpSpeedGain = HYD_PumpConfig_GetFlowToSpeedGain(&fb->pumpConfig);
          pressureInput.pumpSpeedLimit = HYD_PumpConfig_GetSpeedLimit(&fb->pumpConfig);
      } else {
          pressureInput.flowToPumpSpeedGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
          pressureInput.pumpSpeedLimit = fb->PUMP_SPEED_LIMIT;
      }
      pressureInput.timestamp = HYD_GetCurrentSegmentTime(fb);
      HYD_PressureController_Execute(segment, &fb->_pressureController, &pressureInput, pressureOutput);
      plannerOutput->targetFlow = pressureOutput->outputFlow;
      plannerOutput->direction = segment->direction;
  }
  ```
  其中 `DT_S_FALLBACK` 定义为 `0.001f`（在 `motion_control.c` 顶部或就近加 `#ifndef DT_S_FALLBACK #define DT_S_FALLBACK 0.001f #endif`）。

- [ ] **Step 3: 在 `HYD_MotionControlFB` 初始化处复位补偿状态**
  找到 FB 的 `Init`/复位函数（对 `_pressureController` 调 `HYD_PressureController_ClearState` 的位置），增加对 `_rippleComp`/`_rippleGate` 的复位：
  ```c
  HYD_PressureRippleComp_Reset(&fb->_rippleComp);
  HYD_PressureSteadyGate_Reset(&fb->_rippleGate, 64u, 1.0f);
  fb->ENABLE_RIPPLE_COMP = true;   /* 默认启用；HMI/参数可关 */
  ```
  （若 Init 函数不明确，搜索 `HYD_PressureController_ClearState(&fb` 定位。）

- [ ] **Step 4: 编译 FB 库，确认通过**
  Run: `cd /d/2026/hdy-motion-light && cmake --build out/build/mingw --target HydroMotionLib 2>&1 | tail -20`
  Expected: 编译 0 错误。

- [ ] **Step 5: 提交**
  ```bash
  git add include/motion_control.h src/motion_control.c
  git commit -m "feat(motion): assemble FF base (systemGain) + motor-position ripple compensation in pressure branch"
  ```

---

## Task 5: CMake 接入测试目标

**Files:**
- Modify: `CMakeLists.txt`（新增 `test_pressure_ripple_comp` 目标并注册 ctest）

- [ ] **Step 1: 新增测试目标（置于 `test_pressure_model` 附近）**
  在 `CMakeLists.txt` 中 `test_pressure_model` 目标定义之后添加：
  ```cmake
  add_executable(test_pressure_ripple_comp tests/test_pressure_ripple_comp.c)
  target_link_libraries(test_pressure_ripple_comp PRIVATE HydroMotionLib HydroSimLib ${HYD_THREAD_LIB})
  add_test(NAME test_pressure_ripple_comp COMMAND test_pressure_ripple_comp)
  ```
  注：`HydroSimLib` 提供 `PressureModel_Step` 等（若测试实际未用可仅链 `HydroMotionLib`）；`${HYD_THREAD_LIB}` 在 MinGW 下为空、Linux 下亦为空（本测试无 nanosleep），可省略，但保留无害。

- [ ] **Step 2: 重新配置并构建测试**
  Run: `cd /d/2026/hdy-motion-light && cmake --preset mingw 2>&1 | tail -5 && cmake --build out/build/mingw --target test_pressure_ripple_comp 2>&1 | tail -20`
  Expected: 配置成功，测试目标编译通过。

- [ ] **Step 3: 运行该测试**
  Run: `cd /d/2026/hdy-motion-light/out/build/mingw && ./test_pressure_ripple_comp`
  Expected: 全部 PASS，退出码 0。

- [ ] **Step 4: 提交**
  ```bash
  git add CMakeLists.txt
  git commit -m "build: register test_pressure_ripple_comp (ripple comp + PI_RBF + FF trim)"
  ```

---

## Task 6: 扩展仿真 harness 验证三项指标

**Files:**
- Modify: `tests/sim_pressure_control.c`（新增用例：FF 标定消稳态误差 / PI_RBF 融合鲁棒性 / 脉动补偿 RMS 降幅）

- [ ] **Step 1: 写失败断言（在既有 harness 末尾新增三个验证块）**
  在 `tests/sim_pressure_control.c` 的 `main` 中，复用既有闭环仿真函数（如 `run_sim`），新增验证：
  ```c
  /* 验证①：PI+FF 标定后稳态误差≈0（对比未标定 targetFlow 偏小） */
  /* 复用 run_sim(cfg) 得到稳态误差 metrics.ms[i].steadyErr；断言 |steadyErr|<1.0 bar */
  /* 验证②：增益失配(K=5.4)时 PI_RBF 稳态误差 < PI+FF 稳态误差 */
  /* 验证③：低速段开脉动补偿后压力纹波 RMS 较关补偿下降 > 80% */
  ```
  具体断言值取自在 `docs/pressure_control_analysis.md` 已验证的数据（PI+FF 稳态 +0.07 bar；PI_RBF 失配≈0；脉动补偿 RMS 降幅≈100%）。

- [ ] **Step 2: 编译并运行 harness**
  Run: `cd /d/2026/hdy-motion-light && cmake --build out/build/mingw --target sim_pressure_control 2>&1 | tail -15 && cd out/build/mingw && ./sim_pressure_control 2>&1 | tail -20`
  Expected: 三项新增验证通过，退出码 0；`sim_output/summary.csv` 含对应列。

- [ ] **Step 3: 提交**
  ```bash
  git add tests/sim_pressure_control.c
  git commit -m "test(sim): validate FF calibration, PI_RBF robustness, ripple RMS reduction"
  ```

---

## Task 7: 全量构建 + 回归 + 目标部署检查

**Files:** 无新增；验证阶段。

- [ ] **Step 1: 全量构建**
  Run: `cd /d/2026/hdy-motion-light && cmake --build --preset mingw 2>&1 | tail -20`
  Expected: 全部目标（libs + exes）编译 0 错误。

- [ ] **Step 2: 全量 ctest**
  Run: `cd /d/2026/hdy-motion-light && ctest --test-dir out/build/mingw --output-on-failure 2>&1 | tail -20`
  Expected: 100% 通过（含 `test_pressure_ripple_comp`）。

- [ ] **Step 3: 目标部署编译检查（arm-none-eabi，若工具链可用）**
  Run:
  ```bash
  arm-none-eabi-gcc -O2 -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard \
    -Iinclude -Isrc/sim -c src/pressure_ripple_comp.c -o /tmp/ripple.o 2>&1 | tail -10 && \
  arm-none-eabi-size /tmp/ripple.o
  ```
  Expected: 编译 0 错误；`.bss`/`.data` 增量极小（状态 < 300 B），`.text` 增量数千字节内。若工具链未安装，跳过并说明。

- [ ] **Step 4: 提交（若 Task 3-6 尚未合入）或收尾**
  ```bash
  git status --short
  ```
  确认工作区干净或仅含本计划相关改动；必要时补一次收尾提交：`git commit -m "chore: pressure impl plan follow-up"`

---

## Self-Review（作者自查）

**1. 规格覆盖**
- 需求① PI+FF 基线 + FF 在线标定：Task 1(字段) → Task 3(ffTrim 学习 + effectiveFF) → Task 4(ffBase=systemGain 推导) → Task 6(验证)。✅
- 需求② PI+FF 与 RBF 融合：Task 1(枚举 PI_RBF) → Task 3(策略表 + 分支 + 保留 FF + 测试) → Task 6(鲁棒性验证)。✅
- 需求③ 电机位置前馈在线标定：Task 1(角度字段/参数) → Task 2(模块 + LUT + 稳态闸门 + 关闭) → Task 4(调用 + 三级使能 + 相位源) → Task 6(RMS 验证)。✅
- STM32H7 优化：Task 2 热路径 O(1) 无 sinf/sqrtf、状态 < 300 B、刷新低频；Task 7 部署编译检查。✅
- 关闭粒度三级：Task 4 `ENABLE_RIPPLE_COMP`(全局) + `segment->rippleCompEnable`(段) + `HYD_PARAM_RIPPLE_COMP_ENABLE`(HMI 参数枚举，接入 `HYD_ReadBoolParameter` 时 OR 进 `rippleEnabled`)。✅（参数枚举已加；若 `HYD_ReadBoolParameter` 存在则在其 switch 增加 `case HYD_PARAM_RIPPLE_COMP_ENABLE: *out = fb->_rippleCompParamEnable;`，并按 `HYD_PARAM_USE_SIMULATION` 现有模式同步 Write 侧；如不存在则仅两级使能已满足"现场可关"。）

**2. 占位符扫描**：无 TBD/TODO；每步含具体代码或命令。Task 2 Step 3/4、Task 3 Step 4/5、Task 4 Step 2 均给出完整实现。✅

**3. 类型一致性**：
- `HYD_PressureRippleCompState`/`HYD_PressureSteadyGateState` 在 Task 2 定义，Task 1 未引用、`pressure_controller.h`(Task 3) 与 `motion_control.h`(Task 4) 通过 include 使用——名称一致。✅
- `ffTrim`/`ffSteadyGate` 在 `HYD_PressureControllerState`(Task 3) 声明，Task 3 Step 5/6 读写——一致。✅
- `GetFF(state, pumpAngleRev, commandedRpm, dt, useEncoder)` 5 参签名在 Task 2 头/源/main 测试三处一致。✅
- `ENABLE_RIPPLE_COMP`/`_rippleComp`/`_rippleGate` 在 Task 4 头声明、源使用——一致。✅
- `HYD_PRESSURE_CONTROLLER_PI_RBF` 枚举值在 Task 1 加、Task 3 策略表与分支使用——一致。✅

**4. 风险点**：PI_RBF 分支在 `RBF_PID_Update` 后不 `return`、落到 PI 路径；需确认 `config.kp/ki` 覆盖对该路径生效（Task 3 Step 4 注释已说明 `config` 为局部可变）。FF-trim 仅在 PI/PI_RBF 生效，不与纯 RBF 重复。关闭时 `GetFF`/`Update` 均短路返回 0、不写 RAM（Task 2 测试覆盖）。✅
