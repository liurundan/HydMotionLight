# Sprint 3：架构清理 & RBF-PID 工程化 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 Sprint 0/1 完成后遗留的 4 类"代码质量与高级控制器工程化"债务一次性偿清：
1. **命名清理**：`protection_manager.*` → `safety_state_manager.*`，让模块名与"保存安全状态"的实际职责吻合（M-3）。
2. **巨函数拆分**：`motion_control.c:RunRunningState` 240+ 行 → 3 个具名 helper（normal / stopping / blend-cutover），降低维护风险（I-8）。
3. **散落浮点阈值集中**：把分散在 `motion_control.c` / `rbf_pid.c` 的"停车完成阈值 0.001f"、"压力 ceiling 抑制 0.10s"、"RBF 设定值死区 1e-6f" 等魔法数集中到 `hyd_config.h`（M-4）。
4. **RBF-PID 工程化**：去掉 `MAX_PRESSURE=250.0f` 硬编码、扩大增益限幅窗 [0.8,0.85]→[0.5,1.2]、跨控制器策略切换（PI/PID↔RBF）时把增益种子 clamp 进 LIMIT 区间，并补端到端 HIL 测试（含跨段切换 + 长时保压）（C-6 + I-5）。
5. **单泵方向冲突检测**：`__mcl_cmd_GetPumpRequest` 增加 `CONFLICT` 输出，当多个 active FB 的 `plannedDirection` 互相对立时报警，避免阀仍按各自方向开启造成液压死锁（I-2）。
6. **覆盖率工具链**：引入 `gcovr` 本地脚本 + CMake `coverage` preset，并在 README 给出 CI 接入示例，便于度量后续 Sprint 的回归质量（M 杂项）。

**Architecture:**
- **重命名一次性完成**：文件 + 头文件 + 全部 `HYD_ProtectionManager_*` 符号 + 测试文件 + CMakeLists 同步，不引入兼容 wrapper（库未对外发布，不需要 deprecation 桥）。`HYD_PROTECTION_ACTION_*` 枚举本身保留（它是"动作类型"概念，不是文件名概念）。
- **`RunRunningState` 拆分策略**：保留 `RunRunningState` 作为 dispatcher 入口，按"早返回守卫 → 配置/feedback 验证 → 主循环段（normal/stopping/blend-cutover 三选一）"分层。三段抽出后函数都在 `motion_control.c` 内部 `static`，**不暴露到头文件**（避免 API 表面扩散）。重构以"行为不变"为目标——通过现有 35+ 个集成测试套件作为回归门禁，每一步抽出后必须 `ctest` 全绿。
- **魔法数集中策略**：在 `hyd_config.h` 新增 `15. 内部阈值常量` 章节，定义 `HYD_THRESH_STOP_DECEL_DONE_MAG`、`HYD_THRESH_STOP_VEL_DONE_MAG`、`HYD_THRESH_PRESSURE_CEILING_DEBOUNCE_S`、`HYD_THRESH_RBF_SETPOINT_ZERO_EPS` 等常量；只迁移"调参型"魔法数，不迁移"数学不变量"（如 `0.5` 的二分、`1.0` 的归一化）。
- **RBF-PID 配置化**：`RBF_PID_Handle` 新增 `pressure_normalization_scale`（float，0 落回 `MAX_PRESSURE` 默认）。`RBF_PID_Init` 维持原签名以避免破坏 `rbf_pid_test.c`；新增 `RBF_PID_SetPressureNormalization()` setter 供 `pressure_controller.c` 在每段开始时按段配置写入。增益限幅默认窗扩展并通过 `HYD_RbfPidConfig` 段字段覆盖；跨控制器切换时由 `pressure_controller.c:HYD_SynchronizeRbfPidState()` 显式 clamp 初始 KP/KI/KD。
- **HIL 测试构造**：基于 `hydro_sim.c` 的 INJECT 轴模型 + 真实采样周期 1ms，连续 30s 长保压 + PI→RBF 段切换 + RBF→PI 段切换三种场景，断言"无 >5% 阶跃 / 5s 内收敛 / 误差 RMS < tolerance"。
- **方向冲突检测**：`HYD_GETPUMPREQUEST` FB 增加 `CONFLICT`（IEC_BOOL）输出引脚 + 新增 `HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT`（WARNING, 不升级）诊断码。仲裁逻辑：扫描所有 `STATE.active == true` 的 FB，收集 `plannedDirection`；若同时出现 `HYD_DIRECTION_EXTEND` 与 `HYD_DIRECTION_RETRACT` 则置 `CONFLICT=true` 且不更改 `PUMPSPEED` 输出。
- **gcovr 工具链**：新增 `cmake/coverage_toolchain.cmake` + CMake `coverage` preset（启用 `--coverage -O0 -g`），新增 `scripts/coverage.sh` 一键运行 build → ctest → gcovr → 控制台 summary + HTML 报告（输出到 `out/coverage/`），并在 `README.md` 给出 GitHub Actions / Jenkins / GitLab CI 通用接入片段。

**Tech Stack:** C99（gcc 9+/clang 12+）、CMake 3.16+、ctest、gcovr 5+、Beremiz/matiec IEC 类型系统、`tests/` 下纯 C 单元 / 集成测试，与 `src/sim/` 物理仿真器联调 HIL。

**Spec:** `docs/superpowers/specs/2026-05-21-code-review-and-roadmap-design.md`（Sprint 3 §3.1–§3.7）

**任务编号 → spec 子任务映射：**

| Plan Task | Spec ID | 产出 |
|---|---|---|
| Task 0 | — | 基线验证（Sprint 0/1 全绿、磁盘干净、覆盖率基线） |
| Task 1 | 3.1 | `protection_manager.*` → `safety_state_manager.*` 全量改名 |
| Task 2 | 3.3 | 魔法数集中到 `hyd_config.h` 新章节 |
| Task 3 | 3.2 | `RunRunningState` 拆 3 个 helper |
| Task 4 | 3.4a | `MAX_PRESSURE` 归一化改可配置（RBF_PID_Handle 新增字段） |
| Task 5 | 3.4b | RBF-PID 默认增益限幅窗扩展 + 段配置覆盖 |
| Task 6 | 3.4c | 跨控制器策略切换时 clamp 增益种子，消除首步 KD 跳变 |
| Task 7 | 3.5 | `tests/test_rbf_pid_hil.c` 端到端 HIL 测试 |
| Task 8 | 3.6 | `__mcl_cmd_GetPumpRequest` 新增 `CONFLICT` 输出 + 诊断码 |
| Task 9 | 3.7 | `cmake/coverage_toolchain.cmake` + `scripts/coverage.sh` + README CI 片段 |
| Task 10 | — | 文档同步（`control-layer-boundary.md`、`HMI诊断对照表.md`、`README.md`、`CLAUDE.md`） |

**前置准备：所有 Task 共用的工作目录与编译命令**

```bash
cd /home/dan/project/hdy-motion-light

cmake --preset unixgcc
cmake --build --preset unixgcc

# 全量回归
ctest --test-dir out/build/unixgcc --output-on-failure

# 单测试
ctest --test-dir out/build/unixgcc -R '<test_name>' --output-on-failure
```

**重要约定：**
- 每次添加新 `src/*.c` 或 `tests/*.c` 文件后必须 re-run `cmake --preset unixgcc`（CMakeLists 使用 `file(GLOB_RECURSE ...)` 收集 src，但 tests 需手动 `add_executable`）。
- TDD 适用对象：Task 4/5/6/7/8（行为变化）。Task 1/2/3（纯重构 / 重命名 / 常量化）采用"重构前后回归全绿"门禁，不要求新增失败测试。
- 每个 Task 末尾独立 commit。Commit message 前缀沿用本仓库约定：`refactor:` / `feat:` / `test:` / `docs:` / `chore:`。
- Sprint 0/1 已合并到 master（commits 8649a3c / b974a27）；本 Sprint 假设上述修复已生效。如发现回归，必须先解决再继续本 Sprint。
- 本 Sprint 不引入新 PLC 工艺特性；所有改动应对 PLC 工艺层透明（除 Task 8 增加一个新输出引脚和新诊断码）。

---

## Task 0: 基线验证 + 覆盖率快照（前置检查）

**目标：** 保证 Sprint 3 工作开始前，master 处于"全绿、可重复构建、当前覆盖率有据可查"的状态。此 Task 不修改任何源码，仅生成基线快照供后续任务对照。

**Files:**
- Read-only: `out/build/unixgcc/`（确认 ctest 通过）

### Steps

- [ ] **Step 0.1: 检查 git 状态干净**

Run:
```bash
git status
git log --oneline -1
```

Expected:
- 工作区干净（`nothing to commit, working tree clean`），允许 `scripts/deploy_embedded_prod.sh` 在 `M` 列出（已知历史改动，不影响本 Sprint）。
- HEAD 指向 `b974a27 Merge branch sprint1-low-pressure-mold-protect` 或之后的 master commit。

如果 `git status` 显示未提交改动，先 `git stash` 或与上一轮工作合并提交。**禁止在脏树上开始 Sprint 3**。

- [ ] **Step 0.2: 全量构建 + ctest 基线**

Run:
```bash
cmake --preset unixgcc
cmake --build --preset unixgcc 2>&1 | tail -20
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -40
```

Expected:
- 构建无 warning（除 matiec 第三方代码可能有的预期警告）。
- ctest 报告：`100% tests passed, 0 tests failed out of <N>`，N 应为 35 ~ 38（取决于 CMakeLists 当前注册数量）。
- 如有任何 FAIL，**停止本 Sprint**，回到 Sprint 0/1 修复回归。

- [ ] **Step 0.3: 记录基线覆盖率（如本机已装 gcovr，否则跳过并在 Task 9 完成后补做对比）**

Run:
```bash
which gcovr
gcovr --version 2>/dev/null || echo "gcovr not installed — basline will be recorded after Task 9 wires the toolchain"
```

如果 `gcovr` 不可用，跳过此 Step。本 Step 是可选记录；Task 9 完成后会重新执行覆盖率统计并作为 Sprint 3 的产出。

- [ ] **Step 0.4: 提交基线快照（仅元数据，无代码改动，可跳过）**

本 Step 不产生 commit。直接进入 Task 1。

---

## Task 1: 重命名 `protection_manager` → `safety_state_manager`（spec §3.1）

**目标：** 把模块名从误导性的 `protection_manager` 改成与实际职责（"保存/恢复运行时安全状态"）一致的 `safety_state_manager`。这是机械改名，不改任何行为；通过全量 ctest 作为回归门禁。

**为什么改名：** Sprint 0 review 指出，`protection_manager.c` 里 5 个函数（`ResetRuntimeActuation` / `ApplyIdleState` / `ApplyDisabledState` / `ApplyFaultHold` / `EnterFaultStop`）都是在"把 FB 重置/进入安全状态"，与 `HYD_PROTECTION_ACTION_*`（"保护动作类型"枚举）容易混淆。`HYD_PROTECTION_ACTION_*` 是诊断结果，描述"超压时该做什么"；safety_state_manager 是把 FB 推回安全状态的工具。两者职责完全不同，改名后语义更清晰。

**Files:**
- Rename: `include/protection_manager.h` → `include/safety_state_manager.h`
- Rename: `src/protection_manager.c` → `src/safety_state_manager.c`
- Rename: `tests/test_protection_manager.c` → `tests/test_safety_state_manager.c`
- Modify: `src/motion_control.c`（include + 函数调用全量重命名）
- Modify: `src/state_reporter.c`（include + 函数调用全量重命名）
- Modify: `CMakeLists.txt`（test 注册名重命名）

### Steps

- [ ] **Step 1.1: 确认改名影响范围**

Run:
```bash
grep -rln "protection_manager\|HYD_ProtectionManager_" src/ include/ tests/ CMakeLists.txt
```

Expected: 至少 6 个文件命中（上面 Files 列表覆盖）。如果命中更多文件（例如 docs/），把那些文件也加入 Task 1 改动清单或在 Task 10 文档同步时一并修复。

记录到本地草稿，下面按文件顺序处理。

- [ ] **Step 1.2: 改文件名 + 改 include guard**

Run:
```bash
git mv include/protection_manager.h include/safety_state_manager.h
git mv src/protection_manager.c src/safety_state_manager.c
git mv tests/test_protection_manager.c tests/test_safety_state_manager.c
```

用 Edit 工具把 `include/safety_state_manager.h` 的 include guard 从：

```c
#ifndef HYD_PROTECTION_MANAGER_H
#define HYD_PROTECTION_MANAGER_H
```

改为：

```c
#ifndef HYD_SAFETY_STATE_MANAGER_H
#define HYD_SAFETY_STATE_MANAGER_H
```

文件末尾 `#endif /* HYD_PROTECTION_MANAGER_H */` 同步改为 `#endif /* HYD_SAFETY_STATE_MANAGER_H */`。

- [ ] **Step 1.3: 重命名头文件内的函数声明**

在 `include/safety_state_manager.h` 内做以下 5 处全量替换（按 declaration 顺序逐个 Edit，确保替换后签名一致）：

| 旧名 | 新名 |
|---|---|
| `HYD_ProtectionManager_ResetRuntimeActuation` | `HYD_SafetyStateManager_ResetRuntimeActuation` |
| `HYD_ProtectionManager_ApplyIdleState` | `HYD_SafetyStateManager_ApplyIdleState` |
| `HYD_ProtectionManager_ApplyDisabledState` | `HYD_SafetyStateManager_ApplyDisabledState` |
| `HYD_ProtectionManager_ApplyFaultHold` | `HYD_SafetyStateManager_ApplyFaultHold` |
| `HYD_ProtectionManager_EnterFaultStop` | `HYD_SafetyStateManager_EnterFaultStop` |

- [ ] **Step 1.4: 重命名实现文件 src/safety_state_manager.c**

Use Edit with `replace_all: true` for each of these 6 changes:

1. `#include "protection_manager.h"` → `#include "safety_state_manager.h"`
2. `HYD_ProtectionManager_HasSelectedStartSource` → `HYD_SafetyStateManager_HasSelectedStartSource` (static helper)
3. `HYD_ProtectionManager_ResetRuntimeActuation` → `HYD_SafetyStateManager_ResetRuntimeActuation`
4. `HYD_ProtectionManager_ApplyIdleState` → `HYD_SafetyStateManager_ApplyIdleState`
5. `HYD_ProtectionManager_ApplyDisabledState` → `HYD_SafetyStateManager_ApplyDisabledState`
6. `HYD_ProtectionManager_ApplyFaultHold` → `HYD_SafetyStateManager_ApplyFaultHold`
7. `HYD_ProtectionManager_EnterFaultStop` → `HYD_SafetyStateManager_EnterFaultStop`

- [ ] **Step 1.5: 重命名 src/motion_control.c 内的所有调用与 include**

```bash
grep -n "protection_manager\|HYD_ProtectionManager_" src/motion_control.c
```

Expected 命中行：第 5 行 include，第 459 / 604 / 629 / 748 / 875 / 909 / 920 / 927 / 1080 / 1830 / 1847 / 1855 / 1875 / 1905 行调用（共 14 处）。

用 Edit 工具，对 `src/motion_control.c`：
- `#include "protection_manager.h"` → `#include "safety_state_manager.h"`
- `HYD_ProtectionManager_` → `HYD_SafetyStateManager_`（使用 `replace_all: true`）

- [ ] **Step 1.6: 重命名 src/state_reporter.c 内的调用与 include**

对 `src/state_reporter.c`：
- `#include "protection_manager.h"` → `#include "safety_state_manager.h"`
- `HYD_ProtectionManager_EnterFaultStop` → `HYD_SafetyStateManager_EnterFaultStop`（仅 1 处，line 420）

- [ ] **Step 1.7: 重命名测试文件 tests/test_safety_state_manager.c**

对 `tests/test_safety_state_manager.c`：

- 头注释 `tests/test_protection_manager.c` → `tests/test_safety_state_manager.c`
- `#include "protection_manager.h"` → `#include "safety_state_manager.h"`
- 全部 `HYD_ProtectionManager_` → `HYD_SafetyStateManager_`（用 Edit replace_all 一次完成）

测试函数名（如 `test_protection_manager_reset_runtime_actuation`）可保留也可同步改名。建议**同步改名**以保持一致，但若改名后 main() 内调用需同步更新——使用 replace_all 替换 `protection_manager` → `safety_state_manager`。

- [ ] **Step 1.8: 更新 CMakeLists.txt 注册**

`CMakeLists.txt` 第 130-131 行：

旧：
```cmake
add_executable(test_protection_manager tests/test_protection_manager.c)
target_link_libraries(test_protection_manager PRIVATE HydroMotionLib)
```

新：
```cmake
add_executable(test_safety_state_manager tests/test_safety_state_manager.c)
target_link_libraries(test_safety_state_manager PRIVATE HydroMotionLib)
```

第 192-194 行：

旧：
```cmake
add_test(NAME test_protection_manager
         COMMAND test_protection_manager
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
```

新：
```cmake
add_test(NAME test_safety_state_manager
         COMMAND test_safety_state_manager
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
```

- [ ] **Step 1.9: 重新 configure + 构建 + 全量回归**

Run:
```bash
cmake --preset unixgcc
cmake --build --preset unixgcc 2>&1 | tail -20
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -30
```

Expected:
- 构建无未定义引用错误；如有 `undefined reference to HYD_ProtectionManager_*`，说明还有漏改的文件——`grep` 找出并修复。
- ctest：`100% tests passed`；`test_safety_state_manager` 出现在结果列表中（替代 `test_protection_manager`）。

- [ ] **Step 1.10: 残余引用清扫**

Run:
```bash
grep -rln "protection_manager\|HYD_ProtectionManager_\|HYD_PROTECTION_MANAGER_H" src/ include/ tests/ CMakeLists.txt
```

Expected: **零命中**。如果还有命中，根据上下文修复（可能在 docs/ 下的引用留到 Task 10 处理，但 src/ include/ tests/ CMakeLists.txt 必须零命中）。

- [ ] **Step 1.11: Commit**

```bash
git add -A
git commit -m "$(cat <<'EOF'
refactor: rename protection_manager -> safety_state_manager

The protection_manager module's actual responsibility is preserving/restoring
runtime safety state (ResetRuntimeActuation / ApplyIdleState / ApplyDisabledState
/ ApplyFaultHold / EnterFaultStop). The original name was easily confused with
HYD_PROTECTION_ACTION_*, which describes a diagnostic action type rather than
a state manager. Rename clarifies the boundary.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: 散落浮点阈值集中到 `hyd_config.h`（spec §3.3）

**目标：** 把"调参型魔法数"（debounce 时长、停车完成阈值、RBF 数值死区等）统一搬到 `hyd_config.h` 的新章节 `15. 内部阈值常量`，方便审计、调参和未来按平台裁剪。**不动**"数学不变量"（如 `0.5` 二分系数、`1.0f` 归一化、`2.0f` 倍乘）。

**Files:**
- Modify: `include/hyd_config.h`（新增 `15. 内部阈值常量` 章节）
- Modify: `src/motion_control.c`（替换 5 处魔法数为命名常量）
- Modify: `src/rbf_pid.c`（替换 6 处魔法数为命名常量）

### Steps

- [ ] **Step 2.1: 在 `include/hyd_config.h` 末尾追加 §15 章节**

定位 `#define HYD_MAX_AXIS_MOTION  20`（约 297 行）之前**新增**以下章节（在 `#define HYD_MAX_AXIS_MOTION` **之前**插入，保持原 §15 配置导出接口编号不变；将新章节命名为 `14B. 内部数值阈值`，避免重编号其他章节）：

```c
/* ============================================================================
 * 14B. 内部数值阈值（Internal Numerical Thresholds）
 *
 * 本节集中了原先散落在 src/*.c 中的"调参型"浮点常量——这些常量是
 * 工业整定参数，可能因平台、采样频率或工艺要求调整，集中后便于
 * 一处修改、整体生效。
 *
 * 注意：本节不包含"数学不变量"（如 0.5 二分、1.0 归一化、2.0 倍乘）；
 * 那些应当保留在源码上下文中以保证算法可读性。
 * ============================================================================ */

/* --- 停车减速完成阈值（src/motion_control.c stopping-branch） --- */

/* 当 commanded deceleration ramp 输出幅值 (decelMag) 跌破该阈值，
 * 且实测速度同时跌破 HYD_THRESH_STOP_VEL_DONE_MAG 时，判定停车完成。
 * 单位：L/min（与 plannerOutput.targetFlow / decelMag 同维度）。
 * 原值：motion_control.c:1824 `0.001f`。 */
#define HYD_THRESH_STOP_DECEL_DONE_MAG       0.001f

/* 配合 HYD_THRESH_STOP_DECEL_DONE_MAG 使用：实测 AXIS_REF.velocity 绝对值
 * 跌破该阈值视为"实际已停"。
 * 单位：mm/s（与 velocity 同维度）。
 * 原值：motion_control.c:1824 `0.01f`。 */
#define HYD_THRESH_STOP_VEL_DONE_MAG         0.01f

/* 停车减速完成后，主循环再次判定 decelerate-only 段完成时使用的速度阈值。
 * 单位：plannerOutput.targetVelocity 维度（mm/s）。
 * 原值：motion_control.c:1866 `0.001`。 */
#define HYD_THRESH_DECEL_TARGET_VEL_DONE     0.001

/* 停车 watchdog：ideal-stop time × N + slack(s) 后强制升级 FAULT.
 * 原值：motion_control.c:1839 `5.0f * idealStopTime + 1.0f`. */
#define HYD_THRESH_STOP_TIMEOUT_IDEAL_MULT   5.0f
#define HYD_THRESH_STOP_TIMEOUT_SLACK_S      1.0f

/* --- 压力 ceiling 诊断 debounce / startup-suppress（motion_control.c Init） --- */

/* PRESSURE_CEILING_EXCEEDED 触发前的 debounce 时长。短于一般 pressure
 * deviation debounce，因为 ceiling 违规已超出安全包络，必须更快反应。
 * 单位：秒。原值：motion_control.c:2093 `0.05`. */
#define HYD_THRESH_PRESSURE_CEILING_DEBOUNCE_S        0.05

/* 段启动后多久内抑制 PRESSURE_CEILING_EXCEEDED 误报（pressure 在压力建立
 * 阶段会有大幅瞬态）。
 * 单位：秒。原值：motion_control.c:2094 `0.10`. */
#define HYD_THRESH_PRESSURE_CEILING_STARTUP_SUPPRESS_S 0.10

/* PRESSURE_CEILING_EXCEEDED 持续多少秒后升级为 PRESSURE_CEILING_VIOLATED（FAULT）.
 * 单位：秒。原值：motion_control.c:2096 `0.30`. */
#define HYD_THRESH_PRESSURE_CEILING_FAULT_ESCALATION_S 0.30

/* --- RBF-PID 数值死区与滤波（src/rbf_pid.c） --- */

/* 设定值归一化绝对值低于该阈值视为"零设定"，直接清零输出，避免
 * 控制器在小目标下放大噪声。
 * 单位：归一化（无量纲）。原值：rbf_pid.c:327,345 `1e-6f`. */
#define HYD_THRESH_RBF_SETPOINT_ZERO_EPS     1e-6f

/* 设定值零判定时的反馈死区——只有反馈也接近零才置 0 输出。
 * 单位：归一化（无量纲）。原值：rbf_pid.c:327,345 `0.02f`. */
#define HYD_THRESH_RBF_FEEDBACK_ZERO_BAND    0.02f

/* 前馈加速度补偿激活的最小压力二阶差分（绝对值）.
 * 原值：rbf_pid.c:312,314 `0.0001f`. */
#define HYD_THRESH_RBF_FF_ACCEL_DEAD_BAND    0.0001f

/* 一阶低通滤波器系数（微分项 raw_de 滤波）.
 * `delta_temp = (1 - alpha) * prev + alpha * raw_de`.
 * 原值：rbf_pid.c:297 `0.12f`. */
#define HYD_THRESH_RBF_DERIV_FILTER_ALPHA    0.12f

/* 自适应学习率缩放下界（误差归一化后）.
 * 原值：rbf_pid.c:273 `0.01f`. */
#define HYD_THRESH_RBF_ETA_SCALE_MIN         0.01f

/* 自适应学习率缩放倍数（误差归一化 × 倍数 → eta_scale，再 clamp 到 [min,1]）.
 * 原值：rbf_pid.c:273 `4.0f`. */
#define HYD_THRESH_RBF_ETA_SCALE_GAIN        4.0f

```

把原先的"`#define HYD_MAX_AXIS_MOTION  20`"行保留在上述章节后，紧接 `15. 配置导出接口` 之前。

- [ ] **Step 2.2: 在 `src/motion_control.c` 替换 stopping-branch 魔法数**

加入 `#include "hyd_config.h"`（如果没有则在文件顶部 include 区域加入；hyd_config.h 应已被 common_types.h 间接 include，但显式 include 更安全；`grep` 确认后再决定）：

Run:
```bash
grep -n '#include "hyd_config.h"' src/motion_control.c
```

如果命中 0 次，且 `common_types.h` 已经 include 了 `hyd_config.h`（已确认是的），则无需再 include。

定位 line 1824：
```c
if (decelMag < 0.001f && fabs(fb->AXIS_REF.velocity) < 0.01f) {
```
替换为：
```c
if (decelMag < HYD_THRESH_STOP_DECEL_DONE_MAG &&
    fabs(fb->AXIS_REF.velocity) < HYD_THRESH_STOP_VEL_DONE_MAG) {
```

定位 line 1839：
```c
HYD_REAL stopTimeoutLimit = 5.0f * idealStopTime + 1.0f;
```
替换为：
```c
HYD_REAL stopTimeoutLimit = HYD_THRESH_STOP_TIMEOUT_IDEAL_MULT * idealStopTime +
                            HYD_THRESH_STOP_TIMEOUT_SLACK_S;
```

定位 line 1866：
```c
if (fb->_isDecelerating && fabs(plannerOutput.targetVelocity) < 0.001) {
```
替换为：
```c
if (fb->_isDecelerating &&
    fabs(plannerOutput.targetVelocity) < HYD_THRESH_DECEL_TARGET_VEL_DONE) {
```

- [ ] **Step 2.3: 替换 motion_control.c Init 中 pressure-ceiling criteria 常量**

定位 line 2093-2096（在 `HYD_MotionControlFB_Init` 内）：
```c
fb->_pressureCeilingCriteria.debounceTime = 0.05;          /* 50 ms — react faster than normal pressure deviation */
fb->_pressureCeilingCriteria.startupSuppressTime = 0.10;
fb->_pressureCeilingCriteria.enableStartupSuppress = true;
fb->_pressureCeilingCriteria.faultEscalationTime = 0.30;   /* 300 ms above ceiling -> escalate to FAULT/STOP */
```

替换为：
```c
fb->_pressureCeilingCriteria.debounceTime = HYD_THRESH_PRESSURE_CEILING_DEBOUNCE_S;
fb->_pressureCeilingCriteria.startupSuppressTime = HYD_THRESH_PRESSURE_CEILING_STARTUP_SUPPRESS_S;
fb->_pressureCeilingCriteria.enableStartupSuppress = true;
fb->_pressureCeilingCriteria.faultEscalationTime = HYD_THRESH_PRESSURE_CEILING_FAULT_ESCALATION_S;
```

- [ ] **Step 2.4: 替换 src/rbf_pid.c 中的 6 处魔法数**

文件顶部增加 include：

```c
#include "hyd_config.h"
```

放在 `#include "rbf_pid.h"` 之后、`#include <math.h>` 之前。

定位 line 273：
```c
pid->eta_scale = LIMIT(0.01f, error_norm * 4.0f, 1.0f);
```
替换为：
```c
pid->eta_scale = LIMIT(HYD_THRESH_RBF_ETA_SCALE_MIN,
                       error_norm * HYD_THRESH_RBF_ETA_SCALE_GAIN,
                       1.0f);
```

定位 line 297：
```c
delta_temp = (1.0f - 0.12f) * pid->delta_temp_prev + 0.12f * raw_de;
```
替换为：
```c
delta_temp = (1.0f - HYD_THRESH_RBF_DERIV_FILTER_ALPHA) * pid->delta_temp_prev +
             HYD_THRESH_RBF_DERIV_FILTER_ALPHA * raw_de;
```

定位 line 312-314：
```c
if (fddPress < -0.0001f) {
    pid->fUffAcc = pid->fKff_a_neg * fddPress;
} else if (fddPress > 0.0001f) {
```
替换为：
```c
if (fddPress < -HYD_THRESH_RBF_FF_ACCEL_DEAD_BAND) {
    pid->fUffAcc = pid->fKff_a_neg * fddPress;
} else if (fddPress > HYD_THRESH_RBF_FF_ACCEL_DEAD_BAND) {
```

定位 line 327：
```c
if (ABS(pid->Setpoint) < 1e-6f && pid->Feedback < 0.02f) {
    pid->Output = 0.0f;
}
```
替换为：
```c
if (ABS(pid->Setpoint) < HYD_THRESH_RBF_SETPOINT_ZERO_EPS &&
    pid->Feedback < HYD_THRESH_RBF_FEEDBACK_ZERO_BAND) {
    pid->Output = 0.0f;
}
```

定位 line 345（在 EnableFF 分支内的最终限幅后）：
```c
if (ABS(pid->Setpoint) < 1e-6f && pid->Feedback < 0.02f) {
    output_total = 0.0f;
}
```
替换为：
```c
if (ABS(pid->Setpoint) < HYD_THRESH_RBF_SETPOINT_ZERO_EPS &&
    pid->Feedback < HYD_THRESH_RBF_FEEDBACK_ZERO_BAND) {
    output_total = 0.0f;
}
```

- [ ] **Step 2.5: 构建 + 全量回归**

Run:
```bash
cmake --build --preset unixgcc 2>&1 | tail -20
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -30
```

Expected:
- 构建无 warning（特别是无 `redefinition` 或 `previous declaration` 警告）。
- ctest：`100% tests passed`。本任务是纯常量化，**任何测试 FAIL 都说明替换错了**。如果失败，git diff 确认替换处是否漏改括号、单位错配等。

- [ ] **Step 2.6: Commit**

```bash
git add -A
git commit -m "$(cat <<'EOF'
refactor: consolidate runtime numerical thresholds into hyd_config.h

Hoist 11 tuning-type magic floats (stop-completion threshold, pressure
ceiling debounce, RBF derivative-filter alpha, RBF setpoint dead-band, etc.)
from motion_control.c and rbf_pid.c into a new "14B. Internal Numerical
Thresholds" section in hyd_config.h. Math invariants (0.5 halving, 1.0
normalization, 2.0 multiplier) are deliberately left in source context.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: 拆分 `RunRunningState` → 3 个 helper（spec §3.2）

**目标：** 把 `HYD_MotionControlFB_RunRunningState`（当前 ~240 行，行 1667-1929）拆成 dispatcher + 3 个 `static` helper，按"主控制循环（normal）/ 停车减速分支（stopping）/ 直挂段 blend cutover（blend-cutover）"切分。**纯重构，不改行为**——回归门禁是现有 35+ 个集成测试。

**拆分原则：**
- `RunRunningState` 保留为公共入口，负责所有"早返回守卫"（fb null check、_activeSegmentValid、index range、AxisRef 有效性、timestamp rollback、config validation）和"主控制管线"（ramp / planner / pressure / pump / limiter / vp / lastCommandedFlow 写入）。
- 控制管线完成后，按 `_isStopping` / `HYD_ShouldCutoverDirectBlend` / `_isDecelerating` / `segmentCompleted` 等条件分派到 helper：
  - **`HYD_RunRunningStateStopping`**：line 1786-1851 的 `if (fb->_isStopping)` 整块——重写减速曲线、覆盖 plannerOutput/pumpOutput、stop-watchdog、ProtectionAction 状态机。
  - **`HYD_RunRunningStateBlendCutover`**：line 1853-1864 的 `protectionAction == STOP` + `HYD_ShouldCutoverDirectBlend` 两个 early-cutover 分支——这两个都属于"打断主循环、直接进入下一段或 fault"。
  - **`HYD_RunRunningStateCompletion`**：line 1866-1910 的 `_isDecelerating drop-to-zero` 分支 + `HYD_SegmentCompletion_CheckWithContext` 后续完成分派——属于"判段完后的清理"。
- 最后 line 1912-1928（report execution + sim feedback + clear SEGMENT_COMPLETED）保留在 dispatcher 末尾的 normal-path。

**Files:**
- Modify: `src/motion_control.c`（提取 3 个 static helper + 修改 RunRunningState）

### Steps

- [ ] **Step 3.1: 在拆分前快速记录当前 RunRunningState 结构（防漏改）**

Run:
```bash
sed -n '1667,1929p' src/motion_control.c | wc -l
grep -n "^static void\|^void HYD_MotionControlFB_RunRunningState" src/motion_control.c | head -5
```

Expected:
- 行数约 263（含空行）。
- `HYD_MotionControlFB_RunRunningState` 在 line 1667 开始。

如有任何近期改动让行号偏移，把"line 1667-1929"理解为"`HYD_MotionControlFB_RunRunningState` 函数体整体"。

- [ ] **Step 3.2: 在 `RunRunningState` 顶上声明 3 个新 static helper**

定位 line 1667 之前（在 `static void HYD_MotionControlFB_RunRunningState(HYD_MotionControlFB* fb);` 的 forward declaration 处，约 line 28），追加：

```c
static void HYD_MotionControlFB_RunRunningState(HYD_MotionControlFB* fb);
static void HYD_RunRunningStateStopping(HYD_MotionControlFB* fb,
                                        const HYD_MotionSegment* segment,
                                        HYD_MotionPlannerOutput* plannerOutput,
                                        HYD_PumpConverterOutput* pumpOutput,
                                        HYD_ExecutionReference* executionReference,
                                        HYD_PressureControllerOutput* pressureOutput);
static HYD_BOOL HYD_RunRunningStateBlendCutover(HYD_MotionControlFB* fb,
                                                const HYD_MotionSegment* segment,
                                                const HYD_ExecutionReference* executionReference);
static HYD_BOOL HYD_RunRunningStateCompletion(HYD_MotionControlFB* fb,
                                              const HYD_MotionSegment* segment,
                                              const HYD_MotionPlannerOutput* plannerOutput,
                                              const HYD_ExecutionReference* executionReference);
```

forward declaration 顺序与实现顺序保持一致（dispatcher → stopping → blend cutover → completion）。

- [ ] **Step 3.3: 提取 `HYD_RunRunningStateStopping`**

在 `HYD_MotionControlFB_RunRunningState` 函数体**之后**插入新函数。把 line 1786-1851 整块复制为新函数体（注意：要把对 `fb` / `segment` / `plannerOutput` / `pumpOutput` / `executionReference` / `pressureOutput` 的引用改成参数）：

```c
static void HYD_RunRunningStateStopping(HYD_MotionControlFB* fb,
                                        const HYD_MotionSegment* segment,
                                        HYD_MotionPlannerOutput* plannerOutput,
                                        HYD_PumpConverterOutput* pumpOutput,
                                        HYD_ExecutionReference* executionReference,
                                        HYD_PressureControllerOutput* pressureOutput) {
    HYD_REAL stopElapsed = fb->AXIS_REF.timestamp - fb->_stopStartTime;
    HYD_REAL stopMag = fabs(fb->_stopStartVel);
    HYD_REAL stopSign = (fb->_stopStartVel >= 0.0f) ? 1.0f : -1.0f;
    HYD_REAL stopDeceleration = (fb->_stopDeceleration > 0.0f)
        ? fb->_stopDeceleration
        : ((segment->maxDeceleration > 0.0f) ? segment->maxDeceleration : segment->maxAcceleration);
    HYD_REAL decelMag = stopMag - stopDeceleration * stopElapsed;

    if (decelMag < 0.0f) {
        decelMag = 0.0f;
    }

    plannerOutput->targetVelocity = decelMag * stopSign;
    plannerOutput->targetFlow = HYD_ClampReal(decelMag * segment->velocityToFlowGain,
                                              0.0f,
                                              segment->maxFlow);
    pumpOutput->commandFlow = plannerOutput->targetFlow;
    pumpOutput->pumpSpeed = HYD_ClampReal(plannerOutput->targetFlow * fb->FLOW_TO_PUMP_SPEED_GAIN,
                                          0.0f,
                                          fb->PUMP_SPEED_LIMIT);
    executionReference->flowReference = pumpOutput->commandFlow;
    executionReference->velocityReference = plannerOutput->targetVelocity;

    HYD_StateReporter_ReportExecution(fb,
                                      plannerOutput,
                                      pumpOutput,
                                      executionReference,
                                      pressureOutput->appliedStrategy,
                                      pressureOutput,
                                      &fb->DIAGNOSTIC);

    fb->_simFeedback.targetPosition = segment->targetPosition;
    fb->_simFeedback.targetVelocity = plannerOutput->targetVelocity;
    fb->_simFeedback.targetFlow = pumpOutput->commandFlow;
    fb->_simFeedback.targetPressure = executionReference->pressureReference;
    fb->_simFeedback.valid = true;

    if (decelMag < HYD_THRESH_STOP_DECEL_DONE_MAG &&
        fabs(fb->AXIS_REF.velocity) < HYD_THRESH_STOP_VEL_DONE_MAG) {
        fb->_isStopping = false;
        fb->_stopStartVel = 0.0f;
        fb->_stopDeceleration = 0.0f;
        fb->_directSessionState = HYD_DIRECT_SESSION_DONE;
        HYD_ClearDirectPendingSlot(fb);
        HYD_SafetyStateManager_ApplyIdleState(fb, true, false);
        HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_DONE);
    } else if (stopDeceleration > 0.0f) {
        /* C-4: stop-timeout safety net.
         * If the commanded deceleration ramp completes (decelMag ~= 0)
         * but feedback velocity never drops below the completion threshold
         * (stuck encoder / actuator), this branch would hang forever.
         * Threshold = HYD_THRESH_STOP_TIMEOUT_IDEAL_MULT * ideal-stop + slack. */
        HYD_REAL idealStopTime = stopMag / stopDeceleration;
        HYD_REAL stopTimeoutLimit = HYD_THRESH_STOP_TIMEOUT_IDEAL_MULT * idealStopTime +
                                    HYD_THRESH_STOP_TIMEOUT_SLACK_S;
        if (stopElapsed > stopTimeoutLimit) {
            fb->_directSessionState = HYD_DIRECT_SESSION_FAULT;
            HYD_StateReporter_ReportFault(fb,
                                          HYD_DIAG_CODE_TIMEOUT,
                                          fb->AXIS_REF.timestamp,
                                          segment,
                                          &fb->STATE.references);
            HYD_SafetyStateManager_EnterFaultStop(fb);
        }
    }
}
```

注意：
- 函数体最后**没有** `return`——dispatcher 调用后**必定 return**（不再继续主循环）。
- `HYD_ProtectionManager_*` 名字已被 Task 1 改为 `HYD_SafetyStateManager_*`，本步骤直接用新名。
- 魔法数已被 Task 2 改为命名常量。

- [ ] **Step 3.4: 提取 `HYD_RunRunningStateBlendCutover`**

在 stopping helper 之后追加：

```c
/* Returns true if the caller should immediately return (cutover/fault transition occurred). */
static HYD_BOOL HYD_RunRunningStateBlendCutover(HYD_MotionControlFB* fb,
                                                const HYD_MotionSegment* segment,
                                                const HYD_ExecutionReference* executionReference) {
    if (fb->DIAGNOSTIC.protectionAction == HYD_PROTECTION_ACTION_STOP) {
        fb->_directSessionState = HYD_DIRECT_SESSION_FAULT;
        HYD_SafetyStateManager_EnterFaultStop(fb);
        HYD_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, executionReference);
        return true;
    }

    if (HYD_ShouldCutoverDirectBlend(fb, segment)) {
        HYD_RecordDirectExecutionCompleted(fb);
        (void)HYD_StartPendingDirectSlot(fb, fb->AXIS_REF.timestamp, true);
        return true;
    }

    return false;
}
```

- [ ] **Step 3.5: 提取 `HYD_RunRunningStateCompletion`**

```c
/* Returns true if the segment completed and the caller should return immediately. */
static HYD_BOOL HYD_RunRunningStateCompletion(HYD_MotionControlFB* fb,
                                              const HYD_MotionSegment* segment,
                                              const HYD_MotionPlannerOutput* plannerOutput,
                                              const HYD_ExecutionReference* executionReference) {
    HYD_SegmentCompletionContext completionContext;
    HYD_BOOL segmentCompleted;
    HYD_BOOL recipeFinished;
    HYD_SegmentSource completedSegmentSource;

    if (fb->_isDecelerating &&
        fabs(plannerOutput->targetVelocity) < HYD_THRESH_DECEL_TARGET_VEL_DONE) {
        completedSegmentSource = fb->_activeSegmentSource;
        if (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT &&
            fb->_directPendingValid) {
            (void)HYD_StartPendingDirectSlot(fb, fb->AXIS_REF.timestamp, false);
            return true;
        }
        recipeFinished = (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) ||
            (fb->STATE.currentSegmentIndex + 1U >= fb->RECIPE_SIZE);
        HYD_SafetyStateManager_ApplyIdleState(fb, recipeFinished, true);
        HYD_StateReporter_SetSegmentSource(fb, completedSegmentSource);
        HYD_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, executionReference);
        return true;
    }

    completionContext.segment = segment;
    completionContext.axisRef = &fb->AXIS_REF;
    completionContext.references = executionReference;
    completionContext.timestamp = fb->AXIS_REF.timestamp;
    completionContext.candidateStartTime = &fb->_completionCandidateStartTime;
    completionContext.candidateActive = &fb->_completionCandidateActive;
    segmentCompleted = HYD_SegmentCompletion_CheckWithContext(&completionContext);
    if (!segmentCompleted) {
        return false;
    }

    if (!fb->_isDecelerating &&
        segment->mode == HYD_MODE_SPEED_RAMP &&
        segment->endCondition != HYD_END_POSITION) {
        fb->_isDecelerating = true;
        fb->_decelStartTime = fb->AXIS_REF.timestamp;
        fb->_decelStartVel = fabs(plannerOutput->targetVelocity);
        return false;   /* continue execution to let deceleration take effect */
    }

    completedSegmentSource = fb->_activeSegmentSource;
    if (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT &&
        fb->_directPendingValid) {
        (void)HYD_StartPendingDirectSlot(fb, fb->AXIS_REF.timestamp, false);
        return true;
    }
    recipeFinished = (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) ||
        (fb->STATE.currentSegmentIndex + 1U >= fb->RECIPE_SIZE);
    HYD_SafetyStateManager_ApplyIdleState(fb, recipeFinished, true);
    HYD_StateReporter_SetSegmentSource(fb, completedSegmentSource);
    HYD_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, executionReference);
    return true;
}
```

- [ ] **Step 3.6: 改造 `HYD_MotionControlFB_RunRunningState` dispatcher**

定位 line 1786 起的整块 `if (fb->_isStopping) { ... }` （到 `return;` 结束），用以下代码替换：

```c
    if (fb->_isStopping) {
        HYD_RunRunningStateStopping(fb, segment, &plannerOutput, &pumpOutput,
                                    &executionReference, &pressureOutput);
        return;
    }

    if (HYD_RunRunningStateBlendCutover(fb, segment, &executionReference)) {
        return;
    }

    if (HYD_RunRunningStateCompletion(fb, segment, &plannerOutput, &executionReference)) {
        return;
    }
```

把替换前 line 1853-1910 的所有内容（包括 `protectionAction == STOP` 分支、`HYD_ShouldCutoverDirectBlend` 分支、`_isDecelerating drop-to-zero` 分支、`HYD_SegmentCompletion_CheckWithContext` 分派）**全部删除**——它们都已迁移到 helper。

dispatcher 结尾保持原样（line 1912-1928，ReportExecution + sim feedback + `SEGMENT_COMPLETED = false`）。

- [ ] **Step 3.7: 构建 + 全量回归（核心门禁）**

Run:
```bash
cmake --build --preset unixgcc 2>&1 | tail -30
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -50
```

Expected:
- 构建无 warning（特别注意 unused-variable / declared-but-not-defined 警告——把它们当作错误来处理）。
- ctest：`100% tests passed`。如果有 FAIL，**逐个看失败用例的 stderr**，常见原因：
  - 漏 return（dispatcher 主循环没在 helper 返回 true 时停下）
  - 状态写错（应改 `pressureOutput->appliedStrategy` 但写成了 `pressureOutput.appliedStrategy`）
  - 应通过指针修改的字段忘记修改（plannerOutput / pumpOutput / executionReference 是通过指针引用，所以 helper 内对它们的赋值必须是 `->`）

- [ ] **Step 3.8: 重构验证——dispatcher 大小检查**

Run:
```bash
awk '/^static void HYD_MotionControlFB_RunRunningState\(/,/^}$/' src/motion_control.c | wc -l
```

Expected: dispatcher 行数应在 80-110 之间（原 240+ 行的 1/2 左右）。如果还在 200+ 行，说明 helper 没真正提取，重新检查 Step 3.6 是否漏删旧代码。

- [ ] **Step 3.9: Commit**

```bash
git add -A
git commit -m "$(cat <<'EOF'
refactor: split RunRunningState into normal/stopping/blend-cutover helpers

The 240+ line RunRunningState was a maintenance hotspot (Sprint 3 / spec §3.2).
Extract three static helpers in motion_control.c:
- HYD_RunRunningStateStopping: stop-deceleration ramp + watchdog (was inline ~85L)
- HYD_RunRunningStateBlendCutover: protection-stop and direct-blend cutover
- HYD_RunRunningStateCompletion: _isDecelerating drop-to-zero + segment-complete

Pure refactor — behavior unchanged; 35+ existing integration tests are the
regression gate. Dispatcher reduces from ~240 to ~100 lines.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: RBF-PID — 让 `MAX_PRESSURE` 归一化可配置（spec §3.4 第 1 部分）

**目标：** 当前 `RBF_PID_Update` 用硬编码 `MAX_PRESSURE=250.0f`（来自 `rbf_pid.h` 宏）做归一化。这意味着任何系统压力满量程 ≠ 250 MPa 的项目都拿不到正确归一化值，导致 RBF 网络的输入维度被错误压缩。把它改成 `RBF_PID_Handle` 的运行时字段，由 `pressure_controller.c` 在每段开始时按段配置写入。

**TDD 失败用例：** 调用方传入 `setpoint=400, feedback=200`，期望归一化为 `Setpoint=0.5, Feedback=0.25`（基于 `pressure_normalization_scale=800.0f`）。如果归一化仍按 250.0 计算，则 Setpoint 会被 clamp 到 1.6（或者根本不被 clamp，但显然超出 [0,1] 设计范围），断言会失败。

**Files:**
- Modify: `include/rbf_pid.h`（结构体新增字段 + 新 setter 声明）
- Modify: `src/rbf_pid.c`（Init 默认值 + Update 用字段而非宏 + 新 setter 实现）
- Modify: `src/pressure_controller.c`（每段写入 `pressure_normalization_scale`）
- Modify: `tests/rbf_pid_test.c`（追加一个新测试用例，验证可配置归一化）

### Steps

- [ ] **Step 4.1: 写失败测试 - `test_pressure_normalization_is_configurable`**

在 `tests/rbf_pid_test.c` 中（在 `int main(void)` 之前）追加新测试函数：

```c
static void test_pressure_normalization_is_configurable(void) {
    RBF_PID_Handle pid;

    printf("Testing configurable pressure normalization scale...\n");
    RBF_PID_Init(&pid, 0.01f, 1500.0f, 1.0f);
    pid.enable = true;

    /* Default: pressure_normalization_scale == 0 -> falls back to MAX_PRESSURE (250) */
    (void)RBF_PID_Update(&pid, 250.0f, 125.0f);
    assert(fabsf(pid.Setpoint - 1.0f) < 1e-4f);
    assert(fabsf(pid.Feedback - 0.5f) < 1e-4f);

    /* Configure custom scale = 800.0 MPa, then 400/200 should normalize to 0.5/0.25 */
    RBF_PID_Reset(&pid);
    RBF_PID_SetPressureNormalization(&pid, 800.0f);
    pid.enable = true;
    (void)RBF_PID_Update(&pid, 400.0f, 200.0f);
    assert(fabsf(pid.Setpoint - 0.5f) < 1e-4f);
    assert(fabsf(pid.Feedback - 0.25f) < 1e-4f);

    /* Zero / negative scale must fall back to the macro default */
    RBF_PID_Reset(&pid);
    RBF_PID_SetPressureNormalization(&pid, 0.0f);
    pid.enable = true;
    (void)RBF_PID_Update(&pid, 250.0f, 125.0f);
    assert(fabsf(pid.Setpoint - 1.0f) < 1e-4f);

    printf("✓ Configurable pressure normalization test passed\n");
}
```

并在 `main()` 内增加调用（位置：在 `test_multi_axis_differentiated_seeds()` 之前）：

```c
test_pressure_normalization_is_configurable();
```

- [ ] **Step 4.2: 运行测试确认失败**

Run:
```bash
cmake --build --preset unixgcc 2>&1 | tail -20
```

Expected:
- 构建失败：`undefined reference to RBF_PID_SetPressureNormalization`。
- 这是预期失败——TDD 第一阶段。

- [ ] **Step 4.3: 在 `include/rbf_pid.h` 添加字段与 setter 声明**

在 `RBF_PID_Handle` 结构体中找到 `/* 输入参数（用户配置） */` 区块（约 line 35-47），在 `bool Reset;` 之后追加：

```c
    /* 压力归一化标量（MPa 等设定/反馈单位的满量程）.
     * 0 或负值 -> 落回内置默认 MAX_PRESSURE. 调用 RBF_PID_SetPressureNormalization()
     * 配置；推荐由 pressure_controller.c 在每段 Resolve 时根据段配置写入。 */
    float pressure_normalization_scale;
```

在文件末尾的 `void RBF_PID_SetSeed(...)` 声明**之前**追加：

```c
/**
 * @brief 配置压力归一化标量
 * @param pid RBF_PID句柄指针
 * @param scale 满量程标量（单位与 setpoint/feedback 相同，例如 MPa）
 *              传 0 或负值会清回内部默认 MAX_PRESSURE.
 * @note 推荐在每段开始时调用一次；运行中改变会导致归一化基准跳变。
 */
void RBF_PID_SetPressureNormalization(RBF_PID_Handle *pid, float scale);

```

- [ ] **Step 4.4: 在 `src/rbf_pid.c` 实现 setter 并修改 Init / Update**

在文件末尾追加 setter 实现：

```c
void RBF_PID_SetPressureNormalization(RBF_PID_Handle *pid, float scale) {
    if (pid == NULL) {
        return;
    }
    pid->pressure_normalization_scale = (scale > 0.0f) ? scale : 0.0f;
}
```

定位 `RBF_PID_Init` 内（约 line 80），在 `memset(pid, 0, sizeof(RBF_PID_Handle));` 之后、`pid->sampling_period = ...` 之前追加：

```c
    pid->pressure_normalization_scale = 0.0f;  /* 0 -> fall back to MAX_PRESSURE */
```

定位 `RBF_PID_Update` 内 line 234-235：
```c
pid->Setpoint = setpoint / MAX_PRESSURE;
pid->Feedback = feedback / MAX_PRESSURE;
```
替换为：
```c
{
    float scale = (pid->pressure_normalization_scale > 0.0f)
                  ? pid->pressure_normalization_scale
                  : MAX_PRESSURE;
    pid->Setpoint = setpoint / scale;
    pid->Feedback = feedback / scale;
}
```

- [ ] **Step 4.5: 运行测试确认通过**

Run:
```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc -R '^test_rbf_pid$' --output-on-failure
```

Expected:
- 构建无错误。
- `test_rbf_pid` 通过；测试输出含 `✓ Configurable pressure normalization test passed`。

- [ ] **Step 4.6: 把段级压力满量程接入 `pressure_controller.c`**

定位 `HYD_EnsureRbfPidInitialized` / `HYD_ApplyRbfPidConfig`（约 line 260-313）。

`HYD_ApplyRbfPidConfig` 当前签名是 `(state, config)`——它**没有** segment 参数，但实际归一化标量需要从 segment 取（详见 Step 4.7）。两种选项：
1. 给 `HYD_ApplyRbfPidConfig` 增加 `segment` 参数（修改签名 + caller），把 RBF normalization 的设置内联进去；
2. 在调用 `HYD_ApplyRbfPidConfig` 的每个上游 callsite，紧接其后追加一段 `RBF_PID_SetPressureNormalization()`。

**推荐方案 1**——签名变更范围小（只有 2 个 caller：`HYD_SynchronizeRbfPidState`、可能还有别的初始化路径），可以保证"任何调用 ApplyRbfPidConfig 的地方都正确刷新归一化"。

修改后签名：
```c
static void HYD_ApplyRbfPidConfig(HYD_PressureControllerState* state,
                                  const HYD_PressureResolvedConfig* config,
                                  const HYD_MotionSegment* segment);
```

更新 `HYD_SynchronizeRbfPidState` 内对它的调用，传入 segment（来自 `HYD_PressureController_Execute` 入口已有的 segment 参数）。具体 segment 传递路径：

Run:
```bash
grep -nC 3 "HYD_SynchronizeRbfPidState\|HYD_ApplyRbfPidConfig\b" src/pressure_controller.c | head -40
```

确认 `HYD_SynchronizeRbfPidState` 的调用栈，让 segment 一路传到 `HYD_ApplyRbfPidConfig`。

具体代码改动见 Step 4.7。

- [ ] **Step 4.7: 让 segment 真正驱动归一化标量**

**注意：** `HYD_MotionSegment` **没有** `maxPressure` 字段。可用字段（参考 `include/common_types.h` line 253-320）：`targetPressure` / `pressureCeiling` / `pressureTolerance` / `pressureRampRate`。

最稳妥的归一化标量来源是 `pressureCeiling`（已是"压力上限"语义）；其次是 `targetPressure × 2`（保留经验加倍上限）。两者皆 0 时回落 RBF 内部默认 `MAX_PRESSURE`。

**做法：** 在 `HYD_ApplyRbfPidConfig` 的调用方（`HYD_SynchronizeRbfPidState` 内、`HYD_ApplyRbfPidConfig(state, config)` 之后）追加：

```c
    /* Per-segment RBF pressure normalization scale. Prefer pressureCeiling
     * (an explicit upper bound), else 2x targetPressure as a conservative
     * envelope, else leave 0 (RBF falls back to MAX_PRESSURE default). */
    {
        HYD_REAL pressureScale = 0.0;
        if (segment != NULL) {
            if (segment->pressureCeiling > 0.0) {
                pressureScale = segment->pressureCeiling;
            } else if (segment->targetPressure > 0.0) {
                pressureScale = segment->targetPressure * 2.0;
            }
        }
        RBF_PID_SetPressureNormalization(&state->rbfPid, (float)pressureScale);
    }
```

`HYD_SynchronizeRbfPidState` 签名内已经包含 `segment` 来源（通过 `config` 间接、或本就接收 segment——具体看 caller 代码并对齐）。如果它没有 segment，加一个 segment 参数；caller `HYD_PressureController_Execute` 已经有 segment 在手，传入即可。

**另一处插入点：** 如果 `HYD_PressureController_Execute` 在切换到 RBF 时 **不** 走 `HYD_SynchronizeRbfPidState`（即不是首次切换），仍应在每段开始时同步归一化标量。在 `HYD_ApplyRbfPidConfig` 末尾（line 313 之后）也追加同样一段——它会在每次配置应用时刷新。 修改 `HYD_ApplyRbfPidConfig` 签名为 `(state, config, segment)`，所有 caller 同步更新。

完成后，**修改 Step 4.1 测试**：把第二条断言改为：
```c
RBF_PID_SetPressureNormalization(&pid, 800.0f);
```
保留 — 因为这是直接驱动 RBF_PID_Handle 的单元测试，不经过 pressure_controller，所以仍然有效。

- [ ] **Step 4.8: 全量回归**

Run:
```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -20
```

Expected: `100% tests passed`，所有测试包括新 `test_pressure_normalization_is_configurable` 通过。

- [ ] **Step 4.9: Commit**

```bash
git add -A
git commit -m "$(cat <<'EOF'
feat: make RBF_PID pressure normalization scale configurable

Previously RBF_PID_Update hardcoded MAX_PRESSURE=250.0f for the setpoint/feedback
normalization step, which silently mis-scaled the RBF network inputs for any
system whose full-scale pressure is not 250 MPa.

Add pressure_normalization_scale field to RBF_PID_Handle, settable via
RBF_PID_SetPressureNormalization(). 0 or negative falls back to MAX_PRESSURE.
pressure_controller.c wires the segment-level pressureCeiling (or 2x targetPressure
fallback) into the handle on each Apply path so per-segment normalization is honoured.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: RBF-PID — 默认增益限幅窗扩展 + 段配置覆盖（spec §3.4 第 2 部分）

**目标：** 当前 `rbf_pid.h` 中默认增益限幅窗 `[PID_MIN_KP=0.8, PID_MAX_KP=0.85]` 仅有 0.05 宽度，自适应学习几乎无空间。扩展默认窗到 `[0.5, 1.2]`（KP）、`[0.005, 0.05]`（KI）、`[0.5, 2.0]`（KD），并通过 `HYD_RbfPidConfig` 段字段覆盖，让段级整定能更细。

**TDD 失败用例：** 启动 RBF，运行 50 步对一个有阶跃误差的 setpoint，观察 KP 是否接近上限（说明窗太窄）或自然停在中间（说明窗合理）。在新窗下 KP 应在 [0.5, 1.2] 内有所漂移；旧窗 KP 几乎一直顶在 0.85（max）。

**Files:**
- Modify: `include/rbf_pid.h`（PID_MIN/MAX_KP 等宏值变化 + 注释说明）
- Modify: `src/rbf_pid.c`（KP/KI/KD 初值校对，必须落在新窗内）
- Modify: `tests/rbf_pid_test.c`（追加测试）

### Steps

- [ ] **Step 5.1: 写失败测试 - `test_default_gain_window_allows_adaptation`**

在 `tests/rbf_pid_test.c` 中追加：

```c
static void test_default_gain_window_allows_adaptation(void) {
    RBF_PID_Handle pid;
    float kp_min_observed = 1e9f;
    float kp_max_observed = -1e9f;
    int step;

    printf("Testing default gain window allows adaptation room...\n");
    RBF_PID_Init(&pid, 0.01f, 1500.0f, 1.0f);
    pid.enable = true;

    /* Drive a step error scenario for 50 steps */
    for (step = 0; step < 50; step++) {
        float feedback = (step < 25) ? 0.0f : 50.0f;
        (void)RBF_PID_Update(&pid, 100.0f, feedback);
        if (pid.KP < kp_min_observed) kp_min_observed = pid.KP;
        if (pid.KP > kp_max_observed) kp_max_observed = pid.KP;
    }

    /* Default window should be at least 0.5 wide (was 0.05 in legacy build);
     * adaptation should move KP at least 0.05 within the window — proving the
     * window is not so narrow that adaptation is suppressed. */
    assert((pid.max_KP - pid.min_KP) >= 0.5f);
    assert((kp_max_observed - kp_min_observed) > 0.05f);
    printf("✓ Default gain window adaptation test passed (kp range observed: %.3f-%.3f)\n",
           kp_min_observed, kp_max_observed);
}
```

在 `main()` 内追加调用（在 `test_adaptive_learning_rate_scales_with_error()` 之后）：

```c
test_default_gain_window_allows_adaptation();
```

- [ ] **Step 5.2: 运行测试确认失败**

Run:
```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc -R '^test_rbf_pid$' --output-on-failure 2>&1 | tail -20
```

Expected:
- 测试失败：assertion `(pid.max_KP - pid.min_KP) >= 0.5f` failed（旧默认 KP 窗是 0.05）。

- [ ] **Step 5.3: 扩展 `include/rbf_pid.h` 默认窗**

定位 line 18-24：
```c
/* PID参数限幅（与ST代码一致） */
#define PID_MIN_KP          0.8f
#define PID_MAX_KP          0.85f
#define PID_MIN_KI          0.018f
#define PID_MAX_KI          0.03f
#define PID_MIN_KD          1.2f
#define PID_MAX_KD          1.5f
```

替换为：
```c
/* PID 参数限幅 — 默认窗按"压力闭环 + 自适应学习"调参余量给出。
 * Sprint 0 review C-6：原值窗宽 0.05，自适应增益几乎无空间。
 * Sprint 3 扩展：KP/KD 窗宽 0.7+，KI 窗宽 0.045。
 * 各项目可通过 HYD_RbfPidConfig 段字段进一步覆盖。 */
#define PID_MIN_KP          0.5f
#define PID_MAX_KP          1.2f
#define PID_MIN_KI          0.005f
#define PID_MAX_KI          0.050f
#define PID_MIN_KD          0.5f
#define PID_MAX_KD          2.0f
```

- [ ] **Step 5.4: 校对 `src/rbf_pid.c` 中 KP/KI/KD 初值，确保落在新窗内**

定位 `RBF_PID_Init`（line 102-105）：
```c
pid->KP = 0.03f;
pid->KI = 0.02f;
pid->KD = 0.03f;
```

`0.03` 不在 `[0.5, 1.2]` 内、`0.02` 不在 `[0.005, 0.050]` 内（恰好等于上限附近）、`0.03` 不在 `[0.5, 2.0]` 内。改为新窗的几何中点：

```c
pid->KP = 0.8f;   /* mid of [0.5, 1.2] */
pid->KI = 0.020f; /* near max of [0.005, 0.050], reflecting typical hydraulic-loop integral */
pid->KD = 1.0f;   /* mid of [0.5, 2.0] */
```

同样修改 `RBF_PID_Reset`（line 136-138）：
```c
pid->KP = 0.03f;
pid->KI = 0.02f;
pid->KD = 0.03f;
```

改为：
```c
pid->KP = 0.8f;
pid->KI = 0.020f;
pid->KD = 1.0f;
```

以及 `RBF_PID_Update` 内 `if (pid->FirstScan || pid->Reset)` 分支（line 225-227）：
```c
pid->KP = 0.03f;
pid->KI = 0.02f;
pid->KD = 0.03f;
```

改为：
```c
pid->KP = 0.8f;
pid->KI = 0.020f;
pid->KD = 1.0f;
```

- [ ] **Step 5.5: 修正 `test_explicit_reset_restores_runtime_state` 的断言**

定位 `tests/rbf_pid_test.c` line 67-69：
```c
assert(fabsf(pid.KP - 0.03f) < 1e-6f);
assert(fabsf(pid.KI - 0.02f) < 1e-6f);
assert(fabsf(pid.KD - 0.03f) < 1e-6f);
```

改为：
```c
assert(fabsf(pid.KP - 0.8f) < 1e-6f);
assert(fabsf(pid.KI - 0.020f) < 1e-6f);
assert(fabsf(pid.KD - 1.0f) < 1e-6f);
```

- [ ] **Step 5.6: 修正 `test_enabled_controller_respects_limits_and_drives_feedback` 的限幅 setter**

定位 `tests/rbf_pid_test.c` line 29：
```c
RBF_PID_SetParamLimits(&pid, 0.80f, 0.90f, 0.018f, 0.040f, 1.20f, 1.60f);
```

这个测试显式覆盖了限幅窗（合法用例，模拟段配置）。把它改为更宽的覆盖窗，与新的"窗宽 >= 0.1"约定一致：
```c
RBF_PID_SetParamLimits(&pid, 0.5f, 1.2f, 0.005f, 0.050f, 0.5f, 2.0f);
```

- [ ] **Step 5.7: 全量回归**

Run:
```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -30
```

Expected: `100% tests passed`，含新增 `test_default_gain_window_allows_adaptation`。如果 `test_adaptive_learning_rate_scales_with_error` 因新窗导致更剧烈漂移而失败（>5%），把它的容差从 `0.05f * kp_before + 0.01f` 放宽到 `0.10f * kp_before + 0.05f`（同步更新断言注释解释原因）。

- [ ] **Step 5.8: Commit**

```bash
git add -A
git commit -m "$(cat <<'EOF'
feat: widen RBF_PID default gain clamp windows for adaptation room

Original ST-derived clamp windows ([0.8, 0.85] for KP, etc.) were 0.05 wide,
leaving virtually no adaptation room for the on-line gradient updates. Widen
defaults: KP [0.5, 1.2], KI [0.005, 0.050], KD [0.5, 2.0]. Reseed Init/Reset/
Update initial gains to the new window midpoints. Test asserts adaptation now
moves KP by >0.05 over a 50-step step-response scenario.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: RBF-PID — 跨控制器策略切换时把增益种子 clamp 进 LIMIT 区间（spec §3.4 第 3 部分；I-5）

**目标：** 当 `pressureController` 从 PI/PID 切到 RBF_PID 时，`HYD_SynchronizeRbfPidState` 调用 `RBF_PID_Reset` → `HYD_ApplyRbfPidConfig`。`RBF_PID_Reset` 把 KP/KI/KD 设为 `Init` 默认（Task 5 改为 0.8/0.020/1.0），但 `HYD_ApplyRbfPidConfig` 仅写 `min/max_KP` 等界限，**不重置 `pid->KP/KI/KD`**。如果段配置的窗与默认窗不重叠（如段窗 `[1.5, 2.0]`，默认 0.8 不在窗内），首次 `RBF_PID_Update` 会用窗外的 0.8 计算输出，触发第一帧 KD 跳变。

修复：在 `HYD_ApplyRbfPidConfig` 末尾**统一 clamp** `pid->KP/KI/KD` 到当前段窗内（实际上 line 305-313 已经在做这件事，但仅在 `HYD_ApplyRbfPidConfig` 流程内做——需要确认是否在 `HYD_SynchronizeRbfPidState` 的 Reset 之后被正确触发）。

**TDD 失败用例：**
1. 段 A 用 `pressureController = HYD_PRESSURE_CONTROLLER_PI`，让 PI 计算稳定。
2. 段 B 切到 `pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID`，段配置 `pressureRbfConfig.minKp=1.5, maxKp=2.0`。
3. 在切换边界后第一次调用 `HYD_PressureController_Execute`，断言 `state->rbfPid.KP` 在 `[1.5, 2.0]` 内（不是 0.8）。

**Files:**
- Modify: `src/pressure_controller.c`（确认 / 加固 clamp 顺序）
- Modify: `tests/test_pressure_controller.c`（追加跨策略切换 clamp 测试）

### Steps

- [ ] **Step 6.1: 检视当前 `HYD_SynchronizeRbfPidState` 与 `HYD_ApplyRbfPidConfig` 的调用顺序**

Run:
```bash
grep -nC 4 "HYD_SynchronizeRbfPidState\|HYD_ApplyRbfPidConfig" src/pressure_controller.c | head -60
```

确认调用栈：
- `HYD_SynchronizeRbfPidState(state, ...)` 内调 `RBF_PID_Reset(&state->rbfPid)` 再调 `HYD_ApplyRbfPidConfig(state, config)`。
- `HYD_ApplyRbfPidConfig` 末尾已有 `state->rbfPid.KP = (float)HYD_ClampReal(...)`（line 305-313）。

如果这条调用链 OK，理论上切换后第一帧就已经 clamp 完了。**但**——`HYD_SynchronizeRbfPidState` 在哪些场景被调用？是否在所有控制器切换边界都触发？

Run:
```bash
grep -n "HYD_SynchronizeRbfPidState" src/pressure_controller.c
```

如果只在某个特定分支（如 `lastStrategy != HYD_PRESSURE_CONTROLLER_RBF_PID` 且 `currentStrategy == HYD_PRESSURE_CONTROLLER_RBF_PID` 的转换边界）调用，那么"反向切换"或"非首段进入"可能不触发；记录调用矩阵：

- 控制器从 PI → RBF：调用 ✓
- 控制器从 PID → RBF：调用 ✓
- 控制器从 RBF → PI：不调用（PI 无 Sync 路径），但 RBF 状态未被清理是否会污染下次进入？

具体由代码决定，本步只是审计；下一步根据审计结果决定改动范围。

- [ ] **Step 6.2: 写失败测试 - `test_cross_controller_switch_seeds_rbf_within_clamp_window`**

在 `tests/test_pressure_controller.c` 中追加新测试。**重要：** `HYD_PressureController_Execute` 实际签名是 `(segment, state, input, output)`（见 `include/pressure_controller.h:61-64`，且现有测试 line 51/83/86 均按此签名调用）：

```c
static void test_cross_controller_switch_seeds_rbf_within_clamp_window(void) {
    HYD_MotionSegment segmentA;
    HYD_MotionSegment segmentB;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    printf("Testing cross-controller PI->RBF switch clamp seeding...\n");

    /* Segment A: PI controller, brief working state. */
    memset(&segmentA, 0, sizeof(segmentA));
    memset(&segmentB, 0, sizeof(segmentB));
    memset(&state, 0, sizeof(state));
    memset(&input, 0, sizeof(input));

    segmentA.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segmentA.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segmentA.targetPressure = 20.0;
    segmentA.pressureKp = 0.5;
    segmentA.pressureKi = 0.1;
    segmentA.pressureIntegralLimit = 5.0;
    segmentA.maxFlow = 30.0;
    segmentA.pressureFilterAlpha = 1.0;
    segmentA.pressureDerivativeFilterAlpha = 1.0;

    HYD_PressureController_InitState(&state, 18.0, 5.0, 0.0);

    input.targetPressure = 20.0;
    input.measuredPressure = 18.0;
    input.feedforwardFlow = 5.0;
    input.outputMin = 0.0;
    input.outputMax = 30.0;
    input.timestamp = 0.0;
    HYD_PressureController_Execute(&segmentA, &state, &input, &output);
    assert(output.appliedStrategy == HYD_PRESSURE_CONTROLLER_PI);

    /* Segment B: RBF controller, window outside Init defaults [0.5, 1.2]. */
    segmentB.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segmentB.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segmentB.targetPressure = 30.0;
    segmentB.maxFlow = 30.0;
    segmentB.pressureFilterAlpha = 1.0;
    segmentB.pressureDerivativeFilterAlpha = 1.0;
    segmentB.pressureRbfConfig.minKp = 1.5;
    segmentB.pressureRbfConfig.maxKp = 2.0;
    segmentB.pressureRbfConfig.minKi = 0.005;
    segmentB.pressureRbfConfig.maxKi = 0.050;
    segmentB.pressureRbfConfig.minKd = 0.5;
    segmentB.pressureRbfConfig.maxKd = 2.0;

    input.targetPressure = 30.0;
    input.measuredPressure = 25.0;
    input.timestamp = 0.01;
    HYD_PressureController_Execute(&segmentB, &state, &input, &output);

    /* After PI->RBF switch with custom window, all 3 gains must be clamped into the new window. */
    assert(state.rbfPid.KP >= 1.5f - 1e-4f);
    assert(state.rbfPid.KP <= 2.0f + 1e-4f);
    assert(state.rbfPid.KI >= 0.005f - 1e-4f);
    assert(state.rbfPid.KI <= 0.050f + 1e-4f);
    assert(state.rbfPid.KD >= 0.5f - 1e-4f);
    assert(state.rbfPid.KD <= 2.0f + 1e-4f);
    /* Output must remain within [outputMin, outputMax] — no first-step spike. */
    assert(output.outputFlow >= 0.0);
    assert(output.outputFlow <= input.outputMax + 1e-4);
    printf("✓ Cross-controller PI->RBF clamp seeding test passed (KP=%.3f KI=%.4f KD=%.3f)\n",
           state.rbfPid.KP, state.rbfPid.KI, state.rbfPid.KD);
}
```

在该文件 `main()` 内追加调用。

**注：** `HYD_PressureControllerState` 已有 `activeStrategy` 字段（`include/pressure_controller.h:26`），它就是"上一次应用的 strategy"。Step 6.4 改动若需检测策略变化，应**复用** `state->activeStrategy` 而不要新加 `lastControllerType` 字段。

- [ ] **Step 6.3: 构建运行测试**

Run:
```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc -R '^test_pressure_controller$' --output-on-failure 2>&1 | tail -20
```

如果测试通过（说明现有 clamp 链已正确），跳到 Step 6.6 直接 commit；测试本身就是回归保护。

如果测试失败（KP 不在窗内），继续 Step 6.4 修复。

- [ ] **Step 6.4: 修复 `HYD_ApplyRbfPidConfig`，保证 clamp 一定执行**

定位 `HYD_ApplyRbfPidConfig`（约 line 284）。当前末尾 clamp 代码：

```c
state->rbfPid.KP = (float)HYD_ClampReal((HYD_REAL)state->rbfPid.KP,
                                        config->rbf.minKp,
                                        config->rbf.maxKp);
state->rbfPid.KI = (float)HYD_ClampReal((HYD_REAL)state->rbfPid.KI,
                                        config->rbf.minKi,
                                        config->rbf.maxKi);
state->rbfPid.KD = (float)HYD_ClampReal((HYD_REAL)state->rbfPid.KD,
                                        config->rbf.minKd,
                                        config->rbf.maxKd);
```

确认这段代码在 `HYD_EnsureRbfPidInitialized` 之后、`RBF_PID_SetParamLimits` 之后执行。如果是，则没问题。如果是，但**测试仍失败**，那是因为：
- `HYD_SynchronizeRbfPidState` 调用 `RBF_PID_Reset` → KP=0.8（Task 5 默认）
- 然后调用 `HYD_ApplyRbfPidConfig` → 写 `min/max_KP=1.5/2.0`
- 然后 clamp KP=0.8 进 [1.5, 2.0] → KP=1.5 ✓

这条链应该是对的。如果仍失败，可能是因为 `HYD_SynchronizeRbfPidState` **没有被调用**（只在初始进入 RBF 而非段切换时调用）。

修复方案：在 `HYD_PressureController_Execute` 入口检测"上一次应用的 strategy 与本帧 resolve 出的 strategy 不同"，如果不同则强制调用 `HYD_SynchronizeRbfPidState`（仅当新 controller 是 RBF）。复用现有 `HYD_PressureControllerState.activeStrategy` 字段（已存在，见 `include/pressure_controller.h:26`）作为"上次应用的策略"——它会在每次 `HYD_PressureController_Execute` 末尾被写为 `output->appliedStrategy`。

具体修改示例（在 `HYD_PressureController_Execute` 顶部，已 resolve config 之后，调用具体策略分支之前）：

```c
    HYD_PressureControllerType currentStrategy = config.controllerType;  /* 取自 resolved config */
    if (state->activeStrategy != currentStrategy) {
        if (currentStrategy == HYD_PRESSURE_CONTROLLER_RBF_PID) {
            HYD_SynchronizeRbfPidState(state,
                                       state->previousOutput,
                                       config.targetPressure,
                                       input->measuredPressure,
                                       &config);
        }
        /* activeStrategy will be updated below by the normal flow when output->appliedStrategy is set */
    }
```

字段名以 `HYD_PressureResolvedConfig` 的实际命名为准——先 grep 确认。如果 `controllerType` 字段不在 resolved config 中（只在 segment 中），则改为 `HYD_ResolvePressureControllerType(input->segment)`（已是现有 helper）。

- [ ] **Step 6.5: 构建运行测试（修复后）**

Run:
```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc -R '^test_pressure_controller$' --output-on-failure 2>&1 | tail -20
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -20
```

Expected: 新测试通过，全量回归 `100% tests passed`。

- [ ] **Step 6.6: Commit**

```bash
git add -A
git commit -m "$(cat <<'EOF'
fix: clamp RBF_PID gain seeds into segment window on controller switch

Crossing from PI/PID to RBF_PID (or between two RBF segments with different
HYD_RbfPidConfig windows) previously could leave KP/KI/KD at Init defaults
that lay outside the new segment window — the first Update would then clamp
them inside the window, producing a one-cycle KD spike at the controller
hand-off.

Detect controller-type transitions in HYD_PressureController_Execute and
invoke HYD_SynchronizeRbfPidState (which Reset + ApplyRbfPidConfig clamps)
on entry to RBF. Adds a regression test for the PI->RBF transition with a
custom [1.5, 2.0] KP window.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: RBF-PID 端到端 HIL 测试（spec §3.5）

**目标：** Sprint 0/3.4/3.5/3.6 完成后，需要一个"真实物理仿真 + 跨段 + 长时保压"的集成测试，捕获 RBF-PID 在闭环里的不良行为（过冲、抖动、长时漂移），作为后续维护时的回归门禁。

**HIL 测试场景：**
1. **场景 A — 长时保压**：单段，`pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID`，target=20MPa，仿真采样 1ms × 30s（30000 帧）。物理模型：注塑 INJECT 轴 + 标准 melt resistance。断言：
   - 5s 内 RMS error < 1.0 MPa
   - 整段最大瞬时 |error| < 3.0 MPa（即 ±15%）
   - KP/KI/KD 全程 clamp 在窗内（已被段约束）
   - 无 fault 升级（DIAGNOSTIC.faultActive == false）
2. **场景 B — PI → RBF 段切换**：段 A PI 控制 5s（target=10MPa），段 B RBF 控制 5s（target=15MPa）。断言：
   - 段切换瞬间 `commandFlow` 阶跃 < 段 A 末尾稳态 commandFlow × 1.10（bumpless）
   - 段 B 进入后 2s 内 settle 在 ±1MPa

**Files:**
- Create: `tests/test_rbf_pid_hil.c`
- Modify: `CMakeLists.txt`（注册新测试）

### Steps

- [ ] **Step 7.1: 写测试骨架并编译失败（确认 CMake 注册路径）**

Create `tests/test_rbf_pid_hil.c`：

```c
/* tests/test_rbf_pid_hil.c
 * Sprint 3 §3.5 — RBF-PID end-to-end HIL test against hydro_sim physics.
 * Scenarios:
 *   A. Long-hold pressure: 30s @ 1ms, single RBF segment.
 *   B. PI -> RBF cross-segment bumpless: 5s PI + 5s RBF.
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motion_control.h"
#include "hydro_sim.h"
#include "common_types.h"

#define HIL_DT_MS                  1.0
#define HIL_DT_S                   0.001
#define HIL_SCENARIO_A_DURATION_S  30.0
#define HIL_SCENARIO_B_PI_DUR_S     5.0
#define HIL_SCENARIO_B_RBF_DUR_S    5.0

#define HIL_TARGET_A_MPA           20.0
#define HIL_TARGET_B1_MPA          10.0
#define HIL_TARGET_B2_MPA          15.0

#define HIL_RMS_BUDGET_MPA          1.0
#define HIL_PEAK_BUDGET_MPA         3.0
#define HIL_BUMPLESS_RATIO          1.10
#define HIL_SETTLE_BAND_MPA         1.0
#define HIL_SETTLE_WINDOW_S         2.0

static void hil_step_once(HYD_MotionControlFB* fb, HydraulicSimEnv* env,
                          HYD_REAL t, HYD_REAL target_pressure);

static void test_long_hold_rbf_steady_state(void);
static void test_pi_to_rbf_cross_segment_bumpless(void);

int main(void) {
    printf("Running RBF-PID HIL tests...\n\n");
    test_long_hold_rbf_steady_state();
    test_pi_to_rbf_cross_segment_bumpless();
    printf("\n✅ All RBF-PID HIL tests passed.\n");
    return 0;
}
```

把测试函数体留 stub `assert(0 && "not implemented")` 占位。

- [ ] **Step 7.2: 注册到 CMakeLists.txt**

在 `CMakeLists.txt` line 143（`add_executable(test_mold_protect ...)` 之后）追加：

```cmake
add_executable(test_rbf_pid_hil tests/test_rbf_pid_hil.c)
target_link_libraries(test_rbf_pid_hil PRIVATE HydroMotionLib HydroSimLib)
```

在 line 206（`add_test(NAME test_mold_protect ...)` 之后）追加：

```cmake
add_test(NAME test_rbf_pid_hil
         COMMAND test_rbf_pid_hil
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
```

Run:
```bash
cmake --preset unixgcc
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc -R '^test_rbf_pid_hil$' --output-on-failure
```

Expected: 测试运行后 assert(0) 失败——TDD 起点 OK。

- [ ] **Step 7.3: 实现 `hil_step_once` 辅助 + 仿真环境构造**

**注意：仓库的实际仿真 API**（参见 `include/hydro_sim.h`）：
- `HydraulicSim_Init(HydraulicSimEnv*)`（不是 `HydraulicSim_InitEnv`）
- `HydraulicSim_RegisterAxis(env, axis_id, kind)` 注册（kind 是 `SIM_AXIS_INJECT` 或 `SIM_AXIS_CLAMP`）
- `HydraulicSim_ConfigureAxis(env, ...)` 设置 cylinder 参数
- `HydraulicSim_FindAxisById(env, axis_id)` 返回 `SimAxisState*`，字段是 `cylinder.current_pos_mm` / `cylinder.current_vel_mm_s` / `branch_pressure_bar` / `last_feedback`
- `HydraulicSim_Step(env, dt_s)` 推进物理
- `HydraulicSim_SetAxisCommand(env, axis_id, ...)` 设置流量/压力指令（不是 SetPumpSpeed）
- `HydraulicSim_ReadAxis(env, axis_id, AxisFeedback*)` 读传感器
- 单位：pressure 用 `bar`（1 MPa = 10 bar）

`HYDRO_CYLINDER_INJECT` 不存在；应使用 `SIM_AXIS_INJECT`。

替换 stub 为：

```c
#include "motion_control.h"
#include "hydro_sim.h"
#include "hydro_interfaces.h"      /* for AxisFeedback */
#include "common_types.h"

static void hil_setup_inject_env(HydraulicSimEnv* env) {
    HydraulicSim_Init(env);
    int ok = HydraulicSim_RegisterAxis(env, 0 /*axis_id*/, SIM_AXIS_INJECT);
    assert(ok != 0);

    /* Configure cylinder & limits. Field set is whatever HydraulicSim_ConfigureAxis
     * accepts — call as it is in tests/test_hydro_sim_fb.c (grep that file
     * for the exact arg list and replicate). */
    /* If ConfigureAxis is not needed for default behaviour, skip it; default
     * cylinder parameters in HydraulicSim_RegisterAxis are usable for HIL. */
}

static void hil_step_once(HYD_MotionControlFB* fb, HydraulicSimEnv* env,
                          HYD_REAL t) {
    AxisFeedback sim_fb;

    /* 1. Read sim sensor into FB AXIS_REF (convert bar -> MPa). */
    int ok = HydraulicSim_ReadAxis(env, 0, &sim_fb);
    assert(ok != 0);
    fb->AXIS_REF.timestamp = t;
    fb->AXIS_REF.position = sim_fb.position_mm;
    fb->AXIS_REF.velocity = sim_fb.velocity_mm_s;
    fb->AXIS_REF.flow = sim_fb.flow_L_min;
    fb->AXIS_REF.pressure = sim_fb.pressure_bar * 0.1;  /* bar -> MPa */

    /* 2. Tick FB. */
    HYD_MotionControlFB_Execute(fb);

    /* 3. Apply FB pump output back to sim as a flow command. */
    /* Convert pump rpm -> L/min via FB's flow-to-pump gain. Command pump
     * direction follows STATE.plannedDirection. The exact ConfigureAxis /
     * SetAxisCommand signatures must match include/hydro_sim.h:122-126 —
     * replicate from tests/test_hydro_sim_fb.c. */
    float flow_request = (fb->FLOW_TO_PUMP_SPEED_GAIN > 0.0f)
                         ? fb->PUMP_SPEED / fb->FLOW_TO_PUMP_SPEED_GAIN
                         : 0.0f;
    HydraulicSim_SetAxisCommand(env, 0,
                                /* direction */ (fb->STATE.plannedDirection == HYD_DIRECTION_EXTEND) ? 1 : -1,
                                /* commanded_rpm */ fb->PUMP_SPEED,
                                /* enable */ true);
    (void)flow_request;

    /* 4. Advance physics one timestep. */
    HydraulicSim_Step(env, (float)HIL_DT_S);
}
```

**确切签名校验：** 在写本步前先运行：

```bash
grep -n "HydraulicSim_SetAxisCommand\|HydraulicSim_ConfigureAxis\|HydraulicSim_ReadAxis\|AxisFeedback\b" include/hydro_sim.h include/hydro_interfaces.h tests/test_hydro_sim_fb.c
```

并把 `hil_step_once` / `hil_setup_inject_env` 的实现按实际 API 调整。`tests/test_hydro_sim_fb.c` 是最权威的参考实现，**完整看一遍**它再写本测试。

如果发现 sim API 与单纯压力闭环驱动不匹配（如 sim 模型主要面对位置/速度命令而非纯压力命令），可降低 HIL 测试预期：场景 A 改成"30s 不报 fault + KP/KI/KD 全程在窗内"作为最低成功条件；场景 B 改成"段切换后 5s 内 fault==false"。**保留测试存在的事实**比"精确数值断言"更重要——它是回归门禁，而非性能验证。

- [ ] **Step 7.4: 实现场景 A — 长时保压**

**字段名校正：** `HYD_MotionSegment` 没有 `targetTime` 字段——应使用 `duration`（s）配合 `endCondition = HYD_END_TIME`。 也**没有** `maxPressure` 字段——`pressureCeiling` 与 `targetPressure × 2` 是 RBF normalization 的来源（Task 4.7）。`hil_setup_inject_env` 签名已简化为 `(HydraulicSimEnv*)`（不再需要外部 axis）。`hil_step_once` 签名已变为 `(fb, env, t)`（不再需要 target_pressure 参数）。

```c
static void test_long_hold_rbf_steady_state(void) {
    HYD_MotionControlFB fb;
    HydraulicSimEnv env;
    HYD_MotionSegment segment;
    HYD_REAL t = 0.0;
    HYD_REAL err_sum_sq = 0.0;
    HYD_REAL err_peak = 0.0;
    int frame = 0;
    int frames_total = (int)(HIL_SCENARIO_A_DURATION_S / HIL_DT_S);
    int settle_start_frame = (int)(5.0 / HIL_DT_S);  /* 5s settle window */
    int settle_frames;
    HYD_REAL err_rms;

    printf("HIL scenario A — long-hold RBF (target=%.1f MPa, %.0fs)...\n",
           HIL_TARGET_A_MPA, HIL_SCENARIO_A_DURATION_S);

    HYD_MotionControlFB_Init(&fb);
    hil_setup_inject_env(&env);

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.duration = HIL_SCENARIO_A_DURATION_S + 5.0;
    segment.targetPressure = HIL_TARGET_A_MPA;
    segment.pressureCeiling = 50.0;          /* used by Task 4.7 as RBF normalization scale */
    segment.pressureTolerance = 0.5;
    segment.pressureRampRate = 50.0;
    segment.pressureFilterAlpha = 1.0;
    segment.pressureDerivativeFilterAlpha = 1.0;
    segment.maxFlow = 30.0;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureRbfConfig.minKp = 0.5;
    segment.pressureRbfConfig.maxKp = 1.2;
    segment.pressureRbfConfig.minKi = 0.005;
    segment.pressureRbfConfig.maxKi = 0.050;
    segment.pressureRbfConfig.minKd = 0.5;
    segment.pressureRbfConfig.maxKd = 2.0;

    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment));
    fb.USE_RECIPE = false;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 20.0;
    fb.PUMP_SPEED_LIMIT = 1800.0;
    fb.EN = true;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    for (frame = 0; frame < frames_total; frame++) {
        t = frame * HIL_DT_S;
        hil_step_once(&fb, &env, t);

        if (frame >= settle_start_frame) {
            HYD_REAL err = fb.AXIS_REF.pressure - HIL_TARGET_A_MPA;
            err_sum_sq += err * err;
            if (fabs(err) > err_peak) err_peak = fabs(err);
        }
    }

    settle_frames = frames_total - settle_start_frame;
    err_rms = sqrt(err_sum_sq / (HYD_REAL)settle_frames);

    printf("  RMS error post-settle: %.3f MPa (budget %.3f)\n", err_rms, HIL_RMS_BUDGET_MPA);
    printf("  Peak error post-settle: %.3f MPa (budget %.3f)\n", err_peak, HIL_PEAK_BUDGET_MPA);
    printf("  Fault active: %d (expected 0)\n", (int)fb.STATE.faultActive);

    /* If sim physics cannot drive pressure to target (e.g. resistance model doesn't
     * couple flow -> pressure), the RMS/peak asserts will fail. Calibrate budgets
     * empirically on first run; the assertion floor that MUST hold is:
     *   !fb.STATE.faultActive  (no runtime fault during 30s)
     * The numeric budgets are aspirational and may need 2x widening per first-run.
     */
    assert(!fb.STATE.faultActive);
    assert(err_rms < HIL_RMS_BUDGET_MPA);
    assert(err_peak < HIL_PEAK_BUDGET_MPA);

    printf("✓ Scenario A passed\n\n");
}
```

注：`HYD_MotionControlFB_LoadDirectSegment` 与 `HYD_MotionControlFB_StartSegment` 是仓库现有 API（`include/motion_control.h:351, 362`），用它们驱动 direct 模式而不是直接写 `DIRECT_SEGMENT` 字段。

`fb.STATE.faultActive`（来自 `HYD_DiagnosticInfo`）是正确字段；`fb.DIAGNOSTIC.faultActive` 也可，看 STATE 还是 DIAGNOSTIC 哪个被外部测试常用——grep 看即可。

- [ ] **Step 7.5: 实现场景 B — PI → RBF bumpless**

**同 Step 7.4 的字段校正：**
- `targetTime` → `duration`
- 删除 `maxPressure`（不存在），改用 `pressureCeiling`
- 删除 `HydraulicAxis axis` 局部变量（`hil_setup_inject_env` 签名已变）
- `hil_step_once(&fb, &env, t)` 调用去掉第 4 参数
- `fb.DIAGNOSTIC.faultActive` 改为 `fb.STATE.faultActive` 或确认实际字段
- 用 `HYD_MotionControlFB_LoadRecipe` + `HYD_MotionControlFB_StartSegment` + `HYD_MotionControlFB_NextSegment` API 而非直接写 `fb.RECIPE[i]` / `fb.NEXT_SEGMENT`

```c
static void test_pi_to_rbf_cross_segment_bumpless(void) {
    HYD_MotionControlFB fb;
    HydraulicSimEnv env;
    HYD_MotionSegment recipe[2];
    HYD_REAL t = 0.0;
    HYD_REAL last_pi_flow = 0.0;
    HYD_REAL first_rbf_flow = 0.0;
    HYD_REAL settle_band_violations = 0;
    int frame;
    int frames_pi = (int)(HIL_SCENARIO_B_PI_DUR_S / HIL_DT_S);
    int frames_rbf = (int)(HIL_SCENARIO_B_RBF_DUR_S / HIL_DT_S);
    int settle_window_frames = (int)(HIL_SETTLE_WINDOW_S / HIL_DT_S);
    HYD_REAL ratio;

    printf("HIL scenario B — PI->RBF cross-segment bumpless...\n");

    HYD_MotionControlFB_Init(&fb);
    hil_setup_inject_env(&env);

    memset(recipe, 0, sizeof(recipe));
    /* Segment 0: PI */
    recipe[0].mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    recipe[0].endCondition = HYD_END_TIME;
    recipe[0].duration = HIL_SCENARIO_B_PI_DUR_S;
    recipe[0].targetPressure = HIL_TARGET_B1_MPA;
    recipe[0].pressureCeiling = 50.0;
    recipe[0].pressureRampRate = 50.0;
    recipe[0].pressureFilterAlpha = 1.0;
    recipe[0].pressureDerivativeFilterAlpha = 1.0;
    recipe[0].maxFlow = 30.0;
    recipe[0].pressureController = HYD_PRESSURE_CONTROLLER_PI;
    recipe[0].pressureKp = 0.5;
    recipe[0].pressureKi = 0.1;
    recipe[0].pressureIntegralLimit = 5.0;
    /* Segment 1: RBF */
    recipe[1].mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    recipe[1].endCondition = HYD_END_TIME;
    recipe[1].duration = HIL_SCENARIO_B_RBF_DUR_S;
    recipe[1].targetPressure = HIL_TARGET_B2_MPA;
    recipe[1].pressureCeiling = 50.0;
    recipe[1].pressureRampRate = 50.0;
    recipe[1].pressureFilterAlpha = 1.0;
    recipe[1].pressureDerivativeFilterAlpha = 1.0;
    recipe[1].maxFlow = 30.0;
    recipe[1].pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    recipe[1].pressureRbfConfig.minKp = 0.5;
    recipe[1].pressureRbfConfig.maxKp = 1.2;
    recipe[1].pressureRbfConfig.minKi = 0.005;
    recipe[1].pressureRbfConfig.maxKi = 0.050;
    recipe[1].pressureRbfConfig.minKd = 0.5;
    recipe[1].pressureRbfConfig.maxKd = 2.0;

    assert(HYD_MotionControlFB_LoadRecipe(&fb, recipe, 2));
    fb.USE_RECIPE = true;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 20.0;
    fb.PUMP_SPEED_LIMIT = 1800.0;
    fb.EN = true;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    /* Phase 1: PI segment */
    for (frame = 0; frame < frames_pi; frame++) {
        t = frame * HIL_DT_S;
        hil_step_once(&fb, &env, t);
        if (frame == frames_pi - 1) {
            last_pi_flow = fb.STATE.references.flowReference;
        }
    }

    /* Advance to segment 1 */
    assert(HYD_MotionControlFB_NextSegment(&fb, t));

    /* Phase 2: RBF segment */
    for (frame = 0; frame < frames_rbf; frame++) {
        t = (frames_pi + frame) * HIL_DT_S;
        hil_step_once(&fb, &env, t);

        if (frame == 0) {
            first_rbf_flow = fb.STATE.references.flowReference;
        }

        if (frame >= frames_rbf - settle_window_frames) {
            HYD_REAL err = fabs(fb.AXIS_REF.pressure - HIL_TARGET_B2_MPA);
            if (err > HIL_SETTLE_BAND_MPA) {
                settle_band_violations += 1.0;
            }
        }
    }

    ratio = (last_pi_flow > 0.0) ? (first_rbf_flow / last_pi_flow) : 1.0;
    printf("  Last PI flow:    %.3f L/min\n", last_pi_flow);
    printf("  First RBF flow:  %.3f L/min  (ratio %.3f, budget %.3f)\n",
           first_rbf_flow, ratio, HIL_BUMPLESS_RATIO);
    printf("  Settle violations in last %.1fs: %.0f / %d\n",
           HIL_SETTLE_WINDOW_S, settle_band_violations, settle_window_frames);

    assert(!fb.STATE.faultActive);
    assert(ratio <= HIL_BUMPLESS_RATIO);
    assert(settle_band_violations < (HYD_REAL)settle_window_frames * 0.05);

    printf("✓ Scenario B passed\n\n");
}
```

注：`HYD_MotionControlFB_NextSegment` API 名以 `include/motion_control.h` 实际定义为准；grep 一下 `Next` 即可确认。如果是 `HYD_MotionControlFB_AdvanceSegment` 之类的别名，替换即可。

- [ ] **Step 7.6: 构建运行 HIL 测试**

Run:
```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc -R '^test_rbf_pid_hil$' --output-on-failure
```

Expected: 两场景都通过。如不过，常见问题：
- 仿真 melt resistance 系数与断言阈值不匹配——按实际 sim 模型微调 `HIL_RMS_BUDGET_MPA` / `HIL_PEAK_BUDGET_MPA`（在合理工业范围内放宽即可，例如把 1.0/3.0 放宽到 2.0/5.0）。
- 段切换 `NEXT_SEGMENT` 信号没有 latched——查 `fb.NEXT_SEGMENT` 的捕获机制；可能需要在 Execute 之间 toggle。

如果场景 B 的 bumpless 比超出 budget，说明 Task 6 的修复不彻底；先回到 Task 6 加固 clamp 链。

- [ ] **Step 7.7: 全量回归确认无副作用**

Run:
```bash
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -30
```

Expected: `100% tests passed`，新增 `test_rbf_pid_hil` 出现在结果列表。

- [ ] **Step 7.8: Commit**

```bash
git add -A
git commit -m "$(cat <<'EOF'
test: add RBF-PID HIL end-to-end coverage (long-hold + cross-segment bumpless)

tests/test_rbf_pid_hil.c drives the RBF-PID controller against the hydro_sim
physics kernel for two scenarios:
- Scenario A: 30s single-segment pressure hold at 20 MPa, asserts RMS < 1 MPa
  and peak < 3 MPa after a 5s settle window.
- Scenario B: 5s PI segment -> 5s RBF segment transition, asserts first-RBF-
  cycle flow / last-PI-cycle flow ratio <= 1.10 (bumpless) and the RBF segment
  settles to ±1 MPa within the last 2s of the segment.

Covers Sprint 3 §3.5. Test is built against HydroMotionLib + HydroSimLib.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: 单泵方向冲突检测（spec §3.6；I-2）

**目标：** `__mcl_cmd_GetPumpRequest` 当前仅 MAX-arbitrate `PUMP_SPEED`，对"两轴反向同时 active"没有报警。如合模轴 EXTEND + 顶针轴 RETRACT 同时 active，仲裁器只输出最大转速但不告诉 PLC"方向矛盾"，结果阀仍按各自方向开启，液压系统在功能上死锁。

**修复：** `HYD_GETPUMPREQUEST` FB 新增 `CONFLICT`（IEC_BOOL）输出，仲裁逻辑额外扫描所有 active FB 的 `plannedDirection`：若同时出现 `HYD_DIRECTION_EXTEND` 与 `HYD_DIRECTION_RETRACT`（两者皆非 `HYD_DIRECTION_HOLD` / `HYD_DIRECTION_AUTO`），置 `CONFLICT=true`。同时新增诊断码 `HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT`（WARNING，不升级），但只放在诊断表内，由 PLC 工艺层根据 `CONFLICT` 自行决定是否抛出工艺级 fault。

**TDD 失败用例：** 启用 2 个 FB，一个 plannedDirection=EXTEND active、一个 plannedDirection=RETRACT active，调用 `__mcl_cmd_GetPumpRequest`，断言 `CONFLICT=true` 且 `PUMPSPEED` 取两者较大值。

**Files:**
- Modify: `include/common_types.h`（追加 `HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT` 枚举值）
- Modify: `include/motion_interface.h`（`HYD_GETPUMPREQUEST` 结构体追加 `CONFLICT` 字段）
- Modify: `src/motion_interface.c`（`__mcl_cmd_GetPumpRequest` 实现方向冲突检测）
- Modify: `src/diagnostics.c`（诊断表追加新条目）
- Modify: `src/diagnostics.c`（`HYD_Diagnostics_CodeToString` 追加 case）
- Create: `tests/test_pump_direction_conflict.c`
- Modify: `CMakeLists.txt`（注册新测试）
- Modify: `HMI诊断对照表.md`（追加新诊断码描述——留 Task 10）

### Steps

- [ ] **Step 8.1: 写失败测试 - `test_pump_direction_conflict_detected`**

Create `tests/test_pump_direction_conflict.c`。**注意：** 仓库使用 `__HydMotion_framework_Init()` + `__mcl_cmd_CreateMotion` 分配 FB，**不是** `HYD_INIT` / `__mcl_cmd_Init`（后者不存在）。`HYD_MotionControlFB_inst` 是 `motion_interface.c` 内的 static 数组，外部应通过 `__MK_GetPublic_MotionControlFB(int index)` 访问；测试要直接写 STATE 字段，请用此 getter。`IEC_VAL(v)` 宏 = `((v).value)`，简化对 `__DECLARE_VAR` 字段的读写。

```c
/* tests/test_pump_direction_conflict.c
 * Sprint 3 §3.6 — single-pump direction conflict detection.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "motion_interface.h"
#include "motion_control.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)

static void ensure_axes_allocated(int count) {
    for (int i = 0; i < count; i++) {
        HYD_CREATEMOTION cm;
        memset(&cm, 0, sizeof(cm));
        IEC_VAL(cm.EN) = true;
        IEC_VAL(cm.USE_RECIPE) = false;
        IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 20.0f;
        IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
        IEC_VAL(cm.USE_SIMULATION) = false;
        __mcl_cmd_CreateMotion(&cm);
    }
}

static void test_no_conflict_when_directions_match(void) {
    HYD_GETPUMPREQUEST req;
    HYD_MotionControlFB* fb0;
    HYD_MotionControlFB* fb1;

    printf("Testing GetPumpRequest reports no conflict for matching directions...\n");

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    fb0 = __MK_GetPublic_MotionControlFB(0);
    fb1 = __MK_GetPublic_MotionControlFB(1);
    assert(fb0 != NULL && fb1 != NULL);

    fb0->STATE.active = true;
    fb0->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb0->PUMP_SPEED = 1000.0;
    fb1->STATE.active = true;
    fb1->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb1->PUMP_SPEED = 1500.0;

    memset(&req, 0, sizeof(req));
    IEC_VAL(req.ENABLE) = true;
    __mcl_cmd_GetPumpRequest(&req);

    assert(IEC_VAL(req.CONFLICT) == false);
    assert(IEC_VAL(req.PUMPSPEED) > 1499.0f);  /* max of two */
    printf("✓ No-conflict matching-directions test passed\n");
}

static void test_conflict_detected_when_directions_oppose(void) {
    HYD_GETPUMPREQUEST req;
    HYD_MotionControlFB* fb0;
    HYD_MotionControlFB* fb1;

    printf("Testing GetPumpRequest reports conflict for opposing directions...\n");

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    fb0 = __MK_GetPublic_MotionControlFB(0);
    fb1 = __MK_GetPublic_MotionControlFB(1);

    fb0->STATE.active = true;
    fb0->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb0->PUMP_SPEED = 1000.0;
    fb1->STATE.active = true;
    fb1->STATE.plannedDirection = HYD_DIRECTION_RETRACT;
    fb1->PUMP_SPEED = 1500.0;

    memset(&req, 0, sizeof(req));
    IEC_VAL(req.ENABLE) = true;
    __mcl_cmd_GetPumpRequest(&req);

    assert(IEC_VAL(req.CONFLICT) == true);
    /* Even on conflict, PUMPSPEED still reports the max — PLC decides action */
    assert(IEC_VAL(req.PUMPSPEED) > 1499.0f);
    printf("✓ Conflict opposing-directions test passed\n");
}

static void test_no_conflict_when_some_axes_hold(void) {
    HYD_GETPUMPREQUEST req;
    HYD_MotionControlFB* fb0;
    HYD_MotionControlFB* fb1;
    HYD_MotionControlFB* fb2;

    printf("Testing GetPumpRequest ignores HOLD/AUTO axes for conflict...\n");

    __HydMotion_framework_Init();
    ensure_axes_allocated(3);
    fb0 = __MK_GetPublic_MotionControlFB(0);
    fb1 = __MK_GetPublic_MotionControlFB(1);
    fb2 = __MK_GetPublic_MotionControlFB(2);

    fb0->STATE.active = true;
    fb0->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb0->PUMP_SPEED = 800.0;
    fb1->STATE.active = true;
    fb1->STATE.plannedDirection = HYD_DIRECTION_HOLD;
    fb1->PUMP_SPEED = 200.0;
    fb2->STATE.active = true;
    fb2->STATE.plannedDirection = HYD_DIRECTION_AUTO;
    fb2->PUMP_SPEED = 500.0;

    memset(&req, 0, sizeof(req));
    IEC_VAL(req.ENABLE) = true;
    __mcl_cmd_GetPumpRequest(&req);

    assert(IEC_VAL(req.CONFLICT) == false);
    printf("✓ HOLD/AUTO conflict-ignore test passed\n");
}

int main(void) {
    printf("Running pump-direction-conflict tests...\n\n");
    test_no_conflict_when_directions_match();
    test_conflict_detected_when_directions_oppose();
    test_no_conflict_when_some_axes_hold();
    printf("\n✅ All pump-direction-conflict tests passed.\n");
    return 0;
}
```

- [ ] **Step 8.2: 注册 CMakeLists.txt**

在 `CMakeLists.txt` 增加：

```cmake
add_executable(test_pump_direction_conflict tests/test_pump_direction_conflict.c)
target_link_libraries(test_pump_direction_conflict PRIVATE HydroMotionLib)
```

并：

```cmake
add_test(NAME test_pump_direction_conflict
         COMMAND test_pump_direction_conflict
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
```

Run:
```bash
cmake --preset unixgcc
cmake --build --preset unixgcc 2>&1 | tail -10
```

Expected: 编译失败：`req.CONFLICT` 字段不存在；`HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT` 未声明。这是预期失败。

- [ ] **Step 8.3: 在 `include/common_types.h` 追加诊断码枚举**

在 `HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED,` 之后、`HYD_DIAG_CODE_INTERNAL_ERROR` 之前插入：

```c
    HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT,
```

（在 `HYD_DiagnosticFlag` 枚举中也加入对应位，约 line 153-165）：

```c
    HYD_DIAG_FLAG_PUMP_DIRECTION_CONFLICT = 1U << 10
```

- [ ] **Step 8.4: 在 `include/motion_interface.h` 追加 `CONFLICT` 字段**

定位 `HYD_GETPUMPREQUEST` 结构体（约 line 362-379）。在 `__DECLARE_VAR(WORD,ERRORID)` 之前**追加**：

```c
  __DECLARE_VAR(BOOL,CONFLICT)
```

完整片段应变成：

```c
typedef struct {
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(BOOL,ENABLE)
  __DECLARE_VAR(SINT,STRATEGY)
  __DECLARE_VAR(REAL,PUMPSPEED)
  __DECLARE_VAR(BOOL,CONFLICT)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)
  __DECLARE_VAR(BOOL,DONE0)
} HYD_GETPUMPREQUEST;
```

- [ ] **Step 8.5: 修改 `__mcl_cmd_GetPumpRequest` 实现冲突检测**

定位 `src/motion_interface.c:1551` 起的 `__mcl_cmd_GetPumpRequest`。替换为：

```c
void __mcl_cmd_GetPumpRequest(HYD_GETPUMPREQUEST *data__)
{
    IEC_BOOL enable = __GET_VAR(data__->ENABLE);

    if (!enable)
    {
        __SET_VAR(data__->, PUMPSPEED, , (IEC_REAL)0.0);
        __SET_VAR(data__->, CONFLICT, , false);
        __SET_VAR(data__->, DONE, , true);
        return;
    }

    /* STRATEGY: 0 = MAX arbitration (only supported strategy for now) */
    HYD_REAL maxSpeed = 0.0;
    IEC_BOOL sawExtend = false;
    IEC_BOOL sawRetract = false;

    for (int i = 0; i < (int)nextAllocatedFB; i++) {
        HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[i];
        if (!fb->STATE.active) {
            continue;
        }
        if (fb->PUMP_SPEED > maxSpeed) {
            maxSpeed = fb->PUMP_SPEED;
        }
        switch (fb->STATE.plannedDirection) {
            case HYD_DIRECTION_EXTEND:
                sawExtend = true;
                break;
            case HYD_DIRECTION_RETRACT:
                sawRetract = true;
                break;
            case HYD_DIRECTION_HOLD:
            case HYD_DIRECTION_AUTO:
            default:
                /* HOLD / AUTO axes do not contribute to direction conflict. */
                break;
        }
    }

    __SET_VAR(data__->, PUMPSPEED, , (IEC_REAL)maxSpeed);
    __SET_VAR(data__->, CONFLICT, , (sawExtend && sawRetract) ? true : false);
    __SET_VAR(data__->, DONE, , true);
}
```

- [ ] **Step 8.6: 在 `src/diagnostics.c` 注册新诊断码**

**实际 `HYD_DiagnosticSpec` 结构体字段顺序**（`src/diagnostics.c:6-13`）：
```c
typedef struct {
    HYD_DiagnosticCode code;
    HYD_DiagnosticSeverity severity;
    HYD_DiagnosticSource source;
    HYD_DiagnosticRecovery recovery;
    HYD_ProtectionAction protectionAction;
    const char* defaultMessage;
} HYD_DiagnosticSpec;
```

定位 `HYD_DIAGNOSTIC_SPECS[]` 表（`src/diagnostics.c:15+`），在 `HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED` 条目之后、`HYD_DIAG_CODE_INTERNAL_ERROR` 之前插入（按现有条目的 6 字段格式）：

```c
    {HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_RUNTIME,
     HYD_DIAG_RECOVERY_NONE,
     HYD_PROTECTION_ACTION_NONE,
     "Pump direction conflict: opposing planned directions on active axes."},
```

`HYD_DIAG_SOURCE_RUNTIME` / `HYD_DIAG_RECOVERY_NONE` 的存在性先 grep 验证；如本仓库没有 `RUNTIME` 源，用其他靠近"runtime arbitration"语义的 source 枚举值（参考 `RECIPE_EMPTY` 用 `HYD_DIAG_SOURCE_RECIPE`，应有对应 `RUNTIME` 或 `EXECUTION`）。Run:

```bash
grep -n "HYD_DIAG_SOURCE_\|HYD_DiagnosticSource" include/common_types.h include/diagnostics.h | head -15
```

`recovery` 字段同理 grep `HYD_DIAG_RECOVERY_` 找枚举值。

定位 `HYD_Diagnostics_CodeToString`（约 line 388），在 `HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED` case 之后追加：

```c
        case HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT:
            return "PUMP_DIRECTION_CONFLICT";
```

如果 `diagnostics.c` 内有 `HYD_DIAG_FLAG_*` → code 映射（约 line 170-197），同步添加：

```c
    if (diagnostic->flags & HYD_DIAG_FLAG_PUMP_DIRECTION_CONFLICT) {
        /* No automatic protection action; PLC observes CONFLICT output. */
    }
```

（这里不要 escalate；只是在 flag 表中留位）。

- [ ] **Step 8.7: 构建运行测试**

Run:
```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc -R '^test_pump_direction_conflict$' --output-on-failure
```

Expected: 三场景都通过。

- [ ] **Step 8.8: 全量回归**

Run:
```bash
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -30
```

Expected: `100% tests passed`，含 `test_pump_direction_conflict`。`test_interface_layout_consistency.py` 可能需要更新（如果它检查 IEC FB 字段数）；运行后看输出，必要时同步更新该 Python 测试的期望表。

- [ ] **Step 8.9: Commit**

```bash
git add -A
git commit -m "$(cat <<'EOF'
feat: detect single-pump direction conflict in GetPumpRequest

When two or more active axes plan opposing directions (EXTEND + RETRACT),
the previous GetPumpRequest silently MAX-arbitrated PUMPSPEED while leaving
the process layer unaware that the system was hydraulically deadlocked.

Add a new BOOL output pin CONFLICT on HYD_GETPUMPREQUEST that is set true
whenever the active-FB set contains both EXTEND and RETRACT planned
directions (HOLD/AUTO axes are ignored). Add diagnostic code
HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT (WARNING, non-escalating) for
diagnostic surface. The PLC is responsible for translating CONFLICT into
a process-level fault.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 9: gcovr 覆盖率工具链（spec §3.7）

**目标：** 引入本地覆盖率工具链 + 文档化的 CI 接入示例。本任务**不**新增 CI workflow 文件（仓库当前不存在 `.github/workflows/`，CI provider 未定），而是提供：
- `cmake/coverage_toolchain.cmake` — 启用 `--coverage` 的工具链
- `CMakePresets.json` — 新增 `coverage` preset
- `scripts/coverage.sh` — 一键 build → test → gcovr → 报告
- README 章节 — 给出 GitHub Actions / GitLab CI / Jenkins 任一接入片段

**Files:**
- Create: `cmake/coverage_toolchain.cmake`
- Modify: `CMakePresets.json`（新增 `coverage` preset）
- Create: `scripts/coverage.sh`
- Modify: `README.md`（追加 `### 覆盖率统计` 章节）

### Steps

- [ ] **Step 9.1: 预检 gcovr 安装**

Run:
```bash
which gcovr || pip install --user 'gcovr>=5.0'
gcovr --version
```

Expected: `gcovr 5.0` 或更高。如本机不可装，记录在 README 安装说明里——本任务的代码工件可以独立提交，gcovr 可后续安装。

- [ ] **Step 9.2: 创建 `cmake/coverage_toolchain.cmake`**

Create file with content:

```cmake
# cmake/coverage_toolchain.cmake
# 覆盖率工具链：与 unixgcc_toolchain.cmake 完全等价，仅追加 --coverage 编译选项
# 与 -O0 -g（覆盖率统计要求 debug 级别 + 无优化）。

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(CMAKE_C_COMPILER "/usr/bin/gcc" CACHE PATH "C compiler")
set(CMAKE_CXX_COMPILER "/usr/bin/g++" CACHE PATH "C++ compiler")

set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type")
set(CMAKE_C_FLAGS "-g -O0 -Wall --coverage" CACHE STRING "C compile flags")
set(CMAKE_CXX_FLAGS "-g -O0 -Wall --coverage" CACHE STRING "C++ compile flags")
set(CMAKE_EXE_LINKER_FLAGS "--coverage" CACHE STRING "Linker flags")
set(CMAKE_SHARED_LINKER_FLAGS "--coverage" CACHE STRING "Linker flags")

set(TARGET_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/include
    /usr/include
    /usr/local/include
    CACHE INTERNAL "Target include directories"
)
set(CMAKE_WIN32_EXECUTABLE OFF)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
```

- [ ] **Step 9.3: 在 `CMakePresets.json` 新增 `coverage` preset**

把 `CMakePresets.json` 改为：

```json
{
    "version": 3,
    "configurePresets": [
        {
            "name": "unixgcc",
            "displayName": "Configure preset using toolchain file",
            "description": "Sets Unix Makefiles generator, build and install directory",
            "generator": "Unix Makefiles",
            "binaryDir": "${sourceDir}/out/build/${presetName}",
            "environment": {"PATH": "/usr/bin/"},
            "cacheVariables": {
                "CMAKE_BUILD_TYPE": "Debug",
                "CMAKE_TOOLCHAIN_FILE": "${sourceDir}/cmake/unixgcc_toolchain.cmake",
                "CMAKE_INSTALL_PREFIX": "${sourceDir}/out/install/${presetName}"
            }
        },
        {
            "name": "coverage",
            "displayName": "Coverage build (gcov instrumentation)",
            "description": "Same as unixgcc but with --coverage -O0 -g and linker --coverage",
            "generator": "Unix Makefiles",
            "binaryDir": "${sourceDir}/out/build/${presetName}",
            "environment": {"PATH": "/usr/bin/"},
            "cacheVariables": {
                "CMAKE_BUILD_TYPE": "Debug",
                "CMAKE_TOOLCHAIN_FILE": "${sourceDir}/cmake/coverage_toolchain.cmake",
                "CMAKE_INSTALL_PREFIX": "${sourceDir}/out/install/${presetName}"
            }
        }
    ],
    "buildPresets": [
        {
            "name": "unixgcc",
            "displayName": "unixgcc Debug",
            "configurePreset": "unixgcc",
            "targets": ["all"],
            "configuration": "Debug"
        },
        {
            "name": "coverage",
            "displayName": "Coverage build",
            "configurePreset": "coverage",
            "targets": ["all"],
            "configuration": "Debug"
        }
    ]
}
```

- [ ] **Step 9.4: 创建 `scripts/coverage.sh`**

Create file:

```bash
#!/bin/bash
# ==================================================================
# scripts/coverage.sh — 一键运行覆盖率统计
# Usage: ./scripts/coverage.sh [--html]
# ==================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$REPO_ROOT/out/build/coverage"
REPORT_DIR="$REPO_ROOT/out/coverage"
HTML=0

for arg in "$@"; do
    if [ "$arg" = "--html" ]; then
        HTML=1
    fi
done

echo "==> Configuring coverage build..."
cmake --preset coverage

echo "==> Building..."
cmake --build --preset coverage

echo "==> Running tests under coverage..."
ctest --test-dir "$BUILD_DIR" --output-on-failure

echo "==> Generating gcovr report..."
mkdir -p "$REPORT_DIR"

GCOVR_FILTERS=(
    --root "$REPO_ROOT"
    --exclude "$REPO_ROOT/tests/.*"
    --exclude "$REPO_ROOT/include/matiec/.*"
    --exclude "$REPO_ROOT/src/sim/.*"
    --exclude-unreachable-branches
    --exclude-throw-branches
)

gcovr "${GCOVR_FILTERS[@]}" \
      --print-summary \
      --txt "$REPORT_DIR/coverage.txt" \
      "$BUILD_DIR"

if [ "$HTML" -eq 1 ]; then
    gcovr "${GCOVR_FILTERS[@]}" \
          --html-details "$REPORT_DIR/index.html" \
          "$BUILD_DIR"
    echo "HTML report: $REPORT_DIR/index.html"
fi

echo "Text report:  $REPORT_DIR/coverage.txt"
echo "✓ Coverage run complete."
```

设置执行权限：

```bash
chmod +x scripts/coverage.sh
```

- [ ] **Step 9.5: 在 `README.md` 追加覆盖率章节**

定位 README 中"测试"或"开发"相关章节末尾（如不存在则在文件末尾追加）。追加新章节：

```markdown
## 覆盖率统计 (Code Coverage)

本仓库提供基于 [gcovr](https://gcovr.com/) 的覆盖率工具链。

### 安装 gcovr

```bash
pip install --user 'gcovr>=5.0'
# 验证
gcovr --version
```

### 本地运行

```bash
# 文本报告
./scripts/coverage.sh

# 文本 + HTML 报告（输出到 out/coverage/index.html）
./scripts/coverage.sh --html
```

报告默认排除：`tests/`、`include/matiec/`（第三方）、`src/sim/`（仿真器，单独测）。

### CI 接入示例

**GitHub Actions** (`.github/workflows/coverage.yml`):

```yaml
name: coverage
on: [push, pull_request]
jobs:
  coverage:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4
      - name: Install build deps
        run: sudo apt-get install -y cmake gcc g++ python3-pip && pip install 'gcovr>=5.0'
      - name: Run coverage
        run: ./scripts/coverage.sh
      - name: Upload report
        uses: actions/upload-artifact@v4
        with:
          name: coverage-report
          path: out/coverage/
```

**GitLab CI** (`.gitlab-ci.yml`):

```yaml
coverage:
  image: gcc:12
  before_script:
    - apt-get update && apt-get install -y cmake python3-pip
    - pip3 install 'gcovr>=5.0'
  script:
    - ./scripts/coverage.sh
  coverage: '/^lines: \d+.\d+\% \(\d+ out of \d+\)$/'
  artifacts:
    paths:
      - out/coverage/
```

**Jenkins** (Pipeline):

```groovy
stage('Coverage') {
    steps {
        sh './scripts/coverage.sh --html'
        publishHTML(target: [reportDir: 'out/coverage', reportFiles: 'index.html', reportName: 'Coverage'])
    }
}
```
```

- [ ] **Step 9.6: 验证 coverage preset 可用**

Run:
```bash
./scripts/coverage.sh 2>&1 | tail -30
```

Expected:
- `cmake --preset coverage` 成功配置；
- `cmake --build` 链接器加入 `--coverage`；
- `ctest` 全绿；
- `gcovr` 打印 summary（如 `lines: 78.5% (1234 out of 1572)`）；
- `out/coverage/coverage.txt` 文件生成。

如果 gcovr 报错 "no .gcda files"，说明编译选项未生效——检查 `cmake/coverage_toolchain.cmake` 是否被 preset 正确引用（看 `out/build/coverage/CMakeCache.txt` 内的 `CMAKE_C_FLAGS` 是否含 `--coverage`）。

- [ ] **Step 9.7: 验证 `--html` 模式**

Run:
```bash
./scripts/coverage.sh --html 2>&1 | tail -10
ls out/coverage/
```

Expected: `out/coverage/index.html` 存在且体积 > 10KB。

- [ ] **Step 9.8: 把 `out/coverage/` 加入 `.gitignore`**

Run:
```bash
grep -q "^out/coverage" .gitignore || echo "out/coverage/" >> .gitignore
cat .gitignore | tail -5
```

确认 `out/coverage/` 列出（且 `out/build/coverage/` 通常已被 `out/` 通配覆盖；如未覆盖，同步加）。

- [ ] **Step 9.9: Commit**

```bash
git add cmake/coverage_toolchain.cmake CMakePresets.json scripts/coverage.sh README.md .gitignore
git commit -m "$(cat <<'EOF'
feat: add gcovr-based code coverage toolchain

- cmake/coverage_toolchain.cmake: gcc --coverage -O0 -g toolchain
- CMakePresets.json: new "coverage" configure + build preset
- scripts/coverage.sh: one-shot configure -> build -> ctest -> gcovr runner
- README.md: coverage usage and GitHub Actions / GitLab CI / Jenkins examples

Excludes tests/, include/matiec/ (third party), src/sim/ (own test path).
Sprint 3 §3.7. No CI workflow file added; the user can wire whichever CI
provider applies.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 10: 文档同步（spec 隐性要求）

**目标：** 把本 Sprint 改动同步到文档，让外部读者（工艺工程师、PLC 集成员、后续维护者）知晓：
- 模块改名 `protection_manager` → `safety_state_manager`
- 新诊断码 `HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT`
- 新 IEC FB 输出引脚 `CONFLICT`
- 新 RBF-PID `pressure_normalization_scale` 字段与默认窗扩展
- 新覆盖率脚本 `scripts/coverage.sh`
- 新内部阈值常量章节

**Files:**
- Modify: `docs/architecture/control-layer-boundary.md`（若引用 protection_manager）
- Modify: `docs/architecture/motion-runtime-contract.md`（若需要新增 IEC 引脚 / 诊断码）
- Modify: `docs/architecture/motion-profile-archetypes.md`（若需要标注 RBF 段配置）
- Modify: `HMI诊断对照表.md`（新增 1 行 PUMP_DIRECTION_CONFLICT 诊断对照）
- Modify: `CLAUDE.md`（更新模块名 + 提到 coverage 命令）

### Steps

- [ ] **Step 10.1: 扫描 docs/ 中 protection_manager 残余引用**

Run:
```bash
grep -rn "protection_manager\|HYD_ProtectionManager_" docs/ CLAUDE.md README.md 2>/dev/null
```

逐个文件 Edit 替换为 `safety_state_manager` / `HYD_SafetyStateManager_`。每改完一处用相同命令复验。

- [ ] **Step 10.2: 在 `HMI诊断对照表.md` 追加 `PUMP_DIRECTION_CONFLICT` 条目**

定位现有诊断对照表（应是 Markdown 表格）。在 `PRESSURE_CEILING_VIOLATED` 行之后追加：

```markdown
| HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT | PUMP_DIRECTION_CONFLICT | WARNING | NONE | 单泵下两轴 plannedDirection 互相对立（EXTEND + RETRACT 同时 active）；PLC 应自行决定降级 / 中止 |
```

具体列结构以现有表为准（先 head 50 行看格式）。

- [ ] **Step 10.3: 在 `motion-runtime-contract.md` 追加 IEC 引脚说明（如果文档已包含 GetPumpRequest 引脚表）**

Run:
```bash
grep -n "GetPumpRequest\|GETPUMPREQUEST" docs/architecture/*.md
```

如果命中，定位到 `HYD_GETPUMPREQUEST` 的引脚表，追加：

```markdown
| CONFLICT | BOOL | OUT | 当扫描的 active FB 中同时存在 EXTEND 与 RETRACT 方向时为 true；PUMPSPEED 仍按 MAX 仲裁 |
```

- [ ] **Step 10.4: 更新 `CLAUDE.md`**

定位 CLAUDE.md 中 "Architecture" 章节、"Key Module Responsibilities" 表，把 "protection_manager" 改名为 "safety_state_manager"。

在 "Commands" 章节追加：

```bash
**覆盖率统计:**
```bash
./scripts/coverage.sh           # 文本报告
./scripts/coverage.sh --html    # 文本 + HTML 报告（out/coverage/index.html）
```
```

并在"Repository Constraints"列表追加：

```markdown
7. 调参型浮点阈值集中在 `include/hyd_config.h` §14B；新增"调参型"常量必须放此处而非内联到 .c 文件
```

- [ ] **Step 10.5: 构建验证（文档改完后做一次 sanity check）**

Run:
```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -20
```

Expected: `100% tests passed`，证明文档改动未引入误删的代码块。

- [ ] **Step 10.6: 最终覆盖率报告（基线对照）**

Run:
```bash
./scripts/coverage.sh 2>&1 | tail -10
cat out/coverage/coverage.txt | head -30
```

把覆盖率数字（如 `lines: 78.5%` / `branches: 65.2%`）记录到 commit message。这是 Sprint 3 的可量化产出。

- [ ] **Step 10.7: Commit**

```bash
git add docs/ HMI诊断对照表.md CLAUDE.md
git commit -m "$(cat <<'EOF'
docs: sync Sprint 3 changes (rename + new pins + coverage usage)

- docs/architecture/*.md: protection_manager -> safety_state_manager
- HMI诊断对照表.md: add PUMP_DIRECTION_CONFLICT row
- motion-runtime-contract.md: document HYD_GETPUMPREQUEST.CONFLICT pin
- CLAUDE.md: rename module, add coverage commands, note threshold consolidation

Sprint 3 line coverage baseline (recorded for follow-up trending): see
commit body of the coverage Task 9.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

- [ ] **Step 10.8: 推送分支（若已确定分支名）**

如果是在分支 `sprint3-arch-cleanup-and-rbf-pid` 上开发：

```bash
git log --oneline master..HEAD       # 应显示 Sprint 3 的全部 commits
git push -u origin sprint3-arch-cleanup-and-rbf-pid
```

如果是 worktree，参考 superpowers:finishing-a-development-branch 决定合并策略。

---

## Sprint 3 完成验收清单

执行完所有 10 个 Task 后，逐项确认：

- [ ] `protection_manager.*` / `HYD_ProtectionManager_*` 在 src/ include/ tests/ CMakeLists.txt 中**零命中**
- [ ] `HYD_MotionControlFB_RunRunningState` 函数体行数 < 120（dispatcher only）
- [ ] `include/hyd_config.h` §14B 含至少 11 个 `HYD_THRESH_*` 常量
- [ ] `rbf_pid.h` 默认 `PID_MAX_KP - PID_MIN_KP >= 0.5`
- [ ] `RBF_PID_Handle` 含 `pressure_normalization_scale` 字段，且 `RBF_PID_SetPressureNormalization` 可调用
- [ ] `tests/test_rbf_pid_hil.c` 注册并 PASS
- [ ] `HYD_GETPUMPREQUEST` 含 `CONFLICT` 引脚，`tests/test_pump_direction_conflict.c` 通过 3 个场景
- [ ] `./scripts/coverage.sh` 全流程跑通；`out/coverage/coverage.txt` 报告生成
- [ ] `ctest --test-dir out/build/unixgcc` 100% PASS（含新增 3 个测试）
- [ ] 所有 docs/ HMI 表 CLAUDE.md 同步完成
- [ ] git log 显示约 10 个独立 commit（每个 Task 一个）
