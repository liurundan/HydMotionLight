# HYD_ → HYD_ 全量前缀重命名实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将项目中所有 `HYD_`/`Hdy_`/`hyd_`/`__Hyd` 前缀统一替换为 `HYD_`/`Hyd_`/`hyd_`/`__Hyd`，包括源码、头文件、测试、XML、文档、文件名。

**Architecture:** 采用 sed 批量替换 + 文件重命名的策略，按"先替换内容、后重命名文件"的顺序执行，避免引用断裂。分5个阶段：源码替换→文件重命名→XML替换→文档替换→构建验证。

**Tech Stack:** C99, sed, cmake/ctest, matiec IEC61131-3

---

## 影响范围汇总

| 类别 | 替换模式 | 出现次数 | 涉及文件数 |
|------|---------|---------|-----------|
| 宏/枚举/类型 | `HYD_` → `HYD_` | ~3,901 | 51 源文件 + 1 XML + ~42 MD |
| CamelCase函数 | `Hdy_` → `Hyd_` | 18 | 2 源文件 |
| 小写(include/文件名) | `hyd_` → `hyd_` | 3 + 文件名 | 3 源文件 + 1 MD |
| IEC框架函数 | `__Hyd` → `__Hyd` | 90 | 4 源文件 |
| IEC FB名称 | `HydMotion`→`HydMotion`, `HydSimulator`→`HydSimulator` | 90 | 4 源文件 |
| 文件重命名 | `hyd_config.h`→`hyd_config.h`, `pousHydMotion.xml`→`pousHydMotion.xml` | 2 | 2 |
| 文档 | `HYD_` / `hyd_` | ~1,586 | ~42 MD |

**不得修改的内容：**
- `include/matiec/` 目录（第三方 IEC 类型系统）
- `HydroMotionLib` / `HydroSimLib`（CMake 目标名，已用 Hydro 前缀）
- `HydraulicSim*` / `HydroValve` / `HydroPump` / `HydroCylinder`（仿真器物理层类型）
- `hydro_sim*` / `hydro_interfaces*` / `hydro_hardware*`（仿真器文件/类型）
- `HYDRO_SIM_H` / `HYDRO_SIM_FB_H` / `HYDRO_INTERFACES_H` / `HYDRO_HARDWARE_H`（仿真器 include guard，已用 HYDRO 前缀）

---

## Task 1: 源码文件 (.c/.h) 批量替换 — HYD_ → HYD_

**Files:** 所有 51 个含 `HYD_` 的 .c/.h 文件（排除 matiec/）

- [ ] **Step 1: 执行 sed 批量替换 HYD_ → HYD_**

```bash
cd /home/dan/project/hdy-motion-light
find include src tests -name '*.c' -o -name '*.h' | grep -v matiec | xargs sed -i 's/\bHDY_/HYD_/g'
```

- [ ] **Step 2: 验证替换结果（无遗漏）**

```bash
grep -rE '\bHDY_' --include='*.c' --include='*.h' | grep -v matiec
```

预期：无输出（所有 HYD_ 已替换为 HYD_）

---

## Task 2: 源码文件 (.c/.h) 批量替换 — Hdy_ → Hyd_

**Files:** `src/sim/hydro_sim_fb.c`, `include/hydro_sim_fb.h`（含 `Hdy_` 前缀的6个静态函数）

- [ ] **Step 1: 执行 sed 批量替换 Hdy_ → Hyd_**

```bash
find include src tests -name '*.c' -o -name '*.h' | grep -v matiec | xargs sed -i 's/\bHdy_/Hyd_/g'
```

- [ ] **Step 2: 验证替换结果**

```bash
grep -rE '\bHdy_' --include='*.c' --include='*.h' | grep -v matiec
```

预期：无输出

---

## Task 3: 源码文件 (.c/.h) 批量替换 — __Hyd → __Hyd（IEC框架函数）

**Files:** `src/motion_interface.c`, `include/motion_interface.h`, `src/sim/hydro_sim_fb.c`, `include/hydro_sim_fb.h` 及测试文件

- [ ] **Step 1: 执行 sed 批量替换 __Hyd → __Hyd**

```bash
find include src tests -name '*.c' -o -name '*.h' | grep -v matiec | xargs sed -i 's/__Hyd/__Hyd/g'
```

- [ ] **Step 2: 验证替换结果**

```bash
grep -rE '__Hyd' --include='*.c' --include='*.h' | grep -v matiec
```

预期：无输出

---

## Task 4: 源码文件 (.c/.h) 批量替换 — hyd_ → hyd_（小写引用）

**Files:** 含 `#include "hyd_config.h"` 和注释中引用 `hyd_config` 的文件

- [ ] **Step 1: 执行 sed 批量替换 hyd_ → hyd_**

```bash
find include src tests -name '*.c' -o -name '*.h' | grep -v matiec | xargs sed -i 's/\bhdy_/hyd_/g'
```

- [ ] **Step 2: 验证替换结果**

```bash
grep -rE '\bhdy_' --include='*.c' --include='*.h' | grep -v matiec
```

预期：无输出

---

## Task 5: 文件重命名

**Files:**
- `include/hyd_config.h` → `include/hyd_config.h`
- `pousHydMotion.xml` → `pousHydMotion.xml`

- [ ] **Step 1: 重命名 hyd_config.h**

```bash
mv include/hyd_config.h include/hyd_config.h
```

- [ ] **Step 2: 重命名 pousHydMotion.xml**

```bash
mv pousHydMotion.xml pousHydMotion.xml
```

- [ ] **Step 3: 验证旧文件不存在**

```bash
ls include/hyd_config.h pousHydMotion.xml 2>&1
```

预期：文件不存在错误

- [ ] **Step 4: 验证新文件存在**

```bash
ls -la include/hyd_config.h pousHydMotion.xml
```

---

## Task 6: XML 文件替换

**Files:** `pousHydMotion.xml`（已重命名）

- [ ] **Step 1: 执行 sed 替换 XML 中的 HYD_ → HYD_**

```bash
sed -i 's/\bHDY_/HYD_/g' pousHydMotion.xml
```

- [ ] **Step 2: 验证替换结果**

```bash
grep -E '\bHDY_' pousHydMotion.xml
```

预期：无输出

---

## Task 7: Markdown 文档批量替换

**Files:** 所有含 `HYD_` / `hyd_` 的 .md 文件（约42个）

- [ ] **Step 1: 替换 MD 文件中的 HYD_ → HYD_**

```bash
find . -name '*.md' -not -path './out/*' -not -path './.git/*' | xargs sed -i 's/\bHDY_/HYD_/g'
```

- [ ] **Step 2: 替换 MD 文件中的 hyd_ → hyd_**

```bash
find . -name '*.md' -not -path './out/*' -not -path './.git/*' | xargs sed -i 's/\bhdy_/hyd_/g'
```

- [ ] **Step 3: 替换 MD 文件中的 __Hyd → __Hyd**

```bash
find . -name '*.md' -not -path './out/*' -not -path './.git/*' | xargs sed -i 's/__Hyd/__Hyd/g'
```

- [ ] **Step 4: 替换 MD 文件中的 HydMotion → HydMotion**

```bash
find . -name '*.md' -not -path './out/*' -not -path './.git/*' | xargs sed -i 's/HydMotion/HydMotion/g'
```

- [ ] **Step 5: 替换 MD 文件中的 HydSimulator → HydSimulator**

```bash
find . -name '*.md' -not -path './out/*' -not -path './.git/*' | xargs sed -i 's/HydSimulator/HydSimulator/g'
```

- [ ] **Step 6: 验证替换结果**

```bash
grep -rE '\bHDY_|\bHdy_|__Hyd' --include='*.md' | head -20
```

预期：无输出或仅含不应替换的上下文

---

## Task 8: Shell 脚本替换

**Files:** `scripts/deploy_embedded_prod.sh`（含 `hyd_config.h` 引用）

- [ ] **Step 1: 替换脚本中的 hyd_config → hyd_config**

```bash
sed -i 's/hyd_config/hyd_config/g' scripts/deploy_embedded_prod.sh
```

- [ ] **Step 2: 验证**

```bash
grep -n 'hyd_config\|HYD_' scripts/deploy_embedded_prod.sh
```

预期：无输出

---

## Task 9: 构建验证

- [ ] **Step 1: CMake 重新配置**

```bash
cmake --preset unixgcc
```

- [ ] **Step 2: 全量构建**

```bash
cmake --build --preset unixgcc
```

预期：0 errors, 0 warnings（与重命名前一致）

- [ ] **Step 3: 运行全部测试**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

预期：所有测试 PASS

- [ ] **Step 4: 运行集成示例**

```bash
./out/build/unixgcc/main
```

预期：正常输出泵速、流量、压力、段切换信息

---

## Task 10: 最终一致性检查

- [ ] **Step 1: 确认源码中无残留 HYD_/Hdy_/hyd_/__Hyd**

```bash
grep -rE '\bHDY_|\bHdy_|\bhdy_|__Hyd' --include='*.c' --include='*.h' | grep -v matiec
```

预期：无输出

- [ ] **Step 2: 确认文档中无残留**

```bash
grep -rE '\bHDY_|\bHdy_|\bhdy_' --include='*.md' | head -10
```

预期：无输出或仅含历史/解释性上下文

- [ ] **Step 3: 确认旧文件名不存在**

```bash
find . -iname '*hdy*' -not -path './out/*' -not -path './.git/*'
```

预期：无输出

- [ ] **Step 4: 确认新命名一致性**

```bash
echo "=== HYD_ count ===" && grep -rE '\bHYD_' --include='*.c' --include='*.h' | grep -v matiec | wc -l && echo "=== Hyd_ count ===" && grep -rE '\bHyd_' --include='*.c' --include='*.h' | grep -v matiec | wc -l && echo "=== hyd_ count ===" && grep -rE '\bhyd_' --include='*.c' --include='*.h' | grep -v matiec | wc -l && echo "=== __Hyd count ===" && grep -rE '__Hyd' --include='*.c' --include='*.h' | grep -v matiec | wc -l
```

---

## 替换规则速查表

| 原始模式 | 替换为 | sed 表达式 | 作用域 |
|---------|--------|-----------|--------|
| `HYD_` | `HYD_` | `s/\bHDY_/HYD_/g` | 宏/枚举/类型/结构体 |
| `Hdy_` | `Hyd_` | `s/\bHdy_/Hyd_/g` | CamelCase 内部函数 |
| `hyd_` | `hyd_` | `s/\bhdy_/hyd_/g` | include/文件名引用 |
| `__Hyd` | `__Hyd` | `s/__Hyd/__Hyd/g` | IEC 框架函数前缀 |
| `HydMotion` | `HydMotion` | `s/HydMotion/HydMotion/g` | IEC 框架名称 |
| `HydSimulator` | `HydSimulator` | `s/HydSimulator/HydSimulator/g` | IEC 仿真框架名称 |

## 不替换清单

| 模式 | 原因 |
|------|------|
| `HydroMotionLib` | CMake 库目标名 |
| `HydroSimLib` | CMake 库目标名 |
| `HydraulicSim*` | 仿真器物理层 API |
| `HydroValve/Pump/Cylinder` | 仿真器硬件抽象类型 |
| `hydro_sim*` / `hydro_interfaces*` / `hydro_hardware*` | 仿真器文件名/类型 |
| `HYDRO_SIM_H` 等 | 仿真器 include guard（已用 HYDRO 前缀，不受 HDY→HYD 影响但需注意不误替换） |
| `include/matiec/*` | 第三方 IEC 类型系统 |
