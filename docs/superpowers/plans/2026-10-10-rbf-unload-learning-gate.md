# RBF 卸压学习冻结 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 1 ms RBF 压力环内冻结无效卸压样本的网络学习和 PID 增益整定，并保持控制输出连续。

**Architecture:** 在 `RBF_PID_Handle` 中保存运行时冻结原因、恢复预热计数和学习许可状态。`RBF_PID_Update()` 继续执行前向计算、增量输出、前馈、阻尼、限幅和历史滚动，只在网络更新与增益整定处读取统一门控。使用现有 C 回归测试目标，避免增加动态内存或独立卸压网络。

**Tech Stack:** C99, CMake, existing assert-based regression tests.

---

### Task 1: Add failing regression tests

**Files:**
- Modify: `tests/rbf_pid_p0_fixes_test.c`

- [ ] **Step 1: Add a test that reverse output freezes network and gains.**
- [ ] **Step 2: Add a test that target drop with positive output does not freeze adaptation.**
- [ ] **Step 3: Add a test that freeze preserves `jac_neg_count` and output remains finite.**
- [ ] **Step 4: Add a test for two-sample network-first recovery.**
- [ ] **Step 5: Build and run `rbf_pid_p0_fixes_test` to confirm the new assertions fail before production changes.**

### Task 2: Implement the smallest runtime gate

**Files:**
- Modify: `include/rbf_pid.h`
- Modify: `src/rbf_pid.c`

- [ ] **Step 1: Add fixed-size runtime fields and freeze reason bits.**
- [ ] **Step 2: Add finite normalized-input and low-pressure checks using existing scales.**
- [ ] **Step 3: Make reverse output and pressure decline participate in the gate without using target drop or relief state.**
- [ ] **Step 4: Keep forward/output/history execution active while skipping both adaptation paths when frozen.**
- [ ] **Step 5: Hold Jacobian sign count during freeze and implement two-sample network-first recovery.**
- [ ] **Step 6: Build and run the focused regression target.**

### Task 3: Verify integration and regressions

**Files:**
- No production files beyond Task 2.

- [ ] **Step 1: Run `ctest --test-dir out/build/unixgcc -R '^(test_rbf_pid|test_pressure_controller|test_rbf_pid_relief_first_order)$' --output-on-failure`.**
- [ ] **Step 2: Run the complete CTest suite.**
- [ ] **Step 3: Inspect the diff for embedded constraints, public API stability, and unrelated changes.**
