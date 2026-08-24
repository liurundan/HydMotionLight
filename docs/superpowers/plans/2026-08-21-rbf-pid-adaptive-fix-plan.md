# RBF-PID 自适应压力闭环有效化 — 开发计划

> 日期：2026-08-21
> 对应设计：`docs/superpowers/specs/2026-08-21-rbf-pid-adaptive-fix-design.md`
> 分支：`rbf-pid-adaptive-fix`
> 执行顺序：严格按阶段编号顺序执行，每阶段完成后跑回归测试

---

## 执行原则

1. **逐项实施 + 自洽检查**：每改一项立即对照设计 §1-§4 的"自洽检查"条款验证无矛盾。
2. **增量验证**：每阶段完成后必须：
   - `ctest --test-dir out/build/unixgcc --output-on-failure` 全绿（重点：`test_rbf_pid`, `test_pressure_controller`, `test_rbf_pid_hil`）
   - `./scripts/deploy_embedded_prod.sh` 可编译（RBF 核心库不依赖 sim）
3. **不破现有数值行为**：PI 短路（§4.3）等改动必须先建基线、后验证无回归。
4. **5% 超调硬门限**：所有场景（一阶模型阶跃、物理模型三段基准）超调 < 5% 作为验收必要条件。

---

## 阶段 0：准备与回归基线建立

**目标**：确认当前分支测试全绿、嵌入式可编译、建立 PI/RBF 数值基线供后续对比。

### 0.1 确认起点状态
- [ ] 当前分支：`rbf-pid-adaptive-fix`（已创建）
- [ ] `ctest --test-dir out/build/unixgcc --output-on-failure` 全绿
- [ ] `./scripts/deploy_embedded_prod.sh` 成功编译

### 0.2 建立数值基线
- [ ] 运行 `test_pressure_controller` 并记录 PI 路径输出（作为对照组）
- [ ] 运行 `test_rbf_pid` 并记录现有 RBF-PID 数值行为
- [ ] 运行 `test_rbf_pid_hil` 记录仿真闭环指标
- [ ] 建立基线文档 `docs/superpowers/baselines/rbf-pid-pre-fix-baseline.md`，包含：
  - 测试输出快照（关键数值：ITAE、超调、稳态误差）
  - 编译器版本、CMake 配置（`unixgcc` preset）

**验收**：基线文档完成，测试全绿，编译通过。

---

## 阶段 1：配置层准备 — 魔法系数入 `hyd_config.h`（设计 §3.3/§3.4）

**目标**：把所有硬编码阈值迁移到 `include/hyd_config.h` §14B，附物理量纲注释；稳态判据相对量程化。

### 1.1 新增配置常量（追加到 `hyd_config.h` §14B，当前最后一条是 `HYD_THRESH_RBF_ETA_SCALE_GAIN` 4