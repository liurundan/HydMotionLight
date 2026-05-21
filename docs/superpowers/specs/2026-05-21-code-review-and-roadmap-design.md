# HydroMotionLib 全面代码评审与开发路线图

Date: 2026-05-21

## 目的

本文档记录对 HydroMotionLib（注塑机液压运动控制库）在 commit `71808bf` 时的全面专家评审结果，并基于评审给出分 Sprint 的开发路线图。本文档是后续每个 Sprint 详细开发计划的总纲。

---

## 总评

**结论**：架构边界清晰、工程化水平高于同类开源项目均值，作为"合模/开模/顶针/座台/简单射胶/保压"五大常规动作的运动控制后端**已基本可用**；但要正式承担**低压护膜、复杂多段射胶 VP 切换、bumpless 全工况**这类高端工艺特性，**仍有 3 处关键缺陷与 5 处重要缺陷需补齐**。

| 维度 | 评分 |
|---|---|
| 架构设计 | ★★★★☆ |
| 代码质量 | ★★★★☆ |
| 完成情况 | ★★★☆☆ |
| 核心动作适配 | ★★★★☆ |
| 复杂工艺适配 | ★★☆☆☆ |

---

## 架构强项（须保持）

1. **三层职责切分文档化**（`docs/architecture/control-layer-boundary.md`）：算法层 / 运动运行时层 / IEC 适配层 / PLC 工艺层各司其职，明确边界。
2. **运行时契约表格化**（`motion-runtime-contract.md`）：FB_STATE × 命令合法性矩阵 / 输出信号语义 / 完成条件。
3. **动作画像模板**（`motion-profile-archetypes.md` + `action_profile.c`）：7 类典型段提供默认 builder。
4. **PLCopen 风格 FB 池 + execution_id 仲裁**：符合 IEC61131-3 工业惯例。
5. **命令仲裁的 epoch 模型 + 采样-消费分离**：严格"一帧一命令"，ABORT 单独允许覆盖 pending。
6. **HOLD 时基保持 + Bumpless tracking infrastructure**：教科书做法。
7. **诊断 criteria 表驱动**：debounce / 滞回 / 启动抑制 / 切段抑制 / 闭环建立抑制 / WARNING→FAULT 升级。
8. **Recipe 验证早失败**：工艺约束硬编码到 `recipe_validator.c`。
9. **静态内存 + C99 + matiec IEC 类型系统**：嵌入式友好。
10. **测试/源码比 1.27×**，集成测试占比 53%，使用真实物理仿真器。

---

## 问题清单（按优先级）

### Critical（生产前必修）

| ID | 问题 | 文件位置 | 影响 |
|---|---|---|---|
| C-1 | `HYD_AXISMOTION` 双向脏写：PLC↔runtime 共享结构无 force-flag 或 dirty-bit 守护，多 FB 实例同轴下后写覆盖前写 | `src/motion_interface.c:626-756` | 多 FB 共用同轴时数据竞争 |
| C-2 | Recipe NextSegment 触发误报 COMMANDABORTED：`_executionId++` 在每段开始递增，MoveProfile FB 在多段配方推进时被误判失去所有权 | `src/motion_control.c:717`、`src/motion_interface.c:289-309` | 多段配方下 PLC 错误地观察到 COMMANDABORTED |
| C-3 | FAULT 状态无 ABORT 退出路径：命令合法性矩阵中 `HYD_CMD_ABORT` mask 不含 FAULT；唯一脱困路径是直接置位 `fb->RESET` | `src/motion_control.c:74-81` | 工艺侧未接 Reset FB 时永久卡死 |
| C-4 | STOP 减速分支跳过所有诊断：`_isStopping` 分支直接 return，不评估 sensor / time-rollback / timeout | `src/motion_control.c:1603-1651` | STOP 过程中传感器异常或时钟回滚不报警 |
| C-5 | OVER_PRESSURE 仅在 PRESSURE_CLOSED_LOOP 模式评估：合模段（POSITION/SPEED_RAMP）过压完全不触发 | `src/motion_control.c:1293-1316` | **阻塞低压护膜支持** |
| C-6 | RBF-PID 半成品：`MAX_PRESSURE=250.0f` 硬编码归一化、增益限幅窗 [0.8, 0.85] 极窄、reset 后首步 KD 跳变 | `include/rbf_pid.h:19-28`、`src/rbf_pid.c:281-283` | 启用即可能异常 |
| C-7 | `diagnostics_monitor.c` 使用 `!= 0.0` 判断误差激活：压力收敛后浮点抖动仍累加 errorDuration | `src/diagnostics_monitor.c:58` | fault escalation 持续时间数据失真 |

### Important（应当修复）

| ID | 问题 | 影响 |
|---|---|---|
| I-1 | `vpTransferReady` 不 latched + 判据优先级硬编码 `position > pressure > time > velocity_drop` | PLC 必须自己捕获上升沿；优先级与工艺界 pressure-first 惯例不符 |
| I-2 | 单泵 `GetPumpRequest` 无方向冲突检测 | 两轴反向同时 active 时仲裁器不报错 |
| I-3 | ACK 不允许在 RUNNING 态调用 | 长保压段中无法清 WARNING 级诊断 |
| I-4 | SPEED_RAMP 段间无 BLENDING | 多段射胶速度曲线出现"建速→恒速→减速→再建速"阶梯 |
| I-5 | 跨控制器策略切换（PID↔RBF）增益跳变 | bumpless transfer 局部失效 |
| I-6 | VP bumpless 反向（压力→速度）未实现 | 保压结束切回速度段时首帧速度阶跃 |
| I-7 | `state_reporter.c` / `diagnostics.c` / `protection_manager.c` 无独立单元测试 | 大量代码仅有集成路径覆盖 |
| I-8 | `RunRunningState` 单函数 ~240 行 | 维护负担集中、bug 注入风险高 |

### Minor（计入下个版本）

| ID | 问题 |
|---|---|
| M-1 | 重复测试 `tests/ramp_controller_test.c` vs `tests/test_ramp_controller.c` |
| M-2 | CMake 注册遗漏 `test_direct_mode_simple` / `benchmark_performance` |
| M-3 | `protection_manager` 命名误导，建议改 `safety_state_manager` |
| M-4 | 硬编码浮点阈值散落，集中到 `hyd_config.h` |
| M-5 | 时钟回滚保护下沉到 `diagnostics_monitor` |
| M-6 | `ramp_controller` 需要公开 `Reseed(current, t)` 接口 |
| M-7 | VP transfer 测试只覆盖 4 判据中 2 个（缺 time / velocity_drop 断言） |

---

## 与注塑机核心动作的适配度

| 动作 | 库内支持 | 风险/限制 |
|---|---|---|
| 合模 / 开模 | ✓ 充分 | 段间无 BLENDING |
| 顶针 | ✓ 充分 | 单泵下与合模/射胶可能竞争 |
| 座台 | ✓ 充分 | 同上 |
| 保压 | ✓ 充分 | PressureHandle 无 terminal DONE |
| 射胶（多段 + VP 观察） | ◐ 基本可用 | vpTransferReady 不 latched；优先级硬编码；段间无 BLENDING |
| VP 瞬态 bumpless | ◐ 单向（V→P） | 反向（P→V）未做 |
| **低压护膜** | ✗ **不足以支持** | OVER_PRESSURE 仅在 PRESSURE_CLOSED_LOOP 模式评估；无段内 `pressureCeiling` 字段；无 position-window-aware 机制；DERATE 固定 0.5 比例 |

---

## 开发路线图（4 个 Sprint）

### Sprint 0：紧急修复 + 风险消除（预计 2 周）

修复 7 项 Critical/Important，使库具备承载 Sprint 1-3 新功能的健康基线。

| # | 任务 | 优先级 | 估时 | 产出 |
|---|---|---|---|---|
| 0.1 | 修复 C-2（recipe execId 误抢占）：引入 `_recipeExecutionId` 双 ID | P0 | 3d + 2d test | 修复 + 多段 NextSegment 集成测试 |
| 0.2 | 修复 C-1（HYD_AXISMOTION 双向脏写）：拆 Setpoint/Actual 半区或加 dirty-bit | P0 | 3d + 2d test | 修复 + race 测试 |
| 0.3 | 修复 C-3（FAULT 出口）：ABORT mask 加上 FAULT 位 | P0 | 1d + 1d test | mask 修正 + FAULT→ABORTED 路径测试 |
| 0.4 | 修复 C-4（STOP 分支跳过诊断）：保留最小安全检查 | P0 | 2d + 1d test | stop 期间 fault 检测测试 |
| 0.5 | 修复 C-7（`!= 0.0` 误差判据）：tolerance 下沉到 monitor | P1 | 1d + 1d test | monitor 重构 + jitter 抑制测试 |
| 0.6 | 给 `motion_control.c` / `state_reporter.c` / `protection_manager.c` 补单元测试 | P1 | 3d | 3 个新测试文件 |
| 0.7 | 删除重复测试、补全 CMake 注册（M-1, M-2） | P2 | 0.5d | CMake 清理 |

### Sprint 1：低压护膜支持（预计 2 周）

把"段内压力软上限 + position-window-aware 保护"下沉为库的一级原语，让 PLC 工艺层不再重复实现。

| # | 任务 | 产出 |
|---|---|---|
| 1.1 | 在 `HYD_MotionSegment` 增加 4 字段：`pressureCeiling`, `pressureCeilingTolerance`, `pressureCeilingPositionStart`, `pressureCeilingPositionEnd` | `common_types.h` 改动 |
| 1.2 | 新增诊断码：`HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED`（WARNING, DERATE）+ `HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED`（FAULT, STOP） | `diagnostics.c` 表更新 + `HMI诊断对照表.md` |
| 1.3 | 在 `RunRunningState` 中**所有 mode 下**评估 ceiling 判据 | `motion_control.c` 改动 |
| 1.4 | `output_limiter.c` 把 `derateRatio` 改为可由段配置 | `output_limiter.c` + segment 字段 |
| 1.5 | 新增 `HYD_ActionProfile_BuildClampCloseWithMoldProtect()` 模板 builder | `action_profile.c` |
| 1.6 | 端到端集成测试：合模段 + hydro_sim 模具阻力 → ceiling → DERATE → STOP 升级 | `tests/test_mold_protect.c` |
| 1.7 | 更新 `motion-profile-archetypes.md` + `motion-runtime-contract.md` | 文档同步 |

### Sprint 2：多段射胶 & VP 切换增强（预计 2 周）

| # | 任务 |
|---|---|
| 2.1 | VP transfer：判据优先级从硬编码改为段内可配置（`vpTransferCriteriaOrder` mask） |
| 2.2 | VP transfer：`vpTransferReady` 可选 latched 模式（段内字段 `vpTransferLatch=true` 保持电平） |
| 2.3 | VP transfer：补齐 time / velocity_drop 判据单元测试 + 4 判据并发优先级测试 |
| 2.4 | VP bumpless 反向：新 SPEED_RAMP 段以 `_lastCommandedFlow / velocityToFlowGain` 作为 previousVelocity 种子 |
| 2.5 | SPEED_RAMP 段间 BLENDING：跨段保留 `lastTargetVelocity` |
| 2.6 | PressureHandle 增加 terminal DONE 语义 |
| 2.7 | 跨控制器策略切换 KP/KI/KD clamp 后再生效（修 I-5） |

### Sprint 3：架构清理 & RBF-PID 工程化（预计 3 周）

| # | 任务 |
|---|---|
| 3.1 | 重命名 `protection_manager.*` → `safety_state_manager.*` |
| 3.2 | 拆分 `motion_control.c:RunRunningState` → 3 个 helper（normal / stopping / blend_cutover） |
| 3.3 | 硬编码浮点阈值集中到 `hyd_config.h` |
| 3.4 | RBF-PID：去掉硬编码 `MAX_PRESSURE=250.0` 归一化；增益限幅窗扩展；与策略切换的种子在 LIMIT 区间内 |
| 3.5 | RBF-PID 端到端 HIL 测试（含跨段切换 + 长时保压） |
| 3.6 | 单泵方向冲突检测：`GetPumpRequest` 返回 direction conflict flag |
| 3.7 | 引入 `gcovr` 覆盖率统计进 CI |

### Sprint 4（可选）：高级 BLENDING & 性能优化

- 实现 PLCopen `BlendingPrevious/Next/Low/High` 四种模式
- 用 perf-sanitizer 测量每周期 worst-case CPU
- 评估 sqrt lookup table 优化

---

## 给项目负责人的关键决策

1. **不要在 Sprint 1 之前把库部署到真实注塑机做合模工艺**——缺低压护膜会导致模具风险。可以先在保压、顶针、座台单元测试。
2. **C-1 与 C-2 是生产级潜在 bug 候选**，建议立即补 race-condition 测试做验证。
3. **RBF-PID 默认关掉**（生产默认 `pressureController = HYD_PRESSURE_CONTROLLER_PI`），等 Sprint 3 工程化完成再开放。
4. **PLC 集成文档需追加章节**：
   - "FAULT 唯一出口是 Reset FB"（在 Sprint 0.3 之前）
   - "vpTransferReady 是瞬时电平，PLC 必须捕获上升沿"
5. **`motion_control.c` 列入"严禁新增功能直加，必须先拆分"**——技术债热点。

---

## 与其他文档的关系

- 与 [implementation-contract-gap-list.md](/home/dan/project/hdy-motion-light/docs/architecture/implementation-contract-gap-list.md) 并行：本文档关注 Critical 级缺陷，gap list 关注 IEC 引脚契约一致性。
- 每个 Sprint 落地时需要在 `docs/superpowers/plans/` 写对应的详细实施计划（YYYY-MM-DD-sprintN-<topic>.md）。

