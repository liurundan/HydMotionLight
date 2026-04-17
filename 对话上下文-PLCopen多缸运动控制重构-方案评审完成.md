# 对话上下文：PLCopen 多缸运动控制重构（方案评审完成）

## 📋 问题背景

### 项目信息
- 项目：`hdy-motion-light`
- 类型：C99 静态内存、嵌入式友好的液压/注塑机场景运动控制库
- 当前核心边界：
  - **工艺层**负责机理、阀逻辑、段切换、互锁、动作编排
  - **运动控制库**负责运动数学、压力/流量规划、泵速换算、诊断输出
- 当前主控入口：`HDY_MotionControlFB`
- 当前运行方式：以 **配方/段（recipe/segment）驱动** 为主，通过 `LoadRecipe -> StartSegment / START_SEGMENT -> Execute` 运行

### 当前讨论主题
围绕 `motion_control.c` 进行重构评估，目标是让 `HDY_MotionControlFB`：
- 更符合 **PLCopen 功能块使用习惯**
- 可作为 **合模、射胶、顶针、座台等多缸控制**的统一 FB 类型复用
- 支持 **命令上升沿触发**
- 接收工艺层下发的运动控制参数
- 周期性回传执行状态给工艺层
- 在 **cycle 函数**中运行状态机与控制逻辑
- 后续可方便扩展更多指令（如 Hold / Resume / Stop / Jump 等）

### 当前代码现状（已评审）
已分析的关键文件包括：
- `include/motion_control.h`
- `src/motion_control.c`
- `include/common_types.h`
- `src/motion_planner.c`
- `src/segment_completion.c`
- `src/state_reporter.c`
- `src/protection_manager.c`
- `src/pump_converter.c`
- `src/diagnostics.c`
- `tests/test_motion_control.c`
- `项目需求与设计说明书.md`

当前架构优点：
- 模块边界基本合理：planner / pressure / pump / completion / diagnostics 已拆分
- 保持了“工艺层 vs 运动层”的职责边界
- 静态内存、C99、固定上限，适合嵌入式/PLC 风格场景

---

## 🔴 核心问题

### 问题表现
当前 `motion_control.c` 仍更像“**单执行器、单活动段、配方型控制器**”，而不是“**命令驱动、状态机明确、适合 PLCopen 扩展的功能块**”。

### 已识别的核心问题
1. **命令入口不够 PLCopen 化**
   - `START_SEGMENT` 当前更接近**电平触发**，而不是严格的**上升沿触发 + 内部锁存**。
   - 如果工艺层/HMI 将启动位保持高电平多个周期，存在重复触发风险。

2. **`Execute()` 过于臃肿**
   - 当前 `Execute()` 同时承担：
     - EN/RESET 处理
     - 启动命令处理
     - 反馈有效性校验
     - 参数校验
     - ramp / planner / pressure controller / pump converter 计算
     - 诊断更新
     - 段完成判断
     - 输出发布
   - 后续若继续扩展 Stop / Hold / Resume / Jump 等命令，维护复杂度会快速失控。

3. **状态模型不显式**
   - 当前更多依赖 `ACTIVE / FINISHED / FAULT / STATUS / SEGMENT_COMPLETED` 组合推导状态。
   - 这对扩展命令合法性判断不够友好。

4. **配方执行模型偏强，工艺层直接下发参数的场景支持不自然**
   - 当前更偏 `LoadRecipe + StartSegment(index)`。
   - 新需求要求支持工艺层在命令触发时直接下发“当前动作参数”。

5. **多缸复用方式需明确**
   - 如果做成“一个 FB 内部同时管理多个缸”，会让状态机、参数、互锁和机理逻辑严重耦合。
   - 更合理方式应是：**一个 FB 管一个运动对象，多缸靠多实例复用**。

6. **未来扩展命令缺少统一入口**
   - 当前一部分命令是输入位，一部分是函数调用，风格不统一。
   - 扩展成本高，不利于演进为 PLCopen 风格接口。

### 根源分析
根本问题不在 `motion_planner.c` 等计算模块，而在于 `motion_control.c` 缺少一层明确的：
- 命令采样/边沿检测
- 命令锁存
- 显式状态机
- 命令仲裁
- 输出发布

换言之：
> 当前主要短板是 **控制器框架层**，不是 **算法层**。

---

## 🎯 实现目标

### 主要目标
把 `HDY_MotionControlFB` 重构成更符合 PLCopen 使用习惯、支持多缸复用、易扩展命令的通用单缸运动功能块。

### 具体要求
1. **一个 FB 实例只管理一个缸/一个运动对象**
   - 合模、射胶、顶针、座台等采用同一 FB 类型，多实例复用。

2. **命令必须支持上升沿触发**
   - 例如 `Start` 只在上升沿被接受一次。
   - 参数必须在上升沿时锁存，锁存后不受输入变化影响。

3. **Execute 与 Cycle 分离**
   - `Execute()`：输入采样、边沿检测、命令锁存
   - `Cycle()`：状态机迁移、控制计算、诊断与输出刷新

4. **建立显式状态机**
   - 至少覆盖：Disabled / Ready / Starting / Running / SegmentComplete / Hold / Done / Aborted / Fault

5. **兼容两种触发模式**
   - Recipe 模式：工艺层先装载多段，再按段运行
   - Direct 模式：工艺层直接下发当前动作参数，Start 边沿时锁存

6. **输出持续回传给工艺层**
   - 包括 Busy / Done / Active / Error / ErrorID / 当前段信息 / 规划值 / 诊断等

7. **为未来扩展预留统一命令模型**
   - Start / Next / Stop / Hold / Resume / Abort / Reset / Ack / Jump 等命令后续可平滑增加

### 约束条件
- 必须保持工艺层与运动层边界：阀动作、顺控、互锁仍归工艺层
- 保持 C99、静态内存、固定上限、无动态分配
- 保持已有模块拆分，不建议把 planner/pressure/pump/conpletion 重新揉回 `motion_control.c`

---

## 🔧 技术约束

### 必须遵守的原则
1. **不把多缸协调逻辑塞进一个大 FB**
2. **不让工艺机理逻辑倒灌进运动控制库**
3. **保持纯 C99 和嵌入式友好设计**
4. **保持 PLCopen 风格但不要机械照搬，需结合本项目泵控/液压特点**
5. **优先重构控制器框架层，不破坏已拆分的算法模块**

### 关键技术限制
- `HDY_MotionControlFB_Init()` 当前会 `memset` 全清，意味着 `RESET` 会清掉配置、配方和运行态，重构时需明确是否保留该语义
- 当前 `PUMP_SPEED` 仍是非负泵侧幅值，方向语义通过 `plannedDirection` 等状态表达
- 当前 `LoadRecipe()` 不会自动启动执行，启动仍依赖 `StartSegment()` 或 `START_SEGMENT`
- 当前测试主要围绕 `Execute()` 展开，若增加 `Cycle()`，测试体系需同步升级

### 关键文件
#### 当前核心文件
- `/home/dan/project/hdy-motion-light/include/motion_control.h`
- `/home/dan/project/hdy-motion-light/src/motion_control.c`
- `/home/dan/project/hdy-motion-light/src/motion_planner.c`
- `/home/dan/project/hdy-motion-light/src/segment_completion.c`
- `/home/dan/project/hdy-motion-light/src/state_reporter.c`
- `/home/dan/project/hdy-motion-light/src/protection_manager.c`
- `/home/dan/project/hdy-motion-light/tests/test_motion_control.c`

#### 需求/架构参考文件
- `/home/dan/project/hdy-motion-light/项目需求与设计说明书.md`
- `/home/dan/project/hdy-motion-light/CODEBUDDY.md`

---

## 🚫 已尝试的方案

> 本轮对话尚未进行代码落地修改，以下为**已评估/已否定或不推荐**的方案。

### 方案 1：一个 `HDY_MotionControlFB` 实例内部统一管理多缸
**结论：不推荐**

**原因：**
- 会导致状态机膨胀
- 参数模型复杂化
- 互锁与工艺顺控逻辑会污染运动层
- 维护成本极高，违背项目边界

### 方案 2：继续沿用当前 `START_SEGMENT` 电平触发模型
**结论：不推荐**

**原因：**
- 不符合 PLCopen 常见命令体验
- 容易被 HMI/工艺层高电平保持误触发
- 不利于做参数锁存和命令仲裁

### 方案 3：继续让 `Execute()` 同时处理命令、状态机、控制算法、输出刷新
**结论：不推荐继续扩展**

**原因：**
- 当前还能工作，但扩展更多指令后将不可维护
- 逻辑耦合过重，难以测试命令行为与状态迁移

### 方案 4：仅靠 `ACTIVE / FINISHED / FAULT` 等布尔量拼状态
**结论：不推荐作为未来演进基础**

**原因：**
- 命令合法性判断会越来越混乱
- 不能清晰表达 Hold / Aborted / SegmentComplete / Done / Fault 等语义差异

---

## ✅ 当前方案 / 最终推荐方案

### 总体设计思路
采用“**通用单缸 FB + 多实例复用 + 命令驱动状态机**”的重构方式。

### 方案核心结论
1. **一个 FB 实例只管一个对象**
   - `ClampFB`
   - `InjectFB`
   - `EjectorFB`
   - `CarriageFB`
   - 由工艺层协调这些实例的先后关系和互锁

2. **`Execute()` 与 `Cycle()` 分层**
   - `Execute()` = 命令采样器 / 边沿检测器 / 参数锁存器
   - `Cycle()` = 状态机执行器 / 控制运算器 / 输出发布器

3. **引入统一命令模型**
   建议内部用统一命令枚举，例如：
   - `HDY_CMD_NONE`
   - `HDY_CMD_START`
   - `HDY_CMD_NEXT`
   - `HDY_CMD_STOP`
   - `HDY_CMD_HOLD`
   - `HDY_CMD_RESUME`
   - `HDY_CMD_ABORT`
   - `HDY_CMD_RESET`
   - `HDY_CMD_ACK`

4. **引入待处理命令锁存区**
   由 `Execute()` 在上升沿产生 `_pendingCommand`，由 `Cycle()` 消费。

5. **建立显式状态机**
   建议内部执行状态包括：
   - `HDY_FB_STATE_DISABLED`
   - `HDY_FB_STATE_IDLE`
   - `HDY_FB_STATE_READY`
   - `HDY_FB_STATE_STARTING`
   - `HDY_FB_STATE_RUNNING`
   - `HDY_FB_STATE_SEGMENT_COMPLETE`
   - `HDY_FB_STATE_HOLD`
   - `HDY_FB_STATE_DONE`
   - `HDY_FB_STATE_ABORTED`
   - `HDY_FB_STATE_FAULT`

6. **支持两种参数来源**
   - `UseRecipe = true`：从 `RECIPE[index]` 锁存当前活动段
   - `UseRecipe = false`：从 `DirectSegment` 锁存当前活动段

7. **保留现有算法模块，重构 `motion_control.c` 为薄协调器**
   不建议推翻：
   - `motion_planner.c`
   - `pressure_controller.c`
   - `pump_converter.c`
   - `segment_completion.c`
   - `diagnostics.c`
   重点改控制器框架，而不是运动数学内核。

### 推荐的内部职责拆分
#### A. 输入采样层
- 采样命令位
- 做边沿检测
- 读取工艺层参数输入
- 读取反馈输入

#### B. 命令锁存层
- 生成 `_pendingCommand`
- 在 Start 上升沿锁存参数
- 统一拒绝非法命令

#### C. 状态机层
- 处理状态迁移
- 做命令合法性判断
- 处理正常完成 / 中止 / 故障 / 保持

#### D. 控制执行层
- ramp / planner / pressure control / pump convert
- 诊断更新
- 段完成判断

#### E. 输出发布层
- Busy / Done / Error / ErrorID / Active
- 当前段状态、规划信息、诊断信息

### 推荐的兼容策略
为了避免一次性打碎现有接口，建议：
- 保留原有：
  - `HDY_MotionControlFB_LoadRecipe()`
  - `HDY_MotionControlFB_StartSegment()`
  - `HDY_MotionControlFB_NextSegment()`
  - `HDY_MotionControlFB_Abort()`
  - `HDY_MotionControlFB_AcknowledgeDiagnostics()`
- 新增：
  - `HDY_MotionControlFB_Cycle()`
  - 可选 `HDY_MotionControlFB_Scan()` 作为 `Execute() + Cycle()` 的兼容入口
- 旧 API 可改为“构造 pending command，再由 Cycle 消费”的包装方式

### 当前讨论中识别出的具体问题修正点
1. **`START_SEGMENT` 需要改成上升沿锁存，不是电平触发**
2. **压力模式下 `plannedDirection` 会被错误覆盖为 `HOLD`，需保留动作方向语义**
3. **段完成后立即清 `_lastCommandedFlow` 不利于速度段 -> 压力段的平滑衔接**
4. **诊断与跟踪偏差建议分层，避免调试期 HMI 过多 warning 噪声**

---

## 📝 关键代码变更

### 本轮实际代码变更
- **本轮未实际修改代码文件**
- 当前完成的是：
  - 需求评估
  - 现有框架分析
  - 重构方向确认
  - 新增需求归纳
  - 关键风险识别
  - 推荐方案成型

### 计划中的关键变更文件
1. `/home/dan/project/hdy-motion-light/include/motion_control.h`
   - 增加执行状态枚举
   - 增加命令模型/输入模型/输出模型（如采用新结构）
   - 增加 `Cycle()` / `Scan()` 接口
   - 为 Direct 模式和 Recipe 模式留接口

2. `/home/dan/project/hdy-motion-light/src/motion_control.c`
   - 拆分 `Execute()` 与 `Cycle()`
   - 引入 pending command / edge detect / state machine
   - 重构为薄协调器

3. `/home/dan/project/hdy-motion-light/tests/test_motion_control.c`
   - 增加上升沿触发测试
   - 增加命令合法性与状态迁移测试
   - 增加 Direct 模式测试
   - 增加多实例隔离测试

4. 可能需要联动检查：
   - `/home/dan/project/hdy-motion-light/src/state_reporter.c`
   - `/home/dan/project/hdy-motion-light/src/protection_manager.c`
   - `/home/dan/project/hdy-motion-light/src/diagnostics.c`

### 代码量评估
- 当前尚未实施，无法给出精确代码量
- 预计第一阶段主要改动集中在 `motion_control.h/.c` 与 `tests/test_motion_control.c`
- 如采取“兼容旧 API + 新增 Cycle/状态机”的方式，改动量中等偏大，但风险可控

---

## 🎯 当前进度

### 已完成
- 已明确项目边界与约束
- 已评审当前 `motion_control.c` 架构适配性
- 已明确多缸场景推荐方案：**同类型 FB 多实例复用**
- 已确认核心重构方向：**命令锁存 + 显式状态机 + Execute/Cycle 分离**
- 已识别现有实现中的几个关键缺陷：
  - Start 命令触发方式
  - 压力模式方向语义丢失
  - `_lastCommandedFlow` 清理时机不合理
  - 诊断噪声需要分层治理

### 进行中
- 方案层已基本定稿
- 尚未进入接口定义与代码骨架落地阶段

### 待处理
- 输出新版 `motion_control.h` 接口草案
- 输出新版 `motion_control.c` 状态机骨架
- 决定兼容策略（旧字段保留多少、是否引入新输入输出结构）
- 编写/更新测试用例
- 评估是否同步调整 README / 设计说明文档

### 当前状态判断
**状态：方案评审完成，尚未进入代码实施阶段。**

---

## 💡 使用方法

### 作为后续开发输入使用
在新对话中，可将本文件作为“恢复上下文文档”直接提供给新的 AI 或开发者，建议做法：

1. 先说明：
   - 这是当前 `motion_control` 重构方案的上下文文档
   - 请先基于文档理解背景、目标、约束和当前推荐方案

2. 再补充你的下一步需求，例如：
   - “请基于该文档输出新的 `motion_control.h` 接口设计”
   - “请先给出 `motion_control.c` 的状态机骨架代码”
   - “请按第一阶段最小闭环直接实施代码修改并补测试”

### 推荐的下一步调用顺序
如果进入实现阶段，建议按以下节奏推进：
1. 先改头文件接口设计
2. 再实现 `motion_control.c` 框架层
3. 再补测试
4. 再跑构建与 CTest 回归

### 建议的开发顺序（最小闭环）
第一阶段先实现：
1. 增加内部显式状态枚举
2. 增加命令上升沿检测
3. 将 `Execute()` 和 `Cycle()` 分离
4. 启动时锁存活动段参数
5. 将运行逻辑收敛到 Running 状态处理函数
6. 修复压力模式方向覆盖问题
7. 调整 `_lastCommandedFlow` 的保留策略

第二阶段再考虑：
1. Hold / Resume
2. Direct 模式与 Recipe 模式并存
3. Busy / Done / Error / ErrorID 标准化输出
4. 诊断分层
5. 多实例联调用例

### 新对话中可直接使用的提示语示例
```text
请先阅读项目根目录下的《对话上下文-PLCopen多缸运动控制重构-方案评审完成.md》，然后基于其中的约束和方案，继续输出：
1）新版 motion_control.h 接口草案；
2）motion_control.c 的 Execute/Cycle 状态机骨架；
3）第一阶段需要补充的测试清单。
```

### 注意事项
- 本文档记录的是**方案结论和上下文**，不是最终实现代码
- 文中提到的结构体/枚举/API 名称部分为**建议草案**，落地时仍需结合现有命名和兼容性细化
- 若后续决定“保留旧接口字段”或“彻底引入新输入/输出结构”，需要在实现前先统一策略

---

## 🐛 已知问题和待解决

### 已知问题
1. 当前 `START_SEGMENT` 语义仍偏电平触发
2. `Execute()` 职责过多，未来扩展风险高
3. 运行态依赖布尔拼接，缺少显式状态机
4. 压力闭环模式下方向语义存在丢失风险
5. 正常段完成后执行量清理策略可能影响平滑切段
6. 诊断与调试偏差未分层，可能导致 HMI 告警噪声偏多

### 待明确问题
1. 新接口是否引入独立的 `Input/Output/Config` 结构
2. `RESET` 是否保持当前“全清配置和配方”的强语义
3. `Done / SegmentCompleted` 是脉冲还是保持量，是否区分“段完成”和“整体完成”
4. `Abort / Stop / Hold / Resume` 的优先级和合法状态表最终怎么定义
5. Direct 模式与 Recipe 模式是否同时公开给上层
6. 是否需要增加对象类型字段（Clamp / Injection / Ejector / Carriage）用于诊断/HMI

---

## 🚀 下一步计划

### 建议优先级 P1
1. 输出新版 `motion_control.h` 草案
2. 明确内部状态枚举和命令枚举
3. 设计 `pending command` 结构和边沿检测字段
4. 明确 `Execute()` / `Cycle()` / `Scan()` 三者关系

### 建议优先级 P2
1. 按“兼容旧 API”的方式重构 `motion_control.c`
2. 把 `StartSegment()` / `NextSegment()` 改为 pending command 包装
3. 将运行算法搬入 `RunRunningState()` 或等效内部函数
4. 增加输出发布与状态迁移辅助函数

### 建议优先级 P3
1. 更新 `tests/test_motion_control.c`
2. 增加上升沿、锁存、多实例、Direct 模式等测试
3. 跑 `cmake --build --preset unixgcc`
4. 跑 `ctest --test-dir out/build/unixgcc --output-on-failure`

### 后续扩展方向
- Hold / Resume
- Busy / Done / Error / ErrorID 标准化
- 多缸联动示例
- 诊断分层和门槛策略
- 更完整的 HMI / 工艺层接口规范

---

## 📝 备注

### 本轮讨论形成的关键结论
- **不要做成一个 FB 控所有缸**
- **应该做成一个通用单缸 FB，多实例复用**
- **本次真正需要重构的是控制器框架层，不是算法内核**
- **Execute 负责命令采样与锁存，Cycle 负责状态机和控制运算**
- **显式状态机是本次重构的核心**

### 经验教训
- 当前代码模块化已经不错，但 PLCopen 风格的“体验”往往不是数学问题，而是命令模型和状态机模型问题
- 如果不先统一命令入口和状态迁移规则，后续每增加一个指令都会带来结构性复杂度
- 先收敛架构，再落地代码，比直接在 `Execute()` 里继续堆逻辑更稳妥

### 参考线索
- `CODEBUDDY.md` 中对项目架构和边界的说明
- `项目需求与设计说明书.md` 中对工艺层/运动层边界的要求
- 当前 `tests/test_motion_control.c` 中已有的行为语义测试，可作为重构后的回归基线

---

## 快速摘要（供新对话快速恢复）
- 当前正在做的是：**将 `motion_control.c` 从配方执行器重构为 PLCopen 风格、支持多缸复用、可扩展命令的单缸通用 FB**。
- 已确认：**多缸应采用多实例，不应在一个 FB 内统一调度所有缸**。
- 已确认：**应将 `Execute()` 与 `Cycle()` 分离**，前者做命令上升沿采样与参数锁存，后者做状态机和控制运算。
- 已确认：**需要显式状态机和统一命令模型**。
- 当前仍未修改代码，处于：**方案评审完成，待接口与实现落地**。
