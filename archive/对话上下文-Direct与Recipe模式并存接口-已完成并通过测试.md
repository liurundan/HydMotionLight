# 对话上下文文档：Direct 与 Recipe 模式并存接口

> 状态：**已完成并通过测试**  
> 适用目的：在新对话中快速恢复当前开发上下文，避免重复分析仓库与重复执行已完成任务。

---

## 📋 问题背景（项目信息、当前状态、涉及组件）

### 项目信息
- 项目名称：`hdy-motion-light`
- 技术栈：纯 **C99**、静态内存、嵌入式友好、PLCopen 风格 Function Block 设计
- 核心定位：液压/注塑类运动控制计算库
- 架构边界：
  - **外部过程层**负责工艺顺序、阀逻辑、段切换决策
  - **本库**负责运动规划、压力/流量计算、泵速换算、诊断与状态输出

### 当前开发主题
本轮开发的重点任务是：
- 在同一个 `HDY_MotionControlFB` 中同时支持：
  1. **Recipe 模式**：基于 `RECIPE[]` 的多段执行
  2. **Direct 模式**：基于单个 `DIRECT_SEGMENT` 的直接执行
- 通过统一接口进行启动，并保证运行时段来源被锁存，不受外部中途修改影响。

### 涉及组件
本轮工作主要涉及以下模块：
- `include/common_types.h`
- `include/motion_control.h`
- `src/motion_control.c`
- `include/state_reporter.h`
- `src/state_reporter.c`
- `include/protection_manager.h`
- `src/protection_manager.c`
- `include/diagnostics.h`
- `src/diagnostics.c`
- `include/recipe_validator.h`
- `src/recipe_validator.c`
- `tests/test_motion_control.c`

### 当前状态
- Direct / Recipe 并存接口已实现
- 运行时来源锁存机制已实现
- Direct 单段完成后直接进入 `DONE`
- Direct 模式下 `NEXT` 已明确拒绝并输出诊断
- 回归测试已补充并通过
- 全量构建与 `ctest` 已通过（结论：**9/9 tests passed**）

---

## 🔴 核心问题（问题表现、根源分析、具体数据）

### 问题表现
原有接口更偏向 **Recipe 多段执行模型**，无法良好支持以下需求：
- 同一 FB 同时保有 recipe 配方与 direct 单段参数
- 启动时根据模式自动选择段来源
- Direct 模式作为单段动作运行，而不是伪装成 recipe 的特殊情况
- 运行中即便外部切换 `USE_RECIPE` 或覆盖 direct buffer，也不应影响当前动作

### 根源分析
根因主要有 4 类：
1. **段参数来源单一**：原执行路径默认围绕 `RECIPE[]` 设计。
2. **运行态来源未锁存**：启动后如果继续修改 `USE_RECIPE` 或 direct 段缓存，可能污染执行语义。
3. **完成态语义不完整**：Direct 模式本质是单段动作，完成后应直接 `DONE`，而不是沿用 recipe 的“可 next”语义。
4. **状态/诊断/Ready 判定依赖 recipe**：一些合法性检查与状态推导默认只看 `RECIPE_SIZE`，导致 direct 场景异常。

### 具体确认结果
- 构建与测试最终通过：**9/9 tests passed**
- 已新增 direct 相关回归测试，覆盖：
  - 无 recipe 时 direct 启动
  - 启动后锁存 direct 参数
  - direct / recipe 共存并可切换
  - direct 模式未配置时拒绝启动

---

## 🎯 实现目标（主要目标、具体要求、约束条件）

### 主要目标
1. 在同一个 `HDY_MotionControlFB` 中支持 **Recipe 模式与 Direct 模式并存**
2. 用统一 `StartSegment()` 接口发起执行
3. 启动时根据 `USE_RECIPE` 选择段来源
4. 启动后锁存当前活动段来源与参数，确保运行稳定
5. 保持原有运动规划/压力控制内核尽量不变

### 具体要求
- `USE_RECIPE = true`：从 `RECIPE[]` 读取段
- `USE_RECIPE = false`：从 `DIRECT_SEGMENT` 读取段
- Direct 模式下 `segmentIndex` 不再作为真实 recipe 索引使用
- Direct 模式完成后直接进入 `DONE`
- Direct 模式下 `NextSegment()` 必须拒绝，并给出明确诊断
- Ready/Idle 判定需根据“当前选中来源是否可启动”决定

### 约束条件
- 必须保持 **C99**
- 不引入动态内存
- 保持 PLCopen 风格 FB 接口
- 不破坏既有 motion planner / pressure controller 内核逻辑
- 不改变“过程层负责段切换决策，控制库负责计算”的总体架构边界

---

## 🔧 技术约束（必须遵守的原则、技术限制、关键文件）

### 必须遵守的原则
- 纯 C99，禁止引入 C++ 风格设计
- 静态内存、固定大小结构体、无动态分配
- `HDY_` 前缀命名规范
- FB 输入/输出/内部状态分层清晰
- 外部过程层决定工艺推进；控制库仅提供计算与状态

### 技术限制
- 运行时状态必须可预测，不能因外部异步写字段而改变已启动段的执行语义
- 诊断码必须可映射为清晰消息
- Reset 行为较强，重新初始化后需重新加载配置
- 原有测试体系需要保持兼容

### 关键文件
- 接口定义：`include/motion_control.h`
- 核心执行：`src/motion_control.c`
- 公共状态与诊断：`include/common_types.h`
- 状态展示：`src/state_reporter.c`
- 保护/Ready 判定：`src/protection_manager.c`
- 诊断文本：`src/diagnostics.c`
- 回归验证：`tests/test_motion_control.c`

---

## 🚫 已尝试的方案（失败的方案及原因）

> 这里记录的是本轮开发中暴露出的问题与已修复的错误，而不是最终保留方案。

### 1. 仅按 recipe 索引校验活动段
- **问题**：执行态索引校验默认要求 `currentSegmentIndex < RECIPE_SIZE`
- **结果**：Direct 模式下出现异常，因为 direct 并不对应真实 recipe 索引
- **原因**：校验逻辑没有按活动段来源分支处理
- **结论**：必须基于 `_activeSegmentSource` 区分 recipe / direct

### 2. 段完成后直接套用 Idle 清理来源
- **问题**：`ApplyIdleState()` 清空或重置来源相关状态
- **结果**：`segmentSource` 在段完成后丢失
- **原因**：完成态回写顺序不正确
- **结论**：完成时应先缓存 `completedSegmentSource`，再进行状态落地

### 3. 测试断言强依赖 `STATUS == RUNNING`
- **问题**：运行过程中状态可能进入 `DEGRADED`
- **结果**：测试误报失败
- **原因**：测试假设过于严格，没有考虑设计允许的运行退化态
- **结论**：测试断言需放宽为符合运行语义的状态集合

### 4. 头文件修改过程中出现结构体字段重复
- **问题**：`motion_control.h` 一次替换后输出字段块被复制两遍
- **结果**：结构体定义异常
- **原因**：接口扩展时未及时清理重复成员
- **结论**：已清理，属于实现过程中的中间错误，不是最终设计问题

---

## ✅ 当前方案/最终方案（设计思路、核心机制、关键代码）

### 设计思路
核心策略是：**不改底层运动/压力算法，只在控制框架层扩展“段来源选择 + 运行时锁存”能力。**

### 核心机制

#### 1. 双来源共存
在 `HDY_MotionControlFB` 中同时保留：
- `RECIPE[]`
- `DIRECT_SEGMENT`
- `DIRECT_SEGMENT_VALID`
- `USE_RECIPE`

这样 FB 可同时持有两套配置，运行前按模式选择，不需要频繁清空另一套数据。

#### 2. 启动时按模式解析段来源
新增内部解析流程：
- `USE_RECIPE = true`：从 `RECIPE[segmentIndex]` 取段
- `USE_RECIPE = false`：从 `DIRECT_SEGMENT` 取段，`segmentIndex` 仅作统一接口参数保留

#### 3. 运行时来源锁存
在真正开始段执行时，将以下信息锁存：
- `_activeSegment`
- `_activeSegmentSource`

这样即使外部在运行过程中：
- 切换 `USE_RECIPE`
- 重写 `DIRECT_SEGMENT`
- 修改 recipe 内容

当前已启动动作仍保持原始执行语义，不被扰动。

#### 4. Direct 模式完成即 DONE
Direct 被定义为单段动作语义，因此：
- 完成后不进入 recipe 的“等待下一段”语义
- 直接落入 `DONE`

#### 5. Direct 模式拒绝 NEXT
由于 direct 没有“下一段”概念：
- `NextSegment()` 在 direct 模式下直接拒绝
- 设置明确诊断，避免上层误用

### 关键接口/字段
在 `include/common_types.h` 中新增：
```c
typedef enum {
    HDY_SEGMENT_SOURCE_NONE,
    HDY_SEGMENT_SOURCE_RECIPE,
    HDY_SEGMENT_SOURCE_DIRECT
} HDY_SegmentSource;
```

新增诊断码：
```c
HDY_DIAG_CODE_NO_DIRECT_SEGMENT
```

在状态中新增：
```c
HDY_MotionState.segmentSource
```

在 `include/motion_control.h` 中新增：
```c
HDY_BOOL USE_RECIPE;
HDY_MotionSegment DIRECT_SEGMENT;
HDY_BOOL DIRECT_SEGMENT_VALID;
```

新增 API：
```c
HDY_BOOL HDY_MotionControlFB_LoadDirectSegment(HDY_MotionControlFB* fb, const HDY_MotionSegment* segment);
void HDY_MotionControlFB_ClearDirectSegment(HDY_MotionControlFB* fb);
HDY_BOOL HDY_MotionControlFB_StartSegment(HDY_MotionControlFB* fb, size_t segmentIndex, HDY_TIME timestamp);
```

新增内部字段：
```c
_activeSegmentSource
```

---

## 📝 关键代码变更（修改的文件、变更内容、代码量）

> 以下代码量为**范围级描述**，用于新对话快速定位，不作为精确统计值。

### 1. `include/common_types.h`
**变更内容**：
- 新增 `HDY_SegmentSource` 枚举
- 新增 `HDY_DIAG_CODE_NO_DIRECT_SEGMENT`
- 在 `HDY_MotionState` 中增加 `segmentSource`

**影响**：
- 统一表达段来源
- 支撑 direct 诊断和状态对外暴露

### 2. `include/motion_control.h`
**变更内容**：
- 新增 `USE_RECIPE`
- 新增 `DIRECT_SEGMENT`
- 新增 `DIRECT_SEGMENT_VALID`
- 新增 direct 相关 API
- 新增内部字段 `_activeSegmentSource`
- 清理过一次结构体重复字段问题

**影响**：
- 完成对外接口层扩展

### 3. `src/motion_control.c`
**变更内容**：
- 新增/调整关键辅助逻辑：
  - `HDY_UsesRecipeSource()`
  - `HDY_HasSelectedStartSource()`
  - `HDY_ResetReadyContextPreview()`
  - `HDY_ResolveStartSourceSegment()`
  - `HDY_ValidateStartRequest()`
  - `HDY_ValidateNextRequest()`
  - `HDY_BeginSegment()`
  - `HDY_AdvanceToNextSegment()`
- 实现 `LoadDirectSegment()` / `ClearDirectSegment()`
- `StartSegment()` 合法状态扩展，支持从 `IDLE` 启动
- 执行期索引校验按 `_activeSegmentSource` 分支
- 段完成后正确保留 `segmentSource`

**影响**：
- 这是本轮改动的核心文件
- 改动规模较大

### 4. `src/state_reporter.c`
**变更内容**：
- 新增 `HDY_StateReporter_SetSegmentSource()`
- Idle/Ready 状态判定改为基于当前选中来源是否可启动

### 5. `src/protection_manager.c`
**变更内容**：
- Ready/Idle 判定不再只依赖 `RECIPE_SIZE`
- 规则改为：
  - recipe 模式看 `RECIPE_SIZE`
  - direct 模式看 `DIRECT_SEGMENT_VALID`

### 6. `src/diagnostics.c`
**变更内容**：
- 新增诊断映射：
  - `HDY_DIAG_CODE_NO_DIRECT_SEGMENT -> "No direct segment configured"`

### 7. `tests/test_motion_control.c`
**变更内容**：
新增或调整以下测试：
- `test_direct_mode_start_without_recipe_uses_direct_segment_buffer`
- `test_direct_mode_latches_segment_parameters_at_start`
- `test_recipe_and_direct_modes_can_coexist_and_switch`
- `test_direct_mode_requires_direct_segment_configuration`

**影响**：
- 提供 direct / recipe 共存回归保障

---

## 🎯 当前进度（已完成、进行中、待处理）

### 已完成
- [x] Direct / Recipe 并存接口设计落地
- [x] `USE_RECIPE` 驱动来源选择
- [x] direct buffer 装载与清空 API
- [x] 运行时活动段来源锁存
- [x] Direct 完成即 `DONE`
- [x] Direct 下 `NEXT` 拒绝与诊断
- [x] Ready/Idle 逻辑适配双模式
- [x] direct 相关回归测试补充
- [x] 全量构建与测试通过

### 进行中
- [ ] 无（本轮目标已完成）

### 待处理
- [ ] 根据后续路线图，继续推进 Direct/Recipe 共存接口之上的更高层能力
- [ ] 结合 `开发阶段与下一步计划.MD` 选择下一开发项
- [ ] 若未来需要，可继续完善上层状态呈现与使用示例文档

---

## 💡 使用方法（配置步骤、示例代码、注意事项）

### 使用方式 1：Recipe 模式
```c
fb.USE_RECIPE = HDY_TRUE;
HDY_MotionControlFB_LoadRecipe(&fb, recipe, recipeSize);
HDY_MotionControlFB_StartSegment(&fb, 0, timestamp);
```

### 使用方式 2：Direct 模式
```c
fb.USE_RECIPE = HDY_FALSE;
HDY_MotionControlFB_LoadDirectSegment(&fb, &segment);
HDY_MotionControlFB_StartSegment(&fb, 0, timestamp);
```

> 说明：Direct 模式下 `segmentIndex` 只是统一接口保留参数，不表示真正的 recipe 索引语义。

### 新对话承接建议
在新对话中可以直接提供本文件，并附带如下说明：

```text
请先阅读项目根目录下的“对话上下文-Direct与Recipe模式并存接口-已完成并通过测试.md”，基于该上下文继续开发，不要重复实现文档中已完成的内容。
```

### 注意事项
1. **不要重复实现** Direct/Recipe 并存接口，本轮已经完成。
2. 运行中的活动段来源已经锁存，后续修改外部配置不应影响当前动作。
3. Direct 模式没有“下一段”，不要在该模式下调用 `NextSegment()` 期待推进。
4. Reset 后如果配置被清空，需要重新加载 recipe 或 direct 段。
5. 新增功能时要继续遵守“过程层负责工艺推进，控制层负责计算”的架构边界。

---

## 🐛 已知问题和待解决

### 已知问题
- 当前没有明确新的阻塞性缺陷留存。
- 本轮发现并修复的问题包括：
  - 结构体重复字段
  - direct 模式索引校验错误
  - 完成后 `segmentSource` 丢失
  - 测试断言过严

### 待解决方向
- 后续尚需根据项目路线图继续推进更高层功能，但不属于本轮 direct/recipe 并存接口任务范围。
- 建议下一步优先查看：
  - `开发阶段与下一步计划.MD`
  - 已有上下文文档 `对话上下文-PLCopen多缸运动控制重构-方案评审完成.md`

---

## 🚀 下一步计划

建议下一轮对话按以下顺序推进：

1. **先读取本上下文文档**，避免重复扫描与重复修改。
2. **对照 `开发阶段与下一步计划.MD`**，确认当前未完成任务优先级。
3. 在 Direct/Recipe 并存接口稳定基础上，继续推进：
   - 更高层命令语义补全
   - 状态呈现/诊断细化
   - 文档与示例更新
   - 其他未完成重构项
4. 修改前优先确认是否会影响：
   - 运行时来源锁存
   - Direct 完成即 DONE 语义
   - Ready/Idle 双模式判定
   - Direct 下 NEXT 禁止规则

---

## 📝 备注（开发心得、经验教训、参考资料）

### 开发心得
- 这次重构的关键不是算法改写，而是**接口层语义澄清**。
- 当一个 FB 同时支持两类输入来源时，**启动时锁存**是最重要的稳定性措施之一。
- “Direct 不是特殊 recipe，而是不同语义模型”这一点必须在状态机与命令校验层明确体现。

### 经验教训
- 不能让 Ready/Idle、索引校验、完成态语义默认为 recipe 模型，否则 direct 支持会处处漏口。
- 状态/诊断/测试要同步更新，单点修改很容易留下隐式不一致。
- 测试断言应体现设计允许范围，不应把实现细节写死成唯一状态值。

### 参考资料
- `CODEBUDDY.md`
- `开发阶段与下一步计划.MD`
- `对话上下文-PLCopen多缸运动控制重构-方案评审完成.md`
- `项目需求与设计说明书.md`

---

## 给新对话的简短交接语

可以直接复制下面这段到新对话：

```text
请先阅读 /home/dan/project/hdy-motion-light/对话上下文-Direct与Recipe模式并存接口-已完成并通过测试.md。
当前已完成 Direct 与 Recipe 模式并存接口，且全量测试通过。请在此基础上继续未完成任务，不要重复实现已完成内容；如果要修改 motion_control 相关逻辑，请优先保持来源锁存、Direct 完成即 DONE、Direct 下 NEXT 禁止、双模式 Ready/Idle 判定这几条核心语义不被破坏。
```
