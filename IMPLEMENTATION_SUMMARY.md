# 实施总结：修复模式切换时油泵转速尖峰问题

## 修改日期
2026-09-09

## 问题描述

在液压注塑机控制系统的模式切换过程中（特别是V/P切换和P/V切换），伺服油泵转速会瞬间降至0 RPM，然后再从0加速，导致：
- 油泵转速出现尖峰和不连续
- 机械振动
- 机械部件使用寿命降低
- 压力曲线不一致，影响制品质量

## 根本原因

**文件:** `src/motion_control.c`  
**函数:** `HYD_PrimeSegmentControllers` (1419-1560行)

在段切换期间无条件清除运动规划器状态，但只为P→V和V→V转换实现了无扰切换（状态继承）。V→P（速度→压力）和Position→P转换没有继承路径，导致规划器从0速度/流量重新开始。

### 关键代码缺陷

**原代码逻辑 (1464-1492行):**
```c
if (allowFlowCarryover && segment->mode == HYD_MODE_SPEED_RAMP) {
    // 仅处理进入速度模式的情况
    if (fb->_previousSegmentMode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
        // P→V: 有继承 ✅
    } else if (fb->_previousSegmentMode == HYD_MODE_SPEED_RAMP) {
        // V→V: 有继承 ✅
    }
}
// V→P 和 Position→P: 无继承 ❌

memset(&fb->_plannerState, 0, sizeof(fb->_plannerState));  // 总是清零
```

**问题:** 当 `segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP`（进入压力模式）时，整个继承块被跳过，因为条件检查的是 `SPEED_RAMP` 模式。

## 实施的解决方案

### 修改1: 扩展继承逻辑以处理V→P和Position→P转换

**位置:** `src/motion_control.c:1449-1533`

**新增代码:**

```c
} else if (allowFlowCarryover && segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
    /* Sprint 3: V->P 和 Position->P 继承以防止油泵转速尖峰 */
    if (fb->_previousSegmentMode == HYD_MODE_SPEED_RAMP ||
        fb->_previousSegmentMode == HYD_MODE_POSITION) {
        /* 保留运动规划状态中的流量 */
        carriedFlow = fb->_plannerState.lastTargetFlow;
        if (carriedFlow <= 0.0) {
            /* 回退：使用最后命令的油泵流量 */
            carriedFlow = fb->_lastCommandedFlow;
        }
        if (carriedFlow > 0.0) {
            doCarryover = true;
        }
    } else if (fb->_previousSegmentMode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
        /* P->P: 保留压力控制器的最后流量 */
        carriedFlow = fb->_lastCommandedFlow;
        if (carriedFlow > 0.0) {
            doCarryover = true;
        }
    }
}
```

### 修改2: 使用继承流量初始化压力控制器跟踪参考

**位置:** `src/motion_control.c:1522-1532`

**新增代码:**

```c
/* Sprint 3: 使用继承流量为压力模式设置trackingFlowReference。
 * 这确保压力控制器初始化反映V->P或Position->P转换前的实际运动
 * 状态，防止模式切换期间油泵转速降至0 RPM。 */
trackingFlowReference = HYD_MotionUtils_AbsReal(fb->AXIS_REF.flow);
if (trackingFlowReference <= 0.0 && allowFlowCarryover) {
    trackingFlowReference = fb->_lastCommandedFlow;
}
if (doCarryover && segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP && carriedFlow > 0.0) {
    trackingFlowReference = carriedFlow;
}
```

### 修改3: 添加Position→Speed继承（可选增强）

**位置:** `src/motion_control.c:1484-1491`

**新增代码:**

```c
} else if (fb->_previousSegmentMode == HYD_MODE_POSITION) {
    /* Position->Speed: 保留任何活动的运动状态 */
    if (fabs(fb->_plannerState.lastTargetVelocity) > 0.0) {
        carriedVelocity = fabs(fb->_plannerState.lastTargetVelocity);
        carriedFlow = fb->_plannerState.lastTargetFlow;
        doCarryover = true;
    }
}
```

## 现在支持的转换路径

| 转换 | 修改前 | 修改后 | 说明 |
|------|--------|--------|------|
| P→V (压力→速度) | ✅ 有继承 | ✅ 有继承 | 已存在，保持不变 |
| V→V (速度→速度) | ✅ 有继承 | ✅ 有继承 | 已存在，保持不变 |
| **V→P (速度→压力)** | ❌ 无继承 | ✅ **新增继承** | **主要修复** |
| **Position→P (位置→压力)** | ❌ 无继承 | ✅ **新增继承** | **主要修复** |
| **Position→Speed** | ❌ 无继承 | ✅ **新增继承** | **可选增强** |
| **P→P (压力→压力)** | ❌ 无继承 | ✅ **新增继承** | **可选增强** |

## 预期效果

### 修改前:
- 在V→P转换时油泵降至0 RPM
- 瞬间流量中断（0 L/min）
- 模腔压力尖峰/下冲
- 制品质量问题（飞边、气孔、缩痕）
- 不必要的循环时间增加
- 机械冲击导致振动

### 修改后:
- 通过所有转换的平滑油泵转速连续性
- 压力控制器从实际运动状态初始化
- 一致的保压压力曲线
- 泵/阀无机械冲击
- 更好的制品质量重复性
- 延长机械部件寿命

## 技术细节

### 内存影响
- 无新增状态变量
- 仅重用现有的 `carriedFlow`/`carriedVelocity` 局部变量
- 纯栈分配，无堆内存

### 兼容性
- C99纯实现
- 符合PLCopen标准
- 向后兼容：当 `allowFlowCarryover=false` 时默认行为不变
- 不影响FB接口

### 安全性
- 压力控制器仍具有输出限制和抗积分饱和
- 流量限制在段的 `maxFlow` 和泵限制内
- 不绕过任何现有的安全机制

## 验证建议

### 1. 单元测试
在 `tests/test_motion_control.c` 中创建测试用例：
- 测试V→P转换时活动运动的继承
- 测试P→V转换时活动压力的继承
- 测试Position→P转换
- 验证P→P段间转换

### 2. 集成测试（仿真器）
使用 `tests/main.c` 或 `test_hydro_sim_fb`：
- 运行完整注射循环（填充→保压→冷却）
- 记录油泵转速曲线
- 验证转换期间无0 RPM命令
- 检查最大油泵转速偏差 < 10%

### 3. 实机测试
部署到测试控制器：
- 监控油泵转速（示波器或PLC趋势）
- 比较修改前后的转换平滑度
- 测量模腔压力曲线一致性
- 评估机械振动（如可用加速度计）

## 文件修改列表

1. **src/motion_control.c** (主要更改)
   - 函数: `HYD_PrimeSegmentControllers` (1419-1560行)
   - 添加V→P和Position→P继承逻辑
   - 使用继承流量初始化压力控制器

## 编译和部署

```bash
# 重新配置（如果添加了新的.c文件，本次无需）
cmake --preset unixgcc

# 构建
cmake --build --preset unixgcc

# 运行测试
ctest --test-dir out/build/unixgcc --output-on-failure

# 嵌入式生产构建（排除仿真器）
./scripts/deploy_embedded_prod.sh
```

## 作者
- 实施者：Claude (Anthropic)
- 审阅者：待定
- 测试者：待定

## 参考
- 原始问题报告：现场观察到的V/P和P/V切换时油泵转速尖峰
- 相关文档：CLAUDE.md - 架构和设计边界
- Sprint标签：Sprint 3 - 无扰转换增强
