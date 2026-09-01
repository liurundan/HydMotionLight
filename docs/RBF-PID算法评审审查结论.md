# RBF-PID 算法评审报告审查结论

## 审查日期
2026-09-01

## 审查范围
基于以下文件进行审查：
- `docs/RBF-PID算法评审报告.md`
- `src/rbf_pid.c`
- `src/pressure_controller.c`
- `include/rbf_pid.h`
- `tests/rbf_pid_test.c`

---

## 1. 评审报告第3章"算法错误与缺陷清单"审查结论

### P0级问题（正确性/安全性）审查结果

| 问题编号 | 评审报告结论 | 审查验证结果 | 严重程度确认 | 备注 |
|---------|------------|------------|------------|------|
| **P0-1** | PID模式无NaN/Inf防护 | **✓ 正确** | **P0** | 代码证据：`rbf_pid_sanitize_runtime_state()`（:616）和`rbf_pid_sanitize_network()`（:291）仅在PI模式下通过`rbf_pid_enforce_control_mode()`调用；PID模式主链路确实无防护 |
| **P0-2** | PID模式权重无界更新 | **✓ 正确** | **P0** | 代码证据：:331行`rbf_pid_same_direction_saturation()`检查仅在PI模式生效；PID模式下权重更新无`WEIGHT_LIMIT`钳位 |
| **P0-3** | 初始KP=0.04低于min_KP=0.4 | **✓ 正确** | **P0** | 代码证据：:133行`rbf_pid_apply_default_gains()`设置`KP=PID_MIN_KP`（0.4），但实际是`:133: pid->KP = PID_MIN_KP` 实际赋值0.4，报告描述有误——**报告错误**，初始化直接用`PID_MIN_KP`，不存在0.04的问题 |
| **P0-4** | 压力加速度前馈开关失效 | **部分正确，已修复** | **P0→已解决** | 原代码`:510-518`确实连续赋值导致开关失效；**已在本次修复中添加近目标抑制逻辑** |
| **P0-5** | w/c/b更新顺序错误 | **✓ 正确** | **P0→已修复** | 代码证据：原:336-368行先更新`w[i]`（:346），再用已更新的`w[i]`计算`delta_center`和`delta_b`，违反同步梯度下降假设。**已在本次修复中调整为正确顺序** |

### P0-3问题勘误
经代码验证，`rbf_pid_apply_default_gains()`（:132-136）直接使用`PID_MIN_KP`（0.4f），**不存在初始化为0.04的问题**。评审报告此处分析有误，可能是早期版本遗留描述。

---

## 2. 已修复问题清单

### 2.1 P0-5: w/c/b更新顺序修复

**修复位置**: `src/rbf_pid.c:336-368`

**原问题**:
```c
// 错误顺序：先更新w，再用新w计算c和b的增量
pid->w[i] = clamp_finite(..., pid->w[i] + delta_w, ...);  // 先更新
for (j...) {
    delta_center = ... * w_old * ...;  // w_old已被污染
}
```

**修复方案**:
```c
// 正确顺序：
// Step 1: 计算norm_val（使用旧c）
// Step 2: 更新c（使用w_old）
// Step 3: 更新b（使用旧c计算的norm_val和w_old）
// Step 4: 最后更新w
```

**验证**: 所有测试通过，包括网络学习测试。

---

### 2.2 P0-4: 压力加速度前馈近目标抑制修复

**修复位置**: `src/rbf_pid.c:510-518`

**原问题**:
```c
float f_uff = 0.0f;
if (pid->pressure_accel_ff_enabled) {
    f_uff = -0.15f * f_dd_press;  // 无近目标抑制
}
```

**修复方案**:
```c
float f_uff = 0.0f;
if (pid->pressure_accel_ff_enabled) {
    float near_target_threshold = pid->P_set * RBF_PID_NEAR_TARGET_RATIO;
    float pressure_error = fabsf(pid->P_set - actual_press);
    
    if (pressure_error >= near_target_threshold) {  // 仅远离目标时生效
        f_uff = -0.15f * f_dd_press;
    }
}
```

**验证**: 
- `test_pressure_accel_feedforward_is_suppressed_inside_near_target_band()` 通过
- `test_pressure_accel_feedforward_remains_active_outside_near_target_band()` 通过

---

## 3. 待修复P0问题优化方案

### 3.1 P0-1: PID模式无NaN/Inf防护

**问题根源**: `rbf_pid_enforce_control_mode()`（:272-280）仅在PI模式下调用`rbf_pid_sanitize_runtime_state()`和`rbf_pid_sanitize_network()`。

**优化方案A（推荐）**: 统一防护，模式无关
```c
float RBF_PID_Update(RBF_PID_Handle *pid, float setpoint, float feedback) {
    pid->P_set = isfinite(setpoint) ? setpoint : 0.0f;
    pid->P_actual = isfinite(feedback) ? feedback : 0.0f;
    
    // 统一清洗，模式无关
    rbf_pid_sanitize_runtime_state(pid);  // 移到enforce之前
    rbf_pid_sanitize_network(pid);        // 移到enforce之前
    
    rbf_pid_enforce_control_mode(pid);
    // ... 后续逻辑
}
```

**优化方案B（最小改动）**: 在PID模式分支补充防护
```c
static void rbf_pid_enforce_control_mode(RBF_PID_Handle *pid) {
    if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
        pid->KD = 0.0f;
        pid->eta_d = 0.0f;
        pid->pressure_accel_ff_enabled = false;
        rbf_pid_sanitize_runtime_state(pid);
        rbf_pid_sanitize_network(pid);
    } else {  // PID模式
        rbf_pid_sanitize_runtime_state(pid);  // 新增
        rbf_pid_sanitize_network(pid);        // 新增
    }
}
```

**推荐**: 方案A，数值防护应是模式无关的底层保障。

---

### 3.2 P0-2: PID模式权重无界更新

**问题根源**: :331行`rbf_pid_same_direction_saturation()`条件检查仅PI模式生效，导致PID模式下权重更新无饱和抑制。

**优化方案A（推荐）**: 权重钳位模式无关
```c
// 在rbf_pid_step_rbf_nn()的权重更新循环中（当前:336-368）
for (i = 0; i < RBF_HNUM; ++i) {
    float delta_w = pid->eta_w * error_rbf_n * h[i]
            + pid->alpha * (pid->w[i] - pid->w_1[i]);
    
    // 统一钳位，移除模式判断
    pid->w[i] = clamp_finite(-RBF_PID_WEIGHT_LIMIT,
            pid->w[i] + delta_w, RBF_PID_WEIGHT_LIMIT, pid->w[i]);
    
    // ... c/b更新
}
```

**优化方案B**: 保留模式差异，PID模式单独钳位
```c
// 在权重更新后补充PID模式钳位
if (pid->control_mode == RBF_PID_CONTROL_MODE_PID) {
    for (i = 0; i < RBF_HNUM; ++i) {
        pid->w[i] = clampf(-RBF_PID_WEIGHT_LIMIT, pid->w[i], RBF_PID_WEIGHT_LIMIT);
    }
}
```

**推荐**: 方案A，权重物理约束不应依赖控制模式。

---

## 4. P1级问题优先级建议

评审报告P1问题（#6-#14）分析基本正确，按工程影响排序：

### 高优先级（影响控制品质）
1. **#8: 压力反馈无滤波** - 高频噪声直接进入D项，泵速抖动
2. **#6: 增量控制律未按采样周期归一** - 与经典分支口径不一致，环频变化时增益漂移
3. **#12: 负流量死区不一致** - RBF/经典分支行为不同，切换时扰动

### 中优先级（影响自适应效果）
4. **#11: 学习冻结阈值过大** - 10bar误差内冻结学习，保压精调段几乎不学习
5. **#14: 策略切换全量复位** - 学习成果不跨段保留，削弱在线自适应价值
6. **#13: RBF分支不叠加前馈** - 策略切换时输出基线不同

### 低优先级（工程规范）
7. **#7: du钳位硬编码** - 应按`fMaxFlow`百分比缩放
8. **#9: pressureDeadband未生效** - 配置字段无效
9. **#10: 软流量限幅用稳态增益** - 瞬态建压时可能约束过紧

---

## 5. P2级问题（可维护性）审查

报告#15-#18分析正确，属于代码质量范畴，不影响核心功能：
- **#15**: 死代码（`control_state`、`ff_flow`、`dde`等）
- **#16**: 魔法数字缺配置入口（0.6626、0.0002、-0.15等）
- **#17**: 网络初始化与工况脱节（初始权重和≈0，du维度激励弱）
- **#18**: `sign()` EPS不一致

建议在P0/P1修复完成后统一清理。

---

## 6. 总体评审结论

### 6.1 评审报告质量评价
**评审报告整体质量：优秀**

**优点**:
- 问题定位精准，提供具体行号和代码证据
- 分层分级清晰（P0/P1/P2），便于优先级决策
- 注塑机场景适用性分析务实（第5章）
- 修复建议路线合理（第6章）

**瑕疵**:
- P0-3问题（初始KP=0.04）描述有误，实际初始化直接用`PID_MIN_KP`（0.4）
- 部分建议（如#10软流量限幅）需要更多现场数据支撑

### 6.2 算法状态评估
**当前状态**: 
- 经典PID/PI分支：**生产可用**（防护完善，口径规范）
- RBF-PID分支：**实验阶段**（P0问题待修复后才可灰度）

**修复进度**:
- ✅ P0-5: w/c/b更新顺序（已修复）
- ✅ P0-4: 压力加速度前馈近目标抑制（已修复）
- ⏳ P0-1: PID模式NaN/Inf防护（待修复，已提供方案）
- ⏳ P0-2: PID模式权重无界更新（待修复，已提供方案）
- ❌ P0-3: 初始KP问题（报告误判，实际不存在）

---

## 7. 后续行动建议

### 立即行动（P0修复）
1. 应用P0-1修复方案A（统一数值防护）
2. 应用P0-2修复方案A（统一权重钳位）
3. 补充NaN输入回归测试用例
4. 补充权重发散测试用例

### 短期优化（P1高优先级）
1. 接入压力滤波（`filterAlpha`配置）
2. 统一负流量死区阈值（RBF/经典分支）
3. 增量控制律按`sampling_period`归一
4. 调整学习冻结阈值（10bar→更小值，如2-3bar）

### 中期改进（架构）
1. 网络参数持久化机制（跨段保留学习成果）
2. 离线辨识工具（记录生产数据→离线整定→写回配方）
3. 死代码清理（`control_state`、`ff_flow`等）
4. 魔法数字配置化

### 策略定位
- **生产默认**: 经典PID/PI + 设定值斜坡 + 前馈
- **RBF-PID定位**: 保压/背压段可选实验策略，需增加参数日志追溯
- **远期方向**: 离线自适应（保留自适应收益 + 注塑工艺可重复性）

---

## 8. 测试验证状态

### 当前测试覆盖
- ✅ 基本初始化和复位
- ✅ PI/PID模式切换
- ✅ 网络学习与饱和抑制
- ✅ 流量归一化和系统增益软限幅
- ✅ 压力加速度前馈（含近目标抑制）
- ✅ RBF输入因果性和分离归一化
- ✅ 负流量输出能力

### 待补充测试
- ⏳ NaN/Inf输入防护测试
- ⏳ PID模式权重发散测试
- ⏳ 模式切换边界条件测试
- ⏳ 采样周期抖动场景测试

---

**审查人**: Claude (RBF-PID算法专家)  
**审查日期**: 2026-09-01  
**文档版本**: v1.0
