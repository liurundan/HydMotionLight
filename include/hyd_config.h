#ifndef HYD_CONFIG_H
#define HYD_CONFIG_H

/*
 * HDY统一平台配置头文件
 * 
 * 此文件定义了注塑机运动控制库的平台相关配置项，支持不同目标平台的
 * 资源裁剪和性能调优。通过修改这些宏定义，可以适配不同资源约束的
 * 嵌入式平台。
 * 
 * 使用说明：
 * 1. 在编译前，根据目标平台的资源情况调整配置项
 * 2. 对于资源受限的平台，可以适当减小各MAX值
 * 3. 对于高精度应用，可以启用额外的诊断和验证功能
 * 
 * 默认配置适用于大多数嵌入式平台（32位MCU，>64KB RAM，>256KB Flash）
 */

/* ============================================================================
 * 1. 精度配置
 * ============================================================================ */

/* 实数精度配置
 * HYD_REAL_PRECISION_FLOAT  - 使用float（32位，节省RAM，精度较低）
 * HYD_REAL_PRECISION_DOUBLE - 使用double（64位，精度较高，默认）
 */
#define HYD_REAL_PRECISION_FLOAT 1

/* 根据精度配置选择HDY_REAL类型 */
#if HYD_REAL_PRECISION_FLOAT
    #include <float.h>
    typedef float HYD_REAL;
    #define HYD_REAL_EPSILON FLT_EPSILON
#else
    #include <math.h>
    typedef double HYD_REAL;
    #define HYD_REAL_EPSILON DBL_EPSILON
#endif

/* ============================================================================
 * 2. 数组大小限制
 * ============================================================================ */

/* 最大配方段数
 * 默认设计: 16段
 * 当前嵌入式目标: 固定为1段
 * 原因: RAM限制要求将配方缓存压缩为单段，当前平台因此禁用多段配方工作流
 * 行为约束: 超过 HYD_MAX_SEGMENTS 的配方必须被拒绝，不能截断为单段继续执行
 * 影响: RAM占用约 sizeof(HYD_MotionSegment) * HYD_MAX_SEGMENTS 字节
 */
#define HYD_MAX_SEGMENTS 1

/* 段标签最大值
 * 默认: 255 (uint8_t full range)
 * 说明: segmentTag 使用 uint8_t，此配置保留用于向后兼容查询接口
 * 影响: 每个段从 char[16] 降到 1 字节，显著节省 RAM
 */
#define HYD_SEGMENT_TAG_MAX 255

/* 段名称最大长度（已弃用，保留用于配置查询接口兼容性）
 * 新代码应使用 HYD_SEGMENT_TAG_MAX 和 segmentTag 字段
 */
#define HYD_NAME_MAX 16

/* 诊断历史深度（已弃用）
 * 保留此宏仅为向后兼容查询接口 (HYD_ConfigInfo.maxHistoryDepth)。
 * 诊断历史已简化为只保留最近一条快照，不再使用环形缓冲区。
 * 新代码不应依赖此宏。
 */
#define HYD_DIAG_HISTORY_DEPTH 1

/* ============================================================================
 * 3. 功能开关
 * ============================================================================ */

/* 启用诊断历史记录
 * 默认: 启用
 * 裁剪后: 不保存历史快照，仅保留当前诊断
 */
#define HYD_ENABLE_DIAGNOSTIC_HISTORY 1

/* 启用诊断标志位
 * 默认: 启用
 * 裁剪后: 不维护紧凑的标志位集合
 */
#define HYD_ENABLE_DIAGNOSTIC_FLAGS 1

/* 启用压力环遥测
 * 默认: 启用
 * 裁剪后: 不提供压力控制器的内部状态详情
 */
#define HYD_ENABLE_PRESSURE_LOOP_TELEMETRY 1

/* 启用执行参考
 * 默认: 启用
 * 裁剪后: 不记录详细的执行参考值（压力/流量/速度参考）
 */
#define HYD_ENABLE_EXECUTION_REFERENCE 1

/* ============================================================================
 * 4. 验证与保护
 * ============================================================================ */

/* 启用配方验证
 * 默认: 启用
 * 裁剪后: 不进行配方完整性验证，提升加载速度但降低安全性
 */
#define HYD_ENABLE_RECIPE_VALIDATION 1

/* 启用运行时配置验证
 * 默认: 启用
 * 裁剪后: 每个周期不验证配置，节省CPU周期
 */
#define HYD_ENABLE_RUNTIME_CONFIG_VALIDATION 1

/* 启用传感器数据验证
 * 默认: 启用
 * 裁剪后: 不检查传感器数据的有效性和时间戳
 */
#define HYD_ENABLE_SENSOR_DATA_VALIDATION 1

/* 启用保护功能
 * 默认: 启用
 * 裁剪后: 不进行过压、低压、超时等保护检查
 */
#define HYD_ENABLE_PROTECTION 1

/* ============================================================================
 * 5. 性能优化
 * ============================================================================ */

/* 启用内联函数优化
 * 默认: 启用
 * 说明: 对高频调用的工具函数使用inline关键字
 */
#define HYD_ENABLE_INLINE_FUNCTIONS 1

/* 启用快速数学函数
 * 默认: 禁用（使用标准库函数以保证精度）
 * 启用后: 使用快速近似函数（如快速平方根），提升速度但降低精度
 */
#define HYD_ENABLE_FAST_MATH 0

/* ============================================================================
 * 6. 调试与测试
 * ============================================================================ */

/* 启用断言检查
 * 默认: 禁用（生产环境）
 * 启用后: 在关键路径插入assert检查，增加安全性但可能影响性能
 */
#define HYD_ENABLE_ASSERT 0

/* 启用统计计数器
 * 默认: 禁用
 * 启用后: 统计函数调用次数、分支预测命中率等，用于性能分析
 */
#define HYD_ENABLE_STATISTICS 0

/* 启用详细日志
 * 默认: 禁用
 * 启用后: 插入调试日志，大幅增加ROM和RAM占用
 */
#define HYD_ENABLE_DEBUG_LOG 0

/* ============================================================================
 * 7. 平台特定配置
 * ============================================================================ */

/* 目标平台类型（用于选择优化策略）
 * HYD_PLATFORM_GENERIC  - 通用平台，使用保守配置
 * HYD_PLATFORM_CORTEX_M - ARM Cortex-M系列MCU
 * HYD_PLATFORM_CORTEX_A - ARM Cortex-A系列处理器
 * HYD_PLATFORM_X86       - x86/x64架构
 */
#define HYD_PLATFORM_GENERIC 1
// #define HYD_PLATFORM_CORTEX_M 0
// #define HYD_PLATFORM_CORTEX_A 0
// #define HYD_PLATFORM_X86 0

/* 字节对齐配置
 * 根据平台选择合适的对齐方式，优化访问性能
 */
#if defined(HYD_PLATFORM_CORTEX_M) || defined(HYD_PLATFORM_CORTEX_A)
    #define HYD_ALIGNMENT 4
#elif defined(HYD_PLATFORM_X86)
    #define HYD_ALIGNMENT 8
#else
    #define HYD_ALIGNMENT 4
#endif

/* ============================================================================
 * 8. 时间配置
 * ============================================================================ */

/* 时间基类型
 * 使用HDY_REAL表示时间（秒），提供高精度
 */
typedef HYD_REAL HYD_TIME;

/* 时间精度
 * 定义时间比较的容差值，用于浮点时间比较
 */
#define HYD_TIME_EPSILON 1e-6  /* 1微秒 */

/* ============================================================================
 * 9. 资源统计
 * ============================================================================ */

/* 启用资源使用统计
 * 默认: 禁用
 * 启用后: 统计栈使用、堆使用、最大调用深度等
 */
#define HYD_ENABLE_RESOURCE_STATISTICS 0

/* 启用内存池
 * 默认: 禁用
 * 启用后: 使用静态内存池替代动态分配，增强确定性
 */
#define HYD_ENABLE_MEMORY_POOL 0

/* ============================================================================
 * 10. 编译器特定配置
 * ============================================================================ */

/* 强制内联（如果编译器支持） */
#if HYD_ENABLE_INLINE_FUNCTIONS
    #define HYD_FORCE_INLINE static inline __attribute__((always_inline))
#else
    #define HYD_FORCE_INLINE static
#endif

/* 可能未使用标记（避免编译器警告） */
#define HYD_UNUSED __attribute__((unused))

/* 弱别名（用于默认实现） */
#define HYD_WEAK __attribute__((weak))

/* ============================================================================
 * 11. 版本信息
 * ============================================================================ */

/* 库版本号 */
#define HYD_VERSION_MAJOR 1
#define HYD_VERSION_MINOR 1
#define HYD_VERSION_PATCH 0
#define HYD_VERSION_STRING "1.1.0"

/* Public state/PLC function-block layouts changed in 1.1.0. Consumers must
 * rebuild all objects against the headers shipped with the static library. */
#define HYD_ABI_VERSION 2

/* 构建时间戳 */
#ifdef HYD_VERSION_BUILD_TIME
    #undef HYD_VERSION_BUILD_TIME
#endif
#define HYD_VERSION_BUILD_TIME __DATE__ " " __TIME__

/* ============================================================================
 * 12. 配置验证
 * ============================================================================ */

/* 编译时验证配置的合理性 */
#if HYD_MAX_SEGMENTS < 1
    #error "HYD_MAX_SEGMENTS must be at least 1"
#endif

/* HYD_DIAG_HISTORY_DEPTH validation removed — depth is no longer used at
 * compile time.  The value 1 is kept for config-export compatibility only. */

/* ============================================================================
 * 13. 兼容性配置
 * ============================================================================ */

/* 向后兼容的宏定义（用于代码迁移） */
#ifndef HYD_REAL_PRECISION_FLOAT
    #define HYD_REAL_PRECISION_FLOAT 0
#endif

#ifndef HYD_REAL_PRECISION_DOUBLE
    #define HYD_REAL_PRECISION_DOUBLE 1
#endif

/* ============================================================================
 * 14. 内部宏定义
 * ============================================================================ */

/* 条件编译辅助宏 */
#if HYD_ENABLE_DIAGNOSTIC_FLAGS
    #define HYD_DIAG_FLAGS_VAR(type) type
#else
    #define HYD_DIAG_FLAGS_VAR(type) /* 空定义，不分配空间 */
#endif

/* 断言宏 */
#if HYD_ENABLE_ASSERT
    #include <assert.h>
    #define HYD_ASSERT(expr) assert(expr)
#else
    #define HYD_ASSERT(expr) ((void)(0))
#endif

/* ============================================================================
 * 14B. 内部数值阈值（Internal Numerical Thresholds）
 *
 * 本节集中了原先散落在 src/ 各 .c 文件中的"调参型"浮点常量——这些常量是
 * 工业整定参数，可能因平台、采样频率或工艺要求调整，集中后便于
 * 一处修改、整体生效。
 *
 * 注意：本节不包含"数学不变量"（如 0.5 二分、1.0 归一化、2.0 倍乘）；
 * 那些应当保留在源码上下文中以保证算法可读性。
 * ============================================================================ */

/* --- 五点式肘杆机构验证默认值（src/toggle_kinematics.c） --- */

#ifndef HYD_TOGGLE_RADICAND_TOLERANCE_FACTOR
    #define HYD_TOGGLE_RADICAND_TOLERANCE_FACTOR (32.0f * HYD_REAL_EPSILON)
#endif

#ifndef HYD_TOGGLE_MIN_NORMALIZED_MAIN_JACOBIAN
    #define HYD_TOGGLE_MIN_NORMALIZED_MAIN_JACOBIAN 0.02f
#endif

#ifndef HYD_TOGGLE_MIN_DRIVE_PROJECTION_RATIO
    #define HYD_TOGGLE_MIN_DRIVE_PROJECTION_RATIO 0.02f
#endif

#ifndef HYD_TOGGLE_MIN_ABS_VELOCITY_RATIO
    #define HYD_TOGGLE_MIN_ABS_VELOCITY_RATIO 0.01f
#endif

#ifndef HYD_TOGGLE_MAX_ABS_VELOCITY_RATIO
    #define HYD_TOGGLE_MAX_ABS_VELOCITY_RATIO 20.0f
#endif

#ifndef HYD_TOGGLE_GEOMETRY_MARGIN_MM
    #define HYD_TOGGLE_GEOMETRY_MARGIN_MM 0.5f
#endif

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

/* Stop-decel watchdog completion threshold for plannerOutput.targetVelocity.
 * 停车减速完成后，主循环再次判定 decelerate-only 段完成时使用的速度阈值。
 * 单位：mm/s（与 HYD_REAL targetVelocity 同维度）。
 * 原值：motion_control.c:1866 `0.001`，已 f-typed 与同组 _MAG 常量保持一致。 */
#define HYD_THRESH_DECEL_TARGET_VEL_DONE     0.001f

/* 停车 watchdog：ideal-stop time × N + slack(s) 后强制升级 FAULT.
 * 原值：motion_control.c:1839 `5.0f * idealStopTime + 1.0f`. */
#define HYD_THRESH_STOP_TIMEOUT_IDEAL_MULT   5.0f
#define HYD_THRESH_STOP_TIMEOUT_SLACK_S      1.0f

/* --- 压力 ceiling 诊断 debounce / startup-suppress（motion_control.c Init） --- */

/* PRESSURE_CEILING_EXCEEDED 触发前的 debounce 时长。短于一般 pressure
 * deviation debounce，因为 ceiling 违规已超出安全包络，必须更快反应。
 * 单位：秒（与 HYD_TIME 字段同维度）。
 * 原值：motion_control.c:2093 `0.05`，已 f-typed 与同组 _MAG 常量保持一致。 */
#define HYD_THRESH_PRESSURE_CEILING_DEBOUNCE_S        0.05f

/* 段启动后多久内抑制 PRESSURE_CEILING_EXCEEDED 误报（pressure 在压力建立
 * 阶段会有大幅瞬态）。
 * 单位：秒（与 HYD_TIME 字段同维度）。
 * 原值：motion_control.c:2094 `0.10`，已 f-typed 与同组 _MAG 常量保持一致。 */
#define HYD_THRESH_PRESSURE_CEILING_STARTUP_SUPPRESS_S 0.10f

/* PRESSURE_CEILING_EXCEEDED 持续多少秒后升级为 PRESSURE_CEILING_VIOLATED（FAULT）.
 * 单位：秒（与 HYD_TIME 字段同维度）。
 * 原值：motion_control.c:2096 `0.30`，已 f-typed 与同组 _MAG 常量保持一致。 */
#define HYD_THRESH_PRESSURE_CEILING_FAULT_ESCALATION_S 0.30f

/* --- 压力限制保护阈值（src/output_limiter.c） --- */

/* 压力限制比例增益：超压比例 overRatio 时的输出缩减系数。
 * scale = 1.0 - Kp * overRatio
 * 例：Kp=3.0, 超压10% → scale = 1.0 - 3.0*0.1 = 0.7（输出降至70%）
 * 取保守值避免振荡，实际整定时可适当增大。
 * 单位：无量纲。 */
#define HYD_THRESH_PRESSURE_LIMIT_KP             3.0f

/* 压力限制最小缩放比：比例限速的下限，不允许完全停泵。
 * 完全停泵由 FAULT/STOP 保护层负责。
 * 单位：无量纲 [0, 1)。 */
#define HYD_THRESH_PRESSURE_LIMIT_MIN_SCALE      0.1f

/* 压力限制 WARNING 触发前的 debounce 时长。
 * 取较宽值避免瞬态压力尖峰误报（如射胶段切换瞬间）。
 * 单位：秒。 */
#define HYD_THRESH_PRESSURE_LIMIT_DEBOUNCE_S     0.20f

/* 压力限制 WARNING → FAULT 升级时长。
 * 射胶段压力波动较大，取 1.0s 避免误停机。
 * 单位：秒。 */
#define HYD_THRESH_PRESSURE_LIMIT_FAULT_ESCALATION_S  1.0f

/* --- 软限位保护阈值（src/output_limiter.c） --- */

/* 软限位默认减速带宽度（当 cylinderConfig.softLimitBandMm 未设置时的参考值）。
 * 实际使用 cylinderConfig.softLimitBandMm 字段，此处仅作文档参考。
 * 单位：mm。 */
#define HYD_THRESH_SOFT_LIMIT_DEFAULT_BAND_MM    5.0f

/* 软限位 FAULT 触发前的 debounce 时长（position 到达极限后）。
 * 单位：秒。 */
#define HYD_THRESH_SOFT_LIMIT_FAULT_DEBOUNCE_S   0.50f

/* --- RBF-PID 数值死区与滤波（src/rbf_pid.c） --- */

/* 设定值归一化绝对值低于该阈值视为"零设定"，直接清零输出，避免
 * 控制器在小目标下放大噪声。
 * 单位：归一化（无量纲）。原值：rbf_pid.c:327,345 `1e-6f`. */
#define HYD_THRESH_RBF_SETPOINT_ZERO_EPS     1e-6f

/* 设定值零判定时的反馈死区——只有反馈也接近零才置 0 输出。
 * 单位：归一化（无量纲）。原值：rbf_pid.c:327,345 `0.02f`. */
#define HYD_THRESH_RBF_FEEDBACK_ZERO_BAND    0.02f

/* 前馈加速度补偿激活的最小压力二阶差分（绝对值）.
 * 单位：归一化压力的二阶差分（无量纲）。fddPress 为 fActPress（即
 * pid->Feedback，归一化压力）的二阶差分，量纲同为归一化（无量纲）。
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

#define HYD_MAX_AXIS_MOTION  20 /* 最大液压轴数量，包括合模、射胶、顶出、座台、中子等 */

#ifndef HYD_MAX_TOGGLE_MECHANISMS
#define HYD_MAX_TOGGLE_MECHANISMS HYD_MAX_AXIS_MOTION
#endif

#ifndef HYD_MAX_TOGGLE_VALIDATIONS
#define HYD_MAX_TOGGLE_VALIDATIONS 1
#endif

#ifndef HYD_ENABLE_MECHANISM_TELEMETRY
#define HYD_ENABLE_MECHANISM_TELEMETRY 1
#endif

/* ============================================================================
 * 14C. 泵负转速下限比例（Pump Negative Speed Ratio）
 *
 * 允许油泵小范围反转以快速卸压时，反转转速上限占 pumpSpeedLimit 的比例。
 * 例：pumpSpeedLimit=1500rpm, RATIO=0.05 → 最大反转转速 = -75rpm
 *
 * 使用此宏的模块：
 *   - output_limiter.c  HYD_OutputLimiter_GetLimits()
 *   - output_limiter.c  HYD_OutputLimiter_Execute() 硬裁剪
 *   - output_limiter.c  HYD_OutputLimiter_ExecuteWithProtection() 硬裁剪
 *   - pump_converter.c  HYD_PumpConverter_Execute()
 *   - motion_interface.c __mcl_cmd_GetPumpRequest()（间接，依赖上游保证）
 * ============================================================================ */
#define HYD_PUMP_NEGATIVE_SPEED_RATIO      0.05f

/* ============================================================================
 * 15. 配置导出接口
 * ============================================================================ */

/* 获取配置信息的接口（用于运行时查询） */
typedef struct {
    int maxSegments;
    int maxSegmentTagValue;
    int maxNameLength;       /* Legacy: kept for backward compatibility, same as maxSegmentTagValue */
    int maxHistoryDepth;       /* Always 1 — history now retains only the latest snapshot */
    int diagnosticHistoryEnabled;
    int pressureLoopTelemetryEnabled;
    int executionReferenceEnabled;
    const char* versionString;
    const char* buildTime;
} HYD_ConfigInfo;

/* 获取配置信息（实现位于motion_utils.c） */
HYD_ConfigInfo HYD_GetConfigInfo(void);

#endif /* HYD_CONFIG_H */
