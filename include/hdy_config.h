#ifndef HDY_CONFIG_H
#define HDY_CONFIG_H

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
 * HDY_REAL_PRECISION_FLOAT  - 使用float（32位，节省RAM，精度较低）
 * HDY_REAL_PRECISION_DOUBLE - 使用double（64位，精度较高，默认）
 */
#define HDY_REAL_PRECISION_DOUBLE 1

/* 根据精度配置选择HDY_REAL类型 */
#if HDY_REAL_PRECISION_FLOAT
    #include <float.h>
    typedef float HDY_REAL;
    #define HDY_REAL_EPSILON FLT_EPSILON
#else
    #include <math.h>
    typedef double HDY_REAL;
    #define HDY_REAL_EPSILON DBL_EPSILON
#endif

/* ============================================================================
 * 2. 数组大小限制
 * ============================================================================ */

/* 最大配方段数
 * 默认: 16段
 * 最小: 4段（至少支持基本的注塑周期）
 * 影响: RAM占用约 sizeof(HDY_MotionSegment) * HDY_MAX_SEGMENTS 字节
 */
#define HDY_MAX_SEGMENTS 16

/* 段名称最大长度
 * 默认: 32字符
 * 最小: 16字符
 * 影响: RAM和Flash占用
 */
#define HDY_NAME_MAX 32

/* 诊断消息最大长度
 * 默认: 64字符
 * 最小: 32字符（可裁剪为0以节省ROM，但将失去诊断消息）
 * 影响: ROM占用约 HDY_MESSAGE_MAX * N 消息字符串
 */
#define HDY_MESSAGE_MAX 64

/* 诊断历史深度
 * 默认: 4条记录
 * 最小: 1条
 * 影响: RAM占用约 sizeof(HDY_DiagnosticSnapshot) * HDY_DIAG_HISTORY_DEPTH 字节
 */
#define HDY_DIAG_HISTORY_DEPTH 4

/* ============================================================================
 * 3. 功能开关
 * ============================================================================ */

/* 启用诊断消息字符串
 * 默认: 启用
 * 裁剪后: 仅保留诊断代码，不生成可读消息，可节省大量ROM
 */
#define HDY_ENABLE_DIAGNOSTIC_MESSAGE 1

/* 启用诊断历史记录
 * 默认: 启用
 * 裁剪后: 不保存历史快照，仅保留当前诊断
 */
#define HDY_ENABLE_DIAGNOSTIC_HISTORY 1

/* 启用诊断标志位
 * 默认: 启用
 * 裁剪后: 不维护紧凑的标志位集合
 */
#define HDY_ENABLE_DIAGNOSTIC_FLAGS 1

/* 启用压力环遥测
 * 默认: 启用
 * 裁剪后: 不提供压力控制器的内部状态详情
 */
#define HDY_ENABLE_PRESSURE_LOOP_TELEMETRY 1

/* 启用执行参考
 * 默认: 启用
 * 裁剪后: 不记录详细的执行参考值（压力/流量/速度参考）
 */
#define HDY_ENABLE_EXECUTION_REFERENCE 1

/* ============================================================================
 * 4. 验证与保护
 * ============================================================================ */

/* 启用配方验证
 * 默认: 启用
 * 裁剪后: 不进行配方完整性验证，提升加载速度但降低安全性
 */
#define HDY_ENABLE_RECIPE_VALIDATION 1

/* 启用运行时配置验证
 * 默认: 启用
 * 裁剪后: 每个周期不验证配置，节省CPU周期
 */
#define HDY_ENABLE_RUNTIME_CONFIG_VALIDATION 1

/* 启用传感器数据验证
 * 默认: 启用
 * 裁剪后: 不检查传感器数据的有效性和时间戳
 */
#define HDY_ENABLE_SENSOR_DATA_VALIDATION 1

/* 启用保护功能
 * 默认: 启用
 * 裁剪后: 不进行过压、低压、超时等保护检查
 */
#define HDY_ENABLE_PROTECTION 1

/* ============================================================================
 * 5. 性能优化
 * ============================================================================ */

/* 启用内联函数优化
 * 默认: 启用
 * 说明: 对高频调用的工具函数使用inline关键字
 */
#define HDY_ENABLE_INLINE_FUNCTIONS 1

/* 启用快速数学函数
 * 默认: 禁用（使用标准库函数以保证精度）
 * 启用后: 使用快速近似函数（如快速平方根），提升速度但降低精度
 */
#define HDY_ENABLE_FAST_MATH 0

/* ============================================================================
 * 6. 调试与测试
 * ============================================================================ */

/* 启用断言检查
 * 默认: 禁用（生产环境）
 * 启用后: 在关键路径插入assert检查，增加安全性但可能影响性能
 */
#define HDY_ENABLE_ASSERT 0

/* 启用统计计数器
 * 默认: 禁用
 * 启用后: 统计函数调用次数、分支预测命中率等，用于性能分析
 */
#define HDY_ENABLE_STATISTICS 0

/* 启用详细日志
 * 默认: 禁用
 * 启用后: 插入调试日志，大幅增加ROM和RAM占用
 */
#define HDY_ENABLE_DEBUG_LOG 0

/* ============================================================================
 * 7. 平台特定配置
 * ============================================================================ */

/* 目标平台类型（用于选择优化策略）
 * HDY_PLATFORM_GENERIC  - 通用平台，使用保守配置
 * HDY_PLATFORM_CORTEX_M - ARM Cortex-M系列MCU
 * HDY_PLATFORM_CORTEX_A - ARM Cortex-A系列处理器
 * HDY_PLATFORM_X86       - x86/x64架构
 */
#define HDY_PLATFORM_GENERIC 1
// #define HDY_PLATFORM_CORTEX_M 0
// #define HDY_PLATFORM_CORTEX_A 0
// #define HDY_PLATFORM_X86 0

/* 字节对齐配置
 * 根据平台选择合适的对齐方式，优化访问性能
 */
#if defined(HDY_PLATFORM_CORTEX_M) || defined(HDY_PLATFORM_CORTEX_A)
    #define HDY_ALIGNMENT 4
#elif defined(HDY_PLATFORM_X86)
    #define HDY_ALIGNMENT 8
#else
    #define HDY_ALIGNMENT 4
#endif

/* ============================================================================
 * 8. 时间配置
 * ============================================================================ */

/* 时间基类型
 * 使用double表示时间（秒），提供高精度
 */
typedef double HDY_TIME;

/* 时间精度
 * 定义时间比较的容差值，用于浮点时间比较
 */
#define HDY_TIME_EPSILON 1e-6  /* 1微秒 */

/* ============================================================================
 * 9. 资源统计
 * ============================================================================ */

/* 启用资源使用统计
 * 默认: 禁用
 * 启用后: 统计栈使用、堆使用、最大调用深度等
 */
#define HDY_ENABLE_RESOURCE_STATISTICS 0

/* 启用内存池
 * 默认: 禁用
 * 启用后: 使用静态内存池替代动态分配，增强确定性
 */
#define HDY_ENABLE_MEMORY_POOL 0

/* ============================================================================
 * 10. 编译器特定配置
 * ============================================================================ */

/* 强制内联（如果编译器支持） */
#if HDY_ENABLE_INLINE_FUNCTIONS
    #define HDY_FORCE_INLINE static inline __attribute__((always_inline))
#else
    #define HDY_FORCE_INLINE static
#endif

/* 可能未使用标记（避免编译器警告） */
#define HDY_UNUSED __attribute__((unused))

/* 弱别名（用于默认实现） */
#define HDY_WEAK __attribute__((weak))

/* ============================================================================
 * 11. 版本信息
 * ============================================================================ */

/* 库版本号 */
#define HDY_VERSION_MAJOR 1
#define HDY_VERSION_MINOR 0
#define HDY_VERSION_PATCH 0
#define HDY_VERSION_STRING "1.0.0"

/* 构建时间戳 */
#ifdef HDY_VERSION_BUILD_TIME
    #undef HDY_VERSION_BUILD_TIME
#endif
#define HDY_VERSION_BUILD_TIME __DATE__ " " __TIME__

/* ============================================================================
 * 12. 配置验证
 * ============================================================================ */

/* 编译时验证配置的合理性 */
#if HDY_MAX_SEGMENTS < 4
    #error "HDY_MAX_SEGMENTS must be at least 4"
#endif

#if HDY_NAME_MAX < 16
    #error "HDY_NAME_MAX must be at least 16"
#endif

#if HDY_DIAG_HISTORY_DEPTH < 1
    #error "HDY_DIAG_HISTORY_DEPTH must be at least 1"
#endif

#if HDY_MESSAGE_MAX < 32 && HDY_ENABLE_DIAGNOSTIC_MESSAGE
    #warning "HDY_MESSAGE_MAX is very small, diagnostic messages may be truncated"
#endif

/* ============================================================================
 * 13. 兼容性配置
 * ============================================================================ */

/* 向后兼容的宏定义（用于代码迁移） */
#ifndef HDY_REAL_PRECISION_FLOAT
    #define HDY_REAL_PRECISION_FLOAT 0
#endif

#ifndef HDY_REAL_PRECISION_DOUBLE
    #define HDY_REAL_PRECISION_DOUBLE 1
#endif

/* ============================================================================
 * 14. 内部宏定义
 * ============================================================================ */

/* 条件编译辅助宏 */
#if HDY_ENABLE_DIAGNOSTIC_MESSAGE
    #define HDY_DIAG_MSG(msg) msg
#else
    #define HDY_DIAG_MSG(msg) ""
#endif

#if HDY_ENABLE_DIAGNOSTIC_FLAGS
    #define HDY_DIAG_FLAGS_VAR(type) type
#else
    #define HDY_DIAG_FLAGS_VAR(type) /* 空定义，不分配空间 */
#endif

/* 断言宏 */
#if HDY_ENABLE_ASSERT
    #include <assert.h>
    #define HDY_ASSERT(expr) assert(expr)
#else
    #define HDY_ASSERT(expr) ((void)(0))
#endif

/* ============================================================================
 * 15. 配置导出接口
 * ============================================================================ */

/* 获取配置信息的接口（用于运行时查询） */
typedef struct {
    int maxSegments;
    int maxNameLength;
    int maxMessageLength;
    int maxHistoryDepth;
    int diagnosticMessageEnabled;
    int diagnosticHistoryEnabled;
    int pressureLoopTelemetryEnabled;
    int executionReferenceEnabled;
    const char* versionString;
    const char* buildTime;
} HDY_ConfigInfo;

/* 获取配置信息（实现位于motion_utils.c） */
HDY_ConfigInfo HDY_GetConfigInfo(void);

#endif /* HDY_CONFIG_H */
