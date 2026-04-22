#ifndef HYDRO_HARDWARE_H
#define HYDRO_HARDWARE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================================================================
 * L1: 物理实体层 (Hardware Models)
 * 本层仅包含纯数据结构（属性与状态），没有任何业务控制逻辑
 * Sprint 1 初版在保持兼容的前提下补充阀角色与泵授予字段
 * ================================================================== */

/**
 * @brief 液压阀类型枚举
 */
typedef enum {
    VALVE_TYPE_DIR = 0,   // 方向控制阀（换向阀）
    VALVE_TYPE_PRESS,     // 压力控制阀
    VALVE_TYPE_PRECHARGE, // 充液阀/预塑阀
    VALVE_TYPE_RELIEF     // 卸压阀
} ValveType;

/**
 * @brief 阀在当前轴内的功能角色
 * @note 用于替代运行时字符串匹配
 */
typedef enum {
    VALVE_ROLE_NONE = 0,
    VALVE_ROLE_FWD,
    VALVE_ROLE_BWD,
    VALVE_ROLE_LOCK,
    VALVE_ROLE_RELIEF,
    VALVE_ROLE_PRECHARGE
} ValveRole;

/**
 * @brief 电磁阀实体模型
 */
typedef struct {
    const char* name;       // 阀名称（用于调试）
    ValveType type;         // 阀类型
    ValveRole role;         // 阀角色（Sprint 1 新增）
    bool safe_default;      // 断电时的安全默认态（false=断电常闭, true=断电常开）
    bool cmd_state;         // 软件下发的输出指令态
    bool fb_state;          // 阀芯位置反馈态（如果硬件支持带反馈信号的阀）
    uint8_t axis_local_index; // 轴内局部索引（可选）
} HydroValve;

/**
 * @brief 伺服定量泵实体模型
 */
typedef struct {
    float displacement_ml_r; // 泵排量 (mL/r)
    float max_rpm;           // 最大允许转速

    /* 兼容当前实现：保留旧字段 */
    float current_rpm;       // 当前指令转速（legacy compatibility）

    /* Sprint 1 新增：请求/授予/反馈分离 */
    float requested_rpm;     // 轴/策略侧请求转速
    float granted_rpm;       // 协调器最终授予转速
    float feedback_rpm;      // 实际反馈转速（预留给真机 HAL）

    // 用于系统内部资源仲裁的请求状态
    float req_flow_lpm;      // 轴请求的流量 (L/min)
    float req_pressure_bar;  // 轴请求的系统压力 (Bar)

    bool  grant_valid;       // 当前周期授权是否有效
    int   owner_axis_id;     // 当前周期授予的轴 ID，-1 表示无 owner
} HydroPump;

/**
 * @brief 执行油缸实体模型
 */
typedef struct {
    float area_rodless_mm2; // 无杆腔有效面积 (推进腔)
    float area_rod_mm2;     // 有杆腔有效面积 (退回腔)
    float stroke_mm;        // 最大机械行程
} HydroCylinder;

#ifdef __cplusplus
}
#endif

#endif // HYDRO_HARDWARE_H
