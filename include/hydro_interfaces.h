#ifndef HYDRO_INTERFACES_H
#define HYDRO_INTERFACES_H

#include "hydro_hardware.h"

/* ==================================================================
 * L2: 反馈与适配层 (Sensor Backend Layer)
 * 采用依赖注入模式隔离底层硬件和纯软件仿真环境
 * ================================================================== */

/**
 * @brief 轴的统一物理反馈数据包
 * 无论数据来自真实 ADC/Encoder 还是 Simulator，均被打包为本结构体
 */
typedef struct {
    float position_mm;    // 实时位移
    float velocity_mm_s;  // 实时速度 (可选，通常由位置做差分获得)
    float pressure_bar;   // 实时压力
    bool  interlock_ok;   // 硬件安全互锁是否满足 (如：安全门关)
    bool  servo_ready;    // 伺服驱动器无故障并就绪
    HYD_PumpFeedback pumpFeedback; // 统一泵反馈包
} AxisFeedback;

/**
 * @brief 传感器后端抽象接口 (Dependency Injection)
 */
typedef struct {
    /**
     * @brief 读取当前轴的反馈状态
     * @param ctx 后端上下文指针 (HAL 句柄 或 仿真器环境)
     * @param fb 输出参数，填入最新的传感器状态
     */
    void (*read_feedback)(void* ctx, AxisFeedback* fb);
    
    /**
     * @brief 将逻辑阀组的状态输出至硬件或仿真器
     * @param ctx 后端上下文指针
     * @param valves 指向逻辑阀指针数组的首地址
     * @param count 要更新的阀的数量
     */
    void (*write_valves)(void* ctx, HydroValve** valves, int count);
    
    /**
     * @brief 将泵的控制指令（如转速）输出至伺服驱动器或仿真器
     * @param ctx 后端上下文指针
     * @param pump 伺服泵对象模型
     */
    void (*write_pump)(void* ctx, HydroPump* pump);
    
    // Opaque 指针，指向实际的实现对象 (实机 HAL Driver 或 HydraulicSimEnv)
    void* ctx; 
} ISensorBackend;

#endif // HYDRO_INTERFACES_H
