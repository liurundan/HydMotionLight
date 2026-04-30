#ifndef HYDRO_SIM_FB_H
#define HYDRO_SIM_FB_H

#include "common_types.h"
#include "hydro_sim.h"
#include "accessor.h"
#include "iec_types_all.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================================================================
 * L1: PLC 适配层
 *
 * - HYD_HydraulicSimFB：单轴实例句柄 / 快照结构
 * - __mcl_cmd_createSimAxis：分配轴实例并建立 AXISID -> 轴槽位映射
 * - __mcl_cmd_moveSimAxis：写入轴命令并切换单泵 owner
 * - __mcl_cmd_readSimAxis：按 AXISID 读取轴快照
 * - __HydSimulator_framework_Publish：共享 env 每扫描只步进一次
 * ================================================================== */

typedef struct {
    HYD_BOOL allocated;
    HYD_BOOL active;
    int axis_id;
    HYD_UINT8 axis_type;

    HYD_REAL maxVel;
    HYD_REAL maxAcc;
    HYD_REAL maxDec;

    HYD_BOOL enable;
    HYD_REAL cmd_rpm;
    int direction;

    HYD_REAL pos_mm;
    HYD_REAL vel_mm_s;
    HYD_REAL pressure_bar;

    HydraulicSimEnv* _env;
    HYD_BOOL _isSharedEnv;
    HYD_BOOL _initialized;
} HYD_HydraulicSimFB;

/* FUNCTION_BLOCK HYD_CREATESIMAXIS */
typedef struct {
    __DECLARE_VAR(BOOL,EN)
    __DECLARE_VAR(BOOL,ENO)
    __DECLARE_VAR(USINT,AXISTYPE)
    __DECLARE_VAR(REAL,MAXVEL)
    __DECLARE_VAR(REAL,MAXACC)
    __DECLARE_VAR(REAL,MAXDEC)

    __DECLARE_VAR(SINT,AXISID)
    __DECLARE_VAR(BOOL,DONE)
} HYD_CREATESIMAXIS;

/* FUNCTION_BLOCK HYD_MOVESIMAXIS */
typedef struct {
    __DECLARE_VAR(BOOL,EN)
    __DECLARE_VAR(BOOL,ENO)
    __DECLARE_VAR(BOOL,ENABLE)
    __DECLARE_VAR(SINT,AXISID)
    __DECLARE_VAR(REAL,CMD_RPM)
    __DECLARE_VAR(SINT,DIRECTION)

    __DECLARE_VAR(BOOL,BUSY)
} HYD_MOVESIMAXIS;

/* FUNCTION_BLOCK HYD_READSIMAXIS */
typedef struct {
    __DECLARE_VAR(BOOL,EN)
    __DECLARE_VAR(BOOL,ENO)
    __DECLARE_VAR(BOOL,ENABLE)
    __DECLARE_VAR(SINT,AXISID)

    __DECLARE_VAR(BOOL,ACTIVE)
    __DECLARE_VAR(REAL,POS_MM)
    __DECLARE_VAR(REAL,VEL_MM_S)
    __DECLARE_VAR(REAL,PRESSURE_BAR)
    __DECLARE_VAR(BOOL,BUSY)
} HYD_READSIMAXIS;

void HYD_HydraulicSimFB_Cycle(HYD_HydraulicSimFB* fb);

extern int  __HydSimulator_framework_Init();
extern void __HydSimulator_framework_Cleanup();
extern void __HydSimulator_framework_Retrieve();
extern void __HydSimulator_framework_Publish();

extern void __mcl_cmd_createSimAxis(HYD_CREATESIMAXIS *data__);
extern void __mcl_cmd_moveSimAxis(HYD_MOVESIMAXIS *data__);
extern void __mcl_cmd_readSimAxis(HYD_READSIMAXIS *data__);

#ifdef __cplusplus
}
#endif

#endif /* HYDRO_SIM_FB_H */
