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
 * - HDY_HydraulicSimFB：单轴实例句柄 / 快照结构
 * - __mcl_cmd_createSimAxis：分配轴实例并建立 AXISID -> 轴槽位映射
 * - __mcl_cmd_moveSimAxis：写入轴命令并切换单泵 owner
 * - __mcl_cmd_readSimAxis：按 AXISID 读取轴快照
 * - __HdySimulator_framework_Publish：共享 env 每扫描只步进一次
 * ================================================================== */

typedef struct {
    HDY_BOOL allocated;
    HDY_BOOL active;
    int axis_id;
    HDY_UINT8 axis_type;

    HDY_REAL maxVel;
    HDY_REAL maxAcc;
    HDY_REAL maxDec;

    HDY_BOOL enable;
    HDY_REAL cmd_rpm;
    int direction;

    HDY_REAL pos_mm;
    HDY_REAL vel_mm_s;
    HDY_REAL pressure_bar;

    HydraulicSimEnv* _env;
    HDY_BOOL _isSharedEnv;
    HDY_BOOL _initialized;
} HDY_HydraulicSimFB;

/* FUNCTION_BLOCK HDY_CREATESIMAXIS */
typedef struct {
    __DECLARE_VAR(BOOL,EN)
    __DECLARE_VAR(BOOL,ENO)
    __DECLARE_VAR(USINT,AXISTYPE)
    __DECLARE_VAR(REAL,MAXVEL)
    __DECLARE_VAR(REAL,MAXACC)
    __DECLARE_VAR(REAL,MAXDEC)

    __DECLARE_VAR(SINT,AXISID)
    __DECLARE_VAR(BOOL,DONE)
} HDY_CREATESIMAXIS;

/* FUNCTION_BLOCK HDY_MOVESIMAXIS */
typedef struct {
    __DECLARE_VAR(BOOL,EN)
    __DECLARE_VAR(BOOL,ENO)
    __DECLARE_VAR(BOOL,ENABLE)
    __DECLARE_VAR(SINT,AXISID)
    __DECLARE_VAR(REAL,CMD_RPM)
    __DECLARE_VAR(SINT,DIRECTION)

    __DECLARE_VAR(BOOL,BUSY)
} HDY_MOVESIMAXIS;

/* FUNCTION_BLOCK HDY_READSIMAXIS */
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
} HDY_READSIMAXIS;

void HDY_HydraulicSimFB_Cycle(HDY_HydraulicSimFB* fb);

extern int  __HdySimulator_framework_Init();
extern void __HdySimulator_framework_Cleanup();
extern void __HdySimulator_framework_Retrieve();
extern void __HdySimulator_framework_Publish();

extern void __mcl_cmd_createSimAxis(HDY_CREATESIMAXIS *data__);
extern void __mcl_cmd_moveSimAxis(HDY_MOVESIMAXIS *data__);
extern void __mcl_cmd_readSimAxis(HDY_READSIMAXIS *data__);

#ifdef __cplusplus
}
#endif

#endif /* HYDRO_SIM_FB_H */
