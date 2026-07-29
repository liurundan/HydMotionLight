#ifndef HYD_TOGGLE_KINEMATICS_H
#define HYD_TOGGLE_KINEMATICS_H

#include <stdint.h>

#include "common_types.h"

typedef enum {
    HYD_TOGGLE_ERROR_NONE = 0,
    HYD_TOGGLE_ERROR_NULL_ARGUMENT,
    HYD_TOGGLE_ERROR_NONFINITE_PARAMETER,
    HYD_TOGGLE_ERROR_NONPOSITIVE_LENGTH,
    HYD_TOGGLE_ERROR_INVALID_BRANCH,
    HYD_TOGGLE_ERROR_FIXED_TRIANGLE_INVALID,
    HYD_TOGGLE_ERROR_MAIN_LINKAGE_UNREACHABLE,
    HYD_TOGGLE_ERROR_DRIVE_LINK_UNREACHABLE,
    HYD_TOGGLE_ERROR_MAIN_JACOBIAN_UNSAFE,
    HYD_TOGGLE_ERROR_DRIVE_PROJECTION_UNSAFE,
    HYD_TOGGLE_ERROR_VELOCITY_RATIO_UNSAFE,
    HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE,
    HYD_TOGGLE_ERROR_NONFINITE_RESULT,
    HYD_TOGGLE_ERROR_NONMONOTONIC
} HYD_ToggleError;

typedef struct {
    HYD_REAL lr, lf, lpf, lpk, ld;
    HYD_REAL hf, hm, dc, sm;
    HYD_REAL xHandoff;
    int8_t sigmaK, signB, tauS;
} HYD_ToggleGeometryConfig;

typedef struct {
    HYD_ToggleGeometryConfig raw;
    HYD_REAL deltaH, aP, bP;
    HYD_REAL lr2, lf2, ld2, invLr;
    HYD_REAL xGeometryMin, xHandoffEffective;
    HYD_REAL xsMin, xsMax, kMin, kMax;
} HYD_TogglePreparedConfig;

typedef struct {
    HYD_REAL xs, velocityRatio, vs;
    HYD_REAL radicandK, radicandS;
    HYD_REAL normalizedMainJacobian;
    HYD_REAL driveProjection;
} HYD_ToggleSolution;

HYD_ToggleGeometryConfig HYD_ToggleKinematics_DefaultConfig(void);
HYD_BOOL HYD_ToggleKinematics_Prepare(const HYD_ToggleGeometryConfig *config, HYD_TogglePreparedConfig *prepared, HYD_ToggleError *error);
HYD_BOOL HYD_ToggleKinematics_SolveOnline(const HYD_TogglePreparedConfig *prepared, HYD_REAL xm, HYD_REAL vm, HYD_ToggleSolution *solution, HYD_ToggleError *error);

#endif
