#ifndef HYD_TOGGLE_KINEMATICS_H
#define HYD_TOGGLE_KINEMATICS_H

#include <stdint.h>

#include "common_types.h"

#define HYD_TOGGLE_VALIDATION_POINTS 257u
#define HYD_TOGGLE_BOUNDARY_ITERATIONS 24u

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

typedef struct {
    HYD_REAL radicandTolerance;
    HYD_REAL minNormalizedMainJacobian;
    HYD_REAL minDriveProjectionRatio;
    HYD_REAL minAbsVelocityRatio;
    HYD_REAL maxAbsVelocityRatio;
    HYD_REAL geometryMarginMm;
} HYD_ToggleValidationLimits;

typedef enum {
    HYD_TOGGLE_VALIDATION_SCAN = 0,
    HYD_TOGGLE_VALIDATION_REFINE,
    HYD_TOGGLE_VALIDATION_COMPLETE,
    HYD_TOGGLE_VALIDATION_FAILED
} HYD_ToggleValidationPhase;

typedef struct {
    HYD_TogglePreparedConfig candidate;
    HYD_ToggleValidationLimits limits;
    HYD_ToggleValidationPhase phase;
    HYD_UINT16 nextPoint;
    HYD_UINT8 refineIteration;
    HYD_BOOL foundSafeSuffix;
    HYD_REAL unsafeX;
    HYD_REAL safeX;
    HYD_REAL previousXs;
} HYD_ToggleValidation;

HYD_ToggleGeometryConfig HYD_ToggleKinematics_DefaultConfig(void);
HYD_ToggleValidationLimits HYD_ToggleKinematics_DefaultValidationLimits(void);
HYD_BOOL HYD_ToggleKinematics_Prepare(const HYD_ToggleGeometryConfig *config, HYD_TogglePreparedConfig *prepared, HYD_ToggleError *error);
HYD_BOOL HYD_ToggleKinematics_SolveOnline(const HYD_TogglePreparedConfig *prepared, HYD_REAL xm, HYD_REAL vm, HYD_ToggleSolution *solution, HYD_ToggleError *error);
HYD_BOOL HYD_ToggleKinematics_BeginValidation(const HYD_ToggleGeometryConfig *config, const HYD_ToggleValidationLimits *limits, HYD_ToggleValidation *validation, HYD_ToggleError *error);
HYD_BOOL HYD_ToggleKinematics_ValidationStep(HYD_ToggleValidation *validation, HYD_UINT16 maxEvaluations, HYD_ToggleError *error);
HYD_BOOL HYD_ToggleKinematics_ValidationDone(const HYD_ToggleValidation *validation);
HYD_BOOL HYD_ToggleKinematics_FinishValidation(const HYD_ToggleValidation *validation, HYD_TogglePreparedConfig *prepared, HYD_ToggleError *error);
HYD_BOOL HYD_ToggleKinematics_ValidateBlocking(const HYD_ToggleGeometryConfig *config, HYD_TogglePreparedConfig *prepared, HYD_ToggleError *error);
HYD_BOOL HYD_ToggleKinematics_InversePosition(const HYD_TogglePreparedConfig *prepared, HYD_REAL xs, HYD_REAL *xm, HYD_ToggleError *error);

#endif
