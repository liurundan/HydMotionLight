#include "toggle_kinematics.h"

#include <math.h>
#include <string.h>

static void set_error(HYD_ToggleError *error, HYD_ToggleError value)
{
    if (error != NULL) {
        *error = value;
    }
}

static HYD_BOOL is_branch_selector_valid(int8_t value)
{
    return (value == (int8_t)1) || (value == (int8_t)-1);
}

static HYD_BOOL all_config_parameters_finite(const HYD_ToggleGeometryConfig *config)
{
    return isfinite(config->lr) && isfinite(config->lf) &&
           isfinite(config->lpf) && isfinite(config->lpk) &&
           isfinite(config->ld) && isfinite(config->hf) &&
           isfinite(config->hm) && isfinite(config->dc) &&
           isfinite(config->sm) && isfinite(config->xHandoff);
}

static HYD_BOOL lengths_and_stroke_positive(const HYD_ToggleGeometryConfig *config)
{
    return (config->lr > 0.0f) && (config->lf > 0.0f) &&
           (config->lpf > 0.0f) && (config->lpk > 0.0f) &&
           (config->ld > 0.0f) && (config->sm > 0.0f);
}

static HYD_BOOL clamp_roundoff_radicand(HYD_REAL *radicand)
{
    if (*radicand >= 0.0f) {
        return 1;
    }

    if (*radicand >= -HYD_TOGGLE_RADICAND_EPS) {
        *radicand = 0.0f;
        return 1;
    }

    return 0;
}

HYD_ToggleGeometryConfig HYD_ToggleKinematics_DefaultConfig(void)
{
    HYD_ToggleGeometryConfig config = {
        150.0f, 230.0f, 135.0f, 75.0f, 60.0f,
        130.0f, 100.0f, 378.0f, 202.0f,
        0.0f, (int8_t)-1, (int8_t)-1, (int8_t)-1
    };

    return config;
}

HYD_BOOL HYD_ToggleKinematics_Prepare(const HYD_ToggleGeometryConfig *config,
                                      HYD_TogglePreparedConfig *prepared,
                                      HYD_ToggleError *error)
{
    HYD_TogglePreparedConfig next;
    HYD_REAL lr2;
    HYD_REAL lf2;
    HYD_REAL lpf2;
    HYD_REAL lpk2;
    HYD_REAL fixed_radicand;

    if ((config == NULL) || (prepared == NULL)) {
        set_error(error, HYD_TOGGLE_ERROR_NULL_ARGUMENT);
        return 0;
    }

    if (!all_config_parameters_finite(config)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_PARAMETER);
        return 0;
    }

    if (!lengths_and_stroke_positive(config)) {
        set_error(error, HYD_TOGGLE_ERROR_NONPOSITIVE_LENGTH);
        return 0;
    }

    if (!is_branch_selector_valid(config->sigmaK) ||
        !is_branch_selector_valid(config->signB) ||
        !is_branch_selector_valid(config->tauS)) {
        set_error(error, HYD_TOGGLE_ERROR_INVALID_BRANCH);
        return 0;
    }

    lr2 = config->lr * config->lr;
    lf2 = config->lf * config->lf;
    lpf2 = config->lpf * config->lpf;
    lpk2 = config->lpk * config->lpk;

    memset(&next, 0, sizeof(next));
    next.raw = *config;
    next.deltaH = config->hm - config->hf;
    next.aP = (lr2 + lpf2 - lpk2) / (2.0f * config->lr);
    fixed_radicand = lpf2 - (next.aP * next.aP);

    if (!clamp_roundoff_radicand(&fixed_radicand) ||
        (fixed_radicand <= HYD_TOGGLE_UNSAFE_EPS)) {
        set_error(error, HYD_TOGGLE_ERROR_FIXED_TRIANGLE_INVALID);
        return 0;
    }

    next.bP = (HYD_REAL)config->signB * sqrtf(fixed_radicand);
    next.lr2 = lr2;
    next.lf2 = lf2;
    next.ld2 = config->ld * config->ld;
    next.invLr = 1.0f / config->lr;
    next.xGeometryMin = 0.0f;
    next.xHandoffEffective = config->xHandoff;
    next.xsMin = 0.0f;
    next.xsMax = 0.0f;
    next.kMin = 0.0f;
    next.kMax = 0.0f;

    if (!isfinite(next.aP) || !isfinite(next.bP) || !isfinite(next.invLr)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_RESULT);
        return 0;
    }

    *prepared = next;
    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}

HYD_BOOL HYD_ToggleKinematics_SolveOnline(const HYD_TogglePreparedConfig *prepared,
                                          HYD_REAL xm,
                                          HYD_REAL vm,
                                          HYD_ToggleSolution *solution,
                                          HYD_ToggleError *error)
{
    HYD_ToggleSolution next;
    HYD_REAL d;
    HYD_REAL D2;
    HYD_REAL D;
    HYD_REAL invD;
    HYD_REAL aK;
    HYD_REAL hK;
    HYD_REAL u;
    HYD_REAL v;
    HYD_REAL v_from_hf;
    HYD_REAL px;
    HYD_REAL py;
    HYD_REAL g;
    HYD_REAL invG;
    HYD_REAL deltaJ;
    HYD_REAL invDeltaJ;
    HYD_REAL uPrime;
    HYD_REAL vPrime;
    HYD_REAL pxPrime;
    HYD_REAL pyPrime;

    if ((prepared == NULL) || (solution == NULL)) {
        set_error(error, HYD_TOGGLE_ERROR_NULL_ARGUMENT);
        return 0;
    }

    memset(&next, 0, sizeof(next));

    if (!isfinite(xm) || !isfinite(vm)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_PARAMETER);
        return 0;
    }

    if ((xm < 0.0f) || (xm > prepared->raw.sm)) {
        set_error(error, HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE);
        return 0;
    }

    d = prepared->raw.dc - xm;
    D2 = (d * d) + (prepared->deltaH * prepared->deltaH);
    if (!isfinite(D2) || (D2 <= HYD_TOGGLE_UNSAFE_EPS)) {
        set_error(error, HYD_TOGGLE_ERROR_MAIN_LINKAGE_UNREACHABLE);
        return 0;
    }

    D = sqrtf(D2);
    invD = 1.0f / D;
    aK = (prepared->lr2 - prepared->lf2 + D2) * 0.5f * invD;
    next.radicandK = prepared->lr2 - (aK * aK);
    if (!clamp_roundoff_radicand(&next.radicandK)) {
        set_error(error, HYD_TOGGLE_ERROR_MAIN_LINKAGE_UNREACHABLE);
        return 0;
    }

    hK = sqrtf(next.radicandK);
    u = ((aK * d) - ((HYD_REAL)prepared->raw.sigmaK * hK * prepared->deltaH)) * invD;
    v = prepared->raw.hf +
        (((aK * prepared->deltaH) + ((HYD_REAL)prepared->raw.sigmaK * hK * d)) * invD);
    v_from_hf = v - prepared->raw.hf;
    px = ((prepared->aP * u) - (prepared->bP * v_from_hf)) * prepared->invLr;
    py = prepared->raw.hf +
         (((prepared->aP * v_from_hf) + (prepared->bP * u)) * prepared->invLr);

    next.radicandS = prepared->ld2 - (py * py);
    if (!clamp_roundoff_radicand(&next.radicandS)) {
        set_error(error, HYD_TOGGLE_ERROR_DRIVE_LINK_UNREACHABLE);
        return 0;
    }

    g = sqrtf(next.radicandS);
    if (!isfinite(g) || (fabsf(g) <= HYD_TOGGLE_UNSAFE_EPS)) {
        set_error(error, HYD_TOGGLE_ERROR_DRIVE_PROJECTION_UNSAFE);
        return 0;
    }

    invG = 1.0f / g;
    next.xs = px + ((HYD_REAL)prepared->raw.tauS * g);
    deltaJ = (u * (v - prepared->raw.hm)) -
             ((u - d) * (v - prepared->raw.hf));
    if (!isfinite(deltaJ) || (fabsf(deltaJ) <= HYD_TOGGLE_UNSAFE_EPS)) {
        set_error(error, HYD_TOGGLE_ERROR_MAIN_JACOBIAN_UNSAFE);
        return 0;
    }

    next.normalizedMainJacobian = fabsf(deltaJ) * prepared->invLr / prepared->raw.lf;
    next.driveProjection = px - next.xs;
    if (!isfinite(next.driveProjection) ||
        (fabsf(next.driveProjection) <= HYD_TOGGLE_UNSAFE_EPS)) {
        set_error(error, HYD_TOGGLE_ERROR_DRIVE_PROJECTION_UNSAFE);
        return 0;
    }

    invDeltaJ = 1.0f / deltaJ;
    uPrime = (v - prepared->raw.hf) * (u - d) * invDeltaJ;
    vPrime = -u * (u - d) * invDeltaJ;
    pxPrime = ((prepared->aP * uPrime) - (prepared->bP * vPrime)) * prepared->invLr;
    pyPrime = ((prepared->aP * vPrime) + (prepared->bP * uPrime)) * prepared->invLr;
    next.velocityRatio = pxPrime - ((HYD_REAL)prepared->raw.tauS * py * pyPrime * invG);
    next.vs = next.velocityRatio * vm;

    if (!isfinite(next.xs) || !isfinite(next.velocityRatio) || !isfinite(next.vs) ||
        !isfinite(next.radicandK) || !isfinite(next.radicandS) ||
        !isfinite(next.normalizedMainJacobian)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_RESULT);
        return 0;
    }

    if (fabsf(next.velocityRatio) > HYD_TOGGLE_VELOCITY_RATIO_LIMIT) {
        set_error(error, HYD_TOGGLE_ERROR_VELOCITY_RATIO_UNSAFE);
        return 0;
    }

    *solution = next;
    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}
