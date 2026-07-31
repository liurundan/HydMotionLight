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

static HYD_REAL max_real(HYD_REAL left, HYD_REAL right)
{
    return (left > right) ? left : right;
}

static HYD_REAL scaled_radicand_tolerance(HYD_REAL relative_tolerance,
                                          HYD_REAL first_length_squared,
                                          HYD_REAL second_length_squared)
{
    return relative_tolerance * max_real(first_length_squared,
                                         second_length_squared);
}

static HYD_BOOL clamp_roundoff_radicand(HYD_REAL *radicand,
                                        HYD_REAL tolerance)
{
    if (*radicand >= 0.0f) {
        return 1;
    }

    if (*radicand >= -tolerance) {
        *radicand = 0.0f;
        return 1;
    }

    return 0;
}

static HYD_BOOL validation_limits_valid(const HYD_ToggleValidationLimits *limits)
{
    return isfinite(limits->radicandTolerance) &&
           isfinite(limits->minNormalizedMainJacobian) &&
           isfinite(limits->minDriveProjectionRatio) &&
           isfinite(limits->minAbsVelocityRatio) &&
           isfinite(limits->maxAbsVelocityRatio) &&
           isfinite(limits->geometryMarginMm) &&
           (limits->radicandTolerance > 0.0f) &&
           (limits->minNormalizedMainJacobian > 0.0f) &&
           (limits->minDriveProjectionRatio > 0.0f) &&
           (limits->minAbsVelocityRatio > 0.0f) &&
           (limits->maxAbsVelocityRatio >= limits->minAbsVelocityRatio) &&
           (limits->geometryMarginMm >= 0.0f);
}

HYD_ToggleGeometryConfig HYD_ToggleKinematics_DefaultConfig(void)
{
    HYD_ToggleGeometryConfig config = {
        150.0f, 230.0f, 135.0f, 75.0f, 60.0f,
        130.0f, 100.0f, 378.0f, 202.0f,
        0.0f, (int8_t)-1, (int8_t)-1, (int8_t)-1, (int8_t)-1
    };

    return config;
}

HYD_ToggleValidationLimits HYD_ToggleKinematics_DefaultValidationLimits(void)
{
    HYD_ToggleValidationLimits limits = {
        HYD_TOGGLE_RADICAND_TOLERANCE_FACTOR,
        HYD_TOGGLE_MIN_NORMALIZED_MAIN_JACOBIAN,
        HYD_TOGGLE_MIN_DRIVE_PROJECTION_RATIO,
        HYD_TOGGLE_MIN_ABS_VELOCITY_RATIO,
        HYD_TOGGLE_MAX_ABS_VELOCITY_RATIO,
        HYD_TOGGLE_GEOMETRY_MARGIN_MM
    };

    return limits;
}

HYD_BOOL HYD_ToggleKinematics_Prepare(const HYD_ToggleGeometryConfig *config,
                                      HYD_TogglePreparedConfig *prepared,
                                      HYD_ToggleError *error)
{
    HYD_TogglePreparedConfig next;
    HYD_ToggleValidationLimits default_limits;
    HYD_REAL lr2;
    HYD_REAL lf2;
    HYD_REAL lpf2;
    HYD_REAL lpk2;
    HYD_REAL fixed_radicand;
    HYD_REAL fixed_tolerance;

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
        !is_branch_selector_valid(config->tauS) ||
        !is_branch_selector_valid(config->sigmaC)) {
        set_error(error, HYD_TOGGLE_ERROR_INVALID_BRANCH);
        return 0;
    }

    lr2 = config->lr * config->lr;
    lf2 = config->lf * config->lf;
    lpf2 = config->lpf * config->lpf;
    lpk2 = config->lpk * config->lpk;
    default_limits = HYD_ToggleKinematics_DefaultValidationLimits();
    fixed_tolerance = scaled_radicand_tolerance(default_limits.radicandTolerance,
                                                lpf2, lpk2);

    memset(&next, 0, sizeof(next));
    next.raw = *config;
    next.deltaH = config->hm - config->hf;
    next.aP = (lr2 + lpf2 - lpk2) / (2.0f * config->lr);
    fixed_radicand = lpf2 - (next.aP * next.aP);

    if (!clamp_roundoff_radicand(&fixed_radicand, fixed_tolerance) ||
        (fixed_radicand <= fixed_tolerance)) {
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

    if (!isfinite(next.aP) || !isfinite(next.bP) || !isfinite(next.invLr) ||
        !isfinite(next.ld2)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_RESULT);
        return 0;
    }

    *prepared = next;
    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}

static HYD_BOOL solve_at(const HYD_TogglePreparedConfig *prepared,
                         HYD_REAL xm,
                         HYD_REAL vm,
                         HYD_REAL radicand_tolerance,
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
    HYD_REAL main_radicand_tolerance;
    HYD_REAL drive_radicand_tolerance;
    HYD_REAL main_divisor_tolerance;
    HYD_REAL drive_divisor_tolerance;

    memset(&next, 0, sizeof(next));
    d = prepared->raw.dc - xm;
    D2 = (d * d) + (prepared->deltaH * prepared->deltaH);
    main_radicand_tolerance = scaled_radicand_tolerance(radicand_tolerance,
                                                        prepared->lr2,
                                                        prepared->lf2);
    drive_radicand_tolerance = radicand_tolerance * prepared->ld2;
    main_divisor_tolerance = HYD_REAL_EPSILON * prepared->raw.lr *
                             prepared->raw.lf;
    drive_divisor_tolerance = HYD_REAL_EPSILON * prepared->raw.ld;

    if (!isfinite(D2) || (D2 <= main_radicand_tolerance)) {
        set_error(error, HYD_TOGGLE_ERROR_MAIN_LINKAGE_UNREACHABLE);
        return 0;
    }

    D = sqrtf(D2);
    invD = 1.0f / D;
    aK = (prepared->lr2 - prepared->lf2 + D2) * 0.5f * invD;
    next.radicandK = prepared->lr2 - (aK * aK);
    if (!clamp_roundoff_radicand(&next.radicandK,
                                 main_radicand_tolerance)) {
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
    if (!clamp_roundoff_radicand(&next.radicandS,
                                 drive_radicand_tolerance)) {
        set_error(error, HYD_TOGGLE_ERROR_DRIVE_LINK_UNREACHABLE);
        return 0;
    }

    g = sqrtf(next.radicandS);
    if (!isfinite(g) || (fabsf(g) <= drive_divisor_tolerance)) {
        set_error(error, HYD_TOGGLE_ERROR_DRIVE_PROJECTION_UNSAFE);
        return 0;
    }

    invG = 1.0f / g;
    next.xs = px + ((HYD_REAL)prepared->raw.tauS * g);
    deltaJ = (u * (v - prepared->raw.hm)) -
             ((u - d) * (v - prepared->raw.hf));
    if (!isfinite(deltaJ) || (fabsf(deltaJ) <= main_divisor_tolerance)) {
        set_error(error, HYD_TOGGLE_ERROR_MAIN_JACOBIAN_UNSAFE);
        return 0;
    }

    next.normalizedMainJacobian = fabsf(deltaJ) * prepared->invLr /
                                  prepared->raw.lf;
    next.driveProjection = px - next.xs;
    if (!isfinite(next.driveProjection) ||
        (fabsf(next.driveProjection) <= drive_divisor_tolerance)) {
        set_error(error, HYD_TOGGLE_ERROR_DRIVE_PROJECTION_UNSAFE);
        return 0;
    }

    invDeltaJ = 1.0f / deltaJ;
    uPrime = (v - prepared->raw.hf) * (u - d) * invDeltaJ;
    vPrime = -u * (u - d) * invDeltaJ;
    pxPrime = ((prepared->aP * uPrime) - (prepared->bP * vPrime)) * prepared->invLr;
    pyPrime = ((prepared->aP * vPrime) + (prepared->bP * uPrime)) * prepared->invLr;
    next.velocityRatio = pxPrime -
                         ((HYD_REAL)prepared->raw.tauS * py * pyPrime * invG);
    next.vs = next.velocityRatio * vm;
    next.xs *= (HYD_REAL)prepared->raw.sigmaC;
    next.velocityRatio *= (HYD_REAL)prepared->raw.sigmaC;
    next.vs *= (HYD_REAL)prepared->raw.sigmaC;

    if (!isfinite(next.xs) || !isfinite(next.velocityRatio) ||
        !isfinite(next.vs) || !isfinite(next.radicandK) ||
        !isfinite(next.radicandS) ||
        !isfinite(next.normalizedMainJacobian)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_RESULT);
        return 0;
    }

    *solution = next;
    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}

HYD_BOOL HYD_ToggleKinematics_SolveOnline(const HYD_TogglePreparedConfig *prepared,
                                          HYD_REAL xm,
                                          HYD_REAL vm,
                                          HYD_ToggleSolution *solution,
                                          HYD_ToggleError *error)
{
    HYD_ToggleValidationLimits limits;
    HYD_ToggleSolution next;

    if ((prepared == NULL) || (solution == NULL)) {
        set_error(error, HYD_TOGGLE_ERROR_NULL_ARGUMENT);
        return 0;
    }

    if (!isfinite(xm) || !isfinite(vm)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_PARAMETER);
        return 0;
    }

    if ((xm < prepared->xHandoffEffective) || (xm > prepared->raw.sm)) {
        set_error(error, HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE);
        return 0;
    }

    limits = HYD_ToggleKinematics_DefaultValidationLimits();
    if (!solve_at(prepared, xm, vm, limits.radicandTolerance, &next, error)) {
        return 0;
    }

    *solution = next;
    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}

static HYD_BOOL solution_meets_limits(const HYD_TogglePreparedConfig *candidate,
                                      const HYD_ToggleValidationLimits *limits,
                                      const HYD_ToggleSolution *solution,
                                      HYD_ToggleError *error)
{
    HYD_REAL projection_ratio = fabsf(solution->driveProjection) /
                                candidate->raw.ld;
    HYD_REAL abs_velocity_ratio = fabsf(solution->velocityRatio);

    if (solution->normalizedMainJacobian < limits->minNormalizedMainJacobian) {
        set_error(error, HYD_TOGGLE_ERROR_MAIN_JACOBIAN_UNSAFE);
        return 0;
    }

    if (projection_ratio < limits->minDriveProjectionRatio) {
        set_error(error, HYD_TOGGLE_ERROR_DRIVE_PROJECTION_UNSAFE);
        return 0;
    }

    if ((abs_velocity_ratio < limits->minAbsVelocityRatio) ||
        (abs_velocity_ratio > limits->maxAbsVelocityRatio)) {
        set_error(error, HYD_TOGGLE_ERROR_VELOCITY_RATIO_UNSAFE);
        return 0;
    }

    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}

static HYD_BOOL evaluate_validation_point(HYD_ToggleValidation *validation,
                                          HYD_ToggleSolution *solution,
                                          HYD_ToggleError *error)
{
    return solution_meets_limits(&validation->candidate, &validation->limits,
                                 solution, error);
}

static void update_safe_extrema(HYD_TogglePreparedConfig *candidate,
                                const HYD_ToggleSolution *solution,
                                HYD_BOOL first)
{
    if (first) {
        candidate->xsMin = solution->xs;
        candidate->xsMax = solution->xs;
        candidate->kMin = solution->velocityRatio;
        candidate->kMax = solution->velocityRatio;
        return;
    }

    candidate->xsMin = fminf(candidate->xsMin, solution->xs);
    candidate->xsMax = fmaxf(candidate->xsMax, solution->xs);
    candidate->kMin = fminf(candidate->kMin, solution->velocityRatio);
    candidate->kMax = fmaxf(candidate->kMax, solution->velocityRatio);
}

static HYD_BOOL safe_point_is_continuous(const HYD_ToggleValidation *validation,
                                         const HYD_ToggleSolution *solution)
{
    HYD_REAL xs_delta;

    if (!validation->foundSafeSuffix) {
        return 1;
    }

    xs_delta = solution->xs - validation->previousXs;
    if ((solution->velocityRatio * validation->candidate.kMin) <= 0.0f) {
        return 0;
    }

    return (xs_delta * solution->velocityRatio) < 0.0f;
}

static HYD_BOOL safe_point_keeps_branch(const HYD_ToggleValidation *validation,
                                        const HYD_ToggleSolution *solution)
{
    return (solution->velocityRatio * validation->candidate.kMin) > 0.0f;
}

static HYD_BOOL resolve_handoff(HYD_ToggleValidation *validation,
                                HYD_ToggleError *error)
{
    HYD_REAL raw_handoff = validation->candidate.raw.xHandoff;

    validation->candidate.xHandoffEffective =
        (raw_handoff == 0.0f) ? validation->candidate.xGeometryMin : raw_handoff;

    if ((validation->candidate.xHandoffEffective <
         validation->candidate.xGeometryMin) ||
        (validation->candidate.xHandoffEffective >
         validation->candidate.raw.sm)) {
        validation->phase = HYD_TOGGLE_VALIDATION_FAILED;
        set_error(error, HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE);
        return 0;
    }

    validation->phase = HYD_TOGGLE_VALIDATION_HANDOFF;
    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}

static HYD_BOOL validate_handoff(HYD_ToggleValidation *validation,
                                 HYD_ToggleError *error)
{
    HYD_ToggleSolution solution;
    HYD_ToggleError point_error = HYD_TOGGLE_ERROR_NONE;

    if (!solve_at(&validation->candidate,
                  validation->candidate.xHandoffEffective,
                  0.0f,
                  validation->limits.radicandTolerance,
                  &solution,
                  &point_error) ||
        !evaluate_validation_point(validation, &solution, &point_error)) {
        validation->phase = HYD_TOGGLE_VALIDATION_FAILED;
        set_error(error, point_error);
        return 0;
    }

    if (!safe_point_keeps_branch(validation, &solution)) {
        validation->phase = HYD_TOGGLE_VALIDATION_FAILED;
        set_error(error, HYD_TOGGLE_ERROR_NONMONOTONIC);
        return 0;
    }

    validation->phase = HYD_TOGGLE_VALIDATION_COMPLETE;
    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}

HYD_BOOL HYD_ToggleKinematics_BeginValidation(
    const HYD_ToggleGeometryConfig *config,
    const HYD_ToggleValidationLimits *limits,
    HYD_ToggleValidation *validation,
    HYD_ToggleError *error)
{
    HYD_ToggleValidation next;

    if ((config == NULL) || (limits == NULL) || (validation == NULL)) {
        set_error(error, HYD_TOGGLE_ERROR_NULL_ARGUMENT);
        return 0;
    }

    if (!validation_limits_valid(limits)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_PARAMETER);
        return 0;
    }

    memset(&next, 0, sizeof(next));
    if (!HYD_ToggleKinematics_Prepare(config, &next.candidate, error)) {
        return 0;
    }

    next.limits = *limits;
    next.phase = HYD_TOGGLE_VALIDATION_SCAN;
    next.unsafeX = NAN;
    next.safeX = NAN;
    next.previousXs = NAN;

    *validation = next;
    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}

static HYD_BOOL scan_validation_point(HYD_ToggleValidation *validation,
                                      HYD_ToggleError *error)
{
    HYD_ToggleSolution solution;
    HYD_ToggleError point_error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL xm;
    HYD_UINT16 point = validation->nextPoint;

    if (point == 0u) {
        xm = validation->candidate.raw.sm;
    } else if (point == (HYD_TOGGLE_VALIDATION_POINTS - 1u)) {
        xm = 0.0f;
    } else {
        xm = validation->candidate.raw.sm *
             (HYD_REAL)(HYD_TOGGLE_VALIDATION_POINTS - 1u - point) /
             (HYD_REAL)(HYD_TOGGLE_VALIDATION_POINTS - 1u);
    }

    ++validation->nextPoint;
    if (!solve_at(&validation->candidate, xm, 0.0f,
                  validation->limits.radicandTolerance, &solution,
                  &point_error)) {
        validation->phase = HYD_TOGGLE_VALIDATION_FAILED;
        set_error(error, point_error);
        return 0;
    }

    if (evaluate_validation_point(validation, &solution, &point_error)) {
        if (!isnan(validation->unsafeX)) {
            validation->phase = HYD_TOGGLE_VALIDATION_FAILED;
            set_error(error, HYD_TOGGLE_ERROR_NONMONOTONIC);
            return 0;
        }

        if (!safe_point_is_continuous(validation, &solution)) {
            validation->phase = HYD_TOGGLE_VALIDATION_FAILED;
            set_error(error, HYD_TOGGLE_ERROR_NONMONOTONIC);
            return 0;
        }

        update_safe_extrema(&validation->candidate, &solution,
                            !validation->foundSafeSuffix);
        validation->foundSafeSuffix = 1;
        validation->safeX = xm;
        validation->previousXs = solution.xs;
        set_error(error, HYD_TOGGLE_ERROR_NONE);
        return 1;
    }

    if (!validation->foundSafeSuffix) {
        validation->phase = HYD_TOGGLE_VALIDATION_FAILED;
        set_error(error, point_error);
        return 0;
    }

    if (isnan(validation->unsafeX)) {
        validation->unsafeX = xm;
    }

    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}

static void set_refined_endpoint_envelope(HYD_TogglePreparedConfig *candidate,
                                          const HYD_ToggleSolution *solution)
{
    if (solution->velocityRatio < 0.0f) {
        candidate->xsMax = solution->xs;
    } else {
        candidate->xsMin = solution->xs;
    }
    candidate->kMin = fminf(candidate->kMin, solution->velocityRatio);
    candidate->kMax = fmaxf(candidate->kMax, solution->velocityRatio);
}

static HYD_BOOL refine_validation_boundary(HYD_ToggleValidation *validation,
                                           HYD_ToggleError *error)
{
    HYD_ToggleSolution solution;
    HYD_ToggleError point_error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL xm;

    if (validation->refineIteration ==
        (HYD_TOGGLE_BOUNDARY_ITERATIONS - 1u)) {
        xm = fminf(validation->safeX + validation->limits.geometryMarginMm,
                   validation->candidate.raw.sm);
        if (!solve_at(&validation->candidate, xm, 0.0f,
                      validation->limits.radicandTolerance, &solution,
                      &point_error) ||
            !evaluate_validation_point(validation, &solution, &point_error)) {
            validation->phase = HYD_TOGGLE_VALIDATION_FAILED;
            set_error(error, point_error);
            return 0;
        }

        ++validation->refineIteration;
        validation->safeX = xm;
        validation->candidate.xGeometryMin = xm;
        set_refined_endpoint_envelope(&validation->candidate, &solution);
        return resolve_handoff(validation, error);
    }

    xm = 0.5f * (validation->unsafeX + validation->safeX);
    ++validation->refineIteration;
    if (!solve_at(&validation->candidate, xm, 0.0f,
                  validation->limits.radicandTolerance, &solution,
                  &point_error)) {
        validation->phase = HYD_TOGGLE_VALIDATION_FAILED;
        set_error(error, point_error);
        return 0;
    }

    if (evaluate_validation_point(validation, &solution, &point_error)) {
        if (!safe_point_keeps_branch(validation, &solution)) {
            validation->phase = HYD_TOGGLE_VALIDATION_FAILED;
            set_error(error, HYD_TOGGLE_ERROR_NONMONOTONIC);
            return 0;
        }

        validation->safeX = xm;
        validation->previousXs = solution.xs;
        update_safe_extrema(&validation->candidate, &solution, 0);
    } else {
        validation->unsafeX = xm;
    }

    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}

HYD_BOOL HYD_ToggleKinematics_ValidationStep(HYD_ToggleValidation *validation,
                                             HYD_UINT16 maxEvaluations,
                                             HYD_ToggleError *error)
{
    HYD_UINT16 evaluations = 0u;

    if (validation == NULL) {
        set_error(error, HYD_TOGGLE_ERROR_NULL_ARGUMENT);
        return 0;
    }

    if (validation->phase == HYD_TOGGLE_VALIDATION_FAILED) {
        set_error(error, HYD_TOGGLE_ERROR_NONMONOTONIC);
        return 0;
    }

    while ((evaluations < maxEvaluations) &&
           ((validation->phase == HYD_TOGGLE_VALIDATION_SCAN) ||
            (validation->phase == HYD_TOGGLE_VALIDATION_REFINE) ||
            (validation->phase == HYD_TOGGLE_VALIDATION_HANDOFF))) {
        if (validation->phase == HYD_TOGGLE_VALIDATION_SCAN) {
            if (!scan_validation_point(validation, error)) {
                return 0;
            }
            ++evaluations;

            if (validation->nextPoint == HYD_TOGGLE_VALIDATION_POINTS) {
                if (isnan(validation->unsafeX)) {
                    validation->candidate.xGeometryMin = 0.0f;
                    if (!resolve_handoff(validation, error)) {
                        return 0;
                    }
                } else {
                    validation->phase = HYD_TOGGLE_VALIDATION_REFINE;
                }
            }
        } else if (validation->phase == HYD_TOGGLE_VALIDATION_REFINE) {
            if (!refine_validation_boundary(validation, error)) {
                return 0;
            }
            ++evaluations;
        } else {
            if (!validate_handoff(validation, error)) {
                return 0;
            }
            ++evaluations;
        }
    }

    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}

HYD_BOOL HYD_ToggleKinematics_ValidationDone(
    const HYD_ToggleValidation *validation)
{
    if (validation == NULL) {
        return 1;
    }

    return (validation->phase == HYD_TOGGLE_VALIDATION_COMPLETE) ||
           (validation->phase == HYD_TOGGLE_VALIDATION_FAILED);
}

HYD_BOOL HYD_ToggleKinematics_FinishValidation(
    const HYD_ToggleValidation *validation,
    HYD_TogglePreparedConfig *prepared,
    HYD_ToggleError *error)
{
    if ((validation == NULL) || (prepared == NULL)) {
        set_error(error, HYD_TOGGLE_ERROR_NULL_ARGUMENT);
        return 0;
    }

    if (validation->phase != HYD_TOGGLE_VALIDATION_COMPLETE) {
        set_error(error, HYD_TOGGLE_ERROR_NONMONOTONIC);
        return 0;
    }

    *prepared = validation->candidate;
    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}

HYD_BOOL HYD_ToggleKinematics_ValidateBlocking(
    const HYD_ToggleGeometryConfig *config,
    HYD_TogglePreparedConfig *prepared,
    HYD_ToggleError *error)
{
    HYD_ToggleValidationLimits limits;
    HYD_ToggleValidation validation;

    if (prepared == NULL) {
        set_error(error, HYD_TOGGLE_ERROR_NULL_ARGUMENT);
        return 0;
    }

    limits = HYD_ToggleKinematics_DefaultValidationLimits();
    if (!HYD_ToggleKinematics_BeginValidation(config, &limits, &validation,
                                              error)) {
        return 0;
    }

    while (!HYD_ToggleKinematics_ValidationDone(&validation)) {
        if (!HYD_ToggleKinematics_ValidationStep(
                &validation, HYD_TOGGLE_VALIDATION_POINTS, error)) {
            return 0;
        }
    }

    return HYD_ToggleKinematics_FinishValidation(&validation, prepared, error);
}

HYD_BOOL HYD_ToggleKinematics_InversePosition(
    const HYD_TogglePreparedConfig *prepared,
    HYD_REAL xs,
    HYD_REAL *xm,
    HYD_ToggleError *error)
{
    HYD_ToggleSolution low_solution;
    HYD_ToggleSolution high_solution;
    HYD_ToggleSolution middle_solution;
    HYD_REAL low;
    HYD_REAL high;
    HYD_REAL middle = 0.0f;
    HYD_REAL next;
    HYD_BOOL increasing;
    HYD_UINT8 iteration;

    if ((prepared == NULL) || (xm == NULL)) {
        set_error(error, HYD_TOGGLE_ERROR_NULL_ARGUMENT);
        return 0;
    }

    if (!isfinite(xs)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_PARAMETER);
        return 0;
    }

    if ((xs < prepared->xsMin) || (xs > prepared->xsMax)) {
        set_error(error, HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE);
        return 0;
    }

    low = prepared->xHandoffEffective;
    high = prepared->raw.sm;
    if (!HYD_ToggleKinematics_SolveOnline(prepared, low, 0.0f,
                                          &low_solution, error) ||
        !HYD_ToggleKinematics_SolveOnline(prepared, high, 0.0f,
                                          &high_solution, error)) {
        return 0;
    }

    if (low == high) {
        if (xs != low_solution.xs) {
            set_error(error, HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE);
            return 0;
        }
        *xm = low;
        set_error(error, HYD_TOGGLE_ERROR_NONE);
        return 1;
    }

    if ((xs < fminf(low_solution.xs, high_solution.xs)) ||
        (xs > fmaxf(low_solution.xs, high_solution.xs))) {
        set_error(error, HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE);
        return 0;
    }

    if (low_solution.xs == high_solution.xs) {
        set_error(error, HYD_TOGGLE_ERROR_NONMONOTONIC);
        return 0;
    }
    increasing = high_solution.xs > low_solution.xs;

    for (iteration = 0u; iteration < 32u; ++iteration) {
        middle = 0.5f * (low + high);
        if (!HYD_ToggleKinematics_SolveOnline(prepared, middle, 0.0f,
                                              &middle_solution, error)) {
            return 0;
        }

        if ((increasing && (middle_solution.xs < xs)) ||
            (!increasing && (middle_solution.xs > xs))) {
            low = middle;
        } else {
            high = middle;
        }
    }

    next = 0.5f * (low + high);
    if (!isfinite(next)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_RESULT);
        return 0;
    }

    *xm = next;
    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}
