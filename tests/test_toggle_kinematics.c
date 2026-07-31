#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "toggle_kinematics.h"

static void assert_near(HYD_REAL actual, HYD_REAL expected, HYD_REAL tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

static void test_validation_phase_ordinals_remain_stable(void)
{
    assert(HYD_TOGGLE_VALIDATION_SCAN == 0);
    assert(HYD_TOGGLE_VALIDATION_REFINE == 1);
    assert(HYD_TOGGLE_VALIDATION_COMPLETE == 2);
    assert(HYD_TOGGLE_VALIDATION_FAILED == 3);
    assert(HYD_TOGGLE_VALIDATION_HANDOFF == 4);
}

static void test_default_prepare(void)
{
    HYD_ToggleGeometryConfig config = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    assert_near(config.dc, 378.0f, 1e-5f);
    assert(config.sigmaC == (int8_t)-1);
    assert(HYD_ToggleKinematics_ValidateBlocking(&config, &prepared, &error));
    assert(error == HYD_TOGGLE_ERROR_NONE);
    assert_near(prepared.aP, 117.0f, 1e-5f);
    assert_near(prepared.bP, -67.349833f, 1e-4f);
}

static void assert_solution_at(const HYD_TogglePreparedConfig *prepared,
                               HYD_REAL xm,
                               HYD_REAL expected_xs,
                               HYD_REAL expected_k)
{
    HYD_ToggleSolution solution;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    assert(HYD_ToggleKinematics_SolveOnline(prepared, xm, 10.0f, &solution, &error));
    assert(error == HYD_TOGGLE_ERROR_NONE);
    assert_near(solution.xs, expected_xs, 2e-3f);
    assert_near(solution.velocityRatio, expected_k, 2e-4f);
    assert_near(solution.vs, expected_k * 10.0f, 2e-3f);
}

static HYD_BOOL validate_with_limits(const HYD_ToggleGeometryConfig *config,
                                     const HYD_ToggleValidationLimits *limits,
                                     HYD_TogglePreparedConfig *prepared,
                                     HYD_ToggleError *error)
{
    HYD_ToggleValidation validation;

    if (!HYD_ToggleKinematics_BeginValidation(config, limits, &validation,
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

static void test_online_golden_points(void)
{
    HYD_ToggleGeometryConfig config = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    assert(HYD_ToggleKinematics_ValidateBlocking(&config, &prepared, &error));
    assert(error == HYD_TOGGLE_ERROR_NONE);

    assert_solution_at(&prepared, 0.0f, -64.910771f, 10.150074f);
    assert_solution_at(&prepared, 50.0f, 20.397682f, 0.935184f);
    assert_solution_at(&prepared, 101.0f, 63.808094f, 0.808380f);
    assert_solution_at(&prepared, 202.0f, 138.295657f, 0.520517f);
}

static void test_default_validation_and_inverse_round_trip(void)
{
    HYD_ToggleGeometryConfig config = HYD_ToggleKinematics_DefaultConfig();
    HYD_ToggleValidationLimits limits = HYD_ToggleKinematics_DefaultValidationLimits();
    HYD_ToggleValidation validation;
    HYD_TogglePreparedConfig prepared;
    HYD_TogglePreparedConfig before;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL xm = -999.0f;

    assert(HYD_TOGGLE_VALIDATION_POINTS == 257u);
    assert(HYD_TOGGLE_BOUNDARY_ITERATIONS == 24u);
    assert_near(limits.minNormalizedMainJacobian, 0.02f, 1e-7f);
    assert_near(limits.minDriveProjectionRatio, 0.02f, 1e-7f);
    assert_near(limits.minAbsVelocityRatio, 0.01f, 1e-7f);
    assert_near(limits.maxAbsVelocityRatio, 20.0f, 1e-6f);
    assert_near(limits.geometryMarginMm, 0.5f, 1e-7f);
    assert(limits.radicandTolerance > 0.0f);

    assert(HYD_ToggleKinematics_BeginValidation(&config, &limits, &validation, &error));
    assert(error == HYD_TOGGLE_ERROR_NONE);
    assert(validation.phase == HYD_TOGGLE_VALIDATION_SCAN);
    assert(!HYD_ToggleKinematics_ValidationDone(&validation));

    memset(&prepared, 0x69, sizeof(prepared));
    before = prepared;
    assert(!HYD_ToggleKinematics_FinishValidation(&validation, &prepared, &error));
    assert(memcmp(&prepared, &before, sizeof(prepared)) == 0);

    assert(HYD_ToggleKinematics_ValidationStep(&validation, 3u, &error));
    assert(error == HYD_TOGGLE_ERROR_NONE);
    assert(validation.nextPoint == 3u);
    assert(validation.phase == HYD_TOGGLE_VALIDATION_SCAN);

    while (!HYD_ToggleKinematics_ValidationDone(&validation)) {
        assert(HYD_ToggleKinematics_ValidationStep(&validation, 7u, &error));
        assert(error == HYD_TOGGLE_ERROR_NONE);
    }

    assert(validation.phase == HYD_TOGGLE_VALIDATION_COMPLETE);
    assert(HYD_ToggleKinematics_FinishValidation(&validation, &prepared, &error));
    assert(error == HYD_TOGGLE_ERROR_NONE);
    assert_near(prepared.xGeometryMin, 0.0f, 1e-6f);
    assert_near(prepared.xHandoffEffective, prepared.xGeometryMin, 1e-6f);
    assert(prepared.xsMin <= 63.808094f);
    assert(prepared.xsMax >= 63.808094f);

    assert(HYD_ToggleKinematics_InversePosition(&prepared, 63.808094f, &xm, &error));
    assert(error == HYD_TOGGLE_ERROR_NONE);
    assert_near(xm, 101.0f, 2e-4f);
}

static void test_full_stroke_unreachable_and_failure_output_preservation(void)
{
    HYD_ToggleGeometryConfig config = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_TogglePreparedConfig prepared_before;
    HYD_ToggleSolution solution;
    HYD_ToggleSolution before;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    config.dc = 400.0f;
    assert(HYD_ToggleKinematics_Prepare(&config, &prepared, &error));

    memset(&solution, 0x5a, sizeof(solution));
    before = solution;
    assert(!HYD_ToggleKinematics_SolveOnline(&prepared, 0.0f, 1.0f, &solution, &error));
    assert(error == HYD_TOGGLE_ERROR_MAIN_LINKAGE_UNREACHABLE);
    assert(memcmp(&solution, &before, sizeof(solution)) == 0);

    memset(&prepared, 0x7b, sizeof(prepared));
    prepared_before = prepared;
    assert(!HYD_ToggleKinematics_ValidateBlocking(&config, &prepared, &error));
    assert(error == HYD_TOGGLE_ERROR_MAIN_LINKAGE_UNREACHABLE);
    assert(memcmp(&prepared, &prepared_before, sizeof(prepared)) == 0);
}

static void test_analytic_ratio_matches_centered_difference(void)
{
    HYD_ToggleGeometryConfig config = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleSolution center;
    HYD_ToggleSolution lower;
    HYD_ToggleSolution upper;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL centered_difference;

    assert(HYD_ToggleKinematics_ValidateBlocking(&config, &prepared, &error));
    assert(HYD_ToggleKinematics_SolveOnline(&prepared, 100.0f, 1.0f, &center, &error));
    assert(HYD_ToggleKinematics_SolveOnline(&prepared, 99.99f, 0.0f, &lower, &error));
    assert(HYD_ToggleKinematics_SolveOnline(&prepared, 100.01f, 0.0f, &upper, &error));

    centered_difference = (upper.xs - lower.xs) / 0.02f;
    assert_near(center.velocityRatio, centered_difference, 0.002f);
}

static void test_sigma_c_controls_published_actuator_coordinate(void)
{
    HYD_ToggleGeometryConfig config = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_TogglePreparedConfig default_prepared;
    HYD_ToggleSolution default_solution;
    HYD_ToggleSolution geometric_solution;
    HYD_REAL inverse_xm = 0.0f;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    assert(HYD_ToggleKinematics_ValidateBlocking(&config, &prepared, &error));
    default_prepared = prepared;
    assert(HYD_ToggleKinematics_SolveOnline(&prepared, 101.0f, 10.0f,
                                            &default_solution, &error));
    assert(default_solution.velocityRatio > 0.0f);
    assert(default_solution.vs > 0.0f);

    config.sigmaC = (int8_t)1;
    assert(HYD_ToggleKinematics_ValidateBlocking(&config, &prepared, &error));
    assert(HYD_ToggleKinematics_SolveOnline(&prepared, 101.0f, 10.0f,
                                            &geometric_solution, &error));
    assert_near(default_solution.xs, -geometric_solution.xs, 2.0e-3f);
    assert_near(default_solution.velocityRatio,
                -geometric_solution.velocityRatio, 2.0e-4f);
    assert_near(default_solution.vs, -geometric_solution.vs, 2.0e-3f);
    assert_near(prepared.xsMin, -default_prepared.xsMax, 2.0e-3f);
    assert_near(prepared.xsMax, -default_prepared.xsMin, 2.0e-3f);
    assert_near(prepared.kMin, -default_prepared.kMax, 2.0e-3f);
    assert_near(prepared.kMax, -default_prepared.kMin, 2.0e-3f);
    assert(HYD_ToggleKinematics_InversePosition(
        &prepared, geometric_solution.xs, &inverse_xm, &error));
    assert_near(inverse_xm, 101.0f, 2.0e-4f);
}

static void test_automatic_and_explicit_handoff_bounds(void)
{
    HYD_ToggleGeometryConfig config = HYD_ToggleKinematics_DefaultConfig();
    HYD_ToggleValidationLimits limits = HYD_ToggleKinematics_DefaultValidationLimits();
    HYD_TogglePreparedConfig automatic;
    HYD_TogglePreparedConfig explicit_handoff;
    HYD_TogglePreparedConfig preserved;
    HYD_TogglePreparedConfig before;
    HYD_ToggleValidation validation;
    HYD_ToggleSolution solution;
    HYD_ToggleSolution solution_before;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL recovered_xm = 1234.0f;
    HYD_REAL chosen_handoff;
    HYD_UINT8 previous_refine_iteration;

    limits.maxAbsVelocityRatio = 1.0f;
    assert(HYD_ToggleKinematics_BeginValidation(&config, &limits, &validation, &error));
    while (validation.phase == HYD_TOGGLE_VALIDATION_SCAN) {
        assert(HYD_ToggleKinematics_ValidationStep(&validation, 5u, &error));
    }
    assert(validation.phase == HYD_TOGGLE_VALIDATION_REFINE);
    while (!HYD_ToggleKinematics_ValidationDone(&validation)) {
        previous_refine_iteration = validation.refineIteration;
        assert(HYD_ToggleKinematics_ValidationStep(&validation, 1u, &error));
        assert(validation.refineIteration <= (HYD_UINT8)(previous_refine_iteration + 1u));
    }
    assert(HYD_ToggleKinematics_FinishValidation(&validation, &automatic, &error));
    assert_near(automatic.xGeometryMin, 41.83056f, 0.002f);
    assert_near(automatic.xHandoffEffective, automatic.xGeometryMin, 1e-5f);

    chosen_handoff = automatic.xGeometryMin + 10.0f;
    config.xHandoff = chosen_handoff;
    assert(validate_with_limits(&config, &limits, &explicit_handoff, &error));
    assert_near(explicit_handoff.xHandoffEffective, chosen_handoff, 1e-5f);

    memset(&solution, 0x5c, sizeof(solution));
    solution_before = solution;
    assert(!HYD_ToggleKinematics_SolveOnline(&explicit_handoff,
                                             chosen_handoff - 0.25f,
                                             1.0f,
                                             &solution,
                                             &error));
    assert(error == HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE);
    assert(memcmp(&solution, &solution_before, sizeof(solution)) == 0);

    assert(HYD_ToggleKinematics_SolveOnline(&automatic,
                                            chosen_handoff - 0.25f,
                                            0.0f,
                                            &solution,
                                            &error));
    assert(!HYD_ToggleKinematics_InversePosition(&explicit_handoff,
                                                 solution.xs,
                                                 &recovered_xm,
                                                 &error));
    assert(error == HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE);
    assert_near(recovered_xm, 1234.0f, 0.0f);

    memset(&preserved, 0xa5, sizeof(preserved));
    before = preserved;
    config.xHandoff = automatic.xGeometryMin - 0.25f;
    assert(!validate_with_limits(&config, &limits, &preserved, &error));
    assert(error == HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE);
    assert(memcmp(&preserved, &before, sizeof(preserved)) == 0);

    config.xHandoff = config.sm + 0.25f;
    assert(!validate_with_limits(&config, &limits, &preserved, &error));
    assert(error == HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE);
    assert(memcmp(&preserved, &before, sizeof(preserved)) == 0);
}

static void test_nonmonotonic_and_unsafe_validation_protections(void)
{
    HYD_ToggleGeometryConfig config = HYD_ToggleKinematics_DefaultConfig();
    HYD_ToggleValidationLimits limits = HYD_ToggleKinematics_DefaultValidationLimits();
    HYD_TogglePreparedConfig prepared;
    HYD_TogglePreparedConfig before;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    memset(&prepared, 0x3c, sizeof(prepared));
    before = prepared;
    config.tauS = (int8_t)1;
    assert(!validate_with_limits(&config, &limits, &prepared, &error));
    assert(error == HYD_TOGGLE_ERROR_NONMONOTONIC);
    assert(memcmp(&prepared, &before, sizeof(prepared)) == 0);

    config = HYD_ToggleKinematics_DefaultConfig();
    limits.maxAbsVelocityRatio = 0.5f;
    assert(!validate_with_limits(&config, &limits, &prepared, &error));
    assert(error == HYD_TOGGLE_ERROR_VELOCITY_RATIO_UNSAFE);
    assert(memcmp(&prepared, &before, sizeof(prepared)) == 0);
}

static void test_invalid_sigma_c_is_rejected(void)
{
    HYD_ToggleGeometryConfig config = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    config.sigmaC = 0;
    assert(!HYD_ToggleKinematics_Prepare(&config, &prepared, &error));
    assert(error == HYD_TOGGLE_ERROR_INVALID_BRANCH);
}

static void test_inverse_rejects_outside_envelope_without_output_change(void)
{
    HYD_ToggleGeometryConfig config = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL xm = 1234.5f;

    assert(HYD_ToggleKinematics_ValidateBlocking(&config, &prepared, &error));
    assert(!HYD_ToggleKinematics_InversePosition(&prepared,
                                                  prepared.xsMax + 1.0f,
                                                  &xm,
                                                  &error));
    assert(error == HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE);
    assert_near(xm, 1234.5f, 0.0f);
}

static void test_inverse_uses_validated_increasing_direction(void)
{
    HYD_ToggleGeometryConfig config = HYD_ToggleKinematics_DefaultConfig();
    HYD_ToggleValidationLimits limits = HYD_ToggleKinematics_DefaultValidationLimits();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleSolution solution;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL xm = -1.0f;

    config.ld = 300.0f;
    config.sigmaK = (int8_t)1;
    assert(validate_with_limits(&config, &limits, &prepared, &error));
    assert(HYD_ToggleKinematics_SolveOnline(&prepared, 101.0f, 0.0f,
                                            &solution, &error));
    assert(solution.velocityRatio < 0.0f);
    assert(HYD_ToggleKinematics_InversePosition(&prepared, solution.xs,
                                                &xm, &error));
    assert_near(xm, 101.0f, 2e-4f);
}

static void test_inverse_accepts_single_point_handoff_interval(void)
{
    HYD_ToggleGeometryConfig config = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleSolution solution;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL xm = -1.0f;

    config.xHandoff = config.sm;
    assert(HYD_ToggleKinematics_ValidateBlocking(&config, &prepared, &error));
    assert(HYD_ToggleKinematics_SolveOnline(&prepared, config.sm, 0.0f,
                                            &solution, &error));
    assert(HYD_ToggleKinematics_InversePosition(&prepared, solution.xs,
                                                &xm, &error));
    assert_near(xm, config.sm, 0.0f);
}

static void test_explicit_handoff_is_exactly_validated_within_budget(void)
{
    HYD_ToggleGeometryConfig config = {
        0x1.50dd3p+7f, 0x1.470496p+7f, 0x1.575054p+6f,
        0x1.acd65cp+6f, 0x1.e883f2p+7f, 0x1.5b9af2p+6f,
        0x1.10a604p+6f, 0x1.6a408p+7f, 0x1.d0c9aap+7f,
        0x1.663c6ep+7f, (int8_t)-1, (int8_t)1, (int8_t)1,
        (int8_t)1
    };
    HYD_ToggleValidationLimits limits = HYD_ToggleKinematics_DefaultValidationLimits();
    HYD_ToggleValidation validation;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_UINT16 evaluations = 0u;

    limits.minNormalizedMainJacobian = 1.0e-6f;
    limits.minDriveProjectionRatio = 1.0e-6f;
    limits.minAbsVelocityRatio = 1.0e-6f;
    limits.maxAbsVelocityRatio = 0x1.ef6282p+1f;
    limits.geometryMarginMm = 0.0f;
    assert(HYD_ToggleKinematics_BeginValidation(&config, &limits,
                                                &validation, &error));

    while (!HYD_ToggleKinematics_ValidationDone(&validation)) {
        assert(evaluations < (HYD_TOGGLE_VALIDATION_POINTS + 1u));
        if (!HYD_ToggleKinematics_ValidationStep(&validation, 1u, &error)) {
            ++evaluations;
            assert(evaluations == (HYD_TOGGLE_VALIDATION_POINTS + 1u));
            assert(error == HYD_TOGGLE_ERROR_VELOCITY_RATIO_UNSAFE);
            assert(validation.phase == HYD_TOGGLE_VALIDATION_FAILED);
            return;
        }
        ++evaluations;
    }

    assert(0 && "explicit non-grid xHandoff must fail exact validation");
}

int main(void)
{
    test_validation_phase_ordinals_remain_stable();
    test_default_prepare();
    test_online_golden_points();
    test_default_validation_and_inverse_round_trip();
    test_full_stroke_unreachable_and_failure_output_preservation();
    test_analytic_ratio_matches_centered_difference();
    test_sigma_c_controls_published_actuator_coordinate();
    test_automatic_and_explicit_handoff_bounds();
    test_nonmonotonic_and_unsafe_validation_protections();
    test_invalid_sigma_c_is_rejected();
    test_inverse_rejects_outside_envelope_without_output_change();
    test_inverse_uses_validated_increasing_direction();
    test_inverse_accepts_single_point_handoff_interval();
    test_explicit_handoff_is_exactly_validated_within_budget();
    printf("toggle kinematics core tests passed\n");
    return 0;
}
