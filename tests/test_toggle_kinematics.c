#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "toggle_kinematics.h"

static void assert_near(HYD_REAL actual, HYD_REAL expected, HYD_REAL tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

static void test_default_prepare(void)
{
    HYD_ToggleGeometryConfig config = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    assert_near(config.dc, 378.0f, 1e-5f);
    assert(HYD_ToggleKinematics_Prepare(&config, &prepared, &error));
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

static void test_online_golden_points(void)
{
    HYD_ToggleGeometryConfig config = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    assert(HYD_ToggleKinematics_Prepare(&config, &prepared, &error));
    assert(error == HYD_TOGGLE_ERROR_NONE);

    assert_solution_at(&prepared, 0.0f, 64.910771f, -10.150074f);
    assert_solution_at(&prepared, 50.0f, -20.397682f, -0.935184f);
    assert_solution_at(&prepared, 101.0f, -63.808094f, -0.808380f);
    assert_solution_at(&prepared, 202.0f, -138.295657f, -0.520517f);
}

int main(void)
{
    test_default_prepare();
    test_online_golden_points();
    printf("toggle kinematics core tests passed\n");
    return 0;
}
