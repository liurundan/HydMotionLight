#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "pressure_model.h"
#include "rbf_pid.h"

static void test_du_normalization_is_configurable(void) {
    RBF_PID_Handle pid;

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetDuNormalization(&pid, 4.5f);

    assert(fabsf(pid.f_dd_press_prev - 4.5f) < 1.0e-6f);
}

static void test_first_order_benchmark_contract(void) {
    PressureModelParams params;

    PressureModel_InitParams(&params);
    params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    params.first_order_k_bar_per_rpm = 0.66f;
    params.first_order_tau_s = 1.0f;
    params.first_order_delay_s = 0.0f;

    assert(PressureModel_ValidateParams(&params));
    assert(fabsf(params.first_order_k_bar_per_rpm - 0.66f) < 1.0e-6f);
    assert(fabsf(params.first_order_tau_s - 1.0f) < 1.0e-6f);
    assert(fabsf(params.first_order_delay_s) < 1.0e-6f);
}

int main(void) {
    test_du_normalization_is_configurable();
    test_first_order_benchmark_contract();
    printf("RBF-PID contract tests passed.\n");
    return 0;
}
