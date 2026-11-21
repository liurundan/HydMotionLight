#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "pressure_model.h"
#include "rbf_pid.h"

static void test_fixed_sampling_period_is_used(void) {
    RBF_PID_Handle pid;

    RBF_PID_Init(&pid, 0.010f, 90.0f, 1.0f);
    assert(fabsf(pid.sampling_period - RBF_PID_FIXED_SAMPLING_PERIOD) < 1.0e-6f);
}

static void test_first_order_model_contract(void) {
    PressureModelParams params;

    PressureModel_InitParams(&params);
    params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    params.first_order_k_bar_per_rpm = 0.66f;
    params.first_order_tau_s = 1.0f;
    params.first_order_delay_s = 0.0f;
    assert(PressureModel_ValidateParams(&params));
}

int main(void) {
    test_fixed_sampling_period_is_used();
    test_first_order_model_contract();
    puts("RBF-PID contract tests passed.");
    return 0;
}
