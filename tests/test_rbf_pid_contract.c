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

static void test_effective_cap_and_shadow_are_side_effect_free(void) {
    RBF_PID_Handle pid;
    RBF_PID_ShadowState shadow = {0};
    float output_before;

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    output_before = pid.Output;
    RBF_PID_SetEffectiveUpperCap(&pid, 4.25f, true);
    assert(pid.effective_upper_cap_valid);
    assert(fabsf(pid.effective_upper_cap - 4.25f) < 1.0e-6f);

    RBF_PID_ShadowUpdate(&shadow, &pid, 10.0f, 8.0f, 1.0f, 0.001f, true);
    assert(shadow.valid);
    assert(shadow.valid_sample_count == 1U);
    assert(fabsf(shadow.residual + 2.0f) < 1.0e-6f);
    assert(pid.Output == output_before);

    RBF_PID_ShadowUpdate(&shadow, &pid, 10.0f, 8.0f, 1.0f, 0.0f, false);
    assert(!shadow.valid);
    assert(shadow.invalid_sample_count == 1U);
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
    test_effective_cap_and_shadow_are_side_effect_free();
    test_first_order_benchmark_contract();
    printf("RBF-PID contract tests passed.\n");
    return 0;
}
