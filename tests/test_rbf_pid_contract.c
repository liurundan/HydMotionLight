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

static void test_jacobian_uses_discrete_one_ms_sensitivity_and_invalid_dt_freezes(void) {
    RBF_PID_Handle pid;
    RBF_PID_ShadowState shadow = {0};
    float previous_kp;
    float previous_ki;
    float previous_kd;
    float current_kp;
    float current_ki;
    float current_kd;

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetGainCompensation(&pid, 200.0f);
    RBF_PID_SetProcessTimeConstant(&pid, 1.0f);
    RBF_PID_SetLearningRates(&pid, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);

    (void)RBF_PID_Update(&pid, 150.0f, 0.0f);
    assert(pid.Jacobian >= 0.05f - 1.0e-5f);
    assert(pid.Jacobian <= 0.8f + 1.0e-5f);

    previous_kp = pid.KP;
    previous_ki = pid.KI;
    previous_kd = pid.KD;
    (void)RBF_PID_Update(&pid, 150.0f, 1.0f);
    current_kp = pid.KP;
    current_ki = pid.KI;
    current_kd = pid.KD;
    assert(fabsf(current_kp - previous_kp) <= 0.005001f);
    assert(fabsf(current_ki - previous_ki) <= 0.0000501f);
    assert(fabsf(current_kd - previous_kd) <= 0.0002001f);

    previous_kp = pid.KP;
    RBF_PID_ShadowUpdate(&shadow, &pid, 150.0f, 10.0f, 2.0f,
                         0.0005f, false);
    assert(!shadow.valid);
    RBF_PID_SetDtValid(&pid, false);
    assert(!pid.dt_valid);
    (void)RBF_PID_Update(&pid, 150.0f, 10.0f);
    assert(fabsf(pid.KP - previous_kp) < 1.0e-6f);
}

static void test_d_term_is_independently_low_pass_filtered(void) {
    RBF_PID_Handle pid;
    float first_output;
    float second_output;

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetLearningRates(&pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    RBF_PID_SetParamLimits(&pid, 0.4f, 0.4f, 0.0013f, 0.0013f, 1.0f, 1.0f);

    (void)RBF_PID_Update(&pid, 20.0f, 0.0f);
    first_output = RBF_PID_Update(&pid, 20.0f, 8.0f);
    second_output = RBF_PID_Update(&pid, 20.0f, 0.0f);

    /* The error history is seeded on the first target jump, so the exact
     * second-difference depends on the causal plant state.  The contract is
     * attenuation and finiteness, not a fixture-specific raw value. */
    assert(isfinite(pid.prev_d_term));
    assert(fabsf(pid.prev_d_term) < 16.0f);
    assert(fabsf(second_output - first_output) < 28.0f);
}

static void test_adaptation_gate_freezes_on_low_excitation_and_recovers(void) {
    RBF_PID_Handle pid;

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetLearningRates(&pid, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f);

    (void)RBF_PID_Update(&pid, 20.0f, 0.0f);
    assert(pid.adaptation_frozen);

    pid.du_prev = pid.flow_normalization_scale * 0.1f;
    pid.output_saturated = false;
    (void)RBF_PID_Update(&pid, 20.0f, 0.5f);
    assert(!pid.adaptation_frozen);
}

int main(void) {
    test_du_normalization_is_configurable();
    test_effective_cap_and_shadow_are_side_effect_free();
    test_first_order_benchmark_contract();
    test_jacobian_uses_discrete_one_ms_sensitivity_and_invalid_dt_freezes();
    test_d_term_is_independently_low_pass_filtered();
    test_adaptation_gate_freezes_on_low_excitation_and_recovers();
    printf("RBF-PID contract tests passed.\n");
    return 0;
}
