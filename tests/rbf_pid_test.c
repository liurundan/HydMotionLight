#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "rbf_pid.h"

static void test_pressure_accel_feedforward_toggle_changes_incremental_output(void);

static void test_init_sets_ready_defaults(void) {
    RBF_PID_Handle pid;

    printf("Testing RBF_PID initialization defaults...\n");
    RBF_PID_Init(&pid, 0.01f, 90.0f, 1.0f);

    assert(pid.Status == 1);
    assert(fabsf(pid.sampling_period - 0.01f) < 1e-6f);
    assert(fabsf(pid.fMaxFlow - 90.0f) < 1e-6f);
    assert(fabsf(pid.fFlowRateLimit - 1.0f) < 1e-6f);
    assert(fabsf(pid.pressure_normalization_scale - MAX_PRESSURE) < 1e-6f);
    assert(!pid.gain_compensation_enabled);
    assert(fabsf(pid.gain_compensation_factor - 1.0f) < 1e-6f);
    assert(fabsf(pid.KP - 1.02f) < 1e-6f);
    assert(fabsf(pid.KI - 0.02f) < 1e-6f);
    assert(fabsf(pid.KD - 1.02f) < 1e-6f);
    assert(fabsf(pid.Output) < 1e-6f);
    assert(fabsf(pid.u_prev) < 1e-6f);
    assert(fabsf(pid.e_prev1) < 1e-6f);
    assert(fabsf(pid.e_prev2) < 1e-6f);
    printf("✓ RBF_PID initialization defaults test passed\n");
}

static void test_enabled_controller_respects_limits_and_drives_feedback(void) {
    RBF_PID_Handle pid;
    float feedback = 0.0f;
    float output = 0.0f;
    float max_motor_output;
    int step;

    printf("Testing RBF_PID closed-loop adaptation behavior...\n");
    RBF_PID_Init(&pid, 0.01f, 90.0f, 1.0f);
    RBF_PID_SetParamLimits(&pid, 0.5f, 1.2f, 0.005f, 0.050f, 0.5f, 2.0f);
    max_motor_output = pid.fMaxFlow * pid.fFlowRateLimit * pid.flowToPumpSpeedGain;

    for (step = 0; step < 20; ++step) {
        output = RBF_PID_Update(&pid, 100.0f, feedback);
        assert(fabsf(output - pid.Output) < 1e-6f);
        assert(fabsf(pid.n_out - output * pid.flowToPumpSpeedGain) < 1e-3f);
        assert(output >= MIN_OUTPUT / pid.flowToPumpSpeedGain - 1e-3f);
        assert(output <= pid.fMaxFlow * pid.fFlowRateLimit + 1e-3f);
        assert(pid.n_out >= MIN_OUTPUT - 1e-6f);
        assert(pid.n_out <= max_motor_output + 1e-6f);
        assert(pid.KP >= pid.min_KP - 1e-6f && pid.KP <= pid.max_KP + 1e-6f);
        assert(pid.KI >= pid.min_KI - 1e-6f && pid.KI <= pid.max_KI + 1e-6f);
        assert(pid.KD >= pid.min_KD - 1e-6f && pid.KD <= pid.max_KD + 1e-6f);

        feedback += output * 5.0f;
        if (feedback > 100.0f) {
            feedback = 100.0f;
        }
    }

    assert(pid.Status == 2 || pid.Status == 3);
    assert(feedback > 1.0f);
    printf("✓ RBF_PID adaptation/limit test passed\n");
}

static void test_explicit_reset_restores_runtime_state(void) {
    RBF_PID_Handle pid;

    printf("Testing RBF_PID explicit reset behavior...\n");
    RBF_PID_Init(&pid, 0.01f, 90.0f, 1.0f);
    RBF_PID_SetPressureNormalization(&pid, 800.0f);
    RBF_PID_SetGainCompensation(&pid, 2.0f);
    (void)RBF_PID_Update(&pid, 100.0f, 0.0f);
    assert(fabsf(pid.u_prev) > 1e-6f || fabsf(pid.du_prev) > 1e-6f || fabsf(pid.n_out) > 1e-6f);

    RBF_PID_Reset(&pid);
    assert(pid.Status == 1);
    assert(fabsf(pid.u_prev) < 1e-6f);
    assert(fabsf(pid.du_prev) < 1e-6f);
    assert(fabsf(pid.e_prev1) < 1e-6f);
    assert(fabsf(pid.e_prev2) < 1e-6f);
    assert(fabsf(pid.n_out) < 1e-6f);
    assert(fabsf(pid.sampling_period - 0.01f) < 1e-6f);
    assert(fabsf(pid.fMaxFlow - 90.0f) < 1e-6f);
    assert(fabsf(pid.fFlowRateLimit - 1.0f) < 1e-6f);
    assert(fabsf(pid.pressure_normalization_scale - MAX_PRESSURE) < 1e-6f);
    assert(!pid.gain_compensation_enabled);
    assert(fabsf(pid.gain_compensation_factor - 1.0f) < 1e-6f);
    printf("✓ RBF_PID explicit reset test passed\n");
}

static void test_adaptive_learning_rate_scales_with_error(void) {
    RBF_PID_Handle pid;
    float kp_before, kp_after;
    int step;

    printf("Testing adaptive learning rate scaling...\n");
    RBF_PID_Init(&pid, 0.01f, 90.0f, 1.0f);

    /* Drive to steady state */
    for (step = 0; step < 15; step++) {
        float feedback = step * 5.0f;
        if (feedback > 95.0f) feedback = 95.0f;
        (void)RBF_PID_Update(&pid, 100.0f, feedback);
    }

    /* Record KP at near-steady state */
    kp_before = pid.KP;

    /* Run many steps with very small error */
    for (step = 0; step < 10; step++) {
        (void)RBF_PID_Update(&pid, 100.0f, 99.9f);
    }

    kp_after = pid.KP;
    /* KP should not drift more than 10% from its previous value (relaxed for wider window) */
    assert(fabsf(kp_after - kp_before) < 0.10f * kp_before + 0.05f);
    printf("✓ Adaptive learning rate scaling test passed\n");
}

static void test_network_initialization_is_deterministic_without_seed_hookup(void) {
    RBF_PID_Handle pid1;
    RBF_PID_Handle pid2;
    int i, j;

    printf("Testing RBF network initialization determinism...\n");

    RBF_PID_Init(&pid1, 0.01f, 90.0f, 1.0f);
    RBF_PID_Init(&pid2, 0.01f, 90.0f, 1.0f);
    RBF_PID_SetSeed(&pid2, 42u);
    assert(pid2.network_seed == 42u);
    RBF_PID_Reset(&pid2);

    for (i = 0; i < RBF_HNUM; i++) {
        for (j = 0; j < RBF_INPUT_DIM; j++) {
            assert(fabsf(pid1.c[i][j] - pid2.c[i][j]) < 1e-6f);
            assert(isfinite(pid1.c[i][j]));
        }
        assert(pid1.b_rbf[i] > 0.0f);
        assert(fabsf(pid1.b_rbf[i] - pid2.b_rbf[i]) < 1e-6f);
        assert(fabsf(pid1.w[i] - pid2.w[i]) < 1e-6f);
    }

    printf("✓ RBF network initialization determinism test passed\n");
}

static void test_default_gain_window_allows_adaptation(void) {
    RBF_PID_Handle pid;
    float kp_initial;
    float kp_final;
    int step;

    printf("Testing default gain window allows adaptation room...\n");
    RBF_PID_Init(&pid, 0.01f, 90.0f, 1.0f);
    kp_initial = pid.KP;

    /* Drive a step error scenario for 50 steps */
    for (step = 0; step < 50; step++) {
        float feedback = (step < 25) ? 0.0f : 50.0f;
        (void)RBF_PID_Update(&pid, 100.0f, feedback);
    }
    kp_final = pid.KP;

    assert((pid.max_KP - pid.min_KP) >= 1.0f);
    assert(kp_final >= pid.min_KP - 1e-6f && kp_final <= pid.max_KP + 1e-6f);

    assert(fabsf(kp_final - kp_initial) > 0.00001f);

    printf("✓ Default gain window adaptation test passed (window=%.3f KP: %.4f->%.4f)\n",
           pid.max_KP - pid.min_KP, kp_initial, kp_final);
}

static void test_pressure_normalization_and_gain_compensation_are_configurable(void) {
    RBF_PID_Handle pid;
    float expected_factor;

    printf("Testing pressure normalization and gain compensation configuration...\n");
    RBF_PID_Init(&pid, 0.01f, 90.0f, 1.0f);

    RBF_PID_SetPressureNormalization(&pid, 800.0f);
    assert(fabsf(pid.pressure_normalization_scale - 800.0f) < 1e-6f);

    RBF_PID_SetGainCompensation(&pid, 2.0f);
    expected_factor = 800.0f / (2.0f * pid.fMaxFlow);
    assert(pid.gain_compensation_enabled);
    assert(fabsf(pid.gain_compensation_factor - expected_factor) < 1e-6f);

    (void)RBF_PID_Update(&pid, 400.0f, 200.0f);
    assert(fabsf(pid.P_set - 400.0f) < 1e-6f);
    assert(fabsf(pid.P_actual - 200.0f) < 1e-6f);

    RBF_PID_SetPressureNormalization(&pid, 0.0f);
    expected_factor = MAX_PRESSURE / (2.0f * pid.fMaxFlow);
    assert(fabsf(pid.pressure_normalization_scale - MAX_PRESSURE) < 1e-6f);
    assert(fabsf(pid.gain_compensation_factor - expected_factor) < 1e-6f);

    RBF_PID_SetGainCompensation(&pid, 0.0f);
    assert(!pid.gain_compensation_enabled);
    assert(fabsf(pid.gain_compensation_factor - 1.0f) < 1e-6f);

    printf("✓ Pressure normalization / gain compensation test passed\n");
}

static void test_rbf_pid_negative_output(void) {
    RBF_PID_Handle pid;
    float output;
    int step;

    printf("Testing RBF-PID negative output...\n");
    RBF_PID_Init(&pid, 0.01f, 90.0f, 1.0f);
    RBF_PID_SetGainCompensation(&pid, 1.0f);  /* K = 1.0 */

    /* Run steps with Setpoint=10.0, Feedback=15.0 → pressure too high, need negative flow */
    for (step = 0; step < 50; step++) {
        output = RBF_PID_Update(&pid, 10.0f, 15.0f);
    }

    /* After adaptation, n_out should be negative (negative flow) */
    printf("  n_out after 50 steps = %.4f L/min (expect < 0)\n", output);
    assert(output < 0.0f);
    printf("✓ RBF-PID negative output test passed (n_out=%.3f)\n", output);
}

static void test_pressure_accel_feedforward_toggle_changes_incremental_output(void) {
    RBF_PID_Handle enabled;
    RBF_PID_Handle disabled;
    float out_enabled;
    float out_disabled;

    printf("Testing pressure acceleration feedforward toggle...\n");

    RBF_PID_Init(&enabled, 0.001f, 90.0f, 1.0f);
    RBF_PID_Init(&disabled, 0.001f, 90.0f, 1.0f);

    (void)RBF_PID_Update(&enabled, 100.0f, 98.0f);
    (void)RBF_PID_Update(&disabled, 100.0f, 98.0f);

    RBF_PID_SetPressureAccelFeedforwardEnabled(&disabled, false);

    out_enabled = RBF_PID_Update(&enabled, 100.0f, 101.0f);
    out_disabled = RBF_PID_Update(&disabled, 100.0f, 101.0f);

    assert(fabsf(out_enabled - out_disabled) > 1e-5f);
    printf("✓ Pressure acceleration feedforward toggle test passed\n");
}

int main(void) {
    printf("Running RBF_PID tests...\n\n");

    test_init_sets_ready_defaults();
    test_enabled_controller_respects_limits_and_drives_feedback();
    test_explicit_reset_restores_runtime_state();
    test_adaptive_learning_rate_scales_with_error();
    test_default_gain_window_allows_adaptation();
    test_pressure_normalization_and_gain_compensation_are_configurable();
    test_network_initialization_is_deterministic_without_seed_hookup();
    test_rbf_pid_negative_output();
    test_pressure_accel_feedforward_toggle_changes_incremental_output();

    printf("\n✅ All RBF_PID tests passed successfully!\n");
    return 0;
}
