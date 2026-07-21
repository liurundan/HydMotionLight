#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "rbf_pid.h"

static void test_flow_normalization_and_system_gain_soft_cap_are_configurable(void);
static void test_flow_domain_output_is_independent_from_pump_gain(void);
static void test_rbf_input_uses_causal_history_and_split_normalization(void);
static void test_pressure_accel_feedforward_toggle_changes_incremental_output(void);
static void test_pressure_accel_feedforward_is_suppressed_inside_near_target_band(void);
static void test_pressure_accel_feedforward_remains_active_outside_near_target_band(void);
static void test_target_relative_small_error_reduces_gain_drift(void);
static void test_control_mode_round_trip_restores_pid_configuration(void);

static void test_init_sets_ready_defaults(void) {
    RBF_PID_Handle pid;

    printf("Testing RBF_PID initialization defaults...\n");
    RBF_PID_Init(&pid, 0.01f, 90.0f, 1.0f);

    assert(pid.Status == 1);
    assert(fabsf(pid.sampling_period - 0.01f) < 1e-6f);
    assert(fabsf(pid.fMaxFlow - 90.0f) < 1e-6f);
    assert(fabsf(pid.fFlowRateLimit - 1.0f) < 1e-6f);
    assert(fabsf(pid.pressure_normalization_scale - MAX_PRESSURE) < 1e-6f);
    assert(fabsf(pid.flow_normalization_scale - 90.0f) < 1e-6f);
    assert(!pid.gain_compensation_enabled);
    assert(fabsf(pid.gain_compensation_factor - 1.0f) < 1e-6f);
    assert(fabsf(pid.KP - 0.04f) < 1e-6f);
    assert(fabsf(pid.KI - PID_MIN_KI) < 1e-6f);
    assert(fabsf(pid.KD - 0.020f) < 1e-6f);
    assert(fabsf(pid.Output) < 1e-6f);
    assert(fabsf(pid.u_prev) < 1e-6f);
    assert(fabsf(pid.e_prev1) < 1e-6f);
    assert(fabsf(pid.e_prev2) < 1e-6f);
    assert(!pid.output_saturated);
    printf("✓ RBF_PID initialization defaults test passed\n");
}

static void test_control_mode_round_trip_restores_pid_configuration(void) {
    RBF_PID_Handle pid;

    printf("Testing RBF PI/PID mode round-trip configuration...\n");
    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetParamLimits(&pid, 0.01f, 2.0f, 0.001f, 1.0f, 0.0f, 2.0f);
    RBF_PID_SetLearningRates(&pid, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.3f);
    RBF_PID_SetPressureAccelFeedforwardEnabled(&pid, true);
    pid.KD = 0.75f;

    RBF_PID_SetControlMode(&pid, RBF_PID_CONTROL_MODE_PI);
    assert(pid.control_mode == RBF_PID_CONTROL_MODE_PI);
    assert(pid.KD == 0.0f);
    assert(pid.eta_d == 0.0f);
    assert(!pid.pressure_accel_ff_enabled);

    RBF_PID_SetLearningRates(&pid, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.4f);
    RBF_PID_SetPressureAccelFeedforwardEnabled(&pid, true);
    RBF_PID_SetControlMode(&pid, RBF_PID_CONTROL_MODE_PID);

    assert(pid.control_mode == RBF_PID_CONTROL_MODE_PID);
    assert(fabsf(pid.KD - 0.75f) < 1e-6f);
    assert(fabsf(pid.eta_d - 0.4f) < 1e-6f);
    assert(pid.pressure_accel_ff_enabled);
    printf("✓ RBF PI/PID mode round-trip configuration test passed\n");
}

static void test_pid_saturation_does_not_freeze_network_learning(void) {
    RBF_PID_Handle pid;
    float weights_before[RBF_HNUM];
    bool changed = false;
    int i;

    printf("Testing legacy RBF-PID network learning under saturation...\n");
    RBF_PID_Init(&pid, 0.001f, 10.0f, 1.0f);
    RBF_PID_SetLearningRates(&pid, 0.2f, 0.2f, 0.2f,
                             0.1f, 0.1f, 0.1f);
    pid.Output = 10.0f;
    pid.u_prev = 10.0f;
    pid.output_saturated = true;
    for (i = 0; i < RBF_HNUM; ++i) {
        weights_before[i] = pid.w[i];
    }

    (void)RBF_PID_Update(&pid, 100.0f, 20.0f);

    for (i = 0; i < RBF_HNUM; ++i) {
        if (fabsf(pid.w[i] - weights_before[i]) > 1.0e-8f) {
            changed = true;
        }
    }
    assert(changed);
    printf("✓ Legacy RBF-PID network learning under saturation test passed\n");
}

static void test_enabled_controller_respects_limits_and_drives_feedback(void) {
    RBF_PID_Handle pid;
    float feedback = 0.0f;
    float output = 0.0f;
    float max_flow_output;
    int step;

    printf("Testing RBF_PID closed-loop adaptation behavior...\n");
    RBF_PID_Init(&pid, 0.01f, 90.0f, 1.0f);
    RBF_PID_SetParamLimits(&pid, 0.5f, 1.2f, 0.005f, 0.050f, 0.5f, 2.0f);
    max_flow_output = pid.fMaxFlow * pid.fFlowRateLimit;

    for (step = 0; step < 20; ++step) {
        output = RBF_PID_Update(&pid, 100.0f, feedback);
        assert(fabsf(output - pid.Output) < 1e-6f);
        assert(fabsf(pid.n_out - output) < 1e-6f);
        assert(output >= MIN_OUTPUT - 1e-3f);
        assert(output <= pid.fMaxFlow * pid.fFlowRateLimit + 1e-3f);
        assert(pid.n_out >= MIN_OUTPUT - 1e-6f);
        assert(pid.n_out <= max_flow_output + 1e-6f);
        assert(pid.KP >= pid.min_KP - 1e-6f && pid.KP <= pid.max_KP + 1e-6f);
        assert(pid.KI >= pid.min_KI - 1e-6f && pid.KI <= pid.max_KI + 1e-6f);
        assert(pid.KD >= pid.min_KD - 1e-6f && pid.KD <= pid.max_KD + 1e-6f);

        feedback += output * 5.0f;
        if (feedback > 100.0f) {
            feedback = 100.0f;
        }
    }

    assert(pid.Status == 2 || pid.Status == 3);
    assert(fabsf(feedback) > 1.0f);
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
    assert(fabsf(pid.flow_normalization_scale - 90.0f) < 1e-6f);
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
    /* KP should not drift much once error is small and learning is restrained */
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

    assert(fabsf(pid.min_KP - PID_MIN_KP) < 1e-6f);
    assert(fabsf(pid.max_KP - PID_MAX_KP) < 1e-6f);
    assert(kp_final >= pid.min_KP - 1e-6f && kp_final <= pid.max_KP + 1e-6f);

    assert(fabsf(kp_final - kp_initial) > 0.00001f);

    printf("✓ Default gain window adaptation test passed (window=%.3f KP: %.4f->%.4f)\n",
           pid.max_KP - pid.min_KP, kp_initial, kp_final);
}

static void test_flow_normalization_and_system_gain_soft_cap_are_configurable(void) {
    RBF_PID_Handle pid;
    float output;

    printf("Testing flow normalization and system-gain soft cap configuration...\n");
    RBF_PID_Init(&pid, 0.01f, 90.0f, 1.0f);
    RBF_PID_SetFlowNormalization(&pid, 45.0f);
    RBF_PID_SetPressureNormalization(&pid, 180.0f);
    RBF_PID_SetGainCompensation(&pid, 60.0f);
    RBF_PID_SetLearningRates(&pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    assert(fabsf(pid.flow_normalization_scale - 45.0f) < 1e-6f);
    assert(fabsf(pid.pressure_normalization_scale - 180.0f) < 1e-6f);
    assert(pid.gain_compensation_enabled);
    assert(fabsf(pid.gain_compensation_factor - 1.0f) < 1e-6f);

    output = RBF_PID_Update(&pid, 120.0f, 0.0f);
    assert(output <= (120.0f * 1.05f / 60.0f) + 1e-3f);

    RBF_PID_SetGainCompensation(&pid, 0.0f);
    assert(!pid.gain_compensation_enabled);
    assert(fabsf(pid.gain_compensation_factor - 1.0f) < 1e-6f);
    printf("PASS flow normalization / system-gain soft cap test\n");
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

    RBF_PID_Init(&enabled, 0.001f, 120.0f, 1.0f);
    RBF_PID_Init(&disabled, 0.001f, 120.0f, 1.0f);
    RBF_PID_SetFlowNormalization(&enabled, 120.0f);
    RBF_PID_SetFlowNormalization(&disabled, 120.0f);
    RBF_PID_SetLearningRates(&enabled, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    RBF_PID_SetLearningRates(&disabled, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    (void)RBF_PID_Update(&enabled, 20.0f, 10.0f);
    (void)RBF_PID_Update(&disabled, 20.0f, 10.0f);

    RBF_PID_SetPressureAccelFeedforwardEnabled(&disabled, false);

    out_enabled = RBF_PID_Update(&enabled, 20.0f, 11.0f);
    out_disabled = RBF_PID_Update(&disabled, 20.0f, 11.0f);

    assert(fabsf(out_enabled - out_disabled) > 1e-5f);
    printf("✓ Pressure acceleration feedforward toggle test passed\n");
}

static void test_pressure_accel_feedforward_is_suppressed_inside_near_target_band(void) {
    RBF_PID_Handle enabled;
    RBF_PID_Handle disabled;
    float out_enabled;
    float out_disabled;

    printf("Testing near-target pressure acceleration feedforward suppression...\n");

    RBF_PID_Init(&enabled, 0.001f, 90.0f, 1.0f);
    RBF_PID_Init(&disabled, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetLearningRates(&enabled, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    RBF_PID_SetLearningRates(&disabled, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    (void)RBF_PID_Update(&enabled, 100.0f, 70.0f);
    (void)RBF_PID_Update(&disabled, 100.0f, 70.0f);

    RBF_PID_SetPressureAccelFeedforwardEnabled(&disabled, false);

    out_enabled = RBF_PID_Update(&enabled, 100.0f, 98.5f);
    out_disabled = RBF_PID_Update(&disabled, 100.0f, 98.5f);

    assert(fabsf(out_enabled - out_disabled) < 1e-6f);
    printf("PASS near-target feedforward suppression test\n");
}

static void test_pressure_accel_feedforward_remains_active_outside_near_target_band(void) {
    RBF_PID_Handle enabled;
    RBF_PID_Handle disabled;
    float out_enabled;
    float out_disabled;

    printf("Testing outside-band pressure acceleration feedforward activity...\n");

    RBF_PID_Init(&enabled, 0.001f, 90.0f, 1.0f);
    RBF_PID_Init(&disabled, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetLearningRates(&enabled, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    RBF_PID_SetLearningRates(&disabled, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    (void)RBF_PID_Update(&enabled, 1.0f, 0.80f);
    (void)RBF_PID_Update(&disabled, 1.0f, 0.80f);

    RBF_PID_SetPressureAccelFeedforwardEnabled(&disabled, false);

    out_enabled = RBF_PID_Update(&enabled, 1.0f, 0.96f);
    out_disabled = RBF_PID_Update(&disabled, 1.0f, 0.96f);

    assert(fabsf(out_enabled - out_disabled) > 1e-5f);
    printf("PASS outside-band feedforward activity test\n");
}

static void test_flow_domain_output_is_independent_from_pump_gain(void) {
    RBF_PID_Handle base;
    RBF_PID_Handle altered;
    float out_base;
    float out_altered;

    printf("Testing flow-domain controller independence from pump-speed gain...\n");
    RBF_PID_Init(&base, 0.001f, 90.0f, 1.0f);
    RBF_PID_Init(&altered, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetLearningRates(&base, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    RBF_PID_SetLearningRates(&altered, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    altered.flowToPumpSpeedGain = 37.0f;

    (void)RBF_PID_Update(&base, 80.0f, 20.0f);
    (void)RBF_PID_Update(&altered, 80.0f, 20.0f);

    out_base = RBF_PID_Update(&base, 80.0f, 25.0f);
    out_altered = RBF_PID_Update(&altered, 80.0f, 25.0f);

    assert(fabsf(out_base - out_altered) < 1e-6f);
    assert(fabsf(base.n_out - out_base) < 1e-6f);
    assert(fabsf(altered.n_out - out_altered) < 1e-6f);
    printf("PASS flow-domain controller independence test\n");
}

static void test_rbf_input_uses_causal_history_and_split_normalization(void) {
    RBF_PID_Handle pid;
    float prev_du;

    printf("Testing RBF causal input vector and split normalization...\n");
    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetFlowNormalization(&pid, 45.0f);
    RBF_PID_SetPressureNormalization(&pid, 200.0f);
    RBF_PID_SetLearningRates(&pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    (void)RBF_PID_Update(&pid, 100.0f, 40.0f);
    (void)RBF_PID_Update(&pid, 100.0f, 55.0f);
    prev_du = pid.du_prev;
    (void)RBF_PID_Update(&pid, 100.0f, 60.0f);

    assert(RBF_INPUT_DIM == 3);
    assert(fabsf(pid.last_rbf_input[0] - (prev_du / 45.0f)) < 1e-6f);
    assert(fabsf(pid.last_rbf_input[1] - (55.0f / 200.0f)) < 1e-6f);
    assert(fabsf(pid.last_rbf_input[2] - (40.0f / 200.0f)) < 1e-6f);
    assert(fabsf(pid.last_rbf_input[0] - (prev_du / 200.0f)) > 1e-4f);
    printf("PASS RBF causal input vector test\n");
}

static void test_target_relative_small_error_reduces_gain_drift(void) {
    RBF_PID_Handle pid;
    float kp_before;
    float ki_before;
    float kd_before;
    int step;

    printf("Testing target-relative small-error learning restraint...\n");
    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetParamLimits(&pid, 0.030f, 0.090f, 0.0005f, 0.0040f, 0.010f, 0.080f);

    for (step = 0; step < 100; ++step) {
        (void)RBF_PID_Update(&pid, 200.0f, 150.0f + (float)step * 0.4f);
    }

    kp_before = pid.KP;
    ki_before = pid.KI;
    kd_before = pid.KD;

    for (step = 0; step < 200; ++step) {
        (void)RBF_PID_Update(&pid, 200.0f, 198.5f);
    }

    assert(fabsf(pid.KP - kp_before) < 0.0025f);
    assert(fabsf(pid.KI - ki_before) < 0.0003f);
    assert(fabsf(pid.KD - kd_before) < 0.0030f);
    printf("PASS target-relative small-error learning restraint test\n");
}

int main(void) {
    printf("Running RBF_PID tests...\n\n");

    test_init_sets_ready_defaults();
    test_control_mode_round_trip_restores_pid_configuration();
    test_pid_saturation_does_not_freeze_network_learning();
    test_enabled_controller_respects_limits_and_drives_feedback();
    test_explicit_reset_restores_runtime_state();
    test_adaptive_learning_rate_scales_with_error();
    test_default_gain_window_allows_adaptation();
    test_flow_normalization_and_system_gain_soft_cap_are_configurable();
    test_flow_domain_output_is_independent_from_pump_gain();
    test_rbf_input_uses_causal_history_and_split_normalization();
    test_network_initialization_is_deterministic_without_seed_hookup();
    test_rbf_pid_negative_output();
    test_pressure_accel_feedforward_toggle_changes_incremental_output();
    test_pressure_accel_feedforward_is_suppressed_inside_near_target_band();
    test_pressure_accel_feedforward_remains_active_outside_near_target_band();
    test_target_relative_small_error_reduces_gain_drift();

    printf("\n✅ All RBF_PID tests passed successfully!\n");
    return 0;
}
