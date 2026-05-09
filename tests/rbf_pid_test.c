#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "rbf_pid.h"

static void test_disabled_controller_returns_zero_output(void) {
    RBF_PID_Handle pid;
    float output;

    printf("Testing RBF_PID disabled-output semantics...\n");
    RBF_PID_Init(&pid, 0.01f, 1500.0f, 1.0f);
    pid.enable = false;

    output = RBF_PID_Update(&pid, 100.0f, 0.0f);
    assert(fabsf(output) < 1e-6f);
    assert(pid.Status == -1);
    printf("✓ RBF_PID disabled-output test passed\n");
}

static void test_enabled_controller_respects_limits_and_drives_feedback(void) {
    RBF_PID_Handle pid;
    float feedback = 0.0f;
    float output = 0.0f;
    int step;

    printf("Testing RBF_PID closed-loop adaptation behavior...\n");
    RBF_PID_Init(&pid, 0.01f, 1500.0f, 1.0f);
    pid.enable = true;
    RBF_PID_SetParamLimits(&pid, 0.80f, 0.90f, 0.018f, 0.040f, 1.20f, 1.60f);

    for (step = 0; step < 20; ++step) {
        output = RBF_PID_Update(&pid, 100.0f, feedback);
        assert(output >= MIN_OUTPUT - 1e-6f);
        assert(output <= pid.fMaxFlowRate + 1e-6f);
        assert(pid.n_out >= MIN_OUTPUT * pid.fMaxMotorSpeed - 1e-3f);
        assert(pid.n_out <= pid.fMaxMotorSpeed + 1e-3f);
        assert(pid.KP >= pid.min_KP - 1e-6f && pid.KP <= pid.max_KP + 1e-6f);
        assert(pid.KI >= pid.min_KI - 1e-6f && pid.KI <= pid.max_KI + 1e-6f);
        assert(pid.KD >= pid.min_KD - 1e-6f && pid.KD <= pid.max_KD + 1e-6f);

        feedback += output * 5.0f;
        if (feedback > 100.0f) {
            feedback = 100.0f;
        }
    }

    assert(pid.Status == 1);
    assert(pid.TuneResult == 66);
    assert(feedback > 1.0f);
    printf("✓ RBF_PID adaptation/limit test passed\n");
}

static void test_explicit_reset_restores_runtime_state(void) {
    RBF_PID_Handle pid;

    printf("Testing RBF_PID explicit reset behavior...\n");
    RBF_PID_Init(&pid, 0.01f, 1500.0f, 1.0f);
    pid.enable = true;
    (void)RBF_PID_Update(&pid, 100.0f, 0.0f);
    assert(fabsf(pid.u_prev) > 1e-6f || fabsf(pid.du_prev) > 1e-6f || fabsf(pid.n_out) > 1e-6f);

    RBF_PID_Reset(&pid);
    assert(fabsf(pid.u_prev) < 1e-6f);
    assert(fabsf(pid.du_prev) < 1e-6f);
    assert(fabsf(pid.e_prev1) < 1e-6f);
    assert(fabsf(pid.e_prev2) < 1e-6f);
    assert(fabsf(pid.KP - 0.03f) < 1e-6f);
    assert(fabsf(pid.KI - 0.02f) < 1e-6f);
    assert(fabsf(pid.KD - 0.03f) < 1e-6f);
    assert(pid.Status == 0);
    assert(!pid.FirstScan);
    printf("✓ RBF_PID explicit reset test passed\n");
}

static void test_adaptive_learning_rate_scales_with_error(void) {
    RBF_PID_Handle pid;
    float kp_before, kp_after;
    int step;

    printf("Testing adaptive learning rate scaling...\n");
    RBF_PID_Init(&pid, 0.01f, 1500.0f, 1.0f);
    pid.enable = true;

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
    /* KP should not drift more than 5% from its previous value */
    assert(fabsf(kp_after - kp_before) < 0.05f * kp_before + 0.01f);
    printf("✓ Adaptive learning rate scaling test passed\n");
}

static void test_multi_axis_differentiated_seeds(void) {
    RBF_PID_Handle pid1, pid2;
    float max_diff;
    int i, j;

    printf("Testing multi-axis differentiated seeds...\n");

    RBF_PID_Init(&pid1, 0.01f, 1500.0f, 1.0f);
    RBF_PID_SetSeed(&pid1, 12345UL);

    RBF_PID_Init(&pid2, 0.01f, 1500.0f, 1.0f);
    RBF_PID_SetSeed(&pid2, 67890UL);

    /* Different seeds should produce different initial centers */
    max_diff = 0.0f;
    for (i = 0; i < RBF_HNUM; i++) {
        for (j = 0; j < RBF_INPUT_DIM; j++) {
            float diff = fabsf(pid1.c[i][j] - pid2.c[i][j]);
            if (diff > max_diff) max_diff = diff;
        }
    }
    assert(max_diff > 0.01f);

    /* Different seeds should produce different initial weights */
    max_diff = 0.0f;
    for (i = 0; i < RBF_HNUM; i++) {
        float diff = fabsf(pid1.w[i] - pid2.w[i]);
        if (diff > max_diff) max_diff = diff;
    }
    assert(max_diff > 0.01f);
    printf("✓ Multi-axis differentiated seeds test passed\n");
}

int main(void) {
    printf("Running RBF_PID tests...\n\n");

    test_disabled_controller_returns_zero_output();
    test_enabled_controller_respects_limits_and_drives_feedback();
    test_explicit_reset_restores_runtime_state();
    test_adaptive_learning_rate_scales_with_error();
    test_multi_axis_differentiated_seeds();

    printf("\n✅ All RBF_PID tests passed successfully!\n");
    return 0;
}
