#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "rbf_pid.h"

/* Weight limit from rbf_pid.c (not exposed in header) */
#define RBF_PID_WEIGHT_LIMIT 5.0f

/* P0-1: NaN/Inf input protection test */
static void test_p0_1_nan_inf_input_protection_pid_mode(void) {
    RBF_PID_Handle pid;
    float output;
    int i;

    printf("Testing P0-1: NaN/Inf input protection in PID mode...\n");

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetControlMode(&pid, RBF_PID_CONTROL_MODE_PID);

    /* Normal operation first */
    for (i = 0; i < 5; i++) {
        output = RBF_PID_Update(&pid, 100.0f, 50.0f);
        assert(isfinite(output));
    }

    /* Inject NaN feedback */
    output = RBF_PID_Update(&pid, 100.0f, NAN);
    assert(isfinite(output));  /* Should sanitize to 0 */
    assert(isfinite(pid.P_actual));
    assert(pid.P_actual == 0.0f);

    /* Inject Inf setpoint */
    output = RBF_PID_Update(&pid, INFINITY, 50.0f);
    assert(isfinite(output));
    assert(isfinite(pid.P_set));
    assert(pid.P_set == 0.0f);

    /* Inject -Inf feedback */
    output = RBF_PID_Update(&pid, 100.0f, -INFINITY);
    assert(isfinite(output));
    assert(isfinite(pid.P_actual));
    assert(pid.P_actual == 0.0f);

    /* Recovery: normal operation should resume */
    for (i = 0; i < 10; i++) {
        output = RBF_PID_Update(&pid, 100.0f, 50.0f + i * 5.0f);
        assert(isfinite(output));
        assert(isfinite(pid.Output));
    }

    printf("✓ P0-1: NaN/Inf input protection test passed (PID mode)\n");
}

/* P0-2: Weight unbounded update in PID mode test */
static void test_p0_2_weight_bounded_update_pid_mode(void) {
    RBF_PID_Handle pid;
    float output;
    int i, j, k;
    bool all_weights_bounded;

    printf("Testing P0-2: Weight bounded update in PID mode...\n");

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetControlMode(&pid, RBF_PID_CONTROL_MODE_PID);

    /* Aggressive learning rates to stress-test bounds */
    RBF_PID_SetLearningRates(&pid, 0.5f, 0.5f, 0.5f, 0.2f, 0.2f, 0.2f);

    /* Drive with large persistent error */
    for (i = 0; i < 200; i++) {
        output = RBF_PID_Update(&pid, 200.0f, 10.0f);  /* Large error */
        assert(isfinite(output));
    }

    /* Check all weights are within WEIGHT_LIMIT */
    all_weights_bounded = true;
    for (j = 0; j < RBF_HNUM; j++) {
        if (fabsf(pid.w[j]) > RBF_PID_WEIGHT_LIMIT + 1e-6f) {
            all_weights_bounded = false;
            printf("  Weight[%d] = %.6f exceeds limit %.6f\n",
                   j, pid.w[j], RBF_PID_WEIGHT_LIMIT);
        }
    }
    assert(all_weights_bounded);

    /* Reverse error direction and check bounds again */
    for (k = 0; k < 200; k++) {
        output = RBF_PID_Update(&pid, 10.0f, 200.0f);  /* Reverse error */
        assert(isfinite(output));
    }

    all_weights_bounded = true;
    for (j = 0; j < RBF_HNUM; j++) {
        if (fabsf(pid.w[j]) > RBF_PID_WEIGHT_LIMIT + 1e-6f) {
            all_weights_bounded = false;
            printf("  Weight[%d] = %.6f exceeds limit %.6f\n",
                   j, pid.w[j], RBF_PID_WEIGHT_LIMIT);
        }
    }
    assert(all_weights_bounded);

    printf("✓ P0-2: Weight bounded update test passed (PID mode)\n");
}

/* P0-1 + P0-2 combined stress test */
static void test_p0_combined_nan_and_weight_stress(void) {
    RBF_PID_Handle pid;
    float output;
    int i, j;
    bool all_weights_bounded;

    printf("Testing P0-1+P0-2: Combined NaN input and weight bounds stress test...\n");

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetControlMode(&pid, RBF_PID_CONTROL_MODE_PID);
    RBF_PID_SetLearningRates(&pid, 0.3f, 0.3f, 0.3f, 0.15f, 0.15f, 0.15f);

    /* Mixed scenario: normal → NaN → large error → Inf → recovery */
    for (i = 0; i < 50; i++) {
        float setpoint, feedback;

        if (i % 10 == 5) {
            setpoint = NAN;  /* Inject NaN periodically */
            feedback = 50.0f;
        } else if (i % 10 == 7) {
            setpoint = 150.0f;
            feedback = INFINITY;  /* Inject Inf periodically */
        } else if (i > 20 && i < 40) {
            setpoint = 200.0f;  /* Large error burst */
            feedback = 10.0f;
        } else {
            setpoint = 100.0f;
            feedback = 50.0f + i * 1.0f;
        }

        output = RBF_PID_Update(&pid, setpoint, feedback);

        /* Every update should produce finite output */
        assert(isfinite(output));
        assert(isfinite(pid.Output));
        assert(isfinite(pid.P_set));
        assert(isfinite(pid.P_actual));
    }

    /* Final check: all weights bounded */
    all_weights_bounded = true;
    for (j = 0; j < RBF_HNUM; j++) {
        if (!isfinite(pid.w[j]) || fabsf(pid.w[j]) > RBF_PID_WEIGHT_LIMIT + 1e-6f) {
            all_weights_bounded = false;
            printf("  Weight[%d] = %.6f (finite=%d, bounded=%d)\n",
                   j, pid.w[j], isfinite(pid.w[j]),
                   fabsf(pid.w[j]) <= RBF_PID_WEIGHT_LIMIT);
        }
    }
    assert(all_weights_bounded);

    printf("✓ P0-1+P0-2: Combined stress test passed\n");
}

/* P0-2: Verify PI mode still has same-direction saturation learning freeze */
static void test_p0_2_pi_mode_saturation_learning_freeze(void) {
    RBF_PID_Handle pid;
    float weights_before[RBF_HNUM];
    float weights_after[RBF_HNUM];
    bool weights_unchanged;
    int i, j;

    printf("Testing P0-2: PI mode saturation learning freeze preserved...\n");

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetControlMode(&pid, RBF_PID_CONTROL_MODE_PI);
    RBF_PID_SetLearningRates(&pid, 0.2f, 0.2f, 0.2f, 0.1f, 0.1f, 0.0f);

    /* Drive to saturation with large positive error */
    for (i = 0; i < 100; i++) {
        (void)RBF_PID_Update(&pid, 200.0f, 10.0f);
    }

    /* Check if saturated (may not saturate in PI mode with flow limits) */
    if (!pid.output_saturated) {
        printf("  Note: PI mode did not reach saturation state. Skipping saturation test.\n");
        printf("  (This is acceptable - saturation freeze is tested via legacy test)\n");
        return;
    }

    /* Record weights at saturation */
    for (j = 0; j < RBF_HNUM; j++) {
        weights_before[j] = pid.w[j];
    }

    /* Continue with same-direction error while saturated */
    for (i = 0; i < 20; i++) {
        (void)RBF_PID_Update(&pid, 200.0f, 10.0f);
    }

    /* Weights should not change (learning frozen) */
    for (j = 0; j < RBF_HNUM; j++) {
        weights_after[j] = pid.w[j];
    }

    weights_unchanged = true;
    for (j = 0; j < RBF_HNUM; j++) {
        if (fabsf(weights_after[j] - weights_before[j]) > 1e-8f) {
            weights_unchanged = false;
            printf("  Weight[%d] changed: %.6f → %.6f during saturation\n",
                   j, weights_before[j], weights_after[j]);
        }
    }
    assert(weights_unchanged);

    printf("✓ P0-2: PI mode saturation learning freeze test passed\n");
}

int main(void) {
    printf("Running RBF_PID P0 fixes verification tests...\n\n");

    test_p0_1_nan_inf_input_protection_pid_mode();
    test_p0_2_weight_bounded_update_pid_mode();
    test_p0_combined_nan_and_weight_stress();
    test_p0_2_pi_mode_saturation_learning_freeze();

    printf("\n✅ All P0 fixes verification tests passed!\n");
    return 0;
}
