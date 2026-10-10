#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "rbf_pid.h"

static void test_invalid_input_holds_safe_output(void) {
    RBF_PID_Handle pid;
    float before;
    float after;

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    before = RBF_PID_Update(&pid, 100.0f, 50.0f);
    after = RBF_PID_Update(&pid, 100.0f, NAN);
    assert(isfinite(after));
    assert(fabsf(after - before) < 1.0e-6f);
    assert((pid.fault_flags & RBF_PID_FAULT_INPUT_INVALID) != 0u);
}

static void test_network_weights_remain_bounded(void) {
    RBF_PID_Handle pid;
    int i;
    int step;

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetLearningRates(&pid, 0.5f, 0.5f, 0.5f,
                             0.2f, 0.2f, 0.2f);
    for (step = 0; step < 200; ++step) {
        (void)RBF_PID_Update(&pid, 200.0f, 10.0f);
    }
    for (i = 0; i < RBF_HNUM; ++i) {
        assert(isfinite(pid.w[i]));
        assert(fabsf(pid.w[i]) <= 5.0f + 1.0e-6f);
    }
}

static void test_saturation_freezes_network_and_gains(void) {
    RBF_PID_Handle pid;
    float beforeWeight;
    float beforeKp;

    RBF_PID_Init(&pid, 0.001f, 10.0f, 1.0f);
    RBF_PID_SetLearningRates(&pid, 0.2f, 0.2f, 0.2f,
                             0.1f, 0.1f, 0.1f);
    pid.Output = 10.0f;
    pid.u_prev = 10.0f;
    pid.output_saturated = true;
    beforeWeight = pid.w[0];
    beforeKp = pid.KP;
    (void)RBF_PID_Update(&pid, 100.0f, 20.0f);
    assert(fabsf(pid.w[0] - beforeWeight) < 1.0e-7f);
    assert(fabsf(pid.KP - beforeKp) < 1.0e-7f);
}

static void configure_adaptation_test(RBF_PID_Handle *pid) {
    RBF_PID_Init(pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetLearningRates(pid, 0.2f, 0.2f, 0.2f,
                             0.2f, 0.2f, 0.2f);
    pid->output_min_flow = -20.0f;
    pid->output_max_flow = 80.0f;
    pid->u_prev = 10.0f;
    pid->Output = 10.0f;
    pid->KP = 0.6f;
    pid->KI = 0.003f;
    pid->KD = 0.02f;
}

static void test_reverse_output_freezes_network_and_gains(void) {
    RBF_PID_Handle pid;
    float beforeWeight;
    float beforeKp;
    int32_t beforeJacobianCount;

    configure_adaptation_test(&pid);
    pid.u_prev = -5.0f;
    pid.Output = -5.0f;
    pid.jac_neg_count = 37;
    beforeWeight = pid.w[0];
    beforeKp = pid.KP;
    beforeJacobianCount = pid.jac_neg_count;

    (void)RBF_PID_Update(&pid, 120.0f, 100.0f);

    assert(fabsf(pid.w[0] - beforeWeight) < 1.0e-7f);
    assert(fabsf(pid.KP - beforeKp) < 1.0e-7f);
    assert(pid.jac_neg_count == beforeJacobianCount);
    assert((pid.adaptation_freeze_reasons & RBF_PID_FREEZE_REVERSE_FLOW) != 0u);
    assert((pid.adaptation_freeze_reasons & RBF_PID_FREEZE_OUT_OF_COVERAGE) != 0u);
}

static void test_low_pressure_freezes_adaptation(void) {
    RBF_PID_Handle pid;
    float beforeWeight;
    float beforeKp;

    configure_adaptation_test(&pid);
    pid.u_prev = 20.0f;
    pid.Output = 20.0f;
    beforeWeight = pid.w[0];
    beforeKp = pid.KP;

    (void)RBF_PID_Update(&pid, 40.0f, 10.0f);

    assert(fabsf(pid.w[0] - beforeWeight) < 1.0e-7f);
    assert(fabsf(pid.KP - beforeKp) < 1.0e-7f);
    assert((pid.adaptation_freeze_reasons & RBF_PID_FREEZE_LOW_PRESSURE) != 0u);
}

static void test_target_drop_with_positive_output_keeps_learning_enabled(void) {
    RBF_PID_Handle pid;
    float beforeWeight;

    configure_adaptation_test(&pid);
    pid.u_prev = 20.0f;
    pid.Output = 20.0f;
    (void)RBF_PID_Update(&pid, 140.0f, 100.0f);
    beforeWeight = pid.w[0];

    /* A lower hold segment target is normal tracking, not reverse relief. */
    pid.u_prev = 20.0f;
    pid.Output = 20.0f;
    (void)RBF_PID_Update(&pid, 90.0f, 80.0f);

    assert(fabsf(pid.w[0] - beforeWeight) > 1.0e-8f);
}

static void test_recovery_learns_network_before_tuning_gains(void) {
    RBF_PID_Handle pid;
    float beforeWeight;
    float beforeKp;

    configure_adaptation_test(&pid);
    pid.u_prev = -5.0f;
    pid.Output = -5.0f;
    (void)RBF_PID_Update(&pid, 120.0f, 100.0f);

    pid.u_prev = 20.0f;
    pid.Output = 20.0f;
    beforeWeight = pid.w[0];
    beforeKp = pid.KP;
    (void)RBF_PID_Update(&pid, 140.0f, 100.0f);
    assert(fabsf(pid.w[0] - beforeWeight) > 1.0e-8f);
    assert(fabsf(pid.KP - beforeKp) < 1.0e-7f);

    beforeKp = pid.KP;
    (void)RBF_PID_Update(&pid, 140.0f, 100.0f);
    assert(fabsf(pid.KP - beforeKp) < 1.0e-7f);

    beforeKp = pid.KP;
    (void)RBF_PID_Update(&pid, 140.0f, 99.0f);
    assert(fabsf(pid.KP - beforeKp) > 1.0e-8f);
}

int main(void) {
    test_invalid_input_holds_safe_output();
    test_network_weights_remain_bounded();
    test_saturation_freezes_network_and_gains();
    test_reverse_output_freezes_network_and_gains();
    test_low_pressure_freezes_adaptation();
    test_target_drop_with_positive_output_keeps_learning_enabled();
    test_recovery_learns_network_before_tuning_gains();
    puts("RBF-PID safety tests passed.");
    return 0;
}
