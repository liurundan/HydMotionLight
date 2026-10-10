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

int main(void) {
    test_invalid_input_holds_safe_output();
    test_network_weights_remain_bounded();
    test_saturation_freezes_network_and_gains();

    puts("RBF-PID safety tests passed.");
    return 0;
}
