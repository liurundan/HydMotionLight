#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "rbf_pid.h"

static void test_init_and_fixed_discrete_contract(void) {
    RBF_PID_Handle pid;

    RBF_PID_Init(&pid, 0.010f, 90.0f, 1.0f);
    assert(fabsf(pid.sampling_period - RBF_PID_FIXED_SAMPLING_PERIOD) < 1.0e-6f);
    assert(pid.Status == 1);
    assert(fabsf(pid.KP - PID_MIN_KP) < 1.0e-6f);
    assert(fabsf(pid.KI - PID_MIN_KI) < 1.0e-6f);
    assert(fabsf(pid.KD - PID_MIN_KD) < 1.0e-6f);
    assert(fabsf(pid.output_min_flow) < 1.0e-6f);
}
static void test_gain_compensation_and_normalization(void) {
    RBF_PID_Handle pid;

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetFlowNormalization(&pid, 45.0f);
    RBF_PID_SetPressureNormalization(&pid, 180.0f);
    RBF_PID_SetGainCompensation(&pid, 30.0f);
    assert(fabsf(pid.flow_normalization_scale - 45.0f) < 1.0e-6f);
    assert(fabsf(pid.pressure_normalization_scale - 180.0f) < 1.0e-6f);
    assert(pid.ksys_valid);
    assert(fabsf(pid.K - 30.0f) < 1.0e-6f);

    RBF_PID_SetGainCompensation(&pid, 0.0f);
    assert(!pid.ksys_valid);
    assert(fabsf(pid.K) < 1.0e-6f);
}

static void test_control_mode_and_damping_configuration(void) {
    RBF_PID_Handle pid;

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetLearningRates(&pid, 0.01f, 0.02f, 0.03f,
                             0.04f, 0.05f, 0.06f);
    RBF_PID_SetControlMode(&pid, RBF_PID_CONTROL_MODE_PI);
    assert(pid.control_mode == RBF_PID_CONTROL_MODE_PI);
    assert(fabsf(pid.KD) < 1.0e-6f);
    assert(fabsf(pid.eta_d) < 1.0e-6f);
    RBF_PID_SetPressureVelocityDamping(&pid, 0.0f);
    assert(!pid.pressure_vel_damp_enabled);
    RBF_PID_SetExternalDeadbandEnabled(&pid, true);
    assert(pid.external_deadband_enabled);
}

static void test_saturation_freezes_network_learning(void) {
    RBF_PID_Handle pid;
    float before[RBF_HNUM];
    int i;

    RBF_PID_Init(&pid, 0.001f, 10.0f, 1.0f);
    RBF_PID_SetLearningRates(&pid, 0.2f, 0.2f, 0.2f,
                             0.1f, 0.1f, 0.1f);
    pid.Output = 10.0f;
    pid.u_prev = 10.0f;
    pid.output_saturated = true;
    for (i = 0; i < RBF_HNUM; ++i) before[i] = pid.w[i];

    (void)RBF_PID_Update(&pid, 100.0f, 20.0f);
    for (i = 0; i < RBF_HNUM; ++i) {
        assert(fabsf(pid.w[i] - before[i]) < 1.0e-7f);
    }
}

static void test_finite_output_and_reset(void) {
    RBF_PID_Handle pid;
    float output;

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    output = RBF_PID_Update(&pid, 100.0f, 0.0f);
    assert(isfinite(output));
    RBF_PID_Reset(&pid);
    assert(pid.Status == 1);
    assert(fabsf(pid.Output) < 1.0e-6f);
    assert(fabsf(pid.u_prev) < 1.0e-6f);
}

static void test_negative_pressure_error_can_request_negative_flow(void) {
    RBF_PID_Handle pid;
    float output = 0.0f;
    int i;

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    pid.output_min_flow = -5.0f;
    pid.output_max_flow = 20.0f;
    RBF_PID_SetLearningRates(&pid, 0.0f, 0.0f, 0.0f,
                             0.0f, 0.0f, 0.0f);
    for (i = 0; i < 50; ++i) {
        output = RBF_PID_Update(&pid, 10.0f, 15.0f);
    }
    assert(isfinite(output));
    assert(output >= -5.0f - 1.0e-5f);
}

int main(void) {
    test_init_and_fixed_discrete_contract();
    test_gain_compensation_and_normalization();
    test_control_mode_and_damping_configuration();
    test_saturation_freezes_network_learning();
    test_finite_output_and_reset();
    test_negative_pressure_error_can_request_negative_flow();
    puts("RBF-PID tests passed.");
    return 0;
}
