#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pressure_model.h"

#define DT_S 0.001f
#define PRESSURE_EPS 1e-4f
#define RPM_EPS 1e-3f

#define ASSERT_TRUE(condition)                                                         \
    do {                                                                               \
        if (!(condition)) {                                                            \
            fprintf(stderr, "Assertion failed at %s:%d: %s\n", __FILE__, __LINE__,    \
                    #condition);                                                       \
            return 0;                                                                  \
        }                                                                              \
    } while (0)

#define ASSERT_NEAR(actual, expected, tolerance)                                       \
    do {                                                                               \
        if (fabs((double)((actual) - (expected))) > (double)(tolerance)) {            \
            fprintf(stderr,                                                            \
                    "Assertion failed at %s:%d: %s=%f expected=%f tolerance=%f\n",    \
                    __FILE__,                                                          \
                    __LINE__,                                                          \
                    #actual,                                                           \
                    (double)(actual),                                                  \
                    (double)(expected),                                                \
                    (double)(tolerance));                                              \
            return 0;                                                                  \
        }                                                                              \
    } while (0)

static PressureModelParams make_deterministic_params(void) {
    PressureModelParams params;

    PressureModel_InitParams(&params);
    params.sensor_noise_std_bar = 0.0f;
    params.sensor_bias_bar = 0.0f;
    params.motor_noise_std_rpm = 0.0f;
    params.process_noise_std_m3_s = 0.0f;
    params.enable_sensor_noise = 0u;
    params.enable_motor_noise = 0u;
    params.enable_process_noise = 0u;

    return params;
}

static void run_steps(const PressureModelParams *params,
                      PressureModelState *state,
                      float target_rpm,
                      int cycles,
                      float dt_s,
                      PressureModelOutput *out) {
    int i;

    for (i = 0; i < cycles; ++i) {
        PressureModel_Step(params, state, target_rpm, dt_s, out);
    }
}

static int test_zero_speed_holds_zero_pressure(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x12345678u);

    run_steps(&params, &state, 0.0f, 2000, DT_S, &out);

    ASSERT_NEAR(out.actual_motor_rpm, 0.0f, RPM_EPS);
    ASSERT_NEAR(out.real_pressure_bar, 0.0f, PRESSURE_EPS);
    ASSERT_NEAR(out.measured_pressure_bar, 0.0f, PRESSURE_EPS);

    return 1;
}

static int test_ten_rpm_converges_to_forty_bar(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x12345678u);

    run_steps(&params, &state, 10.0f, 15000, DT_S, &out);

    ASSERT_NEAR(out.real_pressure_bar, 40.0f, 1.5f);
    ASSERT_NEAR(out.measured_pressure_bar, out.real_pressure_bar, 1e-3f);

    return 1;
}

static int test_motor_state_is_continuous_across_steps(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out0;
    PressureModelOutput out1;

    memset(&out0, 0, sizeof(out0));
    memset(&out1, 0, sizeof(out1));
    PressureModel_Reset(&state, 0x12345678u);

    PressureModel_Step(&params, &state, 1000.0f, DT_S, &out0);
    PressureModel_Step(&params, &state, 1000.0f, DT_S, &out1);

    ASSERT_TRUE(out0.actual_motor_rpm > 0.0f);
    ASSERT_TRUE(out0.actual_motor_rpm < 1000.0f);
    ASSERT_TRUE(out1.actual_motor_rpm > out0.actual_motor_rpm);

    return 1;
}

static int count_visible_tooth_valleys(const float *measured,
                                       const float *real,
                                       int samples,
                                       float min_gap_bar) {
    int i;
    int valleys = 0;

    for (i = 1; i + 1 < samples; ++i) {
        if (measured[i] < measured[i - 1] &&
            measured[i] <= measured[i + 1] &&
            (real[i] - measured[i]) >= min_gap_bar) {
            ++valleys;
        }
    }

    return valleys;
}

static int test_negative_speed_depressurizes_faster_than_passive_leak(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState charged_state;
    PressureModelState leak_only_state;
    PressureModelState reverse_state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&charged_state, 0x11111111u);
    run_steps(&params, &charged_state, 10.0f, 15000, DT_S, &out);

    leak_only_state = charged_state;
    reverse_state = charged_state;

    run_steps(&params, &leak_only_state, 0.0f, 2000, DT_S, &out);
    run_steps(&params, &reverse_state, -50.0f, 2000, DT_S, &out);

    ASSERT_TRUE(reverse_state.pressure_pa < leak_only_state.pressure_pa);
    ASSERT_TRUE(reverse_state.pressure_pa >= 0.0f);

    return 1;
}

static int test_tooth_drop_is_visible_once_per_tooth(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;
    float measured[100];
    float real[100];
    int i;
    int valleys;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x22222222u);

    run_steps(&params, &state, 600.0f, 2000, DT_S, &out);
    for (i = 0; i < 100; ++i) {
        PressureModel_Step(&params, &state, 600.0f, DT_S, &out);
        measured[i] = out.measured_pressure_bar;
        real[i] = out.real_pressure_bar;
    }

    valleys = count_visible_tooth_valleys(measured, real, 100, 0.05f);
    ASSERT_TRUE(valleys >= 10 && valleys <= 16);

    return 1;
}

static int test_relief_caps_measured_output_at_two_hundred_fifty_bar(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x33333333u);

    run_steps(&params, &state, 2000.0f, 30000, DT_S, &out);

    ASSERT_TRUE(out.measured_pressure_bar <= 250.0f + 1e-3f);
    ASSERT_TRUE(out.relief_active);

    return 1;
}

static int test_noise_control_is_repeatable_with_fixed_seed(void) {
    PressureModelParams params;
    PressureModelState state_a;
    PressureModelState state_b;
    PressureModelOutput out_a;
    PressureModelOutput out_b;
    int i;

    PressureModel_InitParams(&params);
    params.enable_sensor_noise = 1u;
    params.enable_motor_noise = 1u;
    params.enable_process_noise = 1u;
    params.process_noise_std_m3_s = 1.0e-7f;

    memset(&out_a, 0, sizeof(out_a));
    memset(&out_b, 0, sizeof(out_b));
    PressureModel_Reset(&state_a, 0x44444444u);
    PressureModel_Reset(&state_b, 0x44444444u);

    for (i = 0; i < 500; ++i) {
        PressureModel_Step(&params, &state_a, 800.0f, DT_S, &out_a);
        PressureModel_Step(&params, &state_b, 800.0f, DT_S, &out_b);
        ASSERT_NEAR(out_a.measured_pressure_bar, out_b.measured_pressure_bar, 1e-6f);
        ASSERT_NEAR(out_a.actual_motor_rpm, out_b.actual_motor_rpm, 1e-6f);
    }

    return 1;
}

int main(void) {
    int passed = 0;
    int failed = 0;

    if (test_zero_speed_holds_zero_pressure()) {
        ++passed;
        printf("PASS test_zero_speed_holds_zero_pressure\n");
    } else {
        ++failed;
        printf("FAIL test_zero_speed_holds_zero_pressure\n");
    }

    if (test_ten_rpm_converges_to_forty_bar()) {
        ++passed;
        printf("PASS test_ten_rpm_converges_to_forty_bar\n");
    } else {
        ++failed;
        printf("FAIL test_ten_rpm_converges_to_forty_bar\n");
    }

    if (test_motor_state_is_continuous_across_steps()) {
        ++passed;
        printf("PASS test_motor_state_is_continuous_across_steps\n");
    } else {
        ++failed;
        printf("FAIL test_motor_state_is_continuous_across_steps\n");
    }

    if (test_negative_speed_depressurizes_faster_than_passive_leak()) {
        ++passed;
        printf("PASS test_negative_speed_depressurizes_faster_than_passive_leak\n");
    } else {
        ++failed;
        printf("FAIL test_negative_speed_depressurizes_faster_than_passive_leak\n");
    }

    if (test_tooth_drop_is_visible_once_per_tooth()) {
        ++passed;
        printf("PASS test_tooth_drop_is_visible_once_per_tooth\n");
    } else {
        ++failed;
        printf("FAIL test_tooth_drop_is_visible_once_per_tooth\n");
    }

    if (test_relief_caps_measured_output_at_two_hundred_fifty_bar()) {
        ++passed;
        printf("PASS test_relief_caps_measured_output_at_two_hundred_fifty_bar\n");
    } else {
        ++failed;
        printf("FAIL test_relief_caps_measured_output_at_two_hundred_fifty_bar\n");
    }

    if (test_noise_control_is_repeatable_with_fixed_seed()) {
        ++passed;
        printf("PASS test_noise_control_is_repeatable_with_fixed_seed\n");
    } else {
        ++failed;
        printf("FAIL test_noise_control_is_repeatable_with_fixed_seed\n");
    }

    printf("Passed: %d Failed: %d\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
