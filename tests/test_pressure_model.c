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

    printf("Passed: %d Failed: %d\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
