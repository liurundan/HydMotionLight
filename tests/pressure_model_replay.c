#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pressure_model.h"

static int parse_float(const char *text, float *value) {
    char *end = NULL;

    errno = 0;
    *value = strtof(text, &end);
    return errno == 0 && end != text && *end == '\0';
}

static int parse_count(const char *text, int *value) {
    char *end = NULL;
    long parsed;

    errno = 0;
    parsed = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' ||
        parsed < 1 || parsed > INT_MAX) {
        return 0;
    }
    *value = (int)parsed;
    return 1;
}

int main(int argc, char **argv) {
    PressureModelParams params;
    PressureModelState state;
    PressureModelOutput out;
    float rpm;
    int samples;
    int i;

    if (argc != 4 && argc != 5) {
        fprintf(stderr,
                "usage: %s <first_order|physical> <rpm> <samples> [identified_params.kv]\n",
                argv[0]);
        return 2;
    }
    if (!parse_float(argv[2], &rpm) || !parse_count(argv[3], &samples)) {
        fprintf(stderr, "invalid replay RPM or sample count\n");
        return 2;
    }
    if (argc == 5) {
        fprintf(stderr,
                "identified_params.kv loading is deferred to Task 3; replay is uncalibrated\n");
        return 2;
    }
    PressureModel_InitParams(&params);
    if (strcmp(argv[1], "first_order") == 0) {
        params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    } else if (strcmp(argv[1], "physical") == 0) {
        params.model_type = PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED;
        params.enable_sensor_noise = 0u;
        params.enable_motor_noise = 0u;
        params.enable_process_noise = 0u;
    } else {
        fprintf(stderr, "profile must be first_order or physical\n");
        return 2;
    }
    PressureModel_Reset(&state, 0x13572468u);
    puts("# calibration_id=uncalibrated");
    puts("# calibration_status=uncalibrated; identified_params.kv loader deferred to Task 3");
    puts("sample,actual_rpm,real_pressure_bar,measured_pressure_bar,angle_deg,"
         "torque_permille,timestamp_s,valid_flags");
    for (i = 0; i < samples; ++i) {
        PressureModel_Step(&params, &state, rpm, 0.001f, &out);
        printf("%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u\n",
               i,
               out.actual_motor_rpm,
               out.real_pressure_bar,
               out.measured_pressure_bar,
               out.pumpFeedback.angleDeg,
               out.pumpFeedback.torquePermille,
               out.pumpFeedback.timestamp,
               out.pumpFeedback.validFlags);
    }
    return 0;
}
