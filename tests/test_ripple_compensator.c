#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ripple_compensator.h"

static HYD_RippleCompTable make_table(HYD_PressureRippleEntry* entries,
                                      size_t count,
                                      HYD_BOOL calibrated) {
    HYD_RippleCompTable table;

    table.entries = entries;
    table.count = count;
    table.calibrated = calibrated;
    return table;
}

static HYD_PumpFeedback make_feedback(HYD_REAL rpm,
                                      HYD_REAL angleDeg,
                                      HYD_REAL timestamp,
                                      uint32_t flags) {
    HYD_PumpFeedback feedback;

    memset(&feedback, 0, sizeof(feedback));
    feedback.rpm = rpm;
    feedback.angleDeg = angleDeg;
    feedback.timestamp = timestamp;
    feedback.validFlags = flags;
    return feedback;
}

static uint32_t phase_flags(void) {
    return HYD_PUMP_FEEDBACK_VALID_RPM |
           HYD_PUMP_FEEDBACK_VALID_ANGLE |
           HYD_PUMP_FEEDBACK_VALID_TIMESTAMP;
}

static void test_default_table_is_uncalibrated_and_bypasses(void) {
    HYD_RippleCompState state;
    HYD_RippleCompOutput output;
    HYD_RippleCompTable defaultTable = HYD_RippleComp_DefaultTable();
    HYD_PumpFeedback feedback = make_feedback(100.0, 1.0, 0.001, phase_flags());

    printf("Testing uncalibrated production table bypass...\n");
    HYD_RippleComp_Reset(&state);
    assert(!defaultTable.calibrated);
    assert(defaultTable.count == 0U);
    assert(!HYD_RippleComp_Scan(&feedback, &defaultTable, &state,
                                100.0, 1000.0, &output));
    assert(!output.active);
    assert(fabs(output.deltaRpm) < 1e-9);
    assert(!state.initialized);
}

static void test_forward_and_reverse_phase_wrap(void) {
    HYD_RippleCompState state;
    HYD_RippleCompOutput output;
    HYD_PressureRippleEntry entries[] = {
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {200.0f, 10.0f, 0.0f, 0.0f, 0.0f}
    };
    HYD_RippleCompTable table = make_table(entries, 2U, true);

    printf("Testing forward and reverse modulo-360 phase wrap...\n");
    HYD_RippleComp_Reset(&state);
    assert(!HYD_RippleComp_Scan(&(HYD_PumpFeedback){100.0, 359.0, 0.0, 0.0, phase_flags()},
                                &table, &state, 100.0, 1000.0, &output));
    assert(HYD_RippleComp_Scan(&(HYD_PumpFeedback){100.0, 1.0, 0.0, 0.001, phase_flags()},
                               &table, &state, 100.0, 1000.0, &output));
    assert(output.active);
    assert(output.deltaRpm > 0.0);

    HYD_RippleComp_Reset(&state);
    assert(!HYD_RippleComp_Scan(&(HYD_PumpFeedback){-100.0, 1.0, 0.0, 0.0, phase_flags()},
                                &table, &state, -100.0, 1000.0, &output));
    assert(HYD_RippleComp_Scan(&(HYD_PumpFeedback){-100.0, 359.0, 0.0, 0.001, phase_flags()},
                               &table, &state, -100.0, 1000.0, &output));
    assert(output.active);
    assert(output.deltaRpm < 0.0);
}

static void test_invalid_timestamp_angle_and_stopped_pump_bypass(void) {
    HYD_RippleCompState state;
    HYD_RippleCompOutput output;
    HYD_PressureRippleEntry entries[] = {
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {200.0f, 10.0f, 0.0f, 0.0f, 0.0f}
    };
    HYD_RippleCompTable table = make_table(entries, 2U, true);

    printf("Testing timestamp rollback, angle jump, invalid flags, and stopped bypass...\n");
    HYD_RippleComp_Reset(&state);
    assert(!HYD_RippleComp_Scan(&(HYD_PumpFeedback){100.0, 0.0, 0.0, 1.0, phase_flags()},
                                &table, &state, 100.0, 1000.0, &output));
    assert(!HYD_RippleComp_Scan(&(HYD_PumpFeedback){100.0, 1.0, 0.0, 0.5, phase_flags()},
                                &table, &state, 100.0, 1000.0, &output));
    assert(!state.initialized);

    assert(!HYD_RippleComp_Scan(&(HYD_PumpFeedback){100.0, 0.0, 0.0, 1.0, phase_flags()},
                                &table, &state, 100.0, 1000.0, &output));
    assert(!HYD_RippleComp_Scan(&(HYD_PumpFeedback){100.0, 100.0, 0.0, 1.001, phase_flags()},
                                &table, &state, 100.0, 1000.0, &output));
    assert(!state.initialized);

    assert(!HYD_RippleComp_Scan(&(HYD_PumpFeedback){100.0, 1.0, 0.0, 1.0,
                                                     HYD_PUMP_FEEDBACK_VALID_RPM},
                                &table, &state, 100.0, 1000.0, &output));
    assert(!state.initialized);
    assert(!HYD_RippleComp_Scan(&(HYD_PumpFeedback){0.0, 0.0, 0.0, 1.0, phase_flags()},
                                &table, &state, 0.0, 1000.0, &output));
    assert(!output.active);
    assert(fabs(output.deltaRpm) < 1e-9);
}

static void test_orders_are_independently_observable(void) {
    HYD_RippleCompState state;
    HYD_RippleCompOutput output;
    HYD_PressureRippleEntry entries[] = {
        {1000.0f, 10.0f, 0.0f, 10.0f, -1.5707963267948966f},
        {2000.0f, 10.0f, 0.0f, 10.0f, -1.5707963267948966f}
    };
    HYD_RippleCompTable table = make_table(entries, 2U, true);

    printf("Testing separate 13th/26th observability at 1 ms...\n");
    HYD_RippleComp_Reset(&state);
    assert(!HYD_RippleComp_Scan(&(HYD_PumpFeedback){1500.0, 357.923076923, 0.0,
                                                     0.0, phase_flags()},
                                &table, &state, 1500.0, 2000.0, &output));
    assert(HYD_RippleComp_Scan(&(HYD_PumpFeedback){1500.0, 6.923076923, 0.0,
                                                    0.001, phase_flags()},
                               &table, &state, 1500.0, 2000.0, &output));
    /* 13th is 325 Hz and remains observable; 26th is 650 Hz and is gated. */
    assert(output.deltaRpm > 7.0 && output.deltaRpm < 10.1);
}

static void test_table_interpolation_sign_and_limit(void) {
    HYD_RippleCompState state;
    HYD_RippleCompOutput output;
    HYD_PressureRippleEntry interpolationEntries[] = {
        {100.0f, 10.0f, 0.0f, 0.0f, 0.0f},
        {200.0f, 20.0f, 0.0f, 0.0f, 0.0f}
    };
    HYD_PressureRippleEntry cappedEntries[] = {
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {1000.0f, 300.0f, 0.0f, 300.0f, -1.5707963267948966f}
    };
    HYD_RippleCompTable interpolationTable = make_table(interpolationEntries, 2U, true);
    HYD_RippleCompTable cappedTable = make_table(cappedEntries, 2U, true);

    printf("Testing RPM-table interpolation, calibrated sign, and output limit...\n");
    HYD_RippleComp_Reset(&state);
    assert(!HYD_RippleComp_Scan(&(HYD_PumpFeedback){150.0, 6.023076923, 0.0,
                                                     0.0, phase_flags()},
                                &interpolationTable, &state, 150.0, 1000.0, &output));
    assert(HYD_RippleComp_Scan(&(HYD_PumpFeedback){150.0, 6.923076923, 0.0,
                                                    0.001, phase_flags()},
                               &interpolationTable, &state, 150.0, 1000.0, &output));
    assert(output.deltaRpm > 14.8 && output.deltaRpm < 15.1);

    interpolationEntries[0].amp13_rpm = -10.0f;
    interpolationEntries[1].amp13_rpm = -20.0f;
    HYD_RippleComp_Reset(&state);
    assert(!HYD_RippleComp_Scan(&(HYD_PumpFeedback){150.0, 6.023076923, 0.0,
                                                     0.0, phase_flags()},
                                &interpolationTable, &state, 150.0, 1000.0, &output));
    assert(HYD_RippleComp_Scan(&(HYD_PumpFeedback){150.0, 6.923076923, 0.0,
                                                    0.001, phase_flags()},
                               &interpolationTable, &state, 150.0, 1000.0, &output));
    assert(output.deltaRpm < -14.8 && output.deltaRpm > -15.1);

    HYD_RippleComp_Reset(&state);
    assert(!HYD_RippleComp_Scan(&(HYD_PumpFeedback){1000.0, 0.923076923, 0.0,
                                                     0.0, phase_flags()},
                                &cappedTable, &state, 1000.0, 1000.0, &output));
    assert(HYD_RippleComp_Scan(&(HYD_PumpFeedback){1000.0, 6.923076923, 0.0,
                                                    0.001, phase_flags()},
                               &cappedTable, &state, 1000.0, 1000.0, &output));
    assert(fabs(output.deltaRpm - 300.0) < 0.1);
}

int main(void) {
    test_default_table_is_uncalibrated_and_bypasses();
    test_forward_and_reverse_phase_wrap();
    test_invalid_timestamp_angle_and_stopped_pump_bypass();
    test_orders_are_independently_observable();
    test_table_interpolation_sign_and_limit();
    printf("PASS ripple compensator tests\n");
    return 0;
}
