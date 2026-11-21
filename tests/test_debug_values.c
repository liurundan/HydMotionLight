#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "debug_values.h"
#include "motion_interface.h"

#define IEC_VAL(var) ((var).value)

static void test_debug_values_round_trip_and_bounds(void) {
    static const float expected[HYD_DEBUG_VALUE_COUNT] = {
        -1.25f, 0.0f, 2.5f, 3.75f, 100.0f, -50.0f, 0.125f, 999.0f
    };
    unsigned int i;

    HYD_Debug_Reset();
    for (i = 0U; i < HYD_DEBUG_VALUE_COUNT; ++i) {
        assert(HYD_Debug_SetValue((HYD_UINT8)i, expected[i]) == HYD_TRUE);
    }
    for (i = 0U; i < HYD_DEBUG_VALUE_COUNT; ++i) {
        assert(fabsf(HYD_Debug_GetValue((HYD_UINT8)i) - expected[i]) < 1.0e-6f);
    }

    assert(HYD_Debug_SetValue((HYD_UINT8)HYD_DEBUG_VALUE_COUNT, 42.0f) == HYD_FALSE);
    assert(HYD_Debug_GetValue((HYD_UINT8)HYD_DEBUG_VALUE_COUNT) == 0.0f);
    assert(HYD_Debug_GetValue(0U) == expected[0]);
}

static void test_debug_values_reset_clears_all_slots(void) {
    unsigned int i;

    for (i = 0U; i < HYD_DEBUG_VALUE_COUNT; ++i) {
        assert(HYD_Debug_SetValue((HYD_UINT8)i, (float)(i + 1U)) == HYD_TRUE);
    }

    HYD_Debug_Reset();
    for (i = 0U; i < HYD_DEBUG_VALUE_COUNT; ++i) {
        assert(HYD_Debug_GetValue((HYD_UINT8)i) == 0.0f);
    }
}

static void test_framework_init_clears_debug_values(void) {
    assert(HYD_Debug_SetValue(3U, 17.0f) == HYD_TRUE);
    __HydMotion_framework_Init();
    assert(HYD_Debug_GetValue(3U) == 0.0f);
}

static void test_iec_read_debug_publishes_global_values(void) {
    HYD_READDEBUG read_debug;

    memset(&read_debug, 0, sizeof(read_debug));
    IEC_VAL(read_debug.EN) = true;
    IEC_VAL(read_debug.ENABLE) = true;

    HYD_Debug_Reset();
    assert(HYD_Debug_SetValue(0U, 1.0f) == HYD_TRUE);
    assert(HYD_Debug_SetValue(1U, -2.0f) == HYD_TRUE);
    assert(HYD_Debug_SetValue(2U, 3.5f) == HYD_TRUE);
    assert(HYD_Debug_SetValue(3U, 4.25f) == HYD_TRUE);
    assert(HYD_Debug_SetValue(4U, 5.0f) == HYD_TRUE);
    assert(HYD_Debug_SetValue(5U, 6.0f) == HYD_TRUE);
    assert(HYD_Debug_SetValue(6U, 7.0f) == HYD_TRUE);
    assert(HYD_Debug_SetValue(7U, 8.0f) == HYD_TRUE);

    __mcl_cmd_ReadDebug(&read_debug);

    assert(IEC_VAL(read_debug.VALID) == true);
    assert(IEC_VAL(read_debug.BUSY) == false);
    assert(IEC_VAL(read_debug.ERROR) == false);
    assert(IEC_VAL(read_debug.ERRORID) == (IEC_WORD)HYD_DIAG_CODE_NONE);
    assert(IEC_VAL(read_debug.VALUE0) == 1.0f);
    assert(IEC_VAL(read_debug.VALUE1) == -2.0f);
    assert(IEC_VAL(read_debug.VALUE2) == 3.5f);
    assert(IEC_VAL(read_debug.VALUE3) == 4.25f);
    assert(IEC_VAL(read_debug.VALUE4) == 5.0f);
    assert(IEC_VAL(read_debug.VALUE5) == 6.0f);
    assert(IEC_VAL(read_debug.VALUE6) == 7.0f);
    assert(IEC_VAL(read_debug.VALUE7) == 8.0f);
}

static void test_iec_read_debug_clears_outputs_when_disabled(void) {
    HYD_READDEBUG read_debug;

    memset(&read_debug, 0, sizeof(read_debug));
    IEC_VAL(read_debug.EN) = true;
    IEC_VAL(read_debug.ENABLE) = false;
    IEC_VAL(read_debug.VALUE0) = 123.0f;
    IEC_VAL(read_debug.VALUE7) = -456.0f;
    IEC_VAL(read_debug.VALID) = true;

    __mcl_cmd_ReadDebug(&read_debug);

    assert(IEC_VAL(read_debug.VALID) == false);
    assert(IEC_VAL(read_debug.BUSY) == false);
    assert(IEC_VAL(read_debug.ERROR) == false);
    assert(IEC_VAL(read_debug.ERRORID) == (IEC_WORD)HYD_DIAG_CODE_NONE);
    assert(IEC_VAL(read_debug.VALUE0) == 0.0f);
    assert(IEC_VAL(read_debug.VALUE1) == 0.0f);
    assert(IEC_VAL(read_debug.VALUE2) == 0.0f);
    assert(IEC_VAL(read_debug.VALUE3) == 0.0f);
    assert(IEC_VAL(read_debug.VALUE4) == 0.0f);
    assert(IEC_VAL(read_debug.VALUE5) == 0.0f);
    assert(IEC_VAL(read_debug.VALUE6) == 0.0f);
    assert(IEC_VAL(read_debug.VALUE7) == 0.0f);
}

int main(void) {
    test_debug_values_round_trip_and_bounds();
    test_debug_values_reset_clears_all_slots();
    test_framework_init_clears_debug_values();
    test_iec_read_debug_publishes_global_values();
    test_iec_read_debug_clears_outputs_when_disabled();
    puts("debug value interface tests passed");
    return 0;
}
