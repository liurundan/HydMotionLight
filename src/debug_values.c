#include "debug_values.h"

static float HYD_Debug_Values[HYD_DEBUG_VALUE_COUNT];

HYD_BOOL HYD_Debug_SetValue(HYD_UINT8 index, float value) {
    if (index >= HYD_DEBUG_VALUE_COUNT) {
        return HYD_FALSE;
    }

    HYD_Debug_Values[index] = value;
    return HYD_TRUE;
}

float HYD_Debug_GetValue(HYD_UINT8 index) {
    if (index >= HYD_DEBUG_VALUE_COUNT) {
        return 0.0f;
    }

    return HYD_Debug_Values[index];
}

void HYD_Debug_Reset(void) {
    HYD_UINT8 index;

    for (index = 0U; index < HYD_DEBUG_VALUE_COUNT; ++index) {
        HYD_Debug_Values[index] = 0.0f;
    }
}
