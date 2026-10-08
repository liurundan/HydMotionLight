#ifndef HYD_DEBUG_VALUES_H
#define HYD_DEBUG_VALUES_H

#include "common_types.h"

#define HYD_DEBUG_VALUE_COUNT 8U

/* Store one application-selected diagnostic value in the global debug bank.
 * Indices are zero-based and valid in the range [0, 7]. */
HYD_BOOL HYD_Debug_SetValue(HYD_UINT8 index, float value);

/* Read one value from the global debug bank. Invalid indices return 0.0f. */
float HYD_Debug_GetValue(HYD_UINT8 index);

/* Clear all debug values. Called by the IEC framework initializer. */
void HYD_Debug_Reset(void);

#endif /* HYD_DEBUG_VALUES_H */
