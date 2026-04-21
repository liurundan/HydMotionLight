#ifndef HDY_MOTION_UTILS_H
#define HDY_MOTION_UTILS_H

#include "common_types.h"
#include "motion_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file motion_utils.h
 * @brief Motion control utility functions
 *
 * This module provides utility functions used across the motion control system:
 * - Mathematical utilities (min, abs, finite check)
 * - String conversion utilities
 * - Validation utilities
 */

/**
 * @brief Get minimum of two real values
 * @param left First value
 * @param right Second value
 * @return Minimum value
 */
HDY_REAL HDY_MotionUtils_MinReal(HDY_REAL left, HDY_REAL right);

/**
 * @brief Get absolute value of a real number
 * @param value Value
 * @return Absolute value
 */
HDY_REAL HDY_MotionUtils_AbsReal(HDY_REAL value);

/**
 * @brief Check if a real value is finite (not NaN or infinity)
 * @param value Value to check
 * @return true if value is finite
 */
HDY_BOOL HDY_MotionUtils_IsFiniteReal(HDY_REAL value);

/**
 * @brief Check if axis reference feedback is valid
 * @param axisRef Pointer to axis reference
 * @return true if all fields are finite and valid
 */
HDY_BOOL HDY_MotionUtils_AxisRefIsValid(const HDY_AxisRef* axisRef);

/**
 * @brief Get command name as string (for diagnostics)
 * @param command Command enum value
 * @return Command name string
 */
const char* HDY_MotionUtils_CommandToString(HDY_FbCommand command);

/**
 * @brief Get state name as string (for diagnostics)
 * @param state State enum value
 * @return State name string
 */
const char* HDY_MotionUtils_StateToString(HDY_FbState state);

/**
 * @brief Get library configuration information
 * @return Configuration info structure containing current settings
 */
HDY_ConfigInfo HDY_GetConfigInfo(void);

#ifdef __cplusplus
}
#endif

#endif /* HDY_MOTION_UTILS_H */
