#ifndef HYD_MOTION_VALIDATOR_H
#define HYD_MOTION_VALIDATOR_H

#include "common_types.h"
#include "motion_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file motion_validator.h
 * @brief Motion control validation module
 *
 * This module handles validation logic for motion control:
 * - Start request validation
 * - Next request validation
 * - Pump configuration validation
 * - Segment resolution
 */

/**
 * @brief Validate a start segment request
 * @param fb Pointer to motion control function block
 * @param segmentIndex Segment index to validate
 * @param code Output diagnostic code if validation fails (can be NULL)
 * @return true if start request is valid
 */
HYD_BOOL HYD_MotionValidator_ValidateStartRequest(const HYD_MotionControlFB* fb,
                                                   size_t segmentIndex,
                                                   HYD_DiagnosticCode* code);

/**
 * @brief Validate a next segment request
 * @param fb Pointer to motion control function block
 * @param code Output diagnostic code if validation fails (can be NULL)
 * @return true if next request is valid
 */
HYD_BOOL HYD_MotionValidator_ValidateNextRequest(const HYD_MotionControlFB* fb,
                                                  HYD_DiagnosticCode* code);

/**
 * @brief Validate pump converter configuration
 * @param flowToPumpSpeedGain Flow to pump speed gain
 * @param pumpSpeedLimit Pump speed limit
 * @param code Output diagnostic code if validation fails (can be NULL)
 * @return true if configuration is valid
 */
HYD_BOOL HYD_MotionValidator_ValidatePumpConfig(HYD_REAL flowToPumpSpeedGain,
                                                  HYD_REAL pumpSpeedLimit,
                                                  HYD_DiagnosticCode* code);

/**
 * @brief Check if recipe source is being used
 * @param fb Pointer to motion control function block
 * @return true if USE_RECIPE=true or fb is NULL (default)
 */
HYD_BOOL HYD_MotionValidator_UsesRecipeSource(const HYD_MotionControlFB* fb);

/**
 * @brief Check if a start source has been selected (recipe or direct segment)
 * @param fb Pointer to motion control function block
 * @return true if either recipe is loaded or direct segment is valid
 */
HYD_BOOL HYD_MotionValidator_HasSelectedStartSource(const HYD_MotionControlFB* fb);

/**
 * @brief Resolve the start segment for a given index
 * @param fb Pointer to motion control function block
 * @param requestedSegmentIndex Requested segment index
 * @param resolvedSegmentIndex Output resolved segment index (can be NULL)
 * @param resolvedSource Output resolved source type (can be NULL)
 * @return Pointer to resolved segment, or NULL if invalid
 */
const HYD_MotionSegment* HYD_MotionValidator_ResolveStartSourceSegment(
    const HYD_MotionControlFB* fb,
    size_t requestedSegmentIndex,
    size_t* resolvedSegmentIndex,
    HYD_SegmentSource* resolvedSource);

/**
 * @brief Resolve the effective function block state considering EN
 * @param fb Pointer to motion control function block
 * @return Effective state (DISABLED if EN=false, otherwise FB_STATE)
 */
HYD_FbState HYD_MotionValidator_ResolveEffectiveFbState(const HYD_MotionControlFB* fb);

#ifdef __cplusplus
}
#endif

#endif /* HYD_MOTION_VALIDATOR_H */
