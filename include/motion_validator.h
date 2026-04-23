#ifndef HDY_MOTION_VALIDATOR_H
#define HDY_MOTION_VALIDATOR_H

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
HDY_BOOL HDY_MotionValidator_ValidateStartRequest(const HDY_MotionControlFB* fb,
                                                   size_t segmentIndex,
                                                   HDY_DiagnosticCode* code);

/**
 * @brief Validate a next segment request
 * @param fb Pointer to motion control function block
 * @param code Output diagnostic code if validation fails (can be NULL)
 * @return true if next request is valid
 */
HDY_BOOL HDY_MotionValidator_ValidateNextRequest(const HDY_MotionControlFB* fb,
                                                  HDY_DiagnosticCode* code);

/**
 * @brief Validate pump converter configuration
 * @param flowToPumpSpeedGain Flow to pump speed gain
 * @param pumpSpeedLimit Pump speed limit
 * @param code Output diagnostic code if validation fails (can be NULL)
 * @return true if configuration is valid
 */
HDY_BOOL HDY_MotionValidator_ValidatePumpConfig(HDY_REAL flowToPumpSpeedGain,
                                                  HDY_REAL pumpSpeedLimit,
                                                  HDY_DiagnosticCode* code);

/**
 * @brief Check if recipe source is being used
 * @param fb Pointer to motion control function block
 * @return true if USE_RECIPE=true or fb is NULL (default)
 */
HDY_BOOL HDY_MotionValidator_UsesRecipeSource(const HDY_MotionControlFB* fb);

/**
 * @brief Check if a start source has been selected (recipe or direct segment)
 * @param fb Pointer to motion control function block
 * @return true if either recipe is loaded or direct segment is valid
 */
HDY_BOOL HDY_MotionValidator_HasSelectedStartSource(const HDY_MotionControlFB* fb);

/**
 * @brief Resolve the start segment for a given index
 * @param fb Pointer to motion control function block
 * @param requestedSegmentIndex Requested segment index
 * @param resolvedSegmentIndex Output resolved segment index (can be NULL)
 * @param resolvedSource Output resolved source type (can be NULL)
 * @return Pointer to resolved segment, or NULL if invalid
 */
const HDY_MotionSegment* HDY_MotionValidator_ResolveStartSourceSegment(
    const HDY_MotionControlFB* fb,
    size_t requestedSegmentIndex,
    size_t* resolvedSegmentIndex,
    HDY_SegmentSource* resolvedSource);

/**
 * @brief Resolve the effective function block state considering EN
 * @param fb Pointer to motion control function block
 * @return Effective state (DISABLED if EN=false, otherwise FB_STATE)
 */
HDY_FbState HDY_MotionValidator_ResolveEffectiveFbState(const HDY_MotionControlFB* fb);

#ifdef __cplusplus
}
#endif

#endif /* HDY_MOTION_VALIDATOR_H */
