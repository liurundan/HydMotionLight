#ifndef HYD_RECIPE_VALIDATOR_H
#define HYD_RECIPE_VALIDATOR_H

#include "common_types.h"
#include <stddef.h>

HYD_BOOL HYD_RecipeValidator_ValidateSegment(const HYD_MotionSegment* segment,
                                            size_t segmentIndex,
                                            HYD_DiagnosticCode* code,
                                            const HYD_CylinderConfig* cylinderConfig);

HYD_BOOL HYD_RecipeValidator_ValidateRecipe(const HYD_MotionSegment* recipe,
                                           size_t recipeSize,
                                           HYD_DiagnosticCode* code);

HYD_BOOL HYD_RecipeValidator_ValidateRuntimeConfig(HYD_REAL flowToPumpSpeedGain,
                                                  HYD_REAL pumpSpeedLimit,
                                                  HYD_DiagnosticCode* code);

HYD_BOOL HYD_RecipeValidator_ValidateStartContext(const HYD_MotionSegment* segment,
                                                 size_t segmentIndex,
                                                 const HYD_AxisRef* axisRef,
                                                 HYD_DiagnosticCode* code);

#endif /* HYD_RECIPE_VALIDATOR_H */
