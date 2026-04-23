#ifndef HDY_RECIPE_VALIDATOR_H
#define HDY_RECIPE_VALIDATOR_H

#include "common_types.h"
#include <stddef.h>

HDY_BOOL HDY_RecipeValidator_ValidateSegment(const HDY_MotionSegment* segment,
                                            size_t segmentIndex,
                                            HDY_DiagnosticCode* code);

HDY_BOOL HDY_RecipeValidator_ValidateRecipe(const HDY_MotionSegment* recipe,
                                           size_t recipeSize,
                                           HDY_DiagnosticCode* code);

HDY_BOOL HDY_RecipeValidator_ValidateRuntimeConfig(HDY_REAL flowToPumpSpeedGain,
                                                  HDY_REAL pumpSpeedLimit,
                                                  HDY_DiagnosticCode* code);

HDY_BOOL HDY_RecipeValidator_ValidateStartContext(const HDY_MotionSegment* segment,
                                                 size_t segmentIndex,
                                                 const HDY_AxisRef* axisRef,
                                                 HDY_DiagnosticCode* code);

#endif /* HDY_RECIPE_VALIDATOR_H */
