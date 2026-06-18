#ifndef TEST_RECIPE_REJECTION_HELPERS_H
#define TEST_RECIPE_REJECTION_HELPERS_H

#include <assert.h>
#include <stddef.h>

#include "motion_control.h"
#include "recipe_validator.h"

static inline void assert_oversized_recipe_validation_rejected(const HYD_MotionSegment* recipe,
                                                               size_t recipeSize) {
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    assert(recipe != NULL);
    assert(recipeSize > HYD_MAX_SEGMENTS);
    assert(!HYD_RecipeValidator_ValidateRecipe(recipe, recipeSize, &code));
    assert(code == HYD_DIAG_CODE_RECIPE_TOO_LARGE);
}

static inline void assert_oversized_recipe_load_rejected(HYD_MotionControlFB* fb,
                                                         const HYD_MotionSegment* recipe,
                                                         size_t recipeSize) {
    assert(fb != NULL);
    assert(recipe != NULL);
    assert(recipeSize > HYD_MAX_SEGMENTS);
    assert(!HYD_MotionControlFB_LoadRecipe(fb, recipe, recipeSize));
    assert(fb->RECIPE_SIZE == 0U);
    assert(fb->DIAGNOSTIC.code == HYD_DIAG_CODE_RECIPE_TOO_LARGE);
    assert(fb->DIAGNOSTIC.severity == HYD_DIAG_SEVERITY_WARNING);
}

#endif /* TEST_RECIPE_REJECTION_HELPERS_H */
