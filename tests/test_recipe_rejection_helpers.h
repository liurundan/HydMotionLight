#ifndef TEST_RECIPE_REJECTION_HELPERS_H
#define TEST_RECIPE_REJECTION_HELPERS_H

#include <assert.h>
#include <stddef.h>

#include "recipe_validator.h"

static inline void assert_oversized_recipe_validation_rejected(const HYD_MotionSegment* recipe,
                                                               size_t recipeSize) {
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    assert(recipe != NULL);
    assert(recipeSize > HYD_MAX_SEGMENTS);
    assert(!HYD_RecipeValidator_ValidateRecipe(recipe, recipeSize, &code));
    assert(code == HYD_DIAG_CODE_RECIPE_TOO_LARGE);
}

#endif /* TEST_RECIPE_REJECTION_HELPERS_H */
