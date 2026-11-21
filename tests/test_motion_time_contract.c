#include <math.h>
#include <stdio.h>

#include "motion_time.h"

static int tests_run;
static int tests_passed;

#define ASSERT_TRUE(condition, message) do { \
    ++tests_run; \
    if (condition) { \
        ++tests_passed; \
    } else { \
        printf("FAIL: %s\n", message); \
    } \
} while (0)

#define ASSERT_NEAR(actual, expected, tolerance, message) \
    ASSERT_TRUE(fabs((actual) - (expected)) <= (tolerance), message)

int main(void)
{
    ASSERT_NEAR(HYD_NormalizeMotionTimeMs(0.0f), 100.0f, 1e-6f,
                "zero motion time should use 100 ms");
    ASSERT_NEAR(HYD_NormalizeMotionTimeMs(99.0f), 100.0f, 1e-6f,
                "short motion time should use 100 ms");
    ASSERT_NEAR(HYD_NormalizeMotionTimeMs(100.0f), 100.0f, 1e-6f,
                "100 ms motion time should be preserved");
    ASSERT_NEAR(HYD_NormalizePressureTimeMs(0.0f), 20.0f, 1e-6f,
                "zero pressure time should use 20 ms");
    ASSERT_NEAR(HYD_NormalizePressureTimeMs(5.0f), 5.0f, 1e-6f,
                "positive pressure time below 20 ms should be preserved");
    ASSERT_NEAR(HYD_MotionTimeMsToSeconds(100.0f), 0.1f, 1e-6f,
                "motion milliseconds should convert to seconds");
    ASSERT_NEAR(HYD_PressureTimeMsToSeconds(20.0f), 0.02f, 1e-6f,
                "pressure milliseconds should convert to seconds");
    ASSERT_TRUE(!HYD_IsValidMotionTimeMs(-1.0f),
                "negative motion time should be invalid");
    ASSERT_TRUE(!HYD_IsValidMotionTimeMs(NAN),
                "non-finite motion time should be invalid");

    printf("motion time contract: %d/%d passed\n", tests_passed, tests_run);
    return (tests_run == tests_passed) ? 0 : 1;
}
