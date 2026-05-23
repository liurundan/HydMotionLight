#include <stdio.h>
#include <string.h>
#include "motion_control.h"

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { printf("  FAIL: %s\n", msg); } \
} while (0)

#define ASSERT_EQ(a, b, msg) do { \
    tests_run++; \
    if ((a) == (b)) { tests_passed++; } \
    else { printf("  FAIL: %s (got %d, expected %d)\n", msg, (int)(a), (int)(b)); } \
} while (0)

#define ASSERT_FLOAT_EQ(a, b, eps, msg) do { \
    tests_run++; \
    HYD_REAL diff = (a) - (b); \
    if (diff < 0.0f) diff = -diff; \
    if (diff <= (eps)) { tests_passed++; } \
    else { printf("  FAIL: %s (got %.6f, expected %.6f)\n", msg, (double)(a), (double)(b)); } \
} while (0)

/* Test: Init sets expected defaults */
static void test_init_sets_defaults(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    HYD_REAL val;
    HYD_BOOL ok;

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_POSITION_TOLERANCE, &val);
    ASSERT_TRUE(ok, "Read positionTolerance should succeed");
    ASSERT_FLOAT_EQ(val, 0.1f, 0.001f, "Default positionTolerance should be 0.1");

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_TIMEOUT_LIMIT, &val);
    ASSERT_TRUE(ok, "Read timeoutLimit should succeed");
    ASSERT_FLOAT_EQ(val, 0.0f, 0.001f, "Default timeoutLimit should be 0.0 (disabled)");

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_VELOCITY_TO_FLOW_GAIN, &val);
    ASSERT_TRUE(ok, "Read velocityToFlowGain should succeed");
    ASSERT_FLOAT_EQ(val, 0.2f, 0.001f, "Default velocityToFlowGain should be 0.2");

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_FLOW_TO_PUMP_SPEED_GAIN, &val);
    ASSERT_TRUE(ok, "Read flowToPumpSpeedGain should succeed");
    ASSERT_FLOAT_EQ(val, 20.0f, 0.001f, "Default flowToPumpSpeedGain should be 20.0");

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PUMP_SPEED_LIMIT, &val);
    ASSERT_TRUE(ok, "Read pumpSpeedLimit should succeed");
    ASSERT_FLOAT_EQ(val, 1800.0f, 0.001f, "Default pumpSpeedLimit should be 1800.0");

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_VELOCITY_KP, &val);
    ASSERT_TRUE(ok, "Read velocityKp should succeed");
    ASSERT_FLOAT_EQ(val, 0.0f, 0.001f, "Default velocityKp should be disabled");

    HYD_BOOL bval;
    ok = HYD_MotionControlFB_ReadBoolParameter(&fb, HYD_PARAM_USE_SIMULATION, &bval);
    ASSERT_TRUE(ok, "ReadBool useSimulation should succeed");
    ASSERT_TRUE(bval == false, "Default useSimulation should be false");
}

/* Test: Write then Read round-trip */
static void test_write_read_roundtrip(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    HYD_REAL val;
    HYD_BOOL ok;

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_POSITION_TOLERANCE, 0.05f);
    ASSERT_TRUE(ok, "Write positionTolerance should succeed");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_POSITION_TOLERANCE, &val);
    ASSERT_TRUE(ok, "Read after write should succeed");
    ASSERT_FLOAT_EQ(val, 0.05f, 0.001f, "positionTolerance should be 0.05 after write");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PRESSURE_KP, 1.5f);
    ASSERT_TRUE(ok, "Write pressureKp should succeed");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PRESSURE_KP, &val);
    ASSERT_TRUE(ok, "Read after write should succeed");
    ASSERT_FLOAT_EQ(val, 1.5f, 0.001f, "pressureKp should be 1.5 after write");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PRESSURE_CONTROLLER_TYPE, (HYD_REAL)HYD_PRESSURE_CONTROLLER_PID);
    ASSERT_TRUE(ok, "Write pressureControllerType should succeed");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PRESSURE_CONTROLLER_TYPE, &val);
    ASSERT_TRUE(ok, "Read pressureControllerType should succeed");
    ASSERT_EQ((int)val, (int)HYD_PRESSURE_CONTROLLER_PID, "pressureControllerType should be PID(3)");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_VELOCITY_KP, 2.0f);
    ASSERT_TRUE(ok, "Write velocityKp should succeed");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_VELOCITY_KP, &val);
    ASSERT_TRUE(ok, "Read velocityKp should succeed");
    ASSERT_FLOAT_EQ(val, 2.0f, 0.001f, "velocityKp should round-trip");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_VELOCITY_DEADBAND, 0.1f);
    ASSERT_TRUE(ok, "Write velocityDeadband should succeed");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_VELOCITY_DEADBAND, &val);
    ASSERT_TRUE(ok, "Read velocityDeadband should succeed");
    ASSERT_FLOAT_EQ(val, 0.1f, 0.001f, "velocityDeadband should round-trip");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_VELOCITY_CORRECTION_LIMIT, 20.0f);
    ASSERT_TRUE(ok, "Write velocityCorrectionLimit should succeed");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_VELOCITY_CORRECTION_LIMIT, &val);
    ASSERT_TRUE(ok, "Read velocityCorrectionLimit should succeed");
    ASSERT_FLOAT_EQ(val, 20.0f, 0.001f, "velocityCorrectionLimit should round-trip");
}

/* Test: Write then Read Bool round-trip */
static void test_write_read_bool_roundtrip(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    HYD_BOOL bval;
    HYD_BOOL ok;

    ok = HYD_MotionControlFB_WriteBoolParameter(&fb, HYD_PARAM_USE_SIMULATION, true);
    ASSERT_TRUE(ok, "WriteBool useSimulation should succeed");
    ok = HYD_MotionControlFB_ReadBoolParameter(&fb, HYD_PARAM_USE_SIMULATION, &bval);
    ASSERT_TRUE(ok, "ReadBool after write should succeed");
    ASSERT_TRUE(bval == true, "useSimulation should be true after write");
    ASSERT_TRUE(fb._useSimulation == true, "Legacy _useSimulation field should be synced");
}

/* Test: Invalid paramNumber returns false */
static void test_invalid_param_number(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    HYD_REAL val;
    HYD_BOOL bval;
    HYD_BOOL ok;

    ok = HYD_MotionControlFB_ReadParameter(&fb, -1, &val);
    ASSERT_TRUE(!ok, "ReadParameter with -1 should fail");

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_COUNT, &val);
    ASSERT_TRUE(!ok, "ReadParameter with PARAM_COUNT should fail");

    ok = HYD_MotionControlFB_ReadParameter(&fb, 999, &val);
    ASSERT_TRUE(!ok, "ReadParameter with 999 should fail");

    ok = HYD_MotionControlFB_WriteParameter(&fb, -1, 0.0f);
    ASSERT_TRUE(!ok, "WriteParameter with -1 should fail");

    ok = HYD_MotionControlFB_ReadBoolParameter(&fb, HYD_PARAM_POSITION_TOLERANCE, &bval);
    ASSERT_TRUE(!ok, "ReadBoolParameter with REAL param number should fail");

    ok = HYD_MotionControlFB_WriteBoolParameter(&fb, HYD_PARAM_TIMEOUT_LIMIT, true);
    ASSERT_TRUE(!ok, "WriteBoolParameter with REAL param number should fail");
}

/* Test: Type mismatch — WriteParameter with USE_SIMULATION ID fails */
static void test_type_mismatch_rejected(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    HYD_REAL val;
    HYD_BOOL ok;

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_USE_SIMULATION, &val);
    ASSERT_TRUE(!ok, "ReadParameter with USE_SIMULATION should fail (type mismatch)");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_USE_SIMULATION, 1.0f);
    ASSERT_TRUE(!ok, "WriteParameter with USE_SIMULATION should fail (type mismatch)");
}

/* Test: Legacy field sync on WriteParameter for pump fields */
static void test_legacy_field_sync_on_write(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    HYD_BOOL ok;

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_FLOW_TO_PUMP_SPEED_GAIN, 50.0f);
    ASSERT_TRUE(ok, "Write flowToPumpSpeedGain should succeed");
    ASSERT_FLOAT_EQ(fb.FLOW_TO_PUMP_SPEED_GAIN, 50.0f, 0.001f, "Legacy FLOW_TO_PUMP_SPEED_GAIN should be 50.0");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PUMP_SPEED_LIMIT, 3000.0f);
    ASSERT_TRUE(ok, "Write pumpSpeedLimit should succeed");
    ASSERT_FLOAT_EQ(fb.PUMP_SPEED_LIMIT, 3000.0f, 0.001f, "Legacy PUMP_SPEED_LIMIT should be 3000.0");

    ok = HYD_MotionControlFB_WriteBoolParameter(&fb, HYD_PARAM_USE_SIMULATION, true);
    ASSERT_TRUE(ok, "WriteBool useSimulation should succeed");
    ASSERT_TRUE(fb._useSimulation == true, "Legacy _useSimulation should be synced to true");
}

/* Test: Init syncs legacy fields from _params defaults */
static void test_init_syncs_legacy_fields(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    ASSERT_FLOAT_EQ(fb.FLOW_TO_PUMP_SPEED_GAIN, 20.0f, 0.001f,
                    "Legacy FLOW_TO_PUMP_SPEED_GAIN should be 20.0 after Init");
    ASSERT_FLOAT_EQ(fb.PUMP_SPEED_LIMIT, 1800.0f, 0.001f,
                    "Legacy PUMP_SPEED_LIMIT should be 1800.0 after Init");
    ASSERT_TRUE(fb._useSimulation == false,
                "Legacy _useSimulation should be false after Init");
}

int main(void) {
    test_init_sets_defaults();
    test_init_syncs_legacy_fields();
    test_write_read_roundtrip();
    test_write_read_bool_roundtrip();
    test_invalid_param_number();
    test_type_mismatch_rejected();
    test_legacy_field_sync_on_write();

    printf("Parameter access tests: %d/%d passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
