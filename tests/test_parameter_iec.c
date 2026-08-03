#include <stdio.h>
#include <math.h>
#include <string.h>
#include "motion_interface.h"
#include "motion_control.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { printf("  FAIL: %s\n", msg); } \
} while (0)

#define ASSERT_FLOAT_EQ(a, b, eps, msg) do { \
    tests_run++; \
    HYD_REAL diff = (a) - (b); \
    if (diff < 0.0f) diff = -diff; \
    if (diff <= (eps)) { tests_passed++; } \
    else { printf("  FAIL: %s (got %.6f, expected %.6f)\n", msg, (double)(a), (double)(b)); } \
} while (0)

static void ensure_axis_allocated(void) {
    HYD_CREATEMOTION cm;
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 20.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 1800.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);
}

/* Test: ReadParameter works through IEC FB */
static void test_read_parameter_iec(void) {
    __HydMotion_framework_Init();
    ensure_axis_allocated();

    HYD_READPARAMETER rp;
    memset(&rp, 0, sizeof(rp));
    IEC_VAL(rp.EN) = true;
    IEC_VAL(rp.AXISID) = 0;
    IEC_VAL(rp.ENABLE) = true;
    IEC_VAL(rp.PARAMETERNUMBER) = HYD_PARAM_POSITION_TOLERANCE;

    __mcl_cmd_ReadParameter(&rp);

    ASSERT_TRUE(IEC_VAL(rp.VALID) == true, "ReadParameter should set VALID");
    ASSERT_TRUE(IEC_VAL(rp.ERROR) == false, "ReadParameter should not set ERROR");
    ASSERT_FLOAT_EQ((HYD_REAL)IEC_VAL(rp.VALUE), 0.0001f, 0.00001f,
                    "Default positionTolerance should be 0.0001");
}

/* Test: WriteParameter then ReadParameter through IEC FBs */
static void test_write_then_read_iec(void) {
    __HydMotion_framework_Init();
    ensure_axis_allocated();

    HYD_WRITEPARAMETER wp;
    memset(&wp, 0, sizeof(wp));
    IEC_VAL(wp.EN) = true;
    IEC_VAL(wp.AXISID) = 0;
    IEC_VAL(wp.PARAMETERNUMBER) = HYD_PARAM_PRESSURE_KP;
    IEC_VAL(wp.VALUE) = 2.5;
    IEC_VAL(wp.EXECUTE) = true;

    __mcl_cmd_WriteParameter(&wp);

    ASSERT_TRUE(IEC_VAL(wp.DONE) == true, "WriteParameter should set DONE");
    ASSERT_TRUE(IEC_VAL(wp.ERROR) == false, "WriteParameter should not set ERROR");

    /* Reset EXECUTE for next call */
    IEC_VAL(wp.EXECUTE) = false;
    __mcl_cmd_WriteParameter(&wp);

    HYD_READPARAMETER rp;
    memset(&rp, 0, sizeof(rp));
    IEC_VAL(rp.EN) = true;
    IEC_VAL(rp.AXISID) = 0;
    IEC_VAL(rp.ENABLE) = true;
    IEC_VAL(rp.PARAMETERNUMBER) = HYD_PARAM_PRESSURE_KP;

    __mcl_cmd_ReadParameter(&rp);

    ASSERT_TRUE(IEC_VAL(rp.VALID) == true, "ReadParameter after write should set VALID");
    ASSERT_FLOAT_EQ((HYD_REAL)IEC_VAL(rp.VALUE), 2.5f, 0.001f, "pressureKp should be 2.5 after write");
}

/* Test: PI-RBF strategy is selectable through the IEC parameter FB. */
static void test_write_pi_rbf_controller_through_iec(void) {
    __HydMotion_framework_Init();
    ensure_axis_allocated();

    HYD_WRITEPARAMETER wp;
    memset(&wp, 0, sizeof(wp));
    IEC_VAL(wp.EN) = true;
    IEC_VAL(wp.AXISID) = 0;
    IEC_VAL(wp.PARAMETERNUMBER) = HYD_PARAM_PRESSURE_CONTROLLER_TYPE;
    IEC_VAL(wp.VALUE) = (IEC_LREAL)HYD_PRESSURE_CONTROLLER_PI_RBF;
    IEC_VAL(wp.EXECUTE) = true;

    __mcl_cmd_WriteParameter(&wp);

    ASSERT_TRUE(IEC_VAL(wp.DONE) == true,
                "IEC WriteParameter should accept PI-RBF controller type");
    ASSERT_TRUE(IEC_VAL(wp.ERROR) == false,
                "IEC WriteParameter should not reject PI-RBF controller type");

    IEC_VAL(wp.EXECUTE) = false;
    __mcl_cmd_WriteParameter(&wp);

    HYD_READPARAMETER rp;
    memset(&rp, 0, sizeof(rp));
    IEC_VAL(rp.EN) = true;
    IEC_VAL(rp.AXISID) = 0;
    IEC_VAL(rp.ENABLE) = true;
    IEC_VAL(rp.PARAMETERNUMBER) = HYD_PARAM_PRESSURE_CONTROLLER_TYPE;
    __mcl_cmd_ReadParameter(&rp);

    ASSERT_TRUE(IEC_VAL(rp.VALID) == true,
                "IEC ReadParameter should report PI-RBF as valid");
    ASSERT_FLOAT_EQ((HYD_REAL)IEC_VAL(rp.VALUE),
                    (HYD_REAL)HYD_PRESSURE_CONTROLLER_PI_RBF,
                    0.001f,
                    "IEC ReadParameter should return PI-RBF controller type");
}

/* Test: WriteBoolParameter then ReadBoolParameter through IEC FBs */
static void test_write_read_bool_iec(void) {
    __HydMotion_framework_Init();
    ensure_axis_allocated();

    HYD_WRITEBOOLPARAMETER wbp;
    memset(&wbp, 0, sizeof(wbp));
    IEC_VAL(wbp.EN) = true;
    IEC_VAL(wbp.AXISID) = 0;
    IEC_VAL(wbp.PARAMETERNUMBER) = HYD_PARAM_USE_SIMULATION;
    IEC_VAL(wbp.VALUE) = true;
    IEC_VAL(wbp.EXECUTE) = true;

    __mcl_cmd_WriteBoolParameter(&wbp);

    ASSERT_TRUE(IEC_VAL(wbp.DONE) == true, "WriteBoolParameter should set DONE");

    HYD_READBOOLPARAMETER rbp;
    memset(&rbp, 0, sizeof(rbp));
    IEC_VAL(rbp.EN) = true;
    IEC_VAL(rbp.AXISID) = 0;
    IEC_VAL(rbp.ENABLE) = true;
    IEC_VAL(rbp.PARAMETERNUMBER) = HYD_PARAM_USE_SIMULATION;

    __mcl_cmd_ReadBoolParameter(&rbp);

    ASSERT_TRUE(IEC_VAL(rbp.VALID) == true, "ReadBoolParameter should set VALID");
    ASSERT_TRUE(IEC_VAL(rbp.VALUE) == true, "useSimulation should be true after write");
}

/* Test: Invalid AXISID returns ERROR */
static void test_invalid_axisid_iec(void) {
    __HydMotion_framework_Init();

    HYD_READPARAMETER rp;
    memset(&rp, 0, sizeof(rp));
    IEC_VAL(rp.EN) = true;
    IEC_VAL(rp.AXISID) = -1;
    IEC_VAL(rp.ENABLE) = true;

    __mcl_cmd_ReadParameter(&rp);

    ASSERT_TRUE(IEC_VAL(rp.ERROR) == true, "ReadParameter with invalid AXISID should set ERROR");
    ASSERT_TRUE(IEC_VAL(rp.VALID) == false, "ReadParameter with invalid AXISID should not set VALID");
}

/* Test: Segment builder uses FB params after WriteParameter */
static void test_segment_builder_uses_fb_params(void) {
    __HydMotion_framework_Init();
    ensure_axis_allocated();

    HYD_WRITEPARAMETER wp;
    memset(&wp, 0, sizeof(wp));
    IEC_VAL(wp.EN) = true;
    IEC_VAL(wp.AXISID) = 0;
    IEC_VAL(wp.PARAMETERNUMBER) = HYD_PARAM_POSITION_TOLERANCE;
    IEC_VAL(wp.VALUE) = 0.025;
    IEC_VAL(wp.EXECUTE) = true;
    __mcl_cmd_WriteParameter(&wp);

    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "FB should exist");
    ASSERT_FLOAT_EQ(fb->_params.positionTolerance, 0.025f, 0.001f,
                    "FB _params.positionTolerance should be 0.025 after WriteParameter");
}

static void test_read_status_reports_applied_pressure_controller(void) {
    HYD_READSTATUS rs;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axis_allocated();
    fb = __MK_GetPublic_MotionControlFB(0);
    fb->STATE.pressureControllerApplied = HYD_PRESSURE_CONTROLLER_RBF_PI;

    memset(&rs, 0, sizeof(rs));
    IEC_VAL(rs.EN) = true;
    IEC_VAL(rs.AXISID) = 0;
    IEC_VAL(rs.ENABLE) = true;
    __mcl_cmd_ReadStatus(&rs);

    ASSERT_TRUE(IEC_VAL(rs.PRESSURECONTROLLERAPPLIED) ==
                    (IEC_INT)HYD_PRESSURE_CONTROLLER_RBF_PI,
                "ReadStatus should expose the applied RBF-PI controller");

    IEC_VAL(rs.ENABLE) = false;
    __mcl_cmd_ReadStatus(&rs);
    ASSERT_TRUE(IEC_VAL(rs.PRESSURECONTROLLERAPPLIED) ==
                    (IEC_INT)HYD_PRESSURE_CONTROLLER_NONE,
                "ReadStatus should clear applied controller when disabled");
}

static void test_read_status_reports_mechanism_state(void) {
    HYD_READSTATUS rs;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axis_allocated();
    fb = __MK_GetPublic_MotionControlFB(0);
    fb->STATE.mechanismType = HYD_MECHANISM_FIVE_POINT_TOGGLE;
    fb->STATE.actuatorDirection = HYD_DIRECTION_NEGATIVE;
    fb->STATE.mechanismConfigVersion = 7u;
    fb->STATE.limitFlags = HYD_LIMIT_FLAG_FLOW | HYD_LIMIT_FLAG_PUMP_SPEED |
                          HYD_LIMIT_FLAG_PRESSURE | HYD_LIMIT_FLAG_SOFT |
                          HYD_LIMIT_FLAG_DERATE;
#if HYD_ENABLE_FLOW_DIAGNOSTIC_TELEMETRY
    fb->STATE.requestedFlow = 8.5f;
    fb->STATE.maxFlow = 12.0f;
    fb->STATE.maxTemplateVelocity = 42.0f;
    fb->STATE.effectiveCylinderGain = 0.4f;
#endif
#if HYD_ENABLE_MECHANISM_TELEMETRY
    fb->STATE.actuatorPosition = 12.5f;
    fb->STATE.actuatorVelocityCommand = -3.25f;
    fb->STATE.velocityRatio = -0.75f;
#endif

    memset(&rs, 0, sizeof(rs));
    IEC_VAL(rs.AXISID) = 0;
    IEC_VAL(rs.ENABLE) = true;
    __mcl_cmd_ReadStatus(&rs);

    ASSERT_TRUE(IEC_VAL(rs.MECHANISMTYPE) == HYD_MECHANISM_FIVE_POINT_TOGGLE,
                "ReadStatus should expose mechanism type");
    ASSERT_TRUE(IEC_VAL(rs.ACTUATORDIRECTION) == HYD_DIRECTION_NEGATIVE,
                "ReadStatus should expose actuator direction");
    ASSERT_TRUE(IEC_VAL(rs.MECHANISMCONFIGVERSION) == 7u,
                "ReadStatus should expose mechanism config version");
#if HYD_ENABLE_FLOW_DIAGNOSTIC_TELEMETRY
    ASSERT_FLOAT_EQ(IEC_VAL(rs.REQUESTEDFLOW), 8.5f, 0.001f,
                    "ReadStatus should expose requested flow before limiting");
    ASSERT_FLOAT_EQ(IEC_VAL(rs.MAXFLOW), 12.0f, 0.001f,
                    "ReadStatus should expose pump hardware flow capability");
    ASSERT_FLOAT_EQ(IEC_VAL(rs.MAXTEMPLATEVELOCITY), 42.0f, 0.001f,
                    "ReadStatus should expose flow-limited template speed");
    ASSERT_FLOAT_EQ(IEC_VAL(rs.EFFECTIVECYLINDERGAIN), 0.4f, 0.001f,
                    "ReadStatus should expose active chamber gain");
#else
    ASSERT_FLOAT_EQ(IEC_VAL(rs.REQUESTEDFLOW), 0.0f, 0.0f,
                    "ReadStatus should omit detailed flow telemetry when disabled");
#endif
    ASSERT_TRUE(IEC_VAL(rs.FLOWLIMITACTIVE) && IEC_VAL(rs.PUMPSPEEDLIMITACTIVE) &&
                IEC_VAL(rs.PRESSURELIMITACTIVE) && IEC_VAL(rs.SOFTLIMITACTIVE) &&
                IEC_VAL(rs.DERATED),
                "ReadStatus should expose active limit causes");
#if HYD_ENABLE_MECHANISM_TELEMETRY
    ASSERT_FLOAT_EQ(IEC_VAL(rs.ACTUATORPOSITION), 12.5f, 0.001f,
                    "ReadStatus should expose actuator position");
    ASSERT_FLOAT_EQ(IEC_VAL(rs.ACTUATORVELOCITYCOMMAND), -3.25f, 0.001f,
                    "ReadStatus should expose actuator velocity command");
    ASSERT_FLOAT_EQ(IEC_VAL(rs.VELOCITYRATIO), -0.75f, 0.001f,
                    "ReadStatus should expose velocity ratio");
#endif

    IEC_VAL(rs.ENABLE) = false;
    __mcl_cmd_ReadStatus(&rs);
    ASSERT_TRUE(IEC_VAL(rs.MECHANISMTYPE) == HYD_MECHANISM_DIRECT,
                "Disabled ReadStatus should clear mechanism type");
    ASSERT_TRUE(IEC_VAL(rs.ACTUATORDIRECTION) == HYD_DIRECTION_HOLD,
                "Disabled ReadStatus should clear actuator direction");
    ASSERT_TRUE(IEC_VAL(rs.MECHANISMCONFIGVERSION) == 0u,
                "Disabled ReadStatus should clear mechanism version");
    ASSERT_FLOAT_EQ(IEC_VAL(rs.ACTUATORPOSITION), 0.0f, 0.0f,
                    "Disabled ReadStatus should clear actuator position");
    ASSERT_FLOAT_EQ(IEC_VAL(rs.ACTUATORVELOCITYCOMMAND), 0.0f, 0.0f,
                    "Disabled ReadStatus should clear actuator velocity");
    ASSERT_FLOAT_EQ(IEC_VAL(rs.VELOCITYRATIO), 0.0f, 0.0f,
                    "Disabled ReadStatus should clear velocity ratio");
    ASSERT_FLOAT_EQ(IEC_VAL(rs.REQUESTEDFLOW), 0.0f, 0.0f,
                    "Disabled ReadStatus should clear requested flow");
    ASSERT_TRUE(!IEC_VAL(rs.FLOWLIMITACTIVE) && !IEC_VAL(rs.PUMPSPEEDLIMITACTIVE) &&
                !IEC_VAL(rs.PRESSURELIMITACTIVE) && !IEC_VAL(rs.SOFTLIMITACTIVE) &&
                !IEC_VAL(rs.DERATED),
                "Disabled ReadStatus should clear limit causes");
}

static void test_pressure_handle_preserves_legacy_max_flow(void) {
    HYD_PRESSUREHANDLE ph;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axis_allocated();
    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(HYD_MotionControlFB_WriteParameter(fb, HYD_PARAM_MAX_FLOW, 37.0),
                "Max flow parameter should be writable");

    memset(&ph, 0, sizeof(ph));
    IEC_VAL(ph.EN) = true;
    IEC_VAL(ph.EXECUTE) = true;
    IEC_VAL(ph.AXISID) = 0;
    IEC_VAL(ph.PRESSURE) = 10.0;
    IEC_VAL(ph.DURATION) = 1.0;
    __mcl_cmd_PressureHandle(&ph);

    ASSERT_TRUE(fabsf((float)(fb->_activeSegment.maxFlow - 20.0)) < 0.001f,
                "PressureHandle should preserve its established 20 L/min limit");
}

int main(void) {
    test_read_parameter_iec();
    test_write_then_read_iec();
    test_write_pi_rbf_controller_through_iec();
    test_write_read_bool_iec();
    test_invalid_axisid_iec();
    test_segment_builder_uses_fb_params();
    test_read_status_reports_applied_pressure_controller();
    test_read_status_reports_mechanism_state();
    test_pressure_handle_preserves_legacy_max_flow();

    printf("IEC parameter FB tests: %d/%d passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
