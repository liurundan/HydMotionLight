/*
 * IEC servo-pump feedback ingress / read-back contract.
 *
 * Covers:
 *   - native HYD_MotionControlFB_SetPumpFeedback / GetPumpFeedback semantics
 *     (angle canonicalisation, validity-bit masking, non-finite handling)
 *   - IEC __mcl_cmd_SetPumpFeedback / __mcl_cmd_ReadPumpFeedback round-trip
 *   - simulation-axis suppression (the plant model owns feedback there)
 *   - lifetime: cleared by Init() and by SoftReset(), like AXIS_REF
 *
 * Units under test: rpm, permille of rated torque, deg, s.
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { tests_failed++; printf("  FAIL [line %d]: %s\n", __LINE__, msg); } \
} while (0)

#define ASSERT_NEAR(actual, expected, tol, msg) do { \
    tests_run++; \
    if (fabs((double)(actual) - (double)(expected)) <= (double)(tol)) { tests_passed++; } \
    else { \
        tests_failed++; \
        printf("  FAIL [line %d]: %s (got %.6f, want %.6f)\n", \
               __LINE__, msg, (double)(actual), (double)(expected)); \
    } \
} while (0)

static int create_axis(bool useSimulation) {
    HYD_CREATEMOTION cm;

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = useSimulation;
    __mcl_cmd_CreateMotion(&cm);
    return (int)IEC_VAL(cm.AXISID);
}

static HYD_PumpFeedback read_pump_feedback(int axisId, bool* validOut) {
    HYD_READPUMPFEEDBACK rd;
    HYD_PumpFeedback out;

    memset(&rd, 0, sizeof(rd));
    IEC_VAL(rd.EN) = true;
    IEC_VAL(rd.AXISID) = (IEC_SINT)axisId;
    IEC_VAL(rd.ENABLE) = true;
    __mcl_cmd_ReadPumpFeedback(&rd);

    out.rpm = (HYD_REAL)IEC_VAL(rd.FB_RPM);
    out.torquePermille = (HYD_REAL)IEC_VAL(rd.FB_TORQUE);
    out.angleDeg = (HYD_REAL)IEC_VAL(rd.FB_ANGLE);
    out.timestamp = (HYD_REAL)IEC_VAL(rd.FB_TIMESTAMP);
    out.validFlags = (uint32_t)IEC_VAL(rd.FLAGS);
    if (validOut != NULL) {
        *validOut = (IEC_VAL(rd.VALID) == true);
    }
    return out;
}

/* ---------------------------------------------------------------- native --- */

static void test_native_round_trip(void) {
    int axisId = create_axis(false);
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    HYD_PumpFeedback in;
    HYD_PumpFeedback out;

    memset(&in, 0, sizeof(in));
    in.rpm = 1234.5f;
    in.torquePermille = 642.0f;
    in.angleDeg = 137.25f;
    in.timestamp = 12.5f;
    in.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM |
                    HYD_PUMP_FEEDBACK_VALID_TORQUE |
                    HYD_PUMP_FEEDBACK_VALID_ANGLE |
                    HYD_PUMP_FEEDBACK_VALID_TIMESTAMP;

    ASSERT_TRUE(HYD_MotionControlFB_SetPumpFeedback(fb, &in) == true,
                "SetPumpFeedback accepts a valid packet");
    memset(&out, 0, sizeof(out));
    ASSERT_TRUE(HYD_MotionControlFB_GetPumpFeedback(fb, &out) == true,
                "GetPumpFeedback succeeds on a valid FB");

    ASSERT_NEAR(out.rpm, 1234.5f, 1e-3f, "feedback rpm stored verbatim");
    ASSERT_NEAR(out.torquePermille, 642.0f, 1e-3f, "feedback torque stored verbatim");
    ASSERT_NEAR(out.angleDeg, 137.25f, 1e-3f, "feedback angle stored verbatim");
    ASSERT_NEAR(out.timestamp, 12.5f, 1e-3f, "feedback timestamp stored verbatim");
    ASSERT_TRUE(out.validFlags == in.validFlags, "validFlags round-trip");
}

static void test_native_negative_rpm_preserved(void) {
    int axisId = create_axis(false);
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    HYD_PumpFeedback in = {0};
    HYD_PumpFeedback out = {0};

    /* A reversing pump (fast pressure relief) reports negative rpm.
     * It must survive the ingress unchanged. */
    in.rpm = -75.0f;
    in.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM;
    ASSERT_TRUE(HYD_MotionControlFB_SetPumpFeedback(fb, &in) == true, "set negative rpm");
    (void)HYD_MotionControlFB_GetPumpFeedback(fb, &out);
    ASSERT_NEAR(out.rpm, -75.0f, 1e-3f, "negative (reversing) rpm preserved");
    ASSERT_TRUE((out.validFlags & HYD_PUMP_FEEDBACK_VALID_RPM) != 0u,
                "rpm validity bit set for negative rpm");
}

static void test_native_angle_canonicalisation(void) {
    int axisId = create_axis(false);
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    HYD_PumpFeedback in = {0};
    HYD_PumpFeedback out = {0};

    in.validFlags = HYD_PUMP_FEEDBACK_VALID_ANGLE;

    in.angleDeg = 370.0f;
    (void)HYD_MotionControlFB_SetPumpFeedback(fb, &in);
    (void)HYD_MotionControlFB_GetPumpFeedback(fb, &out);
    ASSERT_NEAR(out.angleDeg, 10.0f, 1e-3f, "angle > 360 wraps into [0,360)");

    in.angleDeg = -90.0f;
    (void)HYD_MotionControlFB_SetPumpFeedback(fb, &in);
    (void)HYD_MotionControlFB_GetPumpFeedback(fb, &out);
    ASSERT_NEAR(out.angleDeg, 270.0f, 1e-3f, "negative angle wraps into [0,360)");

    in.angleDeg = 360.0f;
    (void)HYD_MotionControlFB_SetPumpFeedback(fb, &in);
    (void)HYD_MotionControlFB_GetPumpFeedback(fb, &out);
    ASSERT_NEAR(out.angleDeg, 0.0f, 1e-3f, "exactly 360 wraps to 0");
}

static void test_native_masks_unknown_valid_flags(void) {
    int axisId = create_axis(false);
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    HYD_PumpFeedback in = {0};
    HYD_PumpFeedback out = {0};

    in.rpm = 10.0f;
    in.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM | 0xFFFF0000u | (1u << 30);
    ASSERT_TRUE(HYD_MotionControlFB_SetPumpFeedback(fb, &in) == true, "set with junk bits");
    (void)HYD_MotionControlFB_GetPumpFeedback(fb, &out);
    ASSERT_TRUE(out.validFlags == HYD_PUMP_FEEDBACK_VALID_RPM,
                "unknown validity bits are dropped");
}

static void test_native_partial_feedback_keeps_other_bits_clear(void) {
    int axisId = create_axis(false);
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    HYD_PumpFeedback in = {0};
    HYD_PumpFeedback out = {0};

    /* Drive with rpm + angle only (no torque channel). */
    in.rpm = 1200.0f;
    in.angleDeg = 30.0f;
    in.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM | HYD_PUMP_FEEDBACK_VALID_ANGLE;
    (void)HYD_MotionControlFB_SetPumpFeedback(fb, &in);
    (void)HYD_MotionControlFB_GetPumpFeedback(fb, &out);

    ASSERT_TRUE((out.validFlags & HYD_PUMP_FEEDBACK_VALID_RPM) != 0u, "rpm valid");
    ASSERT_TRUE((out.validFlags & HYD_PUMP_FEEDBACK_VALID_ANGLE) != 0u, "angle valid");
    ASSERT_TRUE((out.validFlags & HYD_PUMP_FEEDBACK_VALID_TORQUE) == 0u,
                "torque reported as unknown, not as zero");
}

static void test_native_rejects_non_finite(void) {
    int axisId = create_axis(false);
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    HYD_PumpFeedback in = {0};
    HYD_PumpFeedback out = {0};

    in.rpm = NAN;
    in.torquePermille = INFINITY;
    in.angleDeg = -INFINITY;
    in.timestamp = NAN;
    in.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM |
                    HYD_PUMP_FEEDBACK_VALID_TORQUE |
                    HYD_PUMP_FEEDBACK_VALID_ANGLE |
                    HYD_PUMP_FEEDBACK_VALID_TIMESTAMP;
    ASSERT_TRUE(HYD_MotionControlFB_SetPumpFeedback(fb, &in) == true,
                "non-finite packet is normalised, not rejected outright");
    (void)HYD_MotionControlFB_GetPumpFeedback(fb, &out);

    ASSERT_TRUE(out.rpm == 0.0f, "non-finite rpm -> 0");
    ASSERT_TRUE(out.torquePermille == 0.0f, "non-finite torque -> 0");
    ASSERT_TRUE(out.angleDeg == 0.0f, "non-finite angle -> 0");
    ASSERT_TRUE(out.timestamp == 0.0f, "non-finite timestamp -> 0");
    /* Validity is a separate statement: the driver still claims those fields. */
    ASSERT_TRUE(out.validFlags == in.validFlags,
                "validity bits preserved even when values were sanitised");
}

static void test_native_null_safety(void) {
    int axisId = create_axis(false);
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    HYD_PumpFeedback packet = {0};

    ASSERT_TRUE(HYD_MotionControlFB_SetPumpFeedback(NULL, &packet) == false,
                "SetPumpFeedback(NULL fb) -> false");
    ASSERT_TRUE(HYD_MotionControlFB_SetPumpFeedback(fb, NULL) == false,
                "SetPumpFeedback(NULL packet) -> false");
    ASSERT_TRUE(HYD_MotionControlFB_GetPumpFeedback(NULL, &packet) == false,
                "GetPumpFeedback(NULL fb) -> false");
    ASSERT_TRUE(HYD_MotionControlFB_GetPumpFeedback(fb, NULL) == false,
                "GetPumpFeedback(NULL packet) -> false");
}

/* ------------------------------------------------------------------- IEC --- */

static void test_iec_round_trip(void) {
    int axisId = create_axis(false);
    HYD_SETPUMPFEEDBACK wr;
    HYD_PumpFeedback out = {0};
    bool valid = false;

    memset(&wr, 0, sizeof(wr));
    IEC_VAL(wr.EN) = true;
    IEC_VAL(wr.AXISID) = (IEC_SINT)axisId;
    IEC_VAL(wr.ENABLE) = true;
    IEC_VAL(wr.FB_RPM) = -450.25f;
    IEC_VAL(wr.FB_TORQUE) = 823.0f;
    IEC_VAL(wr.FB_ANGLE) = 412.5f;   /* must be wrapped to 52.5 */
    IEC_VAL(wr.FB_TIMESTAMP) = 33.5f;
    IEC_VAL(wr.VALID_RPM) = true;
    IEC_VAL(wr.VALID_TORQUE) = true;
    IEC_VAL(wr.VALID_ANGLE) = true;
    IEC_VAL(wr.VALID_TIMESTAMP) = true;
    __mcl_cmd_SetPumpFeedback(&wr);

    ASSERT_TRUE(IEC_VAL(wr.DONE) == true, "SetPumpFeedback raises DONE");
    ASSERT_TRUE(IEC_VAL(wr.ERROR) == false, "SetPumpFeedback clears ERROR");

    out = read_pump_feedback(axisId, &valid);
    ASSERT_TRUE(valid == true, "read-back VALID is true after a full write");
    ASSERT_NEAR(out.rpm, -450.25f, 1e-3f, "IEC rpm round-trip");
    ASSERT_NEAR(out.torquePermille, 823.0f, 1e-3f, "IEC torque round-trip");
    ASSERT_NEAR(out.angleDeg, 52.5f, 1e-3f, "IEC angle wrapped on write");
    ASSERT_NEAR(out.timestamp, 33.5f, 1e-3f, "IEC timestamp round-trip");
    ASSERT_TRUE(out.validFlags == (HYD_PUMP_FEEDBACK_VALID_RPM |
                                   HYD_PUMP_FEEDBACK_VALID_TORQUE |
                                   HYD_PUMP_FEEDBACK_VALID_ANGLE |
                                   HYD_PUMP_FEEDBACK_VALID_TIMESTAMP),
                "IEC validFlags carries all four bits");
}

static void test_iec_partial_validity_flags(void) {
    int axisId = create_axis(false);
    HYD_SETPUMPFEEDBACK wr;
    HYD_PumpFeedback out = {0};
    bool valid = false;

    memset(&wr, 0, sizeof(wr));
    IEC_VAL(wr.EN) = true;
    IEC_VAL(wr.AXISID) = (IEC_SINT)axisId;
    IEC_VAL(wr.ENABLE) = true;
    IEC_VAL(wr.FB_RPM) = 900.0f;
    IEC_VAL(wr.VALID_RPM) = true;          /* only rpm is trustworthy */
    IEC_VAL(wr.FB_TORQUE) = 123.0f;        /* present but explicitly invalid */
    IEC_VAL(wr.FB_ANGLE) = 45.0f;
    __mcl_cmd_SetPumpFeedback(&wr);

    out = read_pump_feedback(axisId, &valid);
    ASSERT_TRUE(valid == true, "partial feedback still counts as valid");
    ASSERT_TRUE((out.validFlags & HYD_PUMP_FEEDBACK_VALID_RPM) != 0u, "rpm bit set");
    ASSERT_TRUE((out.validFlags & HYD_PUMP_FEEDBACK_VALID_TORQUE) == 0u,
                "torque bit not set when VALID_TORQUE is false");
    ASSERT_TRUE((out.validFlags & HYD_PUMP_FEEDBACK_VALID_ANGLE) == 0u,
                "angle bit not set when VALID_ANGLE is false");
}

static void test_iec_no_feedback_reports_invalid(void) {
    int axisId = create_axis(false);
    HYD_PumpFeedback out;
    bool valid = true;

    /* Freshly created axis: nothing has been written yet. */
    out = read_pump_feedback(axisId, &valid);
    ASSERT_TRUE(valid == false, "VALID is false when no feedback has been written");
    ASSERT_TRUE(out.validFlags == 0u, "FLAGS is zero when no feedback has been written");
    ASSERT_TRUE(out.rpm == 0.0f, "rpm reads 0 before any write");
}

static void test_iec_simulation_axis_is_suppressed(void) {
    int axisId = create_axis(true);  /* _useSimulation = true */
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    HYD_SETPUMPFEEDBACK wr;
    HYD_PumpFeedback before = {0};
    HYD_PumpFeedback after = {0};
    bool valid = false;

    before = read_pump_feedback(axisId, &valid);

    memset(&wr, 0, sizeof(wr));
    IEC_VAL(wr.EN) = true;
    IEC_VAL(wr.AXISID) = (IEC_SINT)axisId;
    IEC_VAL(wr.ENABLE) = true;
    IEC_VAL(wr.FB_RPM) = 1500.0f;
    IEC_VAL(wr.VALID_RPM) = true;
    __mcl_cmd_SetPumpFeedback(&wr);

    ASSERT_TRUE(IEC_VAL(wr.DONE) == true,
                "SetPumpFeedback still reports DONE on a simulation axis");

    after = read_pump_feedback(axisId, &valid);
    ASSERT_TRUE(after.rpm == before.rpm,
                "simulation axis pump feedback is not overridden by the PLC");
    ASSERT_TRUE(fb->_useSimulation == true, "axis is indeed in simulation mode");
}

static void test_iec_disable_leaves_packet_untouched(void) {
    int axisId = create_axis(false);
    HYD_SETPUMPFEEDBACK wr;
    HYD_PumpFeedback out = {0};
    bool valid = false;

    memset(&wr, 0, sizeof(wr));
    IEC_VAL(wr.EN) = true;
    IEC_VAL(wr.AXISID) = (IEC_SINT)axisId;
    IEC_VAL(wr.ENABLE) = true;
    IEC_VAL(wr.FB_RPM) = 700.0f;
    IEC_VAL(wr.VALID_RPM) = true;
    __mcl_cmd_SetPumpFeedback(&wr);

    /* Now disable: the packet must keep its last good value, not be zeroed. */
    IEC_VAL(wr.ENABLE) = false;
    IEC_VAL(wr.FB_RPM) = 0.0f;
    IEC_VAL(wr.VALID_RPM) = false;
    __mcl_cmd_SetPumpFeedback(&wr);

    out = read_pump_feedback(axisId, &valid);
    ASSERT_NEAR(out.rpm, 700.0f, 1e-3f, "ENABLE=false leaves the last packet in place");
}

static void test_iec_read_invalid_axis_reports_error(void) {
    HYD_READPUMPFEEDBACK rd;

    memset(&rd, 0, sizeof(rd));
    IEC_VAL(rd.EN) = true;
    IEC_VAL(rd.AXISID) = (IEC_SINT)99;   /* out of range / not allocated */
    IEC_VAL(rd.ENABLE) = true;
    __mcl_cmd_ReadPumpFeedback(&rd);

    ASSERT_TRUE(IEC_VAL(rd.ERROR) == true, "read on unallocated axis raises ERROR");
    ASSERT_TRUE(IEC_VAL(rd.ERRORID) == (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID,
                "read on unallocated axis reports START_CONTEXT_INVALID");
    ASSERT_TRUE(IEC_VAL(rd.VALID) == false, "read on unallocated axis reports VALID=false");
    ASSERT_TRUE((HYD_REAL)IEC_VAL(rd.FB_RPM) == 0.0f, "read on unallocated axis zeroes rpm");
}

static void test_iec_set_invalid_axis_reports_error(void) {
    HYD_SETPUMPFEEDBACK wr;

    memset(&wr, 0, sizeof(wr));
    IEC_VAL(wr.EN) = true;
    IEC_VAL(wr.AXISID) = (IEC_SINT)99;
    IEC_VAL(wr.ENABLE) = true;
    IEC_VAL(wr.FB_RPM) = 100.0f;
    IEC_VAL(wr.VALID_RPM) = true;
    __mcl_cmd_SetPumpFeedback(&wr);

    ASSERT_TRUE(IEC_VAL(wr.ERROR) == true, "set on unallocated axis raises ERROR");
    ASSERT_TRUE(IEC_VAL(wr.ERRORID) == (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID,
                "set on unallocated axis reports START_CONTEXT_INVALID");
}

static void test_iec_read_disable_zeroes_outputs(void) {
    int axisId = create_axis(false);
    HYD_SETPUMPFEEDBACK wr;
    HYD_READPUMPFEEDBACK rd;

    memset(&wr, 0, sizeof(wr));
    IEC_VAL(wr.EN) = true;
    IEC_VAL(wr.AXISID) = (IEC_SINT)axisId;
    IEC_VAL(wr.ENABLE) = true;
    IEC_VAL(wr.FB_RPM) = 111.0f;
    IEC_VAL(wr.VALID_RPM) = true;
    __mcl_cmd_SetPumpFeedback(&wr);

    memset(&rd, 0, sizeof(rd));
    IEC_VAL(rd.EN) = true;
    IEC_VAL(rd.AXISID) = (IEC_SINT)axisId;
    IEC_VAL(rd.ENABLE) = false;
    __mcl_cmd_ReadPumpFeedback(&rd);

    ASSERT_TRUE(IEC_VAL(rd.VALID) == false, "ENABLE=false reports VALID=false");
    ASSERT_TRUE((HYD_REAL)IEC_VAL(rd.FB_RPM) == 0.0f, "ENABLE=false zeroes rpm output");
    ASSERT_TRUE((IEC_WORD)IEC_VAL(rd.FLAGS) == 0u, "ENABLE=false zeroes FLAGS");
    ASSERT_TRUE(IEC_VAL(rd.ERROR) == false, "ENABLE=false is not an error");
}

/* --------------------------------------------------------------- lifetime --- */

static void test_feedback_cleared_by_init_and_soft_reset(void) {
    int axisId = create_axis(false);
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    HYD_PumpFeedback in = {0};
    HYD_PumpFeedback out = {0};

    in.rpm = 555.0f;
    in.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM;
    (void)HYD_MotionControlFB_SetPumpFeedback(fb, &in);

    /* SoftReset clears feedback exactly like the AXIS_REF ACT_* half;
     * the HAL refreshes it on the next scan. */
    HYD_MotionControlFB_SoftReset(fb);
    (void)HYD_MotionControlFB_GetPumpFeedback(fb, &out);
    ASSERT_TRUE(out.validFlags == 0u, "SoftReset clears pump feedback validity");
    ASSERT_TRUE(out.rpm == 0.0f, "SoftReset clears pump feedback rpm");

    (void)HYD_MotionControlFB_SetPumpFeedback(fb, &in);
    HYD_MotionControlFB_Init(fb);
    (void)HYD_MotionControlFB_GetPumpFeedback(fb, &out);
    ASSERT_TRUE(out.validFlags == 0u, "Init clears pump feedback validity");
    ASSERT_TRUE(out.rpm == 0.0f, "Init clears pump feedback rpm");
}

int main(void) {
    printf("=== IEC servo-pump feedback (rpm/torque/angle) ===\n\n");

    test_native_round_trip();
    test_native_negative_rpm_preserved();
    test_native_angle_canonicalisation();
    test_native_masks_unknown_valid_flags();
    test_native_partial_feedback_keeps_other_bits_clear();
    test_native_rejects_non_finite();
    test_native_null_safety();

    test_iec_round_trip();
    test_iec_partial_validity_flags();
    test_iec_no_feedback_reports_invalid();
    test_iec_simulation_axis_is_suppressed();
    test_iec_disable_leaves_packet_untouched();
    test_iec_read_invalid_axis_reports_error();
    test_iec_set_invalid_axis_reports_error();
    test_iec_read_disable_zeroes_outputs();

    test_feedback_cleared_by_init_and_soft_reset();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_failed == 0) ? 0 : 1;
}
