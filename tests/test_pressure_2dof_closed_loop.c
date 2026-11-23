/* Closed-loop regression for the 2-DOF pressure loop (spec 2026-11-23 §5, T6/T8).
 *
 * Runs the real HYD_MotionControlFB (PressureHandle-style direct segments,
 * live target updates) against the valve-closed plant identified from
 * docs/data_deg/hor1121: dP/dt = Kq·(Q − P/R), Kq = 64 (+0.4/bar above
 * 100 bar) bar/s per L/min, R = 104 bar per L/min, servo dead time, MA3 on
 * the feedback, pump-side order 12/24 flow ripple. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "debug_values.h"
#include "motion_control.h"

#define DT 0.001
#define TWO_PI 6.283185307179586
#define GAIN 20.0              /* rpm per L/min (FLOW_TO_PUMPSPEED = 20) */
#define MAX_SERVO_DELAY 8
#define SEGMENT_COUNT 5

typedef struct {
    double servoDelay;         /* scans */
    double kqScale;            /* true Kq / identified Kq (3.2 bar/s/rpm); configured is 2.9 */
    HYD_REAL rippleOrder;      /* parameter 39 */
} PlantCase;

typedef struct {
    double overshoot[SEGMENT_COUNT];
    double holdPeakToPeak[SEGMENT_COUNT];
    int t98[SEGMENT_COUNT];
    double maxDebugAbs[HYD_DEBUG_VALUE_COUNT];
} LoopMetrics;

static const double kTargets[SEGMENT_COUNT + 1] = {0.0, 100.0, 140.0, 80.0, 120.0, 50.0};
static const int kSegmentSteps = 3000;
static const int kLeadSteps = 200;

static HYD_MotionSegment make_hold_segment(double target) {
    HYD_MotionSegment segment;
    memset(&segment, 0, sizeof(segment));
    segment.segmentTag = HYD_SEGMENT_TYPE_HOLDING;
    segment.segmentType = HYD_SEGMENT_TYPE_HOLDING;
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = (HYD_REAL)target;
    segment.maxFlow = 20.0f;
    segment.pressureRampRate = 20.0f;
    segment.pressureCeiling = MAX_PRESSURE;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    /* Kp = 2π·5/(Kq·G), Ki = Kp·2π·5/4 with Kq = 2.9 bar/s/rpm (3.2 × 0.9). */
    segment.pressureKp = (HYD_REAL)(TWO_PI * 5.0 / (2.9 * GAIN));
    segment.pressureKi = (HYD_REAL)(TWO_PI * 5.0 / (2.9 * GAIN) * TWO_PI * 5.0 / 4.0);
    segment.pressureIntegralLimit = 10.0f;
    segment.pressureDeadband = 0.005f;
    return segment;
}

static void configure_fb(HYD_MotionControlFB* fb, HYD_REAL rippleOrder) {
    HYD_MotionControlFB_Init(fb);
    fb->USE_RECIPE = false;
    assert(HYD_MotionControlFB_WriteParameter(fb, HYD_PARAM_FLOW_TO_PUMP_SPEED_GAIN, (HYD_REAL)GAIN));
    assert(HYD_MotionControlFB_WriteParameter(fb, HYD_PARAM_PUMP_SPEED_LIMIT, 1700.0f));
    assert(HYD_MotionControlFB_WriteParameter(fb, HYD_PARAM_KSYS_BAR_PER_RPM, 5.2f));
    assert(HYD_MotionControlFB_WriteParameter(fb, HYD_PARAM_KQ_BAR_PER_S_RPM, 2.9f));
    assert(HYD_MotionControlFB_WriteParameter(fb, HYD_PARAM_PUMP_RIPPLE_ORDER, rippleOrder));
}

/* Valve-closed hor1121 plant with servo dead time and MA3 feedback. */
typedef struct {
    double pressure;
    double angle;
    double history[3];
    double measured;
    double speedLine[MAX_SERVO_DELAY];
    int delay;                 /* servo dead time, scans */
    int lineIndex;
    int step;
} PlantState;

static void plant_reset(PlantState* plant, int delay) {
    memset(plant, 0, sizeof(*plant));
    assert(delay >= 1 && delay <= MAX_SERVO_DELAY);
    plant->delay = delay;
}

/* One scan: feed the FB, cycle it, then advance the plant by DT. */
static void plant_scan(PlantState* plant, HYD_MotionControlFB* fb, double kqScale) {
    double speedCommand;
    double appliedSpeed;
    double kq;
    double w12;
    double ripple;

    ++plant->step;
    fb->AXIS_REF.timestamp = (HYD_TIME)(plant->step * DT);
    fb->AXIS_REF.pressure = (HYD_REAL)plant->measured;
    HYD_MotionControl_SetPumpFeedback(&(HYD_PumpFeedback){
        (HYD_REAL)plant->speedLine[plant->lineIndex],
        (HYD_REAL)(fmod(plant->angle, TWO_PI) * 360.0 / TWO_PI),
        0.0f, (HYD_TIME)(plant->step * DT),
        HYD_PUMP_FEEDBACK_VALID_RPM | HYD_PUMP_FEEDBACK_VALID_ANGLE});
    HYD_MotionControlFB_Cycle(fb);
    assert(!fb->STATE.faultActive);
    speedCommand = fb->PUMP_SPEED;

    appliedSpeed = plant->speedLine[plant->lineIndex];
    plant->speedLine[plant->lineIndex] = speedCommand;
    plant->lineIndex = (plant->lineIndex + 1) % plant->delay;

    plant->angle += appliedSpeed / 60.0 * TWO_PI * DT;
    kq = kqScale * (64.0 + 0.4 * fmax(plant->pressure - 100.0, 0.0));
    w12 = 12.0 * fabs(appliedSpeed) / 60.0 * TWO_PI;
    ripple = 1.2 * w12 / 64.0 * sin(12.0 * plant->angle + 0.7) +
             0.36 * w12 / 64.0 * sin(24.0 * plant->angle + 1.9);
    plant->pressure += kq * (appliedSpeed / GAIN + ripple - plant->pressure / 104.0) * DT;
    if (plant->pressure < 0.0) plant->pressure = 0.0;
    plant->history[0] = plant->history[1];
    plant->history[1] = plant->history[2];
    plant->history[2] = plant->pressure;
    plant->measured = (plant->history[0] + plant->history[1] + plant->history[2]) / 3.0;
}

static void start_hold_segment(HYD_MotionControlFB* fb) {
    HYD_MotionSegment segment = make_hold_segment(0.0);
    assert(HYD_MotionControlFB_LoadDirectSegment(fb, &segment));
    fb->AXIS_REF.timestamp = 0.0f;
    assert(HYD_MotionControlFB_StartSegment(fb, 0U, 0.0f));
}

static void apply_live_target(HYD_MotionControlFB* fb, double target) {
    HYD_LiveUpdateRequest request;
    memset(&request, 0, sizeof(request));
    request.flags = HYD_LIVE_UPDATE_TARGET_PRESSURE;
    request.ownerKind = fb->_directOwnerKind;
    request.ownerTicket = fb->_directOwnerTicket;
    request.targetPressure = (HYD_REAL)target;
    assert(HYD_MotionControlFB_ApplyLiveUpdate(fb, &request));
}

static void run_sequence(const PlantCase* plantCase, LoopMetrics* metrics) {
    static HYD_MotionControlFB fb;
    PlantState plant;
    int seg;
    int k;

    memset(metrics, 0, sizeof(*metrics));
    plant_reset(&plant, (int)plantCase->servoDelay);
    HYD_Debug_Reset();
    configure_fb(&fb, plantCase->rippleOrder);
    start_hold_segment(&fb);

    for (seg = 0; seg <= SEGMENT_COUNT; ++seg) {
        int steps = (seg == 0) ? kLeadSteps : kSegmentSteps;
        double target = kTargets[seg];
        double previous = (seg == 0) ? 0.0 : kTargets[seg - 1];
        double maxP = -1.0e9;
        double minP = 1.0e9;
        double holdMax = -1.0e9;
        double holdMin = 1.0e9;
        int t98 = -1;

        if (seg > 0) {
            apply_live_target(&fb, target);
        }

        for (k = 0; k < steps; ++k) {
            double measured;
            int i;

            plant_scan(&plant, &fb, plantCase->kqScale);
            measured = plant.measured;

            if (seg > 0) {
                if (measured > maxP) maxP = measured;
                if (measured < minP) minP = measured;
                if (t98 < 0 && ((target >= previous && measured >= previous + 0.98 * (target - previous)) ||
                                (target < previous && measured <= previous + 0.98 * (target - previous)))) {
                    t98 = k;
                }
                if (k >= steps - 1000) {
                    if (measured > holdMax) holdMax = measured;
                    if (measured < holdMin) holdMin = measured;
                }
            }
            for (i = 0; i < (int)HYD_DEBUG_VALUE_COUNT; ++i) {
                double v = fabs((double)HYD_Debug_GetValue((HYD_UINT8)i));
                if (v > metrics->maxDebugAbs[i]) metrics->maxDebugAbs[i] = v;
            }
        }

        if (seg > 0) {
            metrics->overshoot[seg - 1] = (target >= previous) ? maxP - target : target - minP;
            metrics->holdPeakToPeak[seg - 1] = holdMax - holdMin;
            metrics->t98[seg - 1] = t98;
        }
    }
}

static void print_metrics(const char* name, const LoopMetrics* m) {
    int i;
    printf("%-26s", name);
    for (i = 0; i < SEGMENT_COUNT; ++i) {
        printf(" | %3.0f->%3.0f ov %.2f t98 %3d pp %.2f", kTargets[i], kTargets[i + 1],
               m->overshoot[i], m->t98[i], m->holdPeakToPeak[i]);
    }
    printf("\n");
}

static double max_of(const double* values) {
    double m = values[0];
    int i;
    for (i = 1; i < SEGMENT_COUNT; ++i) if (values[i] > m) m = values[i];
    return m;
}

static void test_nominal_without_ripple_compensation(void) {
    PlantCase plantCase = {5, 1.0, 0.0f};
    LoopMetrics m;
    run_sequence(&plantCase, &m);
    print_metrics("B1 nominal", &m);
    assert(max_of(m.overshoot) <= 1.5);
    assert(max_of(m.holdPeakToPeak) <= 3.0);
}

static void test_nominal_with_ripple_compensation(void) {
    PlantCase plantCase = {5, 1.0, 12.0f};
    LoopMetrics m;
    int i;
    run_sequence(&plantCase, &m);
    print_metrics("B2 nominal", &m);
    assert(max_of(m.overshoot) <= 1.5);
    assert(max_of(m.holdPeakToPeak) <= 1.0);
    for (i = 0; i < SEGMENT_COUNT; ++i) assert(m.t98[i] > 0);
    /* T8: every debug channel carries a value on the model path. */
    for (i = 0; i < (int)HYD_DEBUG_VALUE_COUNT; ++i) assert(m.maxDebugAbs[i] > 0.0);
}

static void test_robust_to_servo_delay_and_lower_configured_kq(void) {
    const PlantCase cases[] = {
        /* true Kq = configured (no 0.9 margin) and true Kq = 1.5 x configured */
        {3, 2.9 / 3.2, 12.0f}, {7, 2.9 / 3.2, 12.0f},
        {3, 1.5 * 2.9 / 3.2, 12.0f}, {7, 1.5 * 2.9 / 3.2, 12.0f},
    };
    size_t c;
    for (c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c) {
        LoopMetrics m;
        char name[40];
        run_sequence(&cases[c], &m);
        snprintf(name, sizeof(name), "B2 servo %d true/cfg %.2f", (int)cases[c].servoDelay,
                 cases[c].kqScale * 3.2 / 2.9);
        print_metrics(name, &m);
        assert(max_of(m.overshoot) <= 1.5);
    }
}

/* Parameter 37 written to 0 while the profiled reference is mid-ramp: the
 * legacy 20 ms ramp takes over and must still reach the target (legacy
 * overshoot is expected, so it is not asserted). */
static void test_kq_switched_off_mid_ramp_still_reaches_target(void) {
    static HYD_MotionControlFB fb;
    PlantState plant;
    double measuredSum = 0.0;
    double measuredMean;
    int k;

    plant_reset(&plant, 5);
    HYD_Debug_Reset();
    configure_fb(&fb, 12.0f);
    start_hold_segment(&fb);
    for (k = 0; k < kLeadSteps; ++k) plant_scan(&plant, &fb, 1.0);

    apply_live_target(&fb, 100.0);
    for (k = 0; k < kSegmentSteps; ++k) {
        if (k == 60) {
            assert(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_KQ_BAR_PER_S_RPM, 0.0f));
        }
        plant_scan(&plant, &fb, 1.0);
        if (k >= kSegmentSteps - 500) measuredSum += plant.measured;
    }
    measuredMean = measuredSum / 500.0;
    printf("%-26s | 0->100 ref %.2f mean(last 500) %.2f\n", "B2 Kq -> 0 at k = 60",
           (double)fb._rampController.rampedPressure, measuredMean);
    assert(fb._rampController.rampedPressure == 100.0f);
    assert(!fb.STATE.faultActive);
    assert(fabs(measuredMean - 100.0) <= 2.0);
}

int main(void) {
    test_nominal_without_ripple_compensation();
    test_nominal_with_ripple_compensation();
    test_robust_to_servo_delay_and_lower_configured_kq();
    test_kq_switched_off_mid_ramp_still_reaches_target();
    puts("2-DOF pressure closed-loop tests passed.");
    return 0;
}
