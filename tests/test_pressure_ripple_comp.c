/* Pump-angle-synchronous ripple compensation (spec 2026-11-23 §4.5, T5). */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pressure_ripple_comp.h"
#include "pressure_controller.h"

#define DT 0.001
#define TWO_PI 6.283185307179586
#define KQ_FLOW 64.0          /* bar/s per L/min */
#define R_FLOW 104.0          /* bar per L/min */
#define PUMP_RPM 20.0
#define LOOP_DELAY 5          /* scans from command to flow */
#define TARGET 100.0

typedef struct {
    HYD_PressureRippleCompState comp;
    double pressure;
    double integral;
    double angle;             /* rad */
    double delayLine[LOOP_DELAY];
    int delayIndex;
    double kp;
    double ki;
} HoldLoop;

static void hold_loop_init(HoldLoop* loop) {
    int i;
    memset(loop, 0, sizeof(*loop));
    HYD_PressureRippleComp_Clear(&loop->comp);
    loop->pressure = TARGET;
    loop->kp = TWO_PI * 5.0 / KQ_FLOW;
    loop->ki = loop->kp * TWO_PI * 5.0 / 4.0;
    for (i = 0; i < LOOP_DELAY; ++i) loop->delayLine[i] = TARGET / R_FLOW;
}

/* Runs the hold loop; returns the order-12 pressure amplitude over the last
 * `windowSteps` steps (use whole ripple periods: 250 steps at 20 rpm).
 * `order` 0 disables compensation. */
static double hold_loop_run(HoldLoop* loop, HYD_UINT8 order, int steps, int windowSteps,
                            HYD_BOOL feedbackValid, HYD_BOOL moving, double* lastFlow) {
    double sumCos = 0.0;
    double sumSin = 0.0;
    int k;

    for (k = 0; k < steps; ++k) {
        HYD_PressureRippleCompInput input;
        HYD_PressureRippleCompOutput output;
        double error = TARGET - loop->pressure;
        double command;
        double applied;
        double w12;
        double ripple;

        memset(&input, 0, sizeof(input));
        input.order = order;
        input.pumpFeedback.rpm = PUMP_RPM;
        input.pumpFeedback.angleDeg = fmod(loop->angle, TWO_PI) * 360.0 / TWO_PI;
        input.pumpFeedback.validFlags = feedbackValid
            ? (HYD_PUMP_FEEDBACK_VALID_RPM | HYD_PUMP_FEEDBACK_VALID_ANGLE) : 0u;
        input.referenceMoving = moving;
        input.error = (HYD_REAL)error;
        input.kqFlow = KQ_FLOW;
        input.leakResistanceFlow = R_FLOW;
        input.kp = (HYD_REAL)loop->kp;
        input.ki = (HYD_REAL)loop->ki;
        input.dt = DT;
        HYD_PressureRippleComp_Execute(&loop->comp, &input, &output);
        if (lastFlow != NULL) *lastFlow = output.flow;

        loop->integral += loop->ki * error * DT;
        command = TARGET / R_FLOW + loop->kp * error + loop->integral + output.flow;
        loop->comp.lastClipped = HYD_FALSE;

        applied = loop->delayLine[loop->delayIndex];
        loop->delayLine[loop->delayIndex] = command;
        loop->delayIndex = (loop->delayIndex + 1) % LOOP_DELAY;

        loop->angle += PUMP_RPM / 60.0 * TWO_PI * DT;
        w12 = 12.0 * PUMP_RPM / 60.0 * TWO_PI;
        ripple = 1.2 * w12 / KQ_FLOW * sin(12.0 * loop->angle + 0.7) +
                 0.36 * w12 / KQ_FLOW * sin(24.0 * loop->angle + 1.9);
        loop->pressure += KQ_FLOW * (applied + ripple - loop->pressure / R_FLOW) * DT;

        if (k >= steps - windowSteps) {
            sumCos += (loop->pressure - TARGET) * cos(12.0 * loop->angle);
            sumSin += (loop->pressure - TARGET) * sin(12.0 * loop->angle);
        }
    }
    return 2.0 * sqrt(sumCos * sumCos + sumSin * sumSin) / (double)windowSteps;
}

static void test_order12_amplitude_drops_80_percent_in_3s(void) {
    HoldLoop loop;
    double baseline;
    double compensated;

    hold_loop_init(&loop);
    baseline = hold_loop_run(&loop, 0U, 1000, 500, HYD_TRUE, HYD_FALSE, NULL);

    hold_loop_init(&loop);
    compensated = hold_loop_run(&loop, 12U, 3000, 500, HYD_TRUE, HYD_FALSE, NULL);

    printf("order-12 amplitude: %.3f bar -> %.3f bar\n", baseline, compensated);
    assert(baseline > 0.5);
    assert(compensated < 0.2 * baseline);
}

static void test_coefficients_survive_and_converge_faster_on_reentry(void) {
    HoldLoop loop;
    HYD_PressureRippleCompState learned;
    double fresh;
    double reentry;

    hold_loop_init(&loop);
    fresh = hold_loop_run(&loop, 12U, 500, 250, HYD_TRUE, HYD_FALSE, NULL);

    hold_loop_init(&loop);
    (void)hold_loop_run(&loop, 12U, 3000, 500, HYD_TRUE, HYD_FALSE, NULL);
    learned = loop.comp;

    hold_loop_init(&loop);
    loop.comp = learned;
    loop.comp.fade = 0.0;
    loop.comp.holdTime = 0.0;
    reentry = hold_loop_run(&loop, 12U, 500, 250, HYD_TRUE, HYD_FALSE, NULL);
    printf("amplitude 0.25-0.5 s: fresh %.3f bar, re-entry %.3f bar\n", fresh, reentry);
    assert(reentry < 0.5 * fresh);
    assert(reentry < 0.2);
}

static void test_moving_reference_neither_learns_nor_outputs(void) {
    HoldLoop loop;
    double lastFlow = 1.0;

    hold_loop_init(&loop);
    (void)hold_loop_run(&loop, 12U, 500, 100, HYD_TRUE, HYD_TRUE, &lastFlow);
    assert(loop.comp.coeffRe[0] == 0.0 && loop.comp.coeffIm[0] == 0.0);
    assert(lastFlow == 0.0);
    assert(loop.comp.fade == 0.0);
}

static void test_invalid_feedback_fades_out_within_20ms(void) {
    HoldLoop loop;
    double lastFlow = 1.0;
    HYD_REAL coeff;

    hold_loop_init(&loop);
    (void)hold_loop_run(&loop, 12U, 1000, 100, HYD_TRUE, HYD_FALSE, NULL);
    assert(loop.comp.fade == 1.0);
    coeff = loop.comp.coeffRe[0];

    (void)hold_loop_run(&loop, 12U, 20, 10, HYD_FALSE, HYD_FALSE, &lastFlow);
    assert(loop.comp.fade < 1.0e-6);
    assert(fabs(lastFlow) < 1.0e-6);
    assert(loop.comp.coeffRe[0] == coeff);
}

static void test_clipped_output_freezes_learning(void) {
    HYD_PressureRippleCompState comp;
    HYD_PressureRippleCompInput input;
    HYD_PressureRippleCompOutput output;
    int k;

    HYD_PressureRippleComp_Clear(&comp);
    memset(&input, 0, sizeof(input));
    input.order = 12U;
    input.pumpFeedback.rpm = PUMP_RPM;
    input.pumpFeedback.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM | HYD_PUMP_FEEDBACK_VALID_ANGLE;
    input.error = 1.0;
    input.kqFlow = KQ_FLOW;
    input.leakResistanceFlow = R_FLOW;
    input.kp = 0.5;
    input.ki = 4.0;
    input.dt = DT;
    for (k = 0; k < 200; ++k) {
        input.pumpFeedback.angleDeg = (HYD_REAL)k * 0.12;
        comp.lastClipped = HYD_TRUE;
        HYD_PressureRippleComp_Execute(&comp, &input, &output);
        assert(!output.learning);
    }
    assert(comp.coeffRe[0] == 0.0 && comp.coeffIm[0] == 0.0);
    assert(comp.fade > 0.0);   /* output gating is independent of the freeze */

    comp.lastClipped = HYD_FALSE;
    HYD_PressureRippleComp_Execute(&comp, &input, &output);
    assert(output.learning);
    assert(comp.coeffRe[0] != 0.0 || comp.coeffIm[0] != 0.0);
}

static void test_amplitude_is_capped(void) {
    HYD_PressureRippleCompState comp;
    HYD_PressureRippleCompInput input;
    HYD_PressureRippleCompOutput output;

    HYD_PressureRippleComp_Clear(&comp);
    comp.order = 12U;
    comp.coeffRe[0] = 1.0;     /* 20 L/min at 20 rpm, far above the cap */
    comp.fade = 1.0;
    comp.holdTime = 1.0;
    comp.lastClipped = HYD_TRUE;
    memset(&input, 0, sizeof(input));
    input.order = 12U;
    input.pumpFeedback.rpm = PUMP_RPM;
    input.pumpFeedback.angleDeg = 0.0;
    input.pumpFeedback.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM | HYD_PUMP_FEEDBACK_VALID_ANGLE;
    input.kqFlow = KQ_FLOW;
    input.dt = DT;
    HYD_PressureRippleComp_Execute(&comp, &input, &output);
    assert(fabs(output.amplitude[0] - HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW) < 1.0e-5);
    assert(fabs(output.flow) <= HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW + 1.0e-5);
}

static void test_order_zero_and_order_change(void) {
    HYD_PressureRippleCompState comp;
    HYD_PressureRippleCompInput input;
    HYD_PressureRippleCompOutput output;

    HYD_PressureRippleComp_Clear(&comp);
    comp.order = 12U;
    comp.coeffRe[0] = 0.01;
    comp.fade = 1.0;
    memset(&input, 0, sizeof(input));
    input.order = 0U;
    input.pumpFeedback.rpm = PUMP_RPM;
    input.pumpFeedback.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM | HYD_PUMP_FEEDBACK_VALID_ANGLE;
    input.dt = DT;
    HYD_PressureRippleComp_Execute(&comp, &input, &output);
    assert(output.flow == 0.0);
    assert(comp.coeffRe[0] == 0.0);   /* order changed 12 -> 0: coefficients cleared */
}

static void test_controller_init_keeps_and_clear_drops_coefficients(void) {
    HYD_PressureRippleCompState comp;
    HYD_PressureControllerState state;

    HYD_PressureRippleComp_Clear(&comp);
    comp.order = 12U;
    comp.coeffRe[1] = 0.02f;
    /* The coefficients live outside the controller state, so the segment
     * start memset in InitState cannot touch them. */
    HYD_PressureController_InitState(&state, 50.0, 0.0, 0.0);
    assert(comp.coeffRe[1] == 0.02f);
    HYD_PressureRippleComp_Clear(&comp);
    assert(comp.coeffRe[1] == 0.0 && comp.order == 0U);
}

int main(void) {
    test_order12_amplitude_drops_80_percent_in_3s();
    test_coefficients_survive_and_converge_faster_on_reentry();
    test_moving_reference_neither_learns_nor_outputs();
    test_invalid_feedback_fades_out_within_20ms();
    test_clipped_output_freezes_learning();
    test_amplitude_is_capped();
    test_order_zero_and_order_change();
    test_controller_init_keeps_and_clear_drops_coefficients();
    puts("Pressure ripple compensation tests passed.");
    return 0;
}
