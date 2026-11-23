#include "pressure_ripple_comp.h"
#include <math.h>
#include <string.h>

#define HYD_RIPPLE_TWO_PI 6.28318530717958647692
#define HYD_RIPPLE_DEG_TO_RAD 0.01745329251994329577

void HYD_PressureRippleComp_Clear(HYD_PressureRippleCompState* state)
{
    if (state == NULL) return;
    memset(state, 0, sizeof(*state));
}

void HYD_PressureRippleComp_Execute(HYD_PressureRippleCompState* state,
                                    const HYD_PressureRippleCompInput* input,
                                    HYD_PressureRippleCompOutput* output)
{
    HYD_REAL speed;
    HYD_REAL phase;
    HYD_REAL cosK[HYD_PRESSURE_RIPPLE_HARMONICS];
    HYD_REAL sinK[HYD_PRESSURE_RIPPLE_HARMONICS];
    HYD_REAL flow = 0.0;
    HYD_BOOL feedbackValid;
    HYD_BOOL active;
    int k;

    if (output == NULL) return;
    memset(output, 0, sizeof(*output));
    if (state == NULL || input == NULL) return;

    if (input->order != state->order) {
        HYD_PressureRippleComp_Clear(state);
        state->order = input->order;
    }
    if (input->order == 0U || !(input->dt > 0.0)) {
        state->fade = 0.0;
        state->holdTime = 0.0;
        return;
    }

    feedbackValid = HYD_PumpFeedback_HasValid(input->pumpFeedback.validFlags,
                                              HYD_PUMP_FEEDBACK_VALID_RPM |
                                              HYD_PUMP_FEEDBACK_VALID_ANGLE) &&
                    isfinite(input->pumpFeedback.rpm) &&
                    isfinite(input->pumpFeedback.angleDeg);
    speed = (feedbackValid && input->pumpFeedback.rpm > 0.0) ? input->pumpFeedback.rpm : 0.0;

    if (input->referenceMoving || !feedbackValid) {
        state->holdTime = 0.0;
    } else if (state->holdTime < HYD_THRESH_PRESSURE_RIPPLE_HOLD_S) {
        state->holdTime += input->dt;
    }
    active = feedbackValid &&
             (2.0 * input->order * speed / 60.0) < HYD_THRESH_PRESSURE_RIPPLE_MAX_FREQ_HZ &&
             state->holdTime >= HYD_THRESH_PRESSURE_RIPPLE_HOLD_S;
    if (active) {
        state->fade += input->dt / HYD_THRESH_PRESSURE_RIPPLE_FADE_IN_S;
        if (state->fade > 1.0) state->fade = 1.0;
    } else {
        state->fade -= input->dt / HYD_THRESH_PRESSURE_RIPPLE_FADE_OUT_S;
        if (state->fade < 0.0) state->fade = 0.0;
    }

    /* N·θ reduced in degrees first keeps float precision at high orders. */
    phase = fmod((HYD_REAL)input->order * input->pumpFeedback.angleDeg, 360.0) *
            HYD_RIPPLE_DEG_TO_RAD;
    cosK[0] = cos(phase);
    sinK[0] = sin(phase);
    cosK[1] = cosK[0] * cosK[0] - sinK[0] * sinK[0];
    sinK[1] = 2.0 * sinK[0] * cosK[0];

    /* Learn only at full output weight: during fade-in the residual still
     * contains the faded-out part and the LMS would over-learn it. */
    output->learning = active && state->fade >= 1.0 && input->kqFlow > 0.0 &&
                       speed >= HYD_THRESH_PRESSURE_RIPPLE_MIN_RPM &&
                       fabs(input->error) < HYD_THRESH_PRESSURE_RIPPLE_MAX_ERROR &&
                       !state->lastClipped;
    if (output->learning) {
        HYD_REAL omega1 = input->order * HYD_RIPPLE_TWO_PI * speed / 60.0;
        HYD_REAL delayCos1 = cos(omega1 * HYD_THRESH_PRESSURE_RIPPLE_PATH_DELAY_S);
        HYD_REAL delaySin1 = sin(omega1 * HYD_THRESH_PRESSURE_RIPPLE_PATH_DELAY_S);
        HYD_REAL delayCos[HYD_PRESSURE_RIPPLE_HARMONICS];
        HYD_REAL delaySin[HYD_PRESSURE_RIPPLE_HARMONICS];
        HYD_REAL leak = (input->leakResistanceFlow > 0.0) ? 1.0 / input->leakResistanceFlow : 0.0;
        HYD_REAL step = HYD_THRESH_PRESSURE_RIPPLE_MU * input->dt * 2.0 * input->error / speed;

        delayCos[0] = delayCos1;
        delaySin[0] = delaySin1;
        delayCos[1] = delayCos1 * delayCos1 - delaySin1 * delaySin1;
        delaySin[1] = 2.0 * delaySin1 * delayCos1;

        for (k = 0; k < HYD_PRESSURE_RIPPLE_HARMONICS; ++k) {
            HYD_REAL omega = omega1 * (HYD_REAL)(k + 1);
            /* 1/Ph = (1/R_f + jω/Kq_f)·e^(jωTd) + Kp + Ki/(jω) */
            HYD_REAL invRe = leak * delayCos[k] - (omega / input->kqFlow) * delaySin[k] + input->kp;
            HYD_REAL invIm = leak * delaySin[k] + (omega / input->kqFlow) * delayCos[k] -
                             input->ki / omega;
            HYD_REAL magnitude;

            /* c += step · e^(-jkθ) / Ph */
            state->coeffRe[k] += step * (cosK[k] * invRe + sinK[k] * invIm);
            state->coeffIm[k] += step * (cosK[k] * invIm - sinK[k] * invRe);
            magnitude = sqrt(state->coeffRe[k] * state->coeffRe[k] +
                             state->coeffIm[k] * state->coeffIm[k]) * speed;
            if (magnitude > HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW) {
                HYD_REAL scale = HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW / magnitude;
                state->coeffRe[k] *= scale;
                state->coeffIm[k] *= scale;
            }
        }
    }

    for (k = 0; k < HYD_PRESSURE_RIPPLE_HARMONICS; ++k) {
        HYD_REAL magnitude = sqrt(state->coeffRe[k] * state->coeffRe[k] +
                                  state->coeffIm[k] * state->coeffIm[k]) * speed;
        HYD_REAL scale = (magnitude > HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW)
            ? HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW / magnitude : 1.0;

        output->amplitude[k] = magnitude * scale;
        flow += scale * speed * (state->coeffRe[k] * cosK[k] - state->coeffIm[k] * sinK[k]);
    }
    output->flow = state->fade * flow;
}
