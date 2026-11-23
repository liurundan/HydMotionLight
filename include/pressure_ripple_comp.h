#ifndef HYD_PRESSURE_RIPPLE_COMP_H
#define HYD_PRESSURE_RIPPLE_COMP_H

#include "common_types.h"

/* Pump-angle-synchronous compensation of the pressure ripple at orders N and
 * 2N of the pump shaft (spec 2026-11-23 §4.5).
 *
 * One complex flow coefficient per order, stored per rpm, learned with an LMS
 * that divides by a model of the closed-loop secondary path
 * Ph = G / (1 + C·G), G = Kq_f·e^(-jωTd) / (jω + Kq_f/R_f), C = Kp + Ki/(jω).
 * Learning and output are limited to pressure hold; the state is owned by the
 * caller so that it survives segment switches. */

#define HYD_PRESSURE_RIPPLE_HARMONICS 2

typedef struct {
    HYD_REAL coeffRe[HYD_PRESSURE_RIPPLE_HARMONICS]; /* L/min per rpm, orders N and 2N */
    HYD_REAL coeffIm[HYD_PRESSURE_RIPPLE_HARMONICS]; /* L/min per rpm */
    HYD_REAL fade;        /* 0..1 output weight */
    HYD_REAL holdTime;    /* s the reference has been still with valid feedback (capped) */
    HYD_UINT8 order;      /* N the coefficients belong to; another N clears them */
    HYD_BOOL lastClipped; /* written by the caller: controller output was limited last scan */
} HYD_PressureRippleCompState;

typedef struct {
    HYD_UINT8 order;               /* N, ripples per pump revolution; 0 disables */
    HYD_PumpFeedback pumpFeedback; /* needs RPM and ANGLE valid flags; angle increases with rpm */
    HYD_BOOL referenceMoving;      /* pressure reference slope is not zero */
    HYD_REAL error;                /* bar, controller error */
    HYD_REAL kqFlow;               /* bar/s per L/min */
    HYD_REAL leakResistanceFlow;   /* bar per L/min; 0 drops the leak term */
    HYD_REAL kp;                   /* L/min per bar */
    HYD_REAL ki;                   /* L/min per (bar·s) */
    HYD_REAL dt;                   /* s */
} HYD_PressureRippleCompInput;

typedef struct {
    HYD_REAL flow;                                     /* L/min, add to the unsaturated output */
    HYD_REAL amplitude[HYD_PRESSURE_RIPPLE_HARMONICS]; /* L/min at the current speed */
    HYD_BOOL learning;
} HYD_PressureRippleCompOutput;

void HYD_PressureRippleComp_Clear(HYD_PressureRippleCompState* state);
void HYD_PressureRippleComp_Execute(HYD_PressureRippleCompState* state,
                                    const HYD_PressureRippleCompInput* input,
                                    HYD_PressureRippleCompOutput* output);

#endif /* HYD_PRESSURE_RIPPLE_COMP_H */
