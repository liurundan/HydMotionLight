#define _POSIX_C_SOURCE 199309L
#include "motion_control.h"
#include <stdio.h>
#include <time.h>

int main(void) {
    HDY_MotionControlFB controller;
    HDY_MotionControlFB_Init(&controller);
    controller.EN = true;
    controller.FLOW_TO_PUMP_SPEED_GAIN = 80.0;
    controller.PUMP_SPEED_LIMIT = 5000.0;

    HDY_MotionSegment recipe[5] = {
        {"Slow Clamping", HDY_SEGMENT_TYPE_CLAMPING, HDY_PLANNER_POSITION_BASED, HDY_MODE_SPEED_RAMP, HDY_END_POSITION,
            10.0, 5.0, 5.5, 50.0, 120.0, 0.0, 0.2, 0.10},
        {"Fast Clamping", HDY_SEGMENT_TYPE_CLAMPING, HDY_PLANNER_POSITION_BASED, HDY_MODE_SPEED_RAMP, HDY_END_POSITION,
            30.0, 10.0, 8.0, 70.0, 320.0, 0.0, 0.2, 0.10},
        {"Low Pressure Hold", HDY_SEGMENT_TYPE_CLAMPING, HDY_PLANNER_TIME_BASED, HDY_MODE_PRESSURE_CLOSED_LOOP, HDY_END_TIME,
            30.0, 8.0, 12.0, 50.0, 250.0, 1.5, 0.1, 0.10},
        {"Injection Stage 1", HDY_SEGMENT_TYPE_INJECTION, HDY_PLANNER_POSITION_BASED, HDY_MODE_SPEED_RAMP, HDY_END_POSITION,
            80.0, 45.0, 5.0, 80.0, 380.0, 0.0, 0.5, 0.10},
        {"Holding Stage 1", HDY_SEGMENT_TYPE_HOLDING, HDY_PLANNER_TIME_BASED, HDY_MODE_PRESSURE_CLOSED_LOOP, HDY_END_TIME,
            80.0, 25.0, 14.0, 40.0, 220.0, 2.0, 0.1, 0.10}
    };

    HDY_MotionControlFB_LoadRecipe(&controller, recipe, 5);
    controller.START_SEGMENT = true;
    controller.START_SEGMENT_INDEX = 0;

    HDY_AxisRef ref = {0};
    HDY_REAL timeStep = 0.1;

    for (int step = 0; step < 250; ++step) {
        ref.timestamp = step * timeStep;
        controller.AXIS_REF = ref;
        HDY_MotionControlFB_Execute(&controller);

        printf("[%.1f s] %s | PumpSpeed=%.1f rpm | Flow=%.1f | Pressure=%.2f MPa | Status=%s | Changed=%s\n",
               ref.timestamp,
               controller.CURRENT_SEGMENT_NAME,
               controller.PUMP_SPEED,
               controller.STATE.plannedFlow,
               ref.pressure,
               controller.SEGMENT_COMPLETED ? "Segment completed" : "Segment running",
               controller.SEGMENT_CHANGED ? "Yes" : "No");

        if (controller.DIAGNOSTIC.message[0] != '\0') {
            printf("    Diag: %s\n", controller.DIAGNOSTIC.message);
        }

        if (controller.SEGMENT_COMPLETED) {
            HDY_MotionControlFB_NextSegment(&controller, ref.timestamp);
            if (controller.FINISHED) {
                printf("Recipe finished.\n");
                break;
            }
            printf("Switching to next segment: %s\n", controller.CURRENT_SEGMENT_NAME);
        }

        if (controller.ACTIVE) {
            ref.velocity = controller.STATE.plannedVelocity;
            ref.position += ref.velocity * timeStep;
            ref.flow = controller.STATE.plannedFlow;
            HDY_REAL pressureIncrement = 0.0;
            if (ref.flow > 3.0) {
                pressureIncrement = (ref.flow / 30.0 - 0.1) * timeStep;
            }
            ref.pressure += pressureIncrement;
            if (ref.pressure > recipe[controller.STATE.currentSegmentIndex].targetPressure) {
                ref.pressure -= (ref.pressure - recipe[controller.STATE.currentSegmentIndex].targetPressure) * 0.2 * timeStep;
            }
        }

        {
            struct timespec delay = {0, 20000000L};
            nanosleep(&delay, NULL);
        }
    }

    return 0;
}
