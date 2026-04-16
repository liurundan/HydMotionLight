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
        {
            .name = "Slow Clamping",
            .type = HDY_SEGMENT_TYPE_CLAMPING,
            .planner = HDY_PLANNER_TIME_BASED,
            .mode = HDY_MODE_SPEED_RAMP,
            .endCondition = HDY_END_POSITION,
            .direction = HDY_DIRECTION_EXTEND,
            .targetPosition = 10.0,
            .targetFlow = 5.0,
            .targetPressure = 5.5,
            .maxAcceleration = 50.0,
            .maxVelocity = 120.0,
            .maxFlow = 12.0,
            .duration = 0.0,
            .positionTolerance = 0.2,
            .pressureTolerance = 0.3,
            .flowTolerance = 0.2,
            .velocityTolerance = 0.5,
            .timeoutLimit = 2.5,
            .velocityToFlowGain = 0.10,
            .pressureRampRate = 5.0
        },
        {
            .name = "Fast Clamping",
            .type = HDY_SEGMENT_TYPE_CLAMPING,
            .planner = HDY_PLANNER_TIME_BASED,
            .mode = HDY_MODE_SPEED_RAMP,
            .endCondition = HDY_END_POSITION,
            .direction = HDY_DIRECTION_EXTEND,
            .targetPosition = 30.0,
            .targetFlow = 10.0,
            .targetPressure = 8.0,
            .maxAcceleration = 70.0,
            .maxVelocity = 320.0,
            .maxFlow = 32.0,
            .duration = 0.0,
            .positionTolerance = 0.2,
            .pressureTolerance = 0.3,
            .flowTolerance = 0.3,
            .velocityTolerance = 1.0,
            .timeoutLimit = 3.0,
            .velocityToFlowGain = 0.10,
            .pressureRampRate = 8.0
        },
        {
            .name = "Low Pressure Hold",
            .type = HDY_SEGMENT_TYPE_CLAMPING,
            .planner = HDY_PLANNER_TIME_BASED,
            .mode = HDY_MODE_PRESSURE_CLOSED_LOOP,
            .endCondition = HDY_END_TIME,
            .direction = HDY_DIRECTION_HOLD,
            .targetPosition = 30.0,
            .targetFlow = 8.0,
            .targetPressure = 12.0,
            .maxAcceleration = 50.0,
            .maxVelocity = 250.0,
            .maxFlow = 20.0,
            .duration = 1.5,
            .pressureTolerance = 0.2,
            .flowTolerance = 0.3,
            .timeoutLimit = 2.5,
            .velocityToFlowGain = 0.10,
            .pressureRampRate = 6.0
        },
        {
            .name = "Injection Stage 1",
            .type = HDY_SEGMENT_TYPE_INJECTION,
            .planner = HDY_PLANNER_TIME_BASED,
            .mode = HDY_MODE_SPEED_RAMP,
            .endCondition = HDY_END_POSITION,
            .direction = HDY_DIRECTION_EXTEND,
            .targetPosition = 80.0,
            .targetFlow = 45.0,
            .targetPressure = 5.0,
            .maxAcceleration = 80.0,
            .maxVelocity = 380.0,
            .maxFlow = 60.0,
            .duration = 0.0,
            .positionTolerance = 0.5,
            .pressureTolerance = 0.4,
            .flowTolerance = 0.5,
            .velocityTolerance = 1.5,
            .timeoutLimit = 4.0,
            .velocityToFlowGain = 0.10,
            .pressureRampRate = 5.0
        },
        {
            .name = "Holding Stage 1",
            .type = HDY_SEGMENT_TYPE_HOLDING,
            .planner = HDY_PLANNER_TIME_BASED,
            .mode = HDY_MODE_PRESSURE_CLOSED_LOOP,
            .endCondition = HDY_END_TIME,
            .direction = HDY_DIRECTION_HOLD,
            .targetPosition = 80.0,
            .targetFlow = 25.0,
            .targetPressure = 14.0,
            .maxAcceleration = 40.0,
            .maxVelocity = 220.0,
            .maxFlow = 30.0,
            .duration = 2.0,
            .pressureTolerance = 0.2,
            .flowTolerance = 0.4,
            .timeoutLimit = 3.0,
            .velocityToFlowGain = 0.10,
            .pressureRampRate = 4.0
        }
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

        printf("[%.1f s] %s | PumpSpeed=%.1f rpm | Flow=%.1f | Velocity=%.2f | Pressure=%.2f MPa | Status=%s | Changed=%s\n",
               ref.timestamp,
               controller.CURRENT_SEGMENT_NAME,
               controller.PUMP_SPEED,
               controller.STATE.plannedFlow,
               controller.STATE.plannedVelocity,
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
            if (controller.STATE.plannedVelocity < 0.0 && ref.position < 0.0) {
                ref.position = 0.0;
            }

            {
                HDY_REAL pressureIncrement = 0.0;
                if (ref.flow > 3.0) {
                    pressureIncrement = (ref.flow / 30.0 - 0.1) * timeStep;
                }
                ref.pressure += pressureIncrement;
                if (ref.pressure > recipe[controller.STATE.currentSegmentIndex].targetPressure) {
                    ref.pressure -= (ref.pressure - recipe[controller.STATE.currentSegmentIndex].targetPressure) * 0.2 * timeStep;
                }
            }
        }

        {
            struct timespec delay = {0, 20000000L};
            nanosleep(&delay, NULL);
        }
    }

    return 0;
}
