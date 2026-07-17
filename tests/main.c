#define _POSIX_C_SOURCE 199309L
#include "diagnostics.h"
#include "motion_control.h"
#include <stdio.h>
#include <time.h>

static void print_live_diagnostic(const HYD_DiagnosticInfo* diagnostic) {
    if (diagnostic == NULL || diagnostic->code == HYD_DIAG_CODE_NONE) {
        return;
    }

    printf("    Diag: code=%s severity=%s source=%s recovery=%s action=%s flags=0x%02X",
           HYD_Diagnostics_CodeToString(diagnostic->code),
           HYD_Diagnostics_SeverityToString(diagnostic->severity),
           HYD_Diagnostics_SourceToString(diagnostic->source),
           HYD_Diagnostics_RecoveryToString(diagnostic->recovery),
           HYD_Diagnostics_ProtectionActionToString(diagnostic->protectionAction),
           (unsigned int)diagnostic->flags);
    printf("\n");
}

static void print_retained_diagnostics(const HYD_MotionControlFB* controller) {
    HYD_DiagnosticSnapshot latestSnapshot;

    if (controller == NULL || controller->DIAGNOSTIC_HISTORY.lastSnapshot.diagnostic.code == HYD_DIAG_CODE_NONE) {
        return;
    }

    printf("    Retained: last=%s severity=%s totalRecorded=%u hasRecord=%u\n",
           HYD_Diagnostics_CodeToString(controller->DIAGNOSTIC_HISTORY.lastSnapshot.diagnostic.code),
           HYD_Diagnostics_SeverityToString(controller->DIAGNOSTIC_HISTORY.lastSnapshot.diagnostic.severity),
           (unsigned int)controller->DIAGNOSTIC_HISTORY.totalRecorded,
           (unsigned int)controller->DIAGNOSTIC_HISTORY.hasRecord);

    if (HYD_DiagnosticsHistory_GetLatest(&controller->DIAGNOSTIC_HISTORY, &latestSnapshot)) {
        printf("    History latest: t=%.2f segment=%s code=%s action=%s\n",
               latestSnapshot.eventTimestamp,
               (latestSnapshot.segmentTag == 0 ? "(none)" : "active"),
               HYD_Diagnostics_CodeToString(latestSnapshot.diagnostic.code),
               HYD_Diagnostics_ProtectionActionToString(latestSnapshot.diagnostic.protectionAction));
    }

    if (controller->LAST_FAULT_SNAPSHOT.valid) {
        printf("    Last fault snapshot: t=%.2f segment=%s code=%s\n",
               controller->LAST_FAULT_SNAPSHOT.eventTimestamp,
               (controller->LAST_FAULT_SNAPSHOT.segmentTag == 0 ? "(none)" : "fault"),
               HYD_Diagnostics_CodeToString(controller->LAST_FAULT_SNAPSHOT.diagnostic.code));
    }
}

int main(void) {
    HYD_MotionControlFB controller;
    HYD_MotionControlFB_Init(&controller);
    controller.FLOW_TO_PUMP_SPEED_GAIN = 80.0;
    controller.PUMP_SPEED_LIMIT = 5000.0;

    HYD_MotionSegment recipe[5] = {
        {
            .segmentTag = 1,
            .segmentType = HYD_SEGMENT_TYPE_CLAMPING,
            .planner = HYD_PLANNER_TIME_BASED,
            .mode = HYD_MODE_SPEED_RAMP,
            .endCondition = HYD_END_POSITION,
            .direction = HYD_DIRECTION_EXTEND,
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
            .segmentTag = 2,
            .segmentType = HYD_SEGMENT_TYPE_CLAMPING,
            .planner = HYD_PLANNER_TIME_BASED,
            .mode = HYD_MODE_SPEED_RAMP,
            .endCondition = HYD_END_POSITION,
            .direction = HYD_DIRECTION_EXTEND,
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
            .segmentTag = 3,
            .segmentType = HYD_SEGMENT_TYPE_CLAMPING,
            .planner = HYD_PLANNER_TIME_BASED,
            .mode = HYD_MODE_PRESSURE_CLOSED_LOOP,
            .endCondition = HYD_END_TIME,
            .direction = HYD_DIRECTION_HOLD,
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
            .pressureRampRate = 6.0,
            .pressureController = HYD_PRESSURE_CONTROLLER_PI,
            .pressureKp = 0.35,
            .pressureKi = 0.80,
            .pressureIntegralLimit = 6.0,
            .pressureDeadband = 0.10,
            .pressureFilterAlpha = 0.35
        },
        {
            .segmentTag = 4,
            .segmentType = HYD_SEGMENT_TYPE_INJECTION,
            .planner = HYD_PLANNER_TIME_BASED,
            .mode = HYD_MODE_SPEED_RAMP,
            .endCondition = HYD_END_POSITION,
            .direction = HYD_DIRECTION_EXTEND,
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
            .segmentTag = 5,
            .segmentType = HYD_SEGMENT_TYPE_HOLDING,
            .planner = HYD_PLANNER_TIME_BASED,
            .mode = HYD_MODE_PRESSURE_CLOSED_LOOP,
            .endCondition = HYD_END_TIME,
            .direction = HYD_DIRECTION_HOLD,
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
            .pressureRampRate = 4.0,
            .pressureController = HYD_PRESSURE_CONTROLLER_PI,
            .pressureKp = 0.45,
            .pressureKi = 1.00,
            .pressureIntegralLimit = 8.0,
            .pressureDeadband = 0.10,
            .pressureFilterAlpha = 0.30
        }
    };

    HYD_MotionControlFB_LoadRecipe(&controller, recipe, 5);
    controller.START_SEGMENT = true;
    controller.START_SEGMENT_INDEX = 0;

    HYD_AxisRef ref = {0};
    HYD_REAL timeStep = 0.1;

    for (int step = 0; step < 250; ++step) {
        ref.timestamp = step * timeStep;
        controller.AXIS_REF = ref;
        HYD_MotionControlFB_Execute(&controller);

        printf("[%.1f s] %s | PumpSpeed=%.1f rpm | Flow=%.1f | Velocity=%.2f | Pressure=%.2f bar | Status=%s | Changed=%s\n",
               ref.timestamp,
               (controller.STATE.currentSegmentTag == 0 ? "(none)" : "active"),
               controller.PUMP_SPEED,
               controller.STATE.plannedFlow,
               controller.STATE.plannedVelocity,
               ref.pressure,
               controller.SEGMENT_COMPLETED ? "Segment completed" : "Segment running",
               controller.SEGMENT_CHANGED ? "Yes" : "No");

        if (controller.DIAGNOSTIC.code != HYD_DIAG_CODE_NONE) {
            print_live_diagnostic(&controller.DIAGNOSTIC);
        } else if (!controller.STATE.active && controller.DIAGNOSTIC_HISTORY.lastSnapshot.diagnostic.code != HYD_DIAG_CODE_NONE) {
            print_retained_diagnostics(&controller);
        }

        if (controller.SEGMENT_COMPLETED) {
            HYD_MotionControlFB_NextSegment(&controller, ref.timestamp);
            if (controller.STATE.finished) {
                printf("Recipe finished.\n");
                break;
            }
            printf("Switching to next segment: %s\n", (controller.STATE.currentSegmentTag == 0 ? "(none)" : "active"));
        }

        if (controller.STATE.active) {
            ref.velocity = controller.STATE.plannedVelocity;
            ref.position += ref.velocity * timeStep;
            ref.flow = controller.STATE.plannedFlow;
            if (controller.STATE.plannedVelocity < 0.0 && ref.position < 0.0) {
                ref.position = 0.0;
            }

            {
                HYD_REAL pressureIncrement = 0.0;
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

    if (!controller.STATE.faultActive && controller.DIAGNOSTIC.code == HYD_DIAG_CODE_NONE &&
        controller.DIAGNOSTIC_HISTORY.lastSnapshot.diagnostic.code != HYD_DIAG_CODE_NONE) {
        print_retained_diagnostics(&controller);
        if (HYD_MotionControlFB_AcknowledgeDiagnostics(&controller)) {
            printf("Service acknowledgment cleared retained diagnostics before exit.\n");
        }
    }

    return 0;
}
