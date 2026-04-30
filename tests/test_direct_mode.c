/**
 * Direct Mode Usage Example for Process Layer Integration
 * 
 * This example demonstrates how the process layer can use DIRECT_SEGMENT mode
 * to implement segment switching without relying on the recipe mechanism.
 * The process layer maintains full control over segment transitions.
 * 
 * Key Features Demonstrated:
 * 1. Direct segment configuration and execution
 * 2. Process-layer controlled segment switching
 * 3. Velocity and acceleration continuity verification
 * 4. Pressure smoothness validation
 * 5. Diagnostics and error handling
 * 6. Integration with PLC cycle timing
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "motion_control.h"
#include "common_types.h"

#define CYCLE_PERIOD 0.001  /* 1ms PLC cycle */
#define SIMULATION_STEPS 5000
#define TOLERANCE 1e-6

/* Test tracking variables */
typedef struct {
    HYD_REAL previousVelocity;
    HYD_REAL previousAcceleration;
    HYD_REAL previousPressure;
    HYD_REAL maxVelocityJump;
    HYD_REAL maxAccelerationJump;
    HYD_REAL maxPressureJump;
    HYD_UINT discontinuityCount;
    HYD_BOOL continuityViolation;
} ContinuityTracker;

/* Simulation plant model */
typedef struct {
    HYD_REAL position;
    HYD_REAL velocity;
    HYD_REAL flow;
    HYD_REAL pressure;
    HYD_TIME timestamp;
    HYD_REAL actuatorArea;      /* cm² */
    HYD_REAL bulkModulus;       /* MPa */
    HYD_REAL volume;            /* L */
} PlantModel;

void Plant_Init(PlantModel* plant) {
    memset(plant, 0, sizeof(*plant));
    plant->actuatorArea = 50.0;   /* 50 cm² */
    plant->bulkModulus = 1400.0;  /* 1400 MPa (hydraulic oil) */
    plant->volume = 2.0;          /* 2L */
    plant->timestamp = 0.0;
}

void Plant_Update(PlantModel* plant, HYD_REAL pumpSpeed, HYD_TIME deltaTime) {
    /* Simple integration model */
    HYD_REAL pumpFlow = pumpSpeed / 60.0;  /* Convert rpm to L/s (simplified) */
    
    /* Update position based on flow and actuator area */
    HYD_REAL positionChange = (pumpFlow * 1000.0) / (plant->actuatorArea * deltaTime * 60.0);
    plant->position += positionChange;
    
    /* Update velocity (simplified) */
    plant->velocity = positionChange / deltaTime;
    
    /* Update flow (magnitude) */
    plant->flow = fabs(pumpFlow * 60.0);  /* L/min */
    
    /* Update pressure based on compression */
    HYD_REAL volumeChange = pumpFlow * deltaTime;
    HYD_REAL strain = volumeChange / plant->volume;
    plant->pressure += plant->bulkModulus * strain;
    
    /* Natural pressure decay */
    plant->pressure *= 0.999;
    
    /* Clamp pressure to reasonable range */
    if (plant->pressure < 0.0) plant->pressure = 0.0;
    if (plant->pressure > 200.0) plant->pressure = 200.0;
    
    plant->timestamp += deltaTime;
}

void ContinuityTracker_Init(ContinuityTracker* tracker) {
    memset(tracker, 0, sizeof(*tracker));
    tracker->previousVelocity = 0.0;
    tracker->previousAcceleration = 0.0;
    tracker->previousPressure = 0.0;
    tracker->maxVelocityJump = 0.0;
    tracker->maxAccelerationJump = 0.0;
    tracker->maxPressureJump = 0.0;
    tracker->discontinuityCount = 0;
    tracker->continuityViolation = false;
}

void ContinuityTracker_Check(ContinuityTracker* tracker, 
                           HYD_REAL currentVelocity,
                           HYD_REAL currentAcceleration,
                           HYD_REAL currentPressure,
                           HYD_TIME deltaTime) {
    HYD_REAL velocityJump, accelerationJump, pressureJump;
    
    /* Calculate velocity jump */
    velocityJump = fabs(currentVelocity - tracker->previousVelocity);
    if (velocityJump > tracker->maxVelocityJump) {
        tracker->maxVelocityJump = velocityJump;
    }
    
    /* Calculate acceleration jump */
    accelerationJump = fabs(currentAcceleration - tracker->previousAcceleration);
    if (accelerationJump > tracker->maxAccelerationJump) {
        tracker->maxAccelerationJump = accelerationJump;
    }
    
    /* Calculate pressure jump */
    pressureJump = fabs(currentPressure - tracker->previousPressure);
    if (pressureJump > tracker->maxPressureJump) {
        tracker->maxPressureJump = pressureJump;
    }
    
        /* Check for discontinuities (tolerance-based) */
        if (deltaTime > 0.0) {
            /* Velocity should be continuous */
            if (velocityJump > 10.0) {  /* mm/s tolerance */
                tracker->discontinuityCount++;
                tracker->continuityViolation = true;
            }
            
            /* Acceleration may have jumps but should be bounded */
            if (accelerationJump > 100.0) {  /* mm/s² tolerance */
                tracker->continuityViolation = true;
            }
        
        /* Pressure should be smooth */
        if (pressureJump > 0.5) {  /* MPa tolerance */
            tracker->continuityViolation = true;
        }
    }
    
    /* Update previous values */
    tracker->previousVelocity = currentVelocity;
    tracker->previousAcceleration = currentAcceleration;
    tracker->previousPressure = currentPressure;
}

void PrintDiagnosticInfo(const HYD_MotionControlFB* fb) {
    printf("  Diagnostic: Code=%d, Severity=%d, Source=%d\n",
           fb->DIAGNOSTIC.code, fb->DIAGNOSTIC.severity, fb->DIAGNOSTIC.source);
    if (fb->DIAGNOSTIC.code != HYD_DIAG_CODE_NONE) {
        printf("    PressureError=%.3f, FlowError=%.3f\n",
               fb->DIAGNOSTIC.pressureError, fb->DIAGNOSTIC.flowError);
    }
}

void PrintStateInfo(const HYD_MotionControlFB* fb) {
    const HYD_MotionState* state = &fb->STATE;
    printf("  State: Active=%d, Finished=%d, Fault=%d\n",
           state->active, state->finished, state->faultActive);
    printf("  PlannedVelocity=%.3f mm/s, PlannedFlow=%.3f L/min\n",
           state->plannedVelocity, state->plannedFlow);
    printf("  PumpSpeed=%.3f rpm, Direction=%d\n",
           state->commandedPumpSpeed, state->plannedDirection);
    printf("  Segment: %s (Source=%d)\n",
           (state->currentSegmentTag == 0 ? "(none)" : "active"), state->segmentSource);
    
    #if HYD_ENABLE_PRESSURE_LOOP_TELEMETRY
    printf("  PressureLoop: Target=%.3f, Filtered=%.3f, Error=%.3f\n",
           state->pressureLoop.targetPressure,
           state->pressureLoop.filteredPressure,
           state->pressureLoop.controlError);
    #endif
}

HYD_MotionSegment CreateInjectionSegment(HYD_TIME startTime) {
    HYD_MotionSegment segment;
    memset(&segment, 0, sizeof(segment));
    
    segment.segmentTag = HYD_SEGMENT_TYPE_INJECTION;
    segment.segmentType = HYD_SEGMENT_TYPE_INJECTION;
    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.planner = HYD_PLANNER_TIME_BASED;
    
    segment.targetPosition = 100.0;  /* mm */
    segment.maxVelocity = 200.0;     /* mm/s */
    segment.maxAcceleration = 500.0;  /* mm/s² */
    segment.maxFlow = 50.0;          /* L/min */
    segment.targetFlow = 40.0;       /* L/min */
    segment.velocityToFlowGain = 0.2; /* L/min per mm/s */
    
    segment.pressureRampRate = 10.0; /* MPa/s */
    
    segment.positionTolerance = 0.1; /* mm */
    segment.pressureTolerance = 0.5; /* MPa */
    segment.flowTolerance = 1.0;    /* L/min */
    segment.timeoutLimit = 5.0;     /* s */
    
    return segment;
}

HYD_MotionSegment CreateHoldingSegment(HYD_TIME startTime) {
    HYD_MotionSegment segment;
    memset(&segment, 0, sizeof(segment));
    
    segment.segmentTag = HYD_SEGMENT_TYPE_HOLDING;
    segment.segmentType = HYD_SEGMENT_TYPE_HOLDING;
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    
    segment.targetPressure = 80.0;   /* MPa */
    segment.targetFlow = 5.0;       /* L/min (holding flow) */
    segment.maxFlow = 20.0;         /* L/min */
    segment.duration = 2.0;         /* s */
    
    segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 0.5;       /* L/min per MPa */
    segment.pressureKi = 0.1;       /* L/min per (MPa*s) */
    segment.pressureIntegralLimit = 10.0; /* L/min */
    segment.pressureDeadband = 0.5; /* MPa */
    segment.pressureRampRate = 5.0; /* MPa/s */
    
    segment.pressureTolerance = 0.5; /* MPa */
    segment.flowTolerance = 1.0;    /* L/min */
    
    return segment;
}

HYD_MotionSegment CreateRetractionSegment(HYD_TIME startTime) {
    HYD_MotionSegment segment;
    memset(&segment, 0, sizeof(segment));
    
    segment.segmentTag = HYD_SEGMENT_TYPE_OPENING;
    segment.segmentType = HYD_SEGMENT_TYPE_OPENING;
    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_RETRACT;
    segment.planner = HYD_PLANNER_TIME_BASED;
    
    segment.targetPosition = 0.0;    /* mm */
    segment.maxVelocity = 150.0;     /* mm/s */
    segment.maxAcceleration = 400.0; /* mm/s² */
    segment.maxFlow = 40.0;         /* L/min */
    segment.targetFlow = 30.0;      /* L/min */
    segment.velocityToFlowGain = 0.2; /* L/min per mm/s */
    
    segment.positionTolerance = 0.1; /* mm */
    segment.timeoutLimit = 3.0;     /* s */
    
    return segment;
}

int main(void) {
    HYD_MotionControlFB fb;
    PlantModel plant;
    ContinuityTracker tracker;
    HYD_TIME currentTime = 0.0;
    HYD_UINT step = 0;
    bool testPassed = true;
    
    printf("=== Direct Mode Usage Example ===\n\n");
    
    /* Initialize function block */
    HYD_MotionControlFB_Init(&fb);
    
    /* Configure function block for direct mode */
    /* EN gate handled by IEC layer */
    fb.USE_RECIPE = false;  /* Direct mode: use DIRECT_SEGMENT */
    fb.FLOW_TO_PUMP_SPEED_GAIN = 1.2;  /* rpm per L/min */
    fb.PUMP_SPEED_LIMIT = 3000.0;       /* rpm */
    
    /* Initialize plant and tracker */
    Plant_Init(&plant);
    ContinuityTracker_Init(&tracker);
    
    printf("=== Phase 1: Injection Phase ===\n");
    
    /* Load and start injection segment */
    HYD_MotionSegment injectionSeg = CreateInjectionSegment(currentTime);
    if (!HYD_MotionControlFB_LoadDirectSegment(&fb, &injectionSeg)) {
        printf("ERROR: Failed to load injection segment\n");
        return 1;
    }
    
    if (!HYD_MotionControlFB_StartSegment(&fb, 0, currentTime)) {
        printf("ERROR: Failed to start injection segment\n");
        return 1;
    }
    
    printf("Started injection segment at t=%.3f s\n", currentTime);
    
    /* Execute injection phase */
    while (step < SIMULATION_STEPS && !fb.SEGMENT_COMPLETED) {
        /* Update plant feedback */
        plant.position += fb.STATE.plannedVelocity * CYCLE_PERIOD;
        plant.velocity = fb.STATE.plannedVelocity;
        plant.flow = fb.STATE.plannedFlow;
        plant.timestamp = currentTime;
        
        /* Update axis reference */
        fb.AXIS_REF.position = plant.position;
        fb.AXIS_REF.velocity = plant.velocity;
        fb.AXIS_REF.flow = plant.flow;
        fb.AXIS_REF.pressure = plant.pressure;
        fb.AXIS_REF.timestamp = currentTime;
        
        /* Execute control cycle */
        HYD_MotionControlFB_Execute(&fb);
        
        /* Check continuity */
        HYD_REAL currentAcceleration = 0.0;
        if (step > 0) {
            currentAcceleration = (fb.STATE.plannedVelocity - tracker.previousVelocity) / CYCLE_PERIOD;
        }
        
        ContinuityTracker_Check(&tracker, fb.STATE.plannedVelocity,
                              currentAcceleration, plant.pressure, CYCLE_PERIOD);
        
        /* Print periodic status */
        if (step % 500 == 0) {
            printf("  Step %lu: t=%.3f s, Pos=%.2f mm, Vel=%.2f mm/s, Flow=%.2f L/min, Press=%.2f MPa\n",
                   (unsigned long)step, currentTime, plant.position, plant.velocity, plant.flow, plant.pressure);
            PrintStateInfo(&fb);
            
            if (fb.DIAGNOSTIC.code != HYD_DIAG_CODE_NONE) {
                PrintDiagnosticInfo(&fb);
            }
        }
        
        /* Check for errors */
        if (fb.STATE.faultActive) {
            printf("ERROR: Fault detected\n");
            PrintDiagnosticInfo(&fb);
            testPassed = false;
            break;
        }
        
        currentTime += CYCLE_PERIOD;
        step++;
    }
    
    printf("Injection phase completed at t=%.3f s\n", currentTime);
    printf("Final position: %.2f mm (target: %.2f mm)\n", plant.position, injectionSeg.targetPosition);
    
    printf("\n=== Phase 2: Holding Phase ===\n");
    
    /* Process layer switches to holding segment */
    HYD_MotionSegment holdingSeg = CreateHoldingSegment(currentTime);
    if (!HYD_MotionControlFB_LoadDirectSegment(&fb, &holdingSeg)) {
        printf("ERROR: Failed to load holding segment\n");
        return 1;
    }
    
    /* Start holding segment - this is process-layer controlled */
    if (!HYD_MotionControlFB_StartSegment(&fb, 0, currentTime)) {
        printf("ERROR: Failed to start holding segment\n");
        return 1;
    }
    
    printf("Started holding segment at t=%.3f s\n", currentTime);
    
    /* Execute holding phase */
    HYD_UINT holdingStep = 0;
    while (holdingStep < 3000) {
        /* Update plant feedback with pressure response */
        /* Simulate pressure buildup towards target */
        HYD_REAL pressureError = holdingSeg.targetPressure - plant.pressure;
        plant.pressure += pressureError * 0.5 * CYCLE_PERIOD;  /* Faster pressure response */
        
        /* Update position/velocity based on pump flow */
        plant.position += fb.STATE.plannedVelocity * CYCLE_PERIOD;
        plant.velocity = fb.STATE.plannedVelocity;
        plant.flow = fb.STATE.plannedFlow;
        plant.timestamp = currentTime;
        
        /* Update axis reference */
        fb.AXIS_REF.position = plant.position;
        fb.AXIS_REF.velocity = plant.velocity;
        fb.AXIS_REF.flow = plant.flow;
        fb.AXIS_REF.pressure = plant.pressure;
        fb.AXIS_REF.timestamp = currentTime;
        
        /* Execute control cycle */
        HYD_MotionControlFB_Execute(&fb);
        
        /* Check if segment completed AFTER executing */
        if (fb.SEGMENT_COMPLETED) {
            break;
        }
        
        /* Check continuity */
        HYD_REAL currentAcceleration = 0.0;
        if (step > 0) {
            currentAcceleration = (fb.STATE.plannedVelocity - tracker.previousVelocity) / CYCLE_PERIOD;
        }
        
        ContinuityTracker_Check(&tracker, fb.STATE.plannedVelocity,
                              currentAcceleration, plant.pressure, CYCLE_PERIOD);
        
        /* Print periodic status */
        if (holdingStep % 500 == 0) {
            printf("  Step %lu: t=%.3f s, Press=%.2f MPa (target: %.2f), Flow=%.2f L/min\n",
                   (unsigned long)holdingStep, currentTime, plant.pressure, holdingSeg.targetPressure, plant.flow);
            PrintStateInfo(&fb);
            
            #if HYD_ENABLE_PRESSURE_LOOP_TELEMETRY
            printf("  PressureLoop: Target=%.3f, Error=%.3f, Output=%.3f\n",
                   fb.STATE.pressureLoop.targetPressure,
                   fb.STATE.pressureLoop.controlError,
                   fb.STATE.pressureLoop.outputFlow);
            #endif
            
            if (fb.DIAGNOSTIC.code != HYD_DIAG_CODE_NONE) {
                PrintDiagnosticInfo(&fb);
            }
        }
        
        /* Check for errors */
        if (fb.STATE.faultActive) {
            printf("ERROR: Fault detected\n");
            PrintDiagnosticInfo(&fb);
            testPassed = false;
            break;
        }
        
        currentTime += CYCLE_PERIOD;
        holdingStep++;
        step++;
    }
    
    printf("Holding phase completed at t=%.3f s\n", currentTime);
    printf("Final pressure: %.2f MPa (target: %.2f MPa)\n", plant.pressure, holdingSeg.targetPressure);
    
    printf("\n=== Phase 3: Retraction Phase ===\n");
    
    /* Process layer switches to retraction segment */
    HYD_MotionSegment retractionSeg = CreateRetractionSegment(currentTime);
    if (!HYD_MotionControlFB_LoadDirectSegment(&fb, &retractionSeg)) {
        printf("ERROR: Failed to load retraction segment\n");
        return 1;
    }
    
    /* Start retraction segment - process-layer controlled */
    if (!HYD_MotionControlFB_StartSegment(&fb, 0, currentTime)) {
        printf("ERROR: Failed to start retraction segment\n");
        return 1;
    }
    
    printf("Started retraction segment at t=%.3f s\n", currentTime);
    
    /* Execute retraction phase */
    HYD_UINT retractionStep = 0;
    while (retractionStep < 3000) {
        /* Update plant feedback */
        plant.position += fb.STATE.plannedVelocity * CYCLE_PERIOD;
        plant.velocity = fb.STATE.plannedVelocity;
        plant.flow = fb.STATE.plannedFlow;
        plant.pressure *= 0.99;  /* Pressure decay during retraction */
        plant.timestamp = currentTime;
        
        /* Update axis reference */
        fb.AXIS_REF.position = plant.position;
        fb.AXIS_REF.velocity = plant.velocity;
        fb.AXIS_REF.flow = plant.flow;
        fb.AXIS_REF.pressure = plant.pressure;
        fb.AXIS_REF.timestamp = currentTime;
        
        /* Execute control cycle */
        HYD_MotionControlFB_Execute(&fb);
        
        /* Check if segment completed AFTER executing */
        if (fb.SEGMENT_COMPLETED) {
            break;
        }
        
        /* Check continuity */
        HYD_REAL currentAcceleration = 0.0;
        if (step > 0) {
            currentAcceleration = (fb.STATE.plannedVelocity - tracker.previousVelocity) / CYCLE_PERIOD;
        }
        
        ContinuityTracker_Check(&tracker, fb.STATE.plannedVelocity,
                              currentAcceleration, plant.pressure, CYCLE_PERIOD);
        
        /* Print periodic status */
        if (retractionStep % 500 == 0) {
            printf("  Step %lu: t=%.3f s, Pos=%.2f mm, Vel=%.2f mm/s, Flow=%.2f L/min\n",
                   (unsigned long)retractionStep, currentTime, plant.position, plant.velocity, plant.flow);
            PrintStateInfo(&fb);
            
            if (fb.DIAGNOSTIC.code != HYD_DIAG_CODE_NONE) {
                PrintDiagnosticInfo(&fb);
            }
        }
        
        /* Check for errors */
        if (fb.STATE.faultActive) {
            printf("ERROR: Fault detected\n");
            PrintDiagnosticInfo(&fb);
            testPassed = false;
            break;
        }
        
        currentTime += CYCLE_PERIOD;
        retractionStep++;
        step++;
    }
    
    printf("Retraction phase completed at t=%.3f s\n", currentTime);
    printf("Final position: %.2f mm (target: %.2f mm)\n", plant.position, retractionSeg.targetPosition);
    
    printf("\n=== Continuity Analysis Results ===\n");
    printf("Total simulation steps: %lu\n", (unsigned long)step);
    printf("Max velocity jump: %.6f mm/s\n", tracker.maxVelocityJump);
    printf("Max acceleration jump: %.6f mm/s²\n", tracker.maxAccelerationJump);
    printf("Max pressure jump: %.6f MPa\n", tracker.maxPressureJump);
    printf("Discontinuity count: %lu\n", (unsigned long)tracker.discontinuityCount);
    printf("Continuity violation: %s\n", tracker.continuityViolation ? "YES" : "NO");
    
    printf("\n=== Final Status ===\n");
    printf("Test result: %s\n", testPassed ? "PASSED" : "FAILED");
    printf("Function block state: %d\n", fb.FB_STATE);
    printf("Finished: %d\n", fb.STATE.finished);
    printf("FAULT: %d\n", fb.STATE.faultActive);
    printf("ERROR: %d\n", HYD_MotionControlFB_IsError(&fb));
    
    if (fb.DIAGNOSTIC.code != HYD_DIAG_CODE_NONE) {
        printf("\n=== Final Diagnostic ===\n");
        PrintDiagnosticInfo(&fb);
    }
    
    printf("\n=== API Usage Summary ===\n");
    printf("Direct mode workflow:\n");
    printf("1. HYD_MotionControlFB_Init() - Initialize function block\n");
    printf("2. fb.USE_RECIPE = false - Enable direct mode\n");
    printf("3. HYD_MotionControlFB_LoadDirectSegment() - Load segment parameters\n");
    printf("4. HYD_MotionControlFB_StartSegment() - Start execution\n");
    printf("5. Update fb.AXIS_REF each cycle - Provide sensor feedback\n");
    printf("6. HYD_MotionControlFB_Execute() - Run control cycle\n");
    printf("7. Read fb.STATE and fb.PUMP_SPEED - Get control outputs\n");
    printf("8. Check fb.SEGMENT_COMPLETED - Monitor completion\n");
    printf("9. Process layer decides when to switch segments\n");
    printf("10. Repeat steps 3-8 for next segment\n");
    
    printf("\n=== Key Advantages of Direct Mode ===\n");
    printf("- Process layer maintains full control over segment transitions\n");
    printf("- No need to manage multi-segment recipe array\n");
    printf("- Dynamic segment configuration possible\n");
    printf("- Simplified integration with existing process logic\n");
    printf("- Direct access to all segment parameters\n");
    
    return testPassed ? 0 : 1;
}
