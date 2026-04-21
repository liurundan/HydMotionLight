/**
 * @file benchmark_performance.c
 * @brief Performance benchmarking for motion control library
 * 
 * This program measures the execution time of key functions in the motion
 * control library under various configuration scenarios. It helps identify
 * performance bottlenecks and validates optimization efforts.
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include "motion_control.h"
#include "motion_utils.h"
#include "motion_planner.h"
#include "pressure_controller.h"
#include "pump_converter.h"
#include "common_types.h"

/* Benchmark configuration */
#define BENCHMARK_ITERATIONS 10000
#define BENCHMARK_WARMUP 1000

/* Timing utilities */
static clock_t start_time;
static clock_t end_time;

static void benchmark_start(void) {
    start_time = clock();
}

static void benchmark_end(const char* name, int iterations) {
    end_time = clock();
    double elapsed_ms = ((double)(end_time - start_time)) * 1000.0 / CLOCKS_PER_SEC;
    double avg_us = elapsed_ms * 1000.0 / iterations;
    
    printf("%-40s %8.2f ms  (%8.2f µs/call)\n", name, elapsed_ms, avg_us);
}

/* Test data creation */
static void create_test_axis_ref(HDY_AxisRef* axis_ref, HDY_TIME time_offset) {
    axis_ref->position = 100.0 + time_offset * 10.0;
    axis_ref->velocity = 50.0;
    axis_ref->flow = 5.0;
    axis_ref->pressure = 2.5;
    axis_ref->timestamp = time_offset;
}

static void create_test_segment(HDY_MotionSegment* segment, HDY_ControlMode mode) {
    snprintf(segment->name, HDY_NAME_MAX, "TestSegment");
    segment->type = HDY_SEGMENT_TYPE_INJECTION;
    segment->planner = HDY_PLANNER_TIME_BASED;
    segment->mode = mode;
    segment->endCondition = HDY_END_POSITION;
    segment->direction = HDY_DIRECTION_EXTEND;
    
    segment->targetPosition = 200.0;
    segment->targetFlow = 10.0;
    segment->targetPressure = 5.0;
    segment->maxAcceleration = 1000.0;
    segment->maxVelocity = 500.0;
    segment->maxFlow = 20.0;
    segment->duration = 2.0;
    
    segment->tolerance = 1.0;
    segment->positionTolerance = 0.1;
    segment->pressureTolerance = 0.1;
    segment->flowTolerance = 0.5;
    segment->velocityTolerance = 10.0;
    segment->timeoutLimit = 10.0;
    
    segment->velocityToFlowGain = 0.02;
    segment->pressureRampRate = 10.0;
    
    segment->pressureController = HDY_PRESSURE_CONTROLLER_PI;
    segment->pressureKp = 2.0;
    segment->pressureKi = 0.5;
    segment->pressureKd = 0.1;
    segment->pressureIntegralLimit = 5.0;
    segment->pressureDeadband = 0.05;
    segment->pressureFilterAlpha = 0.8;
    segment->pressureDerivativeFilterAlpha = 0.9;
}

/* Benchmark functions */

static void benchmark_motion_utils(void) {
    HDY_REAL a = 123.456;
    HDY_REAL b = 789.012;
    HDY_AxisRef axis_ref;
    
    create_test_axis_ref(&axis_ref, 1.0);
    
    printf("\n=== Motion Utils Performance ===\n");
    
    /* Warmup */
    for (int i = 0; i < BENCHMARK_WARMUP; i++) {
        HDY_MotionUtils_MinReal(a, b);
        HDY_MotionUtils_AbsReal(-a);
        HDY_MotionUtils_IsFiniteReal(a);
        HDY_MotionUtils_AxisRefIsValid(&axis_ref);
    }
    
    /* Benchmark MinReal */
    benchmark_start();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        HDY_MotionUtils_MinReal(a, b);
    }
    benchmark_end("MinReal", BENCHMARK_ITERATIONS);
    
    /* Benchmark AbsReal */
    benchmark_start();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        HDY_MotionUtils_AbsReal(-a);
    }
    benchmark_end("AbsReal", BENCHMARK_ITERATIONS);
    
    /* Benchmark IsFiniteReal */
    benchmark_start();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        HDY_MotionUtils_IsFiniteReal(a);
    }
    benchmark_end("IsFiniteReal", BENCHMARK_ITERATIONS);
    
    /* Benchmark AxisRefIsValid */
    benchmark_start();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        HDY_MotionUtils_AxisRefIsValid(&axis_ref);
    }
    benchmark_end("AxisRefIsValid", BENCHMARK_ITERATIONS);
}

static void benchmark_motion_planner(void) {
    HDY_MotionPlannerInput input;
    HDY_MotionPlannerOutput output;
    HDY_AxisRef axis_ref;
    HDY_MotionSegment segment;
    
    create_test_axis_ref(&axis_ref, 1.0);
    create_test_segment(&segment, HDY_MODE_POSITION);
    
    input.axisRef = &axis_ref;
    input.segment = &segment;
    input.elapsedTime = 0.5;
    input.rampedPressure = 2.5;
    
    printf("\n=== Motion Planner Performance ===\n");
    
    /* Warmup */
    for (int i = 0; i < BENCHMARK_WARMUP; i++) {
        axis_ref.position += 0.01;
        axis_ref.timestamp += 0.001;
        HDY_MotionPlanner_Execute(&input, &output);
    }
    
    /* Benchmark Execute (POSITION mode) */
    benchmark_start();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        axis_ref.position += 0.01;
        axis_ref.timestamp += 0.001;
        HDY_MotionPlanner_Execute(&input, &output);
    }
    benchmark_end("Planner Execute (POSITION)", BENCHMARK_ITERATIONS);
    
    /* Benchmark Execute (SPEED_RAMP mode) */
    segment.mode = HDY_MODE_SPEED_RAMP;
    axis_ref.position = 100.0;
    benchmark_start();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        axis_ref.position += 0.01;
        axis_ref.timestamp += 0.001;
        HDY_MotionPlanner_Execute(&input, &output);
    }
    benchmark_end("Planner Execute (SPEED_RAMP)", BENCHMARK_ITERATIONS);
}

static void benchmark_pressure_controller(void) {
    HDY_PressureControllerInput input;
    HDY_PressureControllerOutput output;
    HDY_MotionSegment segment;
    HDY_PressureControllerState state;
    
    create_test_segment(&segment, HDY_MODE_PRESSURE_CLOSED_LOOP);
    
    input.targetPressure = 5.0;
    input.measuredPressure = 4.8;
    input.feedforwardFlow = 2.0;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 1.0;
    
    HDY_PressureController_InitState(&state, 4.8, 2.0, 0.0);
    
    printf("\n=== Pressure Controller Performance ===\n");
    
    /* Warmup */
    for (int i = 0; i < BENCHMARK_WARMUP; i++) {
        input.timestamp += 0.001;
        input.measuredPressure += 0.001;
        HDY_PressureController_Execute(&segment, &state, &input, &output);
    }
    
    /* Benchmark Execute (PI controller) */
    benchmark_start();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        input.timestamp += 0.001;
        input.measuredPressure += 0.001;
        HDY_PressureController_Execute(&segment, &state, &input, &output);
    }
    benchmark_end("Pressure Controller Execute (PI)", BENCHMARK_ITERATIONS);
}

static void benchmark_pump_converter(void) {
    HDY_PumpConverterInput input;
    HDY_PumpConverterOutput output;
    
    input.requestedFlow = 10.0;
    input.flowToPumpSpeedGain = 1500.0;
    input.pumpSpeedLimit = 3000.0;
    input.direction = HDY_DIRECTION_EXTEND;
    
    printf("\n=== Pump Converter Performance ===\n");
    
    /* Warmup */
    for (int i = 0; i < BENCHMARK_WARMUP; i++) {
        input.requestedFlow = 5.0 + (i % 100) / 10.0;
        HDY_PumpConverter_Execute(&input, &output);
    }
    
    /* Benchmark Execute */
    benchmark_start();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        input.requestedFlow = 5.0 + (i % 100) / 10.0;
        HDY_PumpConverter_Execute(&input, &output);
    }
    benchmark_end("Pump Converter Execute", BENCHMARK_ITERATIONS);
}

static void benchmark_motion_control_cycle(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment segment;
    HDY_AxisRef axis_ref;
    
    HDY_MotionControlFB_Init(&fb);
    create_test_segment(&segment, HDY_MODE_POSITION);
    
    fb.FLOW_TO_PUMP_SPEED_GAIN = 1500.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.EN = true;
    fb.USE_RECIPE = true;
    
    HDY_MotionControlFB_LoadRecipe(&fb, &segment, 1);
    HDY_MotionControlFB_StartSegment(&fb, 0, 0.0);
    
    create_test_axis_ref(&axis_ref, 0.001);
    fb.AXIS_REF = axis_ref;
    
    printf("\n=== Motion Control Cycle Performance ===\n");
    
    /* Warmup */
    for (int i = 0; i < BENCHMARK_WARMUP; i++) {
        axis_ref.position += 0.1;
        axis_ref.timestamp += 0.001;
        fb.AXIS_REF = axis_ref;
        HDY_MotionControlFB_Execute(&fb);
        
        /* Reset if completed */
        if (fb.SEGMENT_COMPLETED) {
            HDY_MotionControlFB_StartSegment(&fb, 0, axis_ref.timestamp);
        }
    }
    
    /* Benchmark Execute */
    benchmark_start();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        axis_ref.position += 0.1;
        axis_ref.timestamp += 0.001;
        fb.AXIS_REF = axis_ref;
        HDY_MotionControlFB_Execute(&fb);
        
        /* Reset if completed */
        if (fb.SEGMENT_COMPLETED) {
            HDY_MotionControlFB_StartSegment(&fb, 0, axis_ref.timestamp);
        }
    }
    benchmark_end("Motion Control Cycle (full)", BENCHMARK_ITERATIONS);
}

static void print_system_info(void) {
    HDY_ConfigInfo config = HDY_GetConfigInfo();
    
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║   HydroMotion Library - Performance Benchmark               ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("Library Version: %s\n", config.versionString);
    printf("Build Time:     %s\n", config.buildTime);
    printf("\n");
    printf("Configuration:\n");
    printf("  Max Segments:            %d\n", config.maxSegments);
    printf("  Max Name Length:         %d\n", config.maxNameLength);
    printf("  Max Message Length:      %d\n", config.maxMessageLength);
    printf("  Max History Depth:       %d\n", config.maxHistoryDepth);
    printf("  Diagnostic Message:      %s\n", config.diagnosticMessageEnabled ? "Enabled" : "Disabled");
    printf("  Diagnostic History:      %s\n", config.diagnosticHistoryEnabled ? "Enabled" : "Disabled");
    printf("  Pressure Loop Telemetry: %s\n", config.pressureLoopTelemetryEnabled ? "Enabled" : "Disabled");
    printf("  Execution Reference:      %s\n", config.executionReferenceEnabled ? "Enabled" : "Disabled");
    printf("\n");
    printf("Benchmark Parameters:\n");
    printf("  Iterations:  %d\n", BENCHMARK_ITERATIONS);
    printf("  Warmup:      %d cycles\n", BENCHMARK_WARMUP);
    printf("\n");
}

int main(void) {
    printf("\n");
    printf("┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓\n");
    printf("┃                                                          ┃\n");
    printf("┃          HydroMotion Library Performance Benchmark        ┃\n");
    printf("┃                                                          ┃\n");
    printf("┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");
    printf("\n");
    
    print_system_info();
    
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║                   Performance Results                       ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    
    benchmark_motion_utils();
    benchmark_motion_planner();
    benchmark_pressure_controller();
    benchmark_pump_converter();
    benchmark_motion_control_cycle();
    
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║                    Summary & Analysis                       ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("Key Findings:\n");
    printf("  • Motion control cycle time determines real-time feasibility\n");
    printf("  • < 100 µs per cycle is excellent for 1 kHz control loop\n");
    printf("  • < 10 µs for utility functions indicates efficient implementation\n");
    printf("  • Memory usage can be optimized via configuration flags\n");
    printf("\n");
    printf("Recommendations:\n");
    printf("  • Profile on target hardware for accurate timing\n");
    printf("  • Consider compiler optimization flags (-O2/-O3)\n");
    printf("  • Use configuration to disable unused features\n");
    printf("  • Monitor CPU load in production deployment\n");
    printf("\n");
    
    return 0;
}
