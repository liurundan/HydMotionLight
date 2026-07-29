#define _POSIX_C_SOURCE 200809L

#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "motion_control.h"
#include "toggle_mechanism_pool.h"

#define BENCHMARK_ITERATIONS 100000U
#define BENCHMARK_WARMUP 10000U
#define BENCHMARK_BATCH_SIZE 100U

static volatile HYD_REAL benchmark_checksum;

typedef struct {
    uint64_t totalNs;
    uint64_t maxBatchNs;
} BenchmarkTiming;

static clockid_t benchmark_clock_id(void)
{
#if defined(__linux__) && defined(CLOCK_MONOTONIC_RAW)
    return CLOCK_MONOTONIC_RAW;
#else
    return CLOCK_MONOTONIC;
#endif
}

static uint64_t elapsed_ns(struct timespec start, struct timespec end)
{
    return (uint64_t)(end.tv_sec - start.tv_sec) * 1000000000ULL +
           (uint64_t)(end.tv_nsec - start.tv_nsec);
}

static HYD_REAL sweep_position(const HYD_TogglePreparedConfig *prepared,
                               uint32_t iteration)
{
    HYD_REAL span = prepared->raw.sm - prepared->xHandoffEffective;
    HYD_REAL fraction = (HYD_REAL)(iteration % 1000U) / 999.0f;

    return prepared->xHandoffEffective + span * fraction;
}

static void print_timing(const char *label, BenchmarkTiming timing)
{
    double mean = (double)timing.totalNs / (double)BENCHMARK_ITERATIONS;

    printf("%-30s mean %.2f ns/call, max batch %" PRIu64 " ns\n",
           label, mean, timing.maxBatchNs);
}

static int prepare_default_toggle(HYD_TogglePreparedConfig *prepared)
{
    HYD_ToggleGeometryConfig raw = HYD_ToggleKinematics_DefaultConfig();
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    if (!HYD_ToggleKinematics_ValidateBlocking(&raw, prepared, &error)) {
        fprintf(stderr, "default toggle validation failed: %d\n", (int)error);
        return 0;
    }
    return 1;
}

static BenchmarkTiming benchmark_kinematics(
    const HYD_TogglePreparedConfig *prepared)
{
    BenchmarkTiming timing = {0U, 0U};
    HYD_ToggleSolution solution;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    clockid_t clock_id = benchmark_clock_id();
    uint32_t batch;
    uint32_t i;

    for (i = 0U; i < BENCHMARK_WARMUP; ++i) {
        HYD_REAL xm = sweep_position(prepared, i);
        if (!HYD_ToggleKinematics_SolveOnline(
                prepared, xm, 50.0f, &solution, &error)) {
            fprintf(stderr, "kinematics warmup failed: %d\n", (int)error);
            return timing;
        }
        benchmark_checksum += solution.vs;
    }

    for (batch = 0U;
         batch < BENCHMARK_ITERATIONS / BENCHMARK_BATCH_SIZE;
         ++batch) {
        struct timespec start;
        struct timespec end;
        uint64_t batch_ns;
        HYD_REAL batch_checksum = 0.0f;

        (void)clock_gettime(clock_id, &start);
        for (i = 0U; i < BENCHMARK_BATCH_SIZE; ++i) {
            uint32_t iteration = batch * BENCHMARK_BATCH_SIZE + i;
            HYD_REAL xm = sweep_position(prepared, iteration);
            if (!HYD_ToggleKinematics_SolveOnline(
                    prepared, xm, 50.0f, &solution, &error)) {
                fprintf(stderr, "kinematics benchmark failed: %d\n",
                        (int)error);
                return timing;
            }
            batch_checksum += solution.xs + solution.velocityRatio +
                              solution.vs;
        }
        (void)clock_gettime(clock_id, &end);
        batch_ns = elapsed_ns(start, end);
        timing.totalNs += batch_ns;
        if (batch_ns > timing.maxBatchNs) {
            timing.maxBatchNs = batch_ns;
        }
        benchmark_checksum += batch_checksum;
    }

    return timing;
}

static HYD_MotionSegment make_cycle_segment(void)
{
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.segmentType = HYD_SEGMENT_TYPE_OTHER;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.maxVelocity = 50.0f;
    segment.maxAcceleration = 500.0f;
    segment.maxDeceleration = 500.0f;
    segment.maxFlow = 50.0f;
    segment.velocityToFlowGain = 0.2f;
    return segment;
}

static int initialize_cycle_fb(
    HYD_MotionControlFB *fb,
    HYD_BOOL toggle,
    const HYD_TogglePreparedConfig *prepared)
{
    HYD_MotionSegment segment = make_cycle_segment();

    HYD_MotionControlFB_Init(fb);
    fb->USE_RECIPE = false;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 20.0f;
    fb->PUMP_SPEED_LIMIT = 3000.0f;
    fb->_useFixedCycleTime = true;
    fb->_simulationCycleTime = 0.001f;
    fb->AXIS_REF.position = prepared->xHandoffEffective;
    fb->AXIS_REF.pressure = 20.0f;

    if (toggle) {
        HYD_UINT8 slot = HYD_TOGGLE_SLOT_NONE;

        HYD_ToggleMechanismPool_Reset();
        if (!HYD_ToggleMechanismPool_Reserve(0U, &slot) ||
            !HYD_ToggleMechanismPool_Commit(slot, prepared, true)) {
            return 0;
        }
        fb->mechanismType = (HYD_UINT8)HYD_MECHANISM_FIVE_POINT_TOGGLE;
        fb->mechanismSlot = slot;
        fb->STATE.mechanismType = fb->mechanismType;
        fb->STATE.mechanismConfigVersion =
            HYD_ToggleMechanismPool_GetVersion(slot);
    }

    if (!HYD_MotionControlFB_LoadDirectSegment(fb, &segment) ||
        !HYD_MotionControlFB_StartSegment(fb, 0U, 0.0f)) {
        return 0;
    }
    HYD_MotionControlFB_Execute(fb);
    return fb->FB_STATE != HYD_FB_STATE_FAULT;
}

static BenchmarkTiming benchmark_full_cycle(
    HYD_BOOL toggle,
    const HYD_TogglePreparedConfig *prepared)
{
    BenchmarkTiming timing = {0U, 0U};
    HYD_MotionControlFB fb;
    clockid_t clock_id = benchmark_clock_id();
    uint32_t batch;
    uint32_t i;

    if (!initialize_cycle_fb(&fb, toggle, prepared)) {
        fprintf(stderr, "full-cycle benchmark initialization failed\n");
        return timing;
    }

    for (i = 0U; i < BENCHMARK_WARMUP; ++i) {
        fb.AXIS_REF.position = sweep_position(prepared, i);
        fb.AXIS_REF.velocity = fb.STATE.plannedVelocity;
        fb.AXIS_REF.flow = fb.STATE.plannedFlow;
        fb.AXIS_REF.timestamp += 0.001f;
        HYD_MotionControlFB_Execute(&fb);
        benchmark_checksum += fb.STATE.plannedFlow;
    }

    for (batch = 0U;
         batch < BENCHMARK_ITERATIONS / BENCHMARK_BATCH_SIZE;
         ++batch) {
        struct timespec start;
        struct timespec end;
        uint64_t batch_ns;
        HYD_REAL batch_checksum = 0.0f;

        (void)clock_gettime(clock_id, &start);
        for (i = 0U; i < BENCHMARK_BATCH_SIZE; ++i) {
            uint32_t iteration = batch * BENCHMARK_BATCH_SIZE + i;
            fb.AXIS_REF.position = sweep_position(prepared, iteration);
            fb.AXIS_REF.velocity = fb.STATE.plannedVelocity;
            fb.AXIS_REF.flow = fb.STATE.plannedFlow;
            fb.AXIS_REF.timestamp += 0.001f;
            HYD_MotionControlFB_Execute(&fb);
            batch_checksum += fb.STATE.plannedVelocity +
                              fb.STATE.plannedFlow + fb.PUMP_SPEED;
        }
        (void)clock_gettime(clock_id, &end);
        batch_ns = elapsed_ns(start, end);
        timing.totalNs += batch_ns;
        if (batch_ns > timing.maxBatchNs) {
            timing.maxBatchNs = batch_ns;
        }
        benchmark_checksum += batch_checksum;
    }

    if (fb.FB_STATE == HYD_FB_STATE_FAULT) {
        fprintf(stderr, "full-cycle benchmark entered fault state: %d\n",
                (int)fb.DIAGNOSTIC.code);
        timing.totalNs = 0U;
    }
    return timing;
}

int main(void)
{
    HYD_TogglePreparedConfig prepared;
    BenchmarkTiming kinematics;
    BenchmarkTiming direct_cycle;
    BenchmarkTiming toggle_cycle;
    double direct_mean;
    double toggle_mean;
    double incremental_percent;

    if (!prepare_default_toggle(&prepared)) {
        return 1;
    }

    printf("PC regression evidence only; not STM32 WCET.\n");
    printf("Target estimate basis: Cortex-M7F 480 MHz, -Os, "
           "480000 cycles per 1 ms.\n");
    printf("iterations=%u warmup=%u sweep=[%.3f, %.3f] mm\n",
           BENCHMARK_ITERATIONS, BENCHMARK_WARMUP,
           prepared.xHandoffEffective, prepared.raw.sm);
    printf("toggle slot bytes=%zu validation bytes=%zu motion fb bytes=%zu\n",
           HYD_ToggleMechanismPool_SlotSize(),
           sizeof(HYD_ToggleValidation), sizeof(HYD_MotionControlFB));

    kinematics = benchmark_kinematics(&prepared);
    direct_cycle = benchmark_full_cycle(false, &prepared);
    toggle_cycle = benchmark_full_cycle(true, &prepared);
    if (kinematics.totalNs == 0U || direct_cycle.totalNs == 0U ||
        toggle_cycle.totalNs == 0U) {
        return 1;
    }

    print_timing("toggle kinematics", kinematics);
    print_timing("direct full cycle", direct_cycle);
    print_timing("toggle full cycle", toggle_cycle);
    direct_mean = (double)direct_cycle.totalNs /
                  (double)BENCHMARK_ITERATIONS;
    toggle_mean = (double)toggle_cycle.totalNs /
                  (double)BENCHMARK_ITERATIONS;
    incremental_percent = (toggle_mean - direct_mean) * 100.0 / direct_mean;
    printf("toggle full-cycle increment %.2f%%\n", incremental_percent);
    printf("checksum=%.6f\n", (double)benchmark_checksum);
    return 0;
}
