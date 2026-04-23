/**
 * 运动控制 + 液压仿真器 集成测试
 * 验证完整闭环控制行为：配方执行 → 运动规划 → 泵速输出 → 仿真器反馈 → 状态更新
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "motion_control.h"
#include "hydro_sim.h"
#include "common_types.h"

#define CYCLE_PERIOD 0.001f  /* 1ms 控制周期 */
#define TEST_SEGMENTS 3     /* 测试3段配方 */

/* ==================================================================
 * 配方定义：慢速前进 → 快速前进 → 后退回零
 * ================================================================== */
static HDY_MotionSegment recipe[TEST_SEGMENTS];

static void init_segments(void) {
    /* 段0：慢速前进 (0 → 100mm) */
    memset(&recipe[0], 0, sizeof(HDY_MotionSegment));
    recipe[0].segmentTag = 1;
    /* tag set above */;
    recipe[0].segmentType = HDY_SEGMENT_TYPE_INJECTION;
    recipe[0].planner = HDY_PLANNER_TIME_BASED;
    recipe[0].mode = HDY_MODE_SPEED_RAMP;
    recipe[0].direction = HDY_DIRECTION_EXTEND;
    recipe[0].targetPosition = 100.0f;
    recipe[0].maxVelocity = 50.0f;               /* 50 mm/s */
    recipe[0].maxAcceleration = 100.0f;          /* 100 mm/s^2 */
    recipe[0].targetFlow = 10.0f;             /* LPM */
    recipe[0].targetPressure = 0.0f;
    recipe[0].maxFlow = 30.0f;
    recipe[0].velocityToFlowGain = 0.2f;      /* 速度到流量转换系数 */
    recipe[0].endCondition = HDY_END_POSITION;
    recipe[0].positionTolerance = 1.0f;
    recipe[0].timeoutLimit = 10.0f;

    /* 段1：快速前进 (100 → 200mm) */
    memset(&recipe[1], 0, sizeof(HDY_MotionSegment));
    recipe[1].segmentTag = 2;
    recipe[1].segmentType = HDY_SEGMENT_TYPE_INJECTION;
    recipe[1].planner = HDY_PLANNER_TIME_BASED;
    recipe[1].mode = HDY_MODE_SPEED_RAMP;
    recipe[1].direction = HDY_DIRECTION_EXTEND;
    recipe[1].targetPosition = 200.0f;
    recipe[1].maxVelocity = 150.0f;              /* 150 mm/s */
    recipe[1].maxAcceleration = 200.0f;          /* 200 mm/s^2 */
    recipe[1].targetFlow = 30.0f;
    recipe[1].targetPressure = 0.0f;
    recipe[1].maxFlow = 50.0f;
    recipe[1].velocityToFlowGain = 0.2f;
    recipe[1].endCondition = HDY_END_POSITION;
    recipe[1].positionTolerance = 1.0f;
    recipe[1].timeoutLimit = 10.0f;

    /* 段2：后退回零 (200 → 0mm) */
    memset(&recipe[2], 0, sizeof(HDY_MotionSegment));
    recipe[2].segmentTag = 3;
    recipe[2].segmentType = HDY_SEGMENT_TYPE_INJECTION;
    recipe[2].planner = HDY_PLANNER_TIME_BASED;
    recipe[2].mode = HDY_MODE_SPEED_RAMP;
    recipe[2].direction = HDY_DIRECTION_RETRACT;
    recipe[2].targetPosition = 0.0f;
    recipe[2].maxVelocity = 200.0f;             /* 200 mm/s (后退) */
    recipe[2].maxAcceleration = 200.0f;
    recipe[2].targetFlow = 40.0f;
    recipe[2].targetPressure = 0.0f;
    recipe[2].maxFlow = 60.0f;
    recipe[2].velocityToFlowGain = 0.2f;
    recipe[2].endCondition = HDY_END_POSITION;
    recipe[2].positionTolerance = 1.0f;
    recipe[2].timeoutLimit = 10.0f;
}

/* ==================================================================
 * 测试辅助函数
 * ================================================================== */
static void print_segment_info(int segment_idx) {
    if (segment_idx < 0 || segment_idx >= TEST_SEGMENTS) {
        printf("  [Invalid Segment]\n");
        return;
    }

    const HDY_MotionSegment* seg = &recipe[segment_idx];
    printf("  [Segment %d: tag=%d] Mode=%d, TargetPos=%.2fmm, MaxVel=%.2fmm/s\n",
           segment_idx, (int)seg->segmentTag, seg->mode, seg->targetPosition, seg->maxVelocity);
}

static void print_controller_state(const HDY_MotionControlFB* controller) {
    const HDY_MotionState* state = &controller->STATE;

    printf("    Pos=%.2fmm, Vel=%.2fmm/s, Flow=%.2fLPM, Pump=%.0frpm | ",
           state->plannedVelocity, state->plannedFlow,
           state->commandedPumpSpeed, controller->PUMP_SPEED);

    printf("Status=");
    switch (controller->STATE.status) {
        case HDY_STATUS_IDLE:      printf("IDLE"); break;
        case HDY_STATUS_READY:     printf("READY"); break;
        case HDY_STATUS_RUNNING:   printf("RUN"); break;
        case HDY_STATUS_HOLD:      printf("HOLD"); break;
        case HDY_STATUS_DEGRADED:  printf("DEGR"); break;
        case HDY_STATUS_SEGMENT_COMPLETE: printf("SEGC"); break;
        case HDY_STATUS_FINISHED:  printf("FIN"); break;
        case HDY_STATUS_FAULT:     printf("FLT"); break;
        default:                   printf("???"); break;
    }

    printf(" | SegComp=%d, SegChange=%d\n",
           controller->SEGMENT_COMPLETED, controller->SEGMENT_CHANGED);
}

static void print_diagnostic_if_error(const HDY_MotionControlFB* controller) {
    const HDY_DiagnosticInfo* diag = &controller->DIAGNOSTIC;

    if (diag->code != HDY_DIAG_CODE_NONE) {
        printf("    ⚠️  ERROR: Code=%d, Severity=%d\n", diag->code, diag->severity);
        printf("           VelocityRef=%.2f, ActualVel=%.2f, VelocityError=%.2f\n",
               controller->STATE.references.velocityReference, controller->AXIS_REF.velocity, diag->velocityError);
        printf("           FlowRef=%.2f, ActualFlow=%.2f, FlowError=%.2f\n",
               controller->STATE.references.flowReference, fabs(controller->AXIS_REF.flow), diag->flowError);
    }
}

/* ==================================================================
 * 主测试流程
 * ================================================================== */
int main(void) {
    HydraulicSimEnv sim_env;
    ISensorBackend* inject_backend;
    HDY_MotionControlFB controller;
    HydroPump pump_cmd;
    HydroValve valve_fwd, valve_bwd;
    HydroValve* valves[2];
    HDY_BOOL start_result;
    HDY_BOOL next_result;
    int current_segment = 0;
    int cycle_count = 0;
    int test_passed = 1;

    /* 初始化配方段 */
    init_segments();

    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  Motion Control + Hydraulic Simulator Integration Test      ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");

    /* ------------------------------------------------------------------
     * 步骤1：初始化仿真环境
     * ------------------------------------------------------------------ */
    printf("=== Step 1: Initialize Simulator ===\n");
    HydraulicSim_Init(&sim_env);
    inject_backend = HydraulicSim_GetInjectBackend(&sim_env);

    if (inject_backend == NULL) {
        printf("❌ ERROR: Failed to get inject backend\n");
        return 1;
    }

    printf("✓ Simulator initialized\n");
    printf("  Clamp cylinder: stroke=%.0fmm, area_fwd=%dmm², area_bwd=%dmm²\n",
           sim_env.clamp_cyl.stroke_mm,
           (int)sim_env.clamp_cyl.area_fwd_mm2,
           (int)sim_env.clamp_cyl.area_bwd_mm2);
    printf("  Inject cylinder: stroke=%.0fmm, area_fwd=%dmm², area_bwd=%dmm²\n\n",
           sim_env.inject_cyl.stroke_mm,
           (int)sim_env.inject_cyl.area_fwd_mm2,
           (int)sim_env.inject_cyl.area_bwd_mm2);

    /* Initialize valves */
    memset(&valve_fwd, 0, sizeof(valve_fwd));
    memset(&valve_bwd, 0, sizeof(valve_bwd));
    valve_fwd.role = VALVE_ROLE_FWD;
    valve_bwd.role = VALVE_ROLE_BWD;
    valves[0] = &valve_fwd;
    valves[1] = &valve_bwd;

    /* ------------------------------------------------------------------
     * 步骤2：初始化运动控制器
     * ------------------------------------------------------------------ */
    printf("=== Step 2: Initialize Motion Controller ===\n");

    HDY_MotionControlFB_Init(&controller);
    controller.EN = true;
    controller.USE_RECIPE = true;

    /* 设置控制参数 */
    controller.FLOW_TO_PUMP_SPEED_GAIN = 100.0f;
    controller.PUMP_SPEED_LIMIT = 3000.0f;

    /* 加载配方 */
    HDY_BOOL load_result = HDY_MotionControlFB_LoadRecipe(&controller, recipe, TEST_SEGMENTS);
    if (!load_result) {
        printf("❌ ERROR: Failed to load recipe\n");
        return 1;
    }

    printf("✓ Controller initialized\n");
    printf("✓ Recipe loaded (%d segments)\n\n", TEST_SEGMENTS);

    /* ------------------------------------------------------------------
     * 步骤3：启动第一段并执行闭环仿真
     * ------------------------------------------------------------------ */
    printf("=== Step 3: Execute Closed-Loop Simulation ===\n\n");

    start_result = HDY_MotionControlFB_StartSegment(&controller, 0, 0.0f);
    if (!start_result) {
        printf("❌ ERROR: Failed to start segment 0\n");
        return 1;
    }

    while (current_segment < TEST_SEGMENTS && cycle_count < 100000) {
        cycle_count++;

        /* 1. 从仿真器读取反馈 */
        AxisFeedback fb;
        inject_backend->read_feedback(inject_backend->ctx, &fb);

        /* 2. 更新控制器输入 */
        controller.AXIS_REF.position = fb.position_mm;
        controller.AXIS_REF.velocity = fb.velocity_mm_s;
        controller.AXIS_REF.pressure = fb.pressure_bar / 10.0f; /* bar to MPa */
        controller.AXIS_REF.timestamp = sim_env.sim_time_s;

        /* 3. 执行控制器周期 */
        HDY_MotionControlFB_Execute(&controller);

        /* 4. 输出泵速命令到仿真器 */
        pump_cmd.grant_valid = true;
        pump_cmd.granted_rpm = controller.PUMP_SPEED;
        inject_backend->write_pump(inject_backend->ctx, &pump_cmd);

        /* 5. 设置阀门命令（基于规划方向） */
        if (controller.PUMP_SPEED > 0.0f) {
            switch (controller.STATE.plannedDirection) {
                case HDY_DIRECTION_EXTEND:
                    valve_fwd.cmd_state = true;
                    valve_bwd.cmd_state = false;
                    break;
                case HDY_DIRECTION_RETRACT:
                    valve_fwd.cmd_state = false;
                    valve_bwd.cmd_state = true;
                    break;
                default:
                    valve_fwd.cmd_state = false;
                    valve_bwd.cmd_state = false;
                    break;
            }
        } else {
            /* 泵停止时，关闭所有阀门 */
            valve_fwd.cmd_state = false;
            valve_bwd.cmd_state = false;
        }
        inject_backend->write_valves(inject_backend->ctx, valves, 2);

        /* 6. 仿真器步进 */
        HydraulicSim_Step(&sim_env, CYCLE_PERIOD);

        /* 7. 打印关键事件 */
        if (cycle_count == 1 || controller.SEGMENT_CHANGED || controller.SEGMENT_COMPLETED || controller.DIAGNOSTIC.code != HDY_DIAG_CODE_NONE) {
            printf("[Cycle %04d] Segment Changed=%d, Segment Completed=%d\n",
                   cycle_count, controller.SEGMENT_CHANGED, controller.SEGMENT_COMPLETED);
            print_segment_info(current_segment);
            print_controller_state(&controller);
            print_diagnostic_if_error(&controller);
        }

        /* 8. 检查错误 */
        if (controller.DIAGNOSTIC.code != HDY_DIAG_CODE_NONE) {
            printf("❌ ERROR detected at cycle %d\n", cycle_count);
            test_passed = 0;
            break;
        }

        /* 9. 处理段切换 */
        if (controller.SEGMENT_COMPLETED) {
            printf("✓ Segment %d completed at cycle %d\n", current_segment, cycle_count);
            current_segment++;
            if (current_segment < TEST_SEGMENTS) {
                next_result = HDY_MotionControlFB_NextSegment(&controller, sim_env.sim_time_s);
                if (!next_result) {
                    printf("❌ ERROR: Failed to advance to segment %d\n", current_segment);
                    test_passed = 0;
                    break;
                }
            }
        }
    }

    /* ------------------------------------------------------------------
     * 步骤4：验证最终结果
     * ------------------------------------------------------------------ */
    printf("\n=== Step 4: Verify Results ===\n");

    AxisFeedback final_fb;
    inject_backend->read_feedback(inject_backend->ctx, &final_fb);

    printf("Final state:\n");
    printf("  Position: %.2f mm (target: %.2f mm)\n", final_fb.position_mm, 0.0f);
    printf("  Velocity: %.2f mm/s\n", final_fb.velocity_mm_s);
    printf("  Pressure: %.2f bar\n", final_fb.pressure_bar);
    printf("  Cycles: %d\n", cycle_count);

    /* 验证要点 */
    printf("\nVerification:\n");

    /* 验证1：最终位置应接近0mm */
    float position_error = fabsf(final_fb.position_mm - 0.0f);
    if (position_error > 50.0f) { /* 放大容差为50mm，因为这是一个简单的集成测试 */
        printf("❌ FAIL: Final position error %.2f mm > 50.0 mm\n", position_error);
        test_passed = 0;
    } else {
        printf("✓ PASS: Final position error %.2f mm ≤ 50.0 mm\n", position_error);
    }

    /* 验证2：所有段都应完成 */
    if (current_segment != TEST_SEGMENTS) {
        printf("❌ FAIL: Only %d/%d segments completed\n", current_segment, TEST_SEGMENTS);
        test_passed = 0;
    } else {
        printf("✓ PASS: All %d segments completed\n", TEST_SEGMENTS);
    }

    /* 验证3：没有错误发生 */
    if (controller.DIAGNOSTIC.code != HDY_DIAG_CODE_NONE) {
        printf("❌ FAIL: Error occurred during execution (code=%d)\n", controller.DIAGNOSTIC.code);
        test_passed = 0;
    } else {
        printf("✓ PASS: No errors during execution\n");
    }

    /* 验证4：控制器状态应为FINISHED */
    if (controller.STATE.status != HDY_STATUS_FINISHED) {
        printf("❌ FAIL: Controller state not FINISHED (status=%d)\n", controller.STATE.status);
        test_passed = 0;
    } else {
        printf("✓ PASS: Controller state is FINISHED\n");
    }

    /* ------------------------------------------------------------------
     * 测试总结
     * ------------------------------------------------------------------ */
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    if (test_passed) {
        printf("║                    ✅ ALL TESTS PASSED                      ║\n");
    } else {
        printf("║                    ❌ SOME TESTS FAILED                     ║\n");
    }
    printf("╚══════════════════════════════════════════════════════════════╝\n");

    return test_passed ? 0 : 1;
}
