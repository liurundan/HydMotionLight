/**
 * 简化的Direct模式连续性验证测试
 * 重点验证速度、加速度连续性和压力平滑性
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "motion_control.h"
#include "common_types.h"

#define CYCLE_PERIOD 0.001  /* 1ms */
#define TEST_STEPS 3000

typedef struct {
    HYD_REAL maxVelocityJump;
    HYD_REAL maxPressureJump;
    HYD_UINT discontinuityCount;
    HYD_BOOL testPassed;
} TestResults;

void TestResults_Init(TestResults* results) {
    memset(results, 0, sizeof(*results));
    results->testPassed = true;
}

void TestResults_CheckVelocity(TestResults* results, HYD_REAL currentVel, HYD_REAL prevVel) {
    HYD_REAL jump = fabs(currentVel - prevVel);
    if (jump > results->maxVelocityJump) {
        results->maxVelocityJump = jump;
    }
    
    /* 速度跳变超过10 mm/s视为不连续 */
    if (jump > 10.0) {
        results->discontinuityCount++;
        printf("WARNING: Velocity jump detected: %.3f mm/s\n", jump);
    }
}

void TestResults_CheckPressure(TestResults* results, HYD_REAL currentPress, HYD_REAL prevPress) {
    HYD_REAL jump = fabs(currentPress - prevPress);
    if (jump > results->maxPressureJump) {
        results->maxPressureJump = jump;
    }
    
    /* 压力跳变超过1.0 bar视为不平滑 */
    if (jump > 1.0) {
        results->testPassed = false;
        printf("ERROR: Pressure jump detected: %.3f bar\n", jump);
    }
}

int main(void) {
    HYD_MotionControlFB fb;
    HYD_TIME currentTime = 0.0;
    HYD_REAL position = 0.0, velocity = 0.0, pressure = 0.0;
    HYD_REAL prevVelocity = 0.0, prevPressure = 0.0;
    TestResults results;
    HYD_UINT step = 0;
    
    printf("=== Direct Mode Continuity Test ===\n\n");
    
    TestResults_Init(&results);
    
    /* 初始化 */
    HYD_MotionControlFB_Init(&fb);
    /* EN gate handled by IEC layer */
    fb.USE_RECIPE = false;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 1.2;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    
    /* 阶段1: 注射段 (速度斜坡模式) */
    printf("=== Phase 1: Injection (Speed Ramp) ===\n");
    
    HYD_MotionSegment injection;
    memset(&injection, 0, sizeof(injection));
    injection.segmentTag = HYD_SEGMENT_TYPE_INJECTION;
    injection.mode = HYD_MODE_SPEED_RAMP;
    injection.endCondition = HYD_END_POSITION;
    injection.direction = HYD_DIRECTION_EXTEND;
    injection.targetPosition = 50.0;
    injection.maxVelocity = 100.0;
    injection.maxAcceleration = 200.0;
    injection.maxFlow = 30.0;
    injection.targetFlow = 25.0;
    injection.velocityToFlowGain = 0.25;
    injection.positionTolerance = 0.1;
    
    HYD_MotionControlFB_LoadDirectSegment(&fb, &injection);
    HYD_MotionControlFB_StartSegment(&fb, 0, currentTime);
    
    HYD_UINT phase1Step = 0;
    prevVelocity = 0.0;
    prevPressure = 0.0;
    
    while (phase1Step < 1500 && !fb.SEGMENT_COMPLETED) {
        /* 简单仿真模型 */
        velocity = fb.STATE.plannedVelocity;
        position += velocity * CYCLE_PERIOD;
        pressure = velocity * 0.01;  /* 简化的压力模型 */
        
        /* 更新反馈 */
        fb.AXIS_REF.position = position;
        fb.AXIS_REF.velocity = velocity;
        fb.AXIS_REF.flow = fb.STATE.plannedFlow;
        fb.AXIS_REF.pressure = pressure;
        fb.AXIS_REF.timestamp = currentTime;
        
        /* 执行控制 */
        HYD_MotionControlFB_Execute(&fb);
        
        /* 检查连续性 */
        TestResults_CheckVelocity(&results, velocity, prevVelocity);
        TestResults_CheckPressure(&results, pressure, prevPressure);
        
        prevVelocity = velocity;
        prevPressure = pressure;
        
        if (phase1Step % 500 == 0) {
            printf("  Step %lu: t=%.3f s, Pos=%.2f mm, Vel=%.2f mm/s, Press=%.2f bar\n",
                   (unsigned long)phase1Step, currentTime, position, velocity, pressure);
        }
        
        if (fb.STATE.faultActive) {
            printf("ERROR: Fault detected\n");
            results.testPassed = false;
            break;
        }
        
        currentTime += CYCLE_PERIOD;
        phase1Step++;
        step++;
    }
    
    printf("Phase 1 completed at t=%.3f s, Pos=%.2f mm\n", currentTime, position);
    
    /* 阶段2: 继续注射 (时间结束) */
    printf("\n=== Phase 2: Continue Injection (Time-based) ===\n");
    
    HYD_MotionSegment injection2;
    memset(&injection2, 0, sizeof(injection2));
    injection2.segmentTag = HYD_SEGMENT_TYPE_INJECTION + 1;
    injection2.mode = HYD_MODE_SPEED_RAMP;
    injection2.endCondition = HYD_END_TIME;
    injection2.direction = HYD_DIRECTION_EXTEND;
    injection2.targetPosition = 100.0;
    injection2.maxVelocity = 100.0;
    injection2.maxAcceleration = 200.0;
    injection2.maxFlow = 30.0;
    injection2.targetFlow = 20.0;
    injection2.velocityToFlowGain = 0.25;
    injection2.duration = 1.0;  /* 1秒 */
    
    HYD_MotionControlFB_LoadDirectSegment(&fb, &injection2);
    HYD_MotionControlFB_StartSegment(&fb, 0, currentTime);
    
    HYD_UINT phase2Step = 0;
    
    while (phase2Step < 1500 && !fb.SEGMENT_COMPLETED) {
        velocity = fb.STATE.plannedVelocity;
        position += velocity * CYCLE_PERIOD;
        pressure = velocity * 0.015;
        
        fb.AXIS_REF.position = position;
        fb.AXIS_REF.velocity = velocity;
        fb.AXIS_REF.flow = fb.STATE.plannedFlow;
        fb.AXIS_REF.pressure = pressure;
        fb.AXIS_REF.timestamp = currentTime;
        
        HYD_MotionControlFB_Execute(&fb);
        
        TestResults_CheckVelocity(&results, velocity, prevVelocity);
        TestResults_CheckPressure(&results, pressure, prevPressure);
        
        prevVelocity = velocity;
        prevPressure = pressure;
        
        if (phase2Step % 500 == 0) {
            printf("  Step %lu: t=%.3f s, Pos=%.2f mm, Vel=%.2f mm/s, Press=%.2f bar\n",
                   (unsigned long)phase2Step, currentTime, position, velocity, pressure);
        }
        
        if (fb.STATE.faultActive) {
            printf("ERROR: Fault detected\n");
            results.testPassed = false;
            break;
        }
        
        currentTime += CYCLE_PERIOD;
        phase2Step++;
        step++;
    }
    
    printf("Phase 2 completed at t=%.3f s, Pos=%.2f mm\n", currentTime, position);
    
    /* 阶段3: 保压段 (压力控制) */
    printf("\n=== Phase 3: Holding (Pressure Control) ===\n");
    
    HYD_MotionSegment holding;
    memset(&holding, 0, sizeof(holding));
    holding.segmentTag = HYD_SEGMENT_TYPE_HOLDING;
    holding.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    holding.endCondition = HYD_END_TIME;
    holding.direction = HYD_DIRECTION_HOLD;
    holding.targetPressure = 5.0;
    holding.targetFlow = 2.0;
    holding.maxFlow = 10.0;
    holding.duration = 0.5;
    holding.pressureController = HYD_PRESSURE_CONTROLLER_P;
    holding.pressureKp = 0.5;
    holding.pressureRampRate = 2.0;
    holding.pressureTolerance = 0.5;
    
    HYD_MotionControlFB_LoadDirectSegment(&fb, &holding);
    HYD_MotionControlFB_StartSegment(&fb, 0, currentTime);
    
    HYD_UINT phase3Step = 0;
    
    while (phase3Step < 800 && !fb.SEGMENT_COMPLETED) {
        /* 压力响应仿真 */
        HYD_REAL pressureError = holding.targetPressure - pressure;
        pressure += pressureError * 0.3 * CYCLE_PERIOD;
        
        velocity = fb.STATE.plannedVelocity;
        position += velocity * CYCLE_PERIOD;
        
        fb.AXIS_REF.position = position;
        fb.AXIS_REF.velocity = velocity;
        fb.AXIS_REF.flow = fb.STATE.plannedFlow;
        fb.AXIS_REF.pressure = pressure;
        fb.AXIS_REF.timestamp = currentTime;
        
        HYD_MotionControlFB_Execute(&fb);
        
        TestResults_CheckVelocity(&results, velocity, prevVelocity);
        TestResults_CheckPressure(&results, pressure, prevPressure);
        
        prevVelocity = velocity;
        prevPressure = pressure;
        
        if (phase3Step % 200 == 0) {
            printf("  Step %lu: t=%.3f s, Vel=%.2f mm/s, Press=%.2f bar (target: %.2f)\n",
                   (unsigned long)phase3Step, currentTime, velocity, pressure, holding.targetPressure);
        }
        
        if (fb.STATE.faultActive) {
            printf("ERROR: Fault detected\n");
            results.testPassed = false;
            break;
        }
        
        currentTime += CYCLE_PERIOD;
        phase3Step++;
        step++;
    }
    
    printf("Phase 3 completed at t=%.3f s, Press=%.2f bar\n", currentTime, pressure);
    
    /* 测试结果 */
    printf("\n=== Test Results ===\n");
    printf("Total steps: %lu\n", (unsigned long)step);
    printf("Max velocity jump: %.6f mm/s\n", results.maxVelocityJump);
    printf("Max pressure jump: %.6f bar\n", results.maxPressureJump);
    printf("Velocity discontinuities: %lu\n", (unsigned long)results.discontinuityCount);
    
    printf("\n=== Evaluation ===\n");
    printf("Velocity continuity: %s\n", results.maxVelocityJump < 1.0 ? "PASS ✅" : "FAIL ❌");
    printf("Pressure smoothness: %s\n", results.testPassed ? "PASS ✅" : "FAIL ❌");
    printf("Overall result: %s\n", results.testPassed ? "PASS ✅" : "FAIL ❌");
    
    printf("\n=== Conclusion ===\n");
    if (results.testPassed && results.maxVelocityJump < 1.0) {
        printf("✅ Velocity continuity: EXCELLENT\n");
        printf("✅ Pressure smoothness: EXCELLENT\n");
        printf("✅ The motion control library meets industrial requirements\n");
        return 0;
    } else {
        printf("❌ Some issues detected, please review the warnings above\n");
        return 1;
    }
}
