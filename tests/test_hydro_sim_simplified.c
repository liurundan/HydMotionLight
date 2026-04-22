/**
 * 测试简化版液压仿真器
 * 验证阀门接口被正确解释为方向控制，且不再模拟阀门动力学
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "hydro_sim.h"
#include "hydro_interfaces.h"

#define CYCLE_PERIOD 0.001  /* 1ms */

int main(void) {
    HydraulicSimEnv env;
    ISensorBackend* inject_backend;
    AxisFeedback fb;
    HydroPump pump;
    HydroValve valve_fwd, valve_bwd;
    HydroValve* valves[2];
    int test_passed = 1;

    printf("=== Simplified Hydraulic Simulator Test ===\n\n");

    /* 初始化仿真环境 */
    HydraulicSim_Init(&env);
    inject_backend = HydraulicSim_GetInjectBackend(&env);
    if (inject_backend == NULL) {
        printf("ERROR: Failed to get inject backend\n");
        return 1;
    }

    /* 测试1：前进方向 */
    printf("=== Test 1: Forward Direction ===\n");

    /* 设置泵转速 */
    pump.grant_valid = true;
    pump.granted_rpm = 1000.0f;
    inject_backend->write_pump(inject_backend->ctx, &pump);

    /* 设置阀门（前进）*/
    valve_fwd.role = VALVE_ROLE_FWD;
    valve_fwd.cmd_state = true;
    valve_bwd.role = VALVE_ROLE_BWD;
    valve_bwd.cmd_state = false;
    valves[0] = &valve_fwd;
    valves[1] = &valve_bwd;
    inject_backend->write_valves(inject_backend->ctx, valves, 2);

    /* 运行仿真500步 */
    for (int i = 0; i < 500; i++) {
        HydraulicSim_Step(&env, CYCLE_PERIOD);
    }

    /* 读取反馈 */
    inject_backend->read_feedback(inject_backend->ctx, &fb);

    /* 验证结果 */
    if (fb.position_mm <= 0.0f) {
        printf("ERROR: Expected positive position, got %.2f mm\n", fb.position_mm);
        test_passed = 0;
    } else if (fb.velocity_mm_s <= 0.0f) {
        printf("ERROR: Expected positive velocity, got %.2f mm/s\n", fb.velocity_mm_s);
        test_passed = 0;
    } else {
        printf("  Position: %.2f mm, Velocity: %.2f mm/s\n", fb.position_mm, fb.velocity_mm_s);
        printf("  Result: PASS ✅\n");
    }
    printf("\n");

    /* 测试2：立即切换到后退方向（无延迟）*/
    printf("=== Test 2: Immediate Backward Direction (No Delay) ===\n");

    /* 设置阀门（后退）*/
    valve_fwd.cmd_state = false;
    valve_bwd.cmd_state = true;
    inject_backend->write_valves(inject_backend->ctx, valves, 2);

    /* 运行仿真200步 */
    float start_pos = fb.position_mm;
    for (int i = 0; i < 200; i++) {
        HydraulicSim_Step(&env, CYCLE_PERIOD);
    }

    /* 读取反馈 */
    inject_backend->read_feedback(inject_backend->ctx, &fb);

    /* 验证结果 */
    if (fb.velocity_mm_s >= 0.0f) {
        printf("ERROR: Expected negative velocity, got %.2f mm/s\n", fb.velocity_mm_s);
        test_passed = 0;
    } else if (fb.position_mm >= start_pos) {
        printf("ERROR: Expected position to decrease, got %.2f mm (start: %.2f mm)\n",
               fb.position_mm, start_pos);
        test_passed = 0;
    } else {
        printf("  Position: %.2f mm (started at %.2f mm)\n", fb.position_mm, start_pos);
        printf("  Velocity: %.2f mm/s\n", fb.velocity_mm_s);
        printf("  Result: PASS ✅\n");
    }
    printf("\n");

    /* 测试3：停止（两阀都关闭）*/
    printf("=== Test 3: Stop (Both Valves Off) ===\n");

    /* 设置阀门（都关闭）*/
    valve_fwd.cmd_state = false;
    valve_bwd.cmd_state = false;
    inject_backend->write_valves(inject_backend->ctx, valves, 2);

    /* 运行仿真200步 */
    for (int i = 0; i < 200; i++) {
        HydraulicSim_Step(&env, CYCLE_PERIOD);
    }

    /* 读取反馈 */
    inject_backend->read_feedback(inject_backend->ctx, &fb);

    /* 验证结果 */
    if (fabsf(fb.velocity_mm_s) > 0.01f) {
        printf("ERROR: Expected velocity ~0, got %.2f mm/s\n", fb.velocity_mm_s);
        test_passed = 0;
    } else {
        printf("  Position: %.2f mm, Velocity: %.2f mm/s\n", fb.position_mm, fb.velocity_mm_s);
        printf("  Result: PASS ✅\n");
    }
    printf("\n");

    /* 测试4：设置阀门延迟函数（应该不起作用）*/
    printf("=== Test 4: Valve Switch Delay (Should Have No Effect) ===\n");

    HydraulicSim_SetValveSwitchDelay(&env, 0.1f);  /* 100ms延迟 */

    /* 设置阀门（前进）*/
    valve_fwd.cmd_state = true;
    valve_bwd.cmd_state = false;
    inject_backend->write_valves(inject_backend->ctx, valves, 2);

    /* 检查前10个周期是否有运动 */
    int immediate_motion = 0;
    for (int i = 0; i < 10; i++) {
        HydraulicSim_Step(&env, CYCLE_PERIOD);
        inject_backend->read_feedback(inject_backend->ctx, &fb);
        if (fb.velocity_mm_s > 0.01f) {
            immediate_motion = 1;
            break;
        }
    }

    if (!immediate_motion) {
        printf("ERROR: Expected immediate motion despite delay setting\n");
        test_passed = 0;
    } else {
        printf("  Immediate motion detected: Yes ✅\n");
        printf("  Result: PASS ✅\n");
    }
    printf("\n");

    /* 测试5：压力输出验证 */
    printf("=== Test 5: Pressure Output ===\n");

    /* 重置位置 */
    env.inject_cyl.current_pos_mm = 0.0f;
    env.inject_cyl.current_vel_mm_s = 0.0f;

    /* 设置前进方向并运行 */
    valve_fwd.cmd_state = true;
    valve_bwd.cmd_state = false;
    inject_backend->write_valves(inject_backend->ctx, valves, 2);

    for (int i = 0; i < 500; i++) {
        HydraulicSim_Step(&env, CYCLE_PERIOD);
    }

    /* 验证压力输出 */
    if (env.sim_system_pressure_bar <= 0.0f) {
        printf("ERROR: Expected positive pressure, got %.2f bar\n", env.sim_system_pressure_bar);
        test_passed = 0;
    } else {
        printf("  System pressure: %.2f bar\n", env.sim_system_pressure_bar);
        printf("  Result: PASS ✅\n");
    }
    printf("\n");

    /* 总结 */
    printf("=== Overall Result ===\n");
    printf("Test result: %s\n", test_passed ? "PASS ✅" : "FAIL ❌");

    return test_passed ? 0 : 1;
}
