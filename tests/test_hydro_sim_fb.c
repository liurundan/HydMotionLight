/**
 * @file test_hydro_sim_fb.c
 * @brief 液压仿真器 PLCopen 功能块 单元测试
 *
 * 验证:
 *   1. Init 后状态正确 (EN=false, 输出归零, 默认故障注入值)
 *   2. EN=false 时 Cycle 不步进, 输出归零
 *   3. EN=true 时正常步进, 输出跟随仿真
 *   4. EN true→false→true 行为: 冻结→归零→恢复步进
 *   5. 阀门指令同步
 *   6. 故障注入同步
 *   7. 泵指令 / 泵归属同步
 *   8. 模具障碍物同步
 *   9. GetClampBackend / GetInjectBackend / GetEnv 返回有效指针
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "hydro_sim_fb.h"
#include "hydro_sim.h"

#define CYCLE_PERIOD 0.001
#define TOLERANCE    1e-6

static int tests_run    = 0;
static int tests_passed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { printf("  FAIL: %s\n", msg); } \
} while(0)

#define ASSERT_NEAR(a, b, tol, msg) do { \
    tests_run++; \
    if (fabs((a) - (b)) <= (tol)) { tests_passed++; } \
    else { printf("  FAIL: %s (got %g, expected %g)\n", msg, (double)(a), (double)(b)); } \
} while(0)

/* ==================================================================
 * Test 1: Init 后状态正确
 * ================================================================== */
static void test_init_state(void) {
    HDY_HydraulicSimFB fb;
    HDY_HydraulicSimFB_Init(&fb);

    ASSERT_TRUE(!fb.EN,               "EN should be false after Init");
    ASSERT_TRUE(!fb.ENO,              "ENO should be false after Init");
    ASSERT_TRUE(!fb.ACTIVE,           "ACTIVE should be false after Init");
    ASSERT_TRUE(fb._initialized,      "_initialized should be true after Init");

    /* 故障注入默认值 */
    ASSERT_TRUE(fb.CLAMP_SERVO_READY,    "CLAMP_SERVO_READY default true");
    ASSERT_TRUE(fb.CLAMP_INTERLOCK_OK,   "CLAMP_INTERLOCK_OK default true");
    ASSERT_TRUE(!fb.CLAMP_MOTION_STALL,  "CLAMP_MOTION_STALL default false");
    ASSERT_TRUE(fb.INJECT_SERVO_READY,   "INJECT_SERVO_READY default true");
    ASSERT_TRUE(fb.INJECT_INTERLOCK_OK,  "INJECT_INTERLOCK_OK default true");
    ASSERT_TRUE(!fb.INJECT_MOTION_STALL, "INJECT_MOTION_STALL default false");

    ASSERT_NEAR(fb.CLAMP_PRESSURE_SCALE,  1.0, TOLERANCE, "CLAMP_PRESSURE_SCALE default 1.0");
    ASSERT_NEAR(fb.INJECT_PRESSURE_SCALE, 1.0, TOLERANCE, "INJECT_PRESSURE_SCALE default 1.0");

    /* 输出归零 */
    ASSERT_NEAR(fb.CLAMP_POS_MM, 0.0, TOLERANCE, "CLAMP_POS should be 0");
    ASSERT_NEAR(fb.INJECT_POS_MM, 0.0, TOLERANCE, "INJECT_POS should be 0");
    ASSERT_NEAR(fb.SIM_TIME_S, 0.0, TOLERANCE, "SIM_TIME should be 0");
}

/* ==================================================================
 * Test 2: EN=false 时不步进
 * ================================================================== */
static void test_en_false_no_step(void) {
    HDY_HydraulicSimFB fb;
    HDY_HydraulicSimFB_Init(&fb);

    /* 不设 EN, 直接 Cycle */
    fb.CYCLE_TIME = CYCLE_PERIOD;
    HDY_HydraulicSimFB_Cycle(&fb);

    ASSERT_TRUE(!fb.ENO,   "ENO false when EN false");
    ASSERT_TRUE(!fb.ACTIVE, "ACTIVE false when EN false");
    ASSERT_NEAR(fb.SIM_TIME_S, 0.0, TOLERANCE, "SIM_TIME should remain 0 when EN false");
}

/* ==================================================================
 * Test 3: EN=true 时正常步进
 * ================================================================== */
static void test_en_true_step(void) {
    HDY_HydraulicSimFB fb;
    HDY_HydraulicSimFB_Init(&fb);

    fb.EN         = true;
    fb.CYCLE_TIME = CYCLE_PERIOD;

    /* 设置泵和阀门让合模前进 */
    fb.CMD_RPM         = 1500.0;
    fb.PUMP_OWNER_AXIS = 0;  /* CLAMP */
    fb.CLAMP_VALVE_FWD = true;
    fb.CLAMP_VALVE_BWD = false;

    HDY_HydraulicSimFB_Cycle(&fb);

    ASSERT_TRUE(fb.ENO,   "ENO true when EN true");
    ASSERT_TRUE(fb.ACTIVE, "ACTIVE true when EN true");
    ASSERT_TRUE(fb.SIM_TIME_S > 0.0, "SIM_TIME should advance after step");

    /* 合模前进, 位置应该增大 */
    ASSERT_TRUE(fb.CLAMP_POS_MM > 0.0, "CLAMP_POS should increase when valve fwd + pump on");
    ASSERT_TRUE(fb.CLAMP_VEL_MM_S > 0.0, "CLAMP_VEL should be positive when valve fwd");
}

/* ==================================================================
 * Test 4: EN true→false→true 行为
 * ================================================================== */
static void test_en_toggle(void) {
    HDY_HydraulicSimFB fb;
    HDY_HydraulicSimFB_Init(&fb);

    fb.EN         = true;
    fb.CYCLE_TIME = CYCLE_PERIOD;
    fb.CMD_RPM         = 1500.0;
    fb.PUMP_OWNER_AXIS = 0;
    fb.CLAMP_VALVE_FWD = true;
    fb.CLAMP_VALVE_BWD = false;

    /* 先跑几步 */
    for (int i = 0; i < 10; i++) {
        HDY_HydraulicSimFB_Cycle(&fb);
    }
    HDY_REAL pos_after_run = fb.CLAMP_POS_MM;
    ASSERT_TRUE(pos_after_run > 0.0, "Position should advance after running");

    /* EN=false → 输出归零, 但内部 env 物理状态保留 */
    fb.EN = false;
    HDY_HydraulicSimFB_Cycle(&fb);
    ASSERT_TRUE(!fb.ACTIVE, "ACTIVE false when EN false");
    ASSERT_NEAR(fb.CLAMP_POS_MM, 0.0, TOLERANCE, "Output zeroed when EN false");
    ASSERT_NEAR(fb.SIM_TIME_S, 0.0, TOLERANCE, "SIM_TIME output zeroed when EN false");

    /* EN=true → 恢复, 内部物理状态保留, 输出跟随 */
    fb.EN = true;
    fb.CMD_RPM = 0.0;  /* 泵关, 不产生新运动 */
    HDY_HydraulicSimFB_Cycle(&fb);

    /* 位置输出应该恢复到内部 env 保留的值 (之前运动过的位置) */
    ASSERT_TRUE(fb.ACTIVE, "ACTIVE true when EN restored");
    /* 内部 env 未被复位, 所以位置应该还保持之前的值 */
    ASSERT_NEAR(fb.CLAMP_POS_MM, pos_after_run, 0.01,
                "Position should restore from internal env after EN restored");
}

/* ==================================================================
 * Test 5: 阀门指令同步
 * ================================================================== */
static void test_valve_sync(void) {
    HDY_HydraulicSimFB fb;
    HDY_HydraulicSimFB_Init(&fb);

    fb.EN         = true;
    fb.CYCLE_TIME = CYCLE_PERIOD;
    fb.CMD_RPM         = 1500.0;
    fb.PUMP_OWNER_AXIS = 1;  /* INJECT */

    /* 射胶前进 — 多跑几步, 确保位移足够观察反向运动 */
    fb.INJECT_VALVE_FWD = true;
    fb.INJECT_VALVE_BWD = false;

    for (int i = 0; i < 20; i++) {
        HDY_HydraulicSimFB_Cycle(&fb);
    }
    ASSERT_TRUE(fb.INJECT_POS_MM > 0.0, "INJECT_POS should increase when valve fwd");

    /* 切换到后退 */
    fb.INJECT_VALVE_FWD = false;
    fb.INJECT_VALVE_BWD = true;

    HDY_HydraulicSimFB_Cycle(&fb);
    /* 后退方向, 速度应为负 */
    ASSERT_TRUE(fb.INJECT_VEL_MM_S < 0.0, "INJECT_VEL should be negative when valve bwd");
}

/* ==================================================================
 * Test 6: 故障注入同步 — motion stall
 * ================================================================== */
static void test_fault_injection_stall(void) {
    HDY_HydraulicSimFB fb;
    HDY_HydraulicSimFB_Init(&fb);

    fb.EN         = true;
    fb.CYCLE_TIME = CYCLE_PERIOD;
    fb.CMD_RPM         = 1500.0;
    fb.PUMP_OWNER_AXIS = 0;
    fb.CLAMP_VALVE_FWD = true;
    fb.CLAMP_VALVE_BWD = false;

    /* 正常运行一步 */
    HDY_HydraulicSimFB_Cycle(&fb);
    HDY_REAL pos_normal = fb.CLAMP_POS_MM;
    ASSERT_TRUE(pos_normal > 0.0, "Normal: position should advance");

    /* 注入 motion stall */
    fb.CLAMP_MOTION_STALL = true;
    HDY_HydraulicSimFB_Cycle(&fb);
    /* stall 时速度应为0, 位置不变 */
    ASSERT_NEAR(fb.CLAMP_VEL_MM_S, 0.0, TOLERANCE,
                "Velocity should be 0 when motion stalled");
}

/* ==================================================================
 * Test 7: 泵指令和泵归属同步
 * ================================================================== */
static void test_pump_owner_sync(void) {
    HDY_HydraulicSimFB fb;
    HDY_HydraulicSimFB_Init(&fb);

    fb.EN         = true;
    fb.CYCLE_TIME = CYCLE_PERIOD;
    fb.CMD_RPM         = 1000.0;
    fb.PUMP_OWNER_AXIS = 0;  /* CLAMP */
    fb.CLAMP_VALVE_FWD = true;

    HDY_HydraulicSimFB_Cycle(&fb);
    ASSERT_TRUE(fb.CLAMP_VEL_MM_S > 0.0, "Clamp should move when pump owner=CLAMP");

    /* 切换到射胶 */
    fb.PUMP_OWNER_AXIS = 1;  /* INJECT */
    fb.CLAMP_VALVE_FWD = false;
    fb.INJECT_VALVE_FWD = true;

    HDY_HydraulicSimFB_Cycle(&fb);
    ASSERT_TRUE(fb.INJECT_VEL_MM_S > 0.0, "Inject should move when pump owner=INJECT");
}

/* ==================================================================
 * Test 8: 模具障碍物同步
 * ================================================================== */
static void test_obstacle_sync(void) {
    HDY_HydraulicSimFB fb;
    HDY_HydraulicSimFB_Init(&fb);

    fb.EN         = true;
    fb.CYCLE_TIME = CYCLE_PERIOD;
    fb.CMD_RPM         = 1500.0;
    fb.PUMP_OWNER_AXIS = 0;
    fb.CLAMP_VALVE_FWD = true;

    /* 不设置障碍物 — 正常推进 */
    HDY_HydraulicSimFB_Cycle(&fb);

    /* 设置障碍物, 位置在障碍物处 */
    fb.MOLD_OBSTACLE       = true;
    fb.OBSTACLE_POS_MM     = 400.0;
    fb.OBSTACLE_STIFFNESS  = 80000.0;

    /* 多跑几步, 不应崩溃 */
    for (int i = 0; i < 5; i++) {
        HDY_HydraulicSimFB_Cycle(&fb);
    }
    ASSERT_TRUE(fb.CLAMP_POS_MM >= 0.0, "Clamp position should remain valid with obstacle");
}

/* ==================================================================
 * Test 9: GetBackend / GetEnv 返回有效指针
 * ================================================================== */
static void test_accessor_functions(void) {
    HDY_HydraulicSimFB fb;
    HDY_HydraulicSimFB_Init(&fb);

    ISensorBackend* clamp_be = HDY_HydraulicSimFB_GetClampBackend(&fb);
    ISensorBackend* inject_be = HDY_HydraulicSimFB_GetInjectBackend(&fb);
    HydraulicSimEnv* env = HDY_HydraulicSimFB_GetEnv(&fb);

    ASSERT_TRUE(clamp_be != NULL,  "GetClampBackend should return non-NULL");
    ASSERT_TRUE(inject_be != NULL, "GetInjectBackend should return non-NULL");
    ASSERT_TRUE(env != NULL,       "GetEnv should return non-NULL");

    /* NULL 指针保护 */
    ASSERT_TRUE(HDY_HydraulicSimFB_GetClampBackend(NULL) == NULL,  "NULL fb → NULL clamp backend");
    ASSERT_TRUE(HDY_HydraulicSimFB_GetInjectBackend(NULL) == NULL, "NULL fb → NULL inject backend");
    ASSERT_TRUE(HDY_HydraulicSimFB_GetEnv(NULL) == NULL,           "NULL fb → NULL env");
}

/* ==================================================================
 * Test 10: NULL 指针保护
 * ================================================================== */
static void test_null_safety(void) {
    /* 这些调用不应崩溃 */
    HDY_HydraulicSimFB_Init(NULL);
    HDY_HydraulicSimFB_Cycle(NULL);
    tests_run += 2;
    tests_passed += 2;  /* 到达此处即表示未崩溃 */
}

/* ==================================================================
 * Test 11: 压力偏置/比例因子同步
 * ================================================================== */
static void test_pressure_injection_sync(void) {
    HDY_HydraulicSimFB fb;
    HDY_HydraulicSimFB_Init(&fb);

    fb.EN         = true;
    fb.CYCLE_TIME = CYCLE_PERIOD;
    fb.CMD_RPM         = 1500.0;
    fb.PUMP_OWNER_AXIS = 0;
    fb.CLAMP_VALVE_FWD = true;

    /* 正常一步 */
    HDY_HydraulicSimFB_Cycle(&fb);

    /* 设置偏置 */
    fb.CLAMP_PRESSURE_BIAS  = 50.0;
    fb.CLAMP_PRESSURE_SCALE = 2.0;
    HDY_HydraulicSimFB_Cycle(&fb);

    /* 验证内部 env 的 fault injection 已更新 */
    HydraulicSimEnv* env = HDY_HydraulicSimFB_GetEnv(&fb);
    ASSERT_TRUE(env != NULL, "env should be non-NULL");
    ASSERT_NEAR(env->clamp_feedback_injection.pressure_bias_bar, 50.0, TOLERANCE,
                "Clamp pressure bias should be synced to env");
    ASSERT_NEAR(env->clamp_feedback_injection.pressure_scale, 2.0, TOLERANCE,
                "Clamp pressure scale should be synced to env");
}

/* ==================================================================
 * Test 12: 多周期连续运行稳定性
 * ================================================================== */
static void test_continuous_run(void) {
    HDY_HydraulicSimFB fb;
    HDY_HydraulicSimFB_Init(&fb);

    fb.EN         = true;
    fb.CYCLE_TIME = CYCLE_PERIOD;
    fb.CMD_RPM         = 1500.0;
    fb.PUMP_OWNER_AXIS = 0;
    fb.CLAMP_VALVE_FWD = true;

    for (int i = 0; i < 1000; i++) {
        HDY_HydraulicSimFB_Cycle(&fb);
    }

    ASSERT_TRUE(fb.SIM_TIME_S > 0.0, "SIM_TIME should advance after 1000 cycles");
    ASSERT_TRUE(fb.CLAMP_POS_MM > 0.0, "CLAMP_POS should be positive after 1000 cycles");
    /* 位置不应超过行程 */
    HydraulicSimEnv* env = HDY_HydraulicSimFB_GetEnv(&fb);
    ASSERT_TRUE(fb.CLAMP_POS_MM <= env->clamp_cyl.stroke_mm + TOLERANCE,
                "CLAMP_POS should not exceed stroke");
}

/* ==================================================================
 * Test 13: 使用 Backend 接口读取反馈 (与旧代码兼容)
 * ================================================================== */
static void test_backend_feedback(void) {
    HDY_HydraulicSimFB fb;
    HDY_HydraulicSimFB_Init(&fb);

    fb.EN         = true;
    fb.CYCLE_TIME = CYCLE_PERIOD;
    fb.CMD_RPM         = 1500.0;
    fb.PUMP_OWNER_AXIS = 0;
    fb.CLAMP_VALVE_FWD = true;

    HDY_HydraulicSimFB_Cycle(&fb);

    /* 通过 Backend 接口读取反馈 */
    ISensorBackend* clamp_be = HDY_HydraulicSimFB_GetClampBackend(&fb);
    AxisFeedback axis_fb;
    clamp_be->read_feedback(clamp_be->ctx, &axis_fb);

    ASSERT_TRUE(axis_fb.position_mm > 0.0, "Backend feedback position should be positive");
    ASSERT_TRUE(axis_fb.servo_ready,       "Backend servo_ready should be true (default)");
    ASSERT_TRUE(axis_fb.interlock_ok,      "Backend interlock_ok should be true (default)");
}

/* ==================================================================
 * Test 14: EN=false 时不修改内部 env (物理状态保留)
 * ================================================================== */
static void test_en_false_preserves_env(void) {
    HDY_HydraulicSimFB fb;
    HDY_HydraulicSimFB_Init(&fb);

    fb.EN         = true;
    fb.CYCLE_TIME = CYCLE_PERIOD;
    fb.CMD_RPM         = 1500.0;
    fb.PUMP_OWNER_AXIS = 0;
    fb.CLAMP_VALVE_FWD = true;

    /* 跑几步 */
    for (int i = 0; i < 10; i++) {
        HDY_HydraulicSimFB_Cycle(&fb);
    }
    float env_pos = fb._env.clamp_cyl.current_pos_mm;
    float env_time = fb._env.sim_time_s;
    ASSERT_TRUE(env_pos > 0.0f, "env position should be positive after running");

    /* EN=false */
    fb.EN = false;
    HDY_HydraulicSimFB_Cycle(&fb);

    /* 内部 env 的物理状态应该保留, 不被清零 */
    ASSERT_TRUE(fabsf(fb._env.clamp_cyl.current_pos_mm - env_pos) < 0.001f,
                "Internal env position should be preserved when EN=false");
    ASSERT_TRUE(fabsf(fb._env.sim_time_s - env_time) < 0.001f,
                "Internal env time should be preserved when EN=false");
}

/* ==================================================================
 * Main
 * ================================================================== */
int main(void) {
    printf("=== HydraulicSimFB Unit Tests ===\n\n");

    test_init_state();
    test_en_false_no_step();
    test_en_true_step();
    test_en_toggle();
    test_valve_sync();
    test_fault_injection_stall();
    test_pump_owner_sync();
    test_obstacle_sync();
    test_accessor_functions();
    test_null_safety();
    test_pressure_injection_sync();
    test_continuous_run();
    test_backend_feedback();
    test_en_false_preserves_env();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
