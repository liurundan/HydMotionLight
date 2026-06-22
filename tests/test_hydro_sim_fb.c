/**
 * @file test_hydro_sim_fb.c
 * @brief 液压仿真器 PLC 适配层单元测试
 *
 * 目标：验证 create/move/read/framework_* 的 PLC 接口语义，
 *       以及“共享 env + 单泵单次步进”的架构约束。
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "hydro_sim_fb.h"
#include "hydro_sim.h"
#include "pressure_model.h"

extern HYD_HydraulicSimFB* __MK_GetPublic_HydraulicSimFB(int index);

#define CYCLE_PERIOD 0.001
#define TOLERANCE    1e-6
#define INVALID_AXIS_ID 99

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { printf("  FAIL: %s\n", msg); } \
} while (0)

#define ASSERT_NEAR(a, b, tol, msg) do { \
    tests_run++; \
    if (fabs((double)((a) - (b))) <= (tol)) { tests_passed++; } \
    else { printf("  FAIL: %s (got %g, expected %g)\n", msg, (double)(a), (double)(b)); } \
} while (0)

static void reset_create_cmd(HYD_CREATESIMAXIS* cmd,
                             unsigned char axis_type,
                             double max_vel,
                             double max_acc,
                             double max_dec) {
    memset(cmd, 0, sizeof(*cmd));
    cmd->AXISTYPE.value = axis_type;
    cmd->MAXVEL.value = max_vel;
    cmd->MAXACC.value = max_acc;
    cmd->MAXDEC.value = max_dec;
}

static int create_axis(unsigned char axis_type,
                       double max_vel,
                       double max_acc,
                       double max_dec) {
    HYD_CREATESIMAXIS cmd;
    reset_create_cmd(&cmd, axis_type, max_vel, max_acc, max_dec);
    __mcl_cmd_createSimAxis(&cmd);
    return cmd.AXISID.value;
}

static void move_axis(int axis_id, bool enable, double cmd_rpm, int direction) {
    HYD_MOVESIMAXIS cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.ENABLE.value = enable;
    cmd.AXISID.value = axis_id;
    cmd.CMD_RPM.value = cmd_rpm;
    cmd.DIRECTION.value = direction;
    __mcl_cmd_moveSimAxis(&cmd);
}

static void read_axis(HYD_READSIMAXIS* cmd, int axis_id, bool enable) {
    memset(cmd, 0, sizeof(*cmd));
    cmd->ENABLE.value = enable;
    cmd->AXISID.value = axis_id;
    __mcl_cmd_readSimAxis(cmd);
}

static void reset_pressure_model_cmd(HYD_PRESSUREMODEL *cmd) {
    memset(cmd, 0, sizeof(*cmd));
    cmd->MODEL_TYPE.value = PRESSURE_MODEL_TYPE_PHYSICAL;
    cmd->K_NUM.value = 0.0;
    cmd->TTAU.value = 0.2;
    cmd->DELAYTIME.value = 0.0;
}

/* ==================================================================
 * Test 1: framework init/reset 归零分配器与共享 env
 * ================================================================== */
static void test_framework_init_resets_allocator_and_env(void) {
    HYD_HydraulicSimFB* fb;
    int axis_id;

    __HydSimulator_framework_Init();
    ASSERT_TRUE(__MK_GetPublic_HydraulicSimFB(0) == NULL,
                "No public axis handle should exist immediately after framework init");

    axis_id = create_axis((unsigned char)SIM_AXIS_CLAMP, 120.0, 200.0, 180.0);
    fb = __MK_GetPublic_HydraulicSimFB(axis_id);

    ASSERT_TRUE(axis_id == 0, "First allocated axis id should be 0");
    ASSERT_TRUE(fb != NULL, "Created axis handle should be retrievable");
    ASSERT_TRUE(fb->_env != NULL, "Created axis handle should bind to a shared env");
    ASSERT_NEAR(fb->_env->sim_time_s, 0.0, TOLERANCE,
                "Shared env time should start from zero after init");

    move_axis(axis_id, true, 1500.0, 1);
    __HydSimulator_framework_Publish();
    ASSERT_NEAR(fb->_env->sim_time_s, CYCLE_PERIOD, TOLERANCE,
                "Publish should advance shared env time by one scan period");

    __HydSimulator_framework_Init();
    axis_id = create_axis((unsigned char)SIM_AXIS_CLAMP, 80.0, 90.0, 70.0);
    fb = __MK_GetPublic_HydraulicSimFB(axis_id);

    ASSERT_TRUE(axis_id == 0, "Framework re-init should reset axis allocator to 0");
    ASSERT_TRUE(fb != NULL, "Axis handle should still be retrievable after re-init");
    ASSERT_NEAR(fb->_env->sim_time_s, 0.0, TOLERANCE,
                "Framework re-init should reset shared env time to zero");
    ASSERT_TRUE(fb->allocated, "Axis handle should be marked allocated after create");
}

/* ==================================================================
 * Test 2: create 两个轴，分配唯一 ID，绑定类型并保存配置
 * ================================================================== */
static void test_create_two_axes_assigns_unique_ids_and_keeps_config(void) {
    HYD_HydraulicSimFB* clamp_fb;
    HYD_HydraulicSimFB* inject_fb;
    int clamp_id;
    int inject_id;

    __HydSimulator_framework_Init();

    clamp_id = create_axis((unsigned char)SIM_AXIS_CLAMP, 111.0, 222.0, 333.0);
    inject_id = create_axis((unsigned char)SIM_AXIS_INJECT, 444.0, 555.0, 666.0);

    clamp_fb = __MK_GetPublic_HydraulicSimFB(clamp_id);
    inject_fb = __MK_GetPublic_HydraulicSimFB(inject_id);

    ASSERT_TRUE(clamp_id != inject_id, "Clamp and inject should receive different AXISID values");
    ASSERT_TRUE(clamp_fb != NULL && inject_fb != NULL,
                "Both created axis handles should be retrievable");
    ASSERT_TRUE(clamp_fb->_env == inject_fb->_env,
                "All PLC-created axis handles should share one HydraulicSimEnv");
    ASSERT_TRUE(clamp_fb->axis_type == (HYD_UINT8)SIM_AXIS_CLAMP,
                "Clamp handle should keep clamp axis type");
    ASSERT_TRUE(inject_fb->axis_type == (HYD_UINT8)SIM_AXIS_INJECT,
                "Inject handle should keep inject axis type");
    ASSERT_NEAR(clamp_fb->maxVel, 111.0, TOLERANCE, "Clamp MAXVEL should be stored in handle config");
    ASSERT_NEAR(clamp_fb->maxAcc, 222.0, TOLERANCE, "Clamp MAXACC should be stored in handle config");
    ASSERT_NEAR(clamp_fb->maxDec, 333.0, TOLERANCE, "Clamp MAXDEC should be stored in handle config");
    ASSERT_NEAR(inject_fb->maxVel, 444.0, TOLERANCE, "Inject MAXVEL should be stored in handle config");
    ASSERT_NEAR(inject_fb->maxAcc, 555.0, TOLERANCE, "Inject MAXACC should be stored in handle config");
    ASSERT_NEAR(inject_fb->maxDec, 666.0, TOLERANCE, "Inject MAXDEC should be stored in handle config");
    ASSERT_TRUE(clamp_fb->_env->axis_count == 2,
                "Shared env axis_count should reflect the number of allocated axes");
}

/* ==================================================================
 * Test 3: create 必须校验非法轴类型
 * ================================================================== */
static void test_create_rejects_invalid_axis_type(void) {
    HYD_CREATESIMAXIS cmd;

    __HydSimulator_framework_Init();

    reset_create_cmd(&cmd, 7U, 10.0, 20.0, 30.0);
    cmd.AXISID.value = -1;
    __mcl_cmd_createSimAxis(&cmd);

    ASSERT_TRUE(!cmd.DONE.value, "Invalid AXISTYPE should not set DONE");
    ASSERT_TRUE(cmd.AXISID.value == -1, "Invalid AXISTYPE should not allocate a new AXISID");
    ASSERT_TRUE(__MK_GetPublic_HydraulicSimFB(0) == NULL,
                "Invalid AXISTYPE should not create any public axis handle");
}

/* ==================================================================
 * Test 4: move 指定轴后，只允许该轴在单泵下运动
 * ================================================================== */
static void test_move_only_target_axis_under_single_pump_owner(void) {
    HYD_READSIMAXIS clamp_read;
    HYD_READSIMAXIS inject_read;
    int clamp_id;
    int inject_id;
    double clamp_pos_after_first_publish;

    __HydSimulator_framework_Init();

    clamp_id = create_axis((unsigned char)SIM_AXIS_CLAMP, 120.0, 0.0, 0.0);
    inject_id = create_axis((unsigned char)SIM_AXIS_INJECT, 140.0, 0.0, 0.0);

    move_axis(clamp_id, true, 1500.0, 1);
    __HydSimulator_framework_Publish();

    read_axis(&clamp_read, clamp_id, true);
    read_axis(&inject_read, inject_id, true);

    ASSERT_TRUE(clamp_read.ACTIVE.value, "Clamp should be ACTIVE when it owns the pump");
    ASSERT_TRUE(clamp_read.BUSY.value, "Clamp should be BUSY when it owns the pump");
    ASSERT_TRUE(clamp_read.POS_MM.value > 0.0, "Clamp position should advance after clamp move");
    ASSERT_NEAR(inject_read.POS_MM.value, 0.0, TOLERANCE,
                "Inject axis should remain still while clamp owns the pump");

    clamp_pos_after_first_publish = clamp_read.POS_MM.value;

    move_axis(inject_id, true, 1500.0, 1);
    __HydSimulator_framework_Publish();

    read_axis(&clamp_read, clamp_id, true);
    read_axis(&inject_read, inject_id, true);

    ASSERT_TRUE(inject_read.POS_MM.value > 0.0,
                "Inject axis should advance after pump ownership switches to inject");
    ASSERT_NEAR(clamp_read.POS_MM.value, clamp_pos_after_first_publish, TOLERANCE,
                "Clamp position should stay frozen once the pump owner switches away");
}

/* ==================================================================
 * Test 5: read 必须严格按 AXISID 返回各自反馈
 * ================================================================== */
static void test_read_uses_axisid_not_current_pump_owner(void) {
    HYD_READSIMAXIS clamp_read;
    HYD_READSIMAXIS inject_read;
    int clamp_id;
    int inject_id;
    double clamp_pos_before_switch;

    __HydSimulator_framework_Init();

    clamp_id = create_axis((unsigned char)SIM_AXIS_CLAMP, 120.0, 0.0, 0.0);
    inject_id = create_axis((unsigned char)SIM_AXIS_INJECT, 120.0, 0.0, 0.0);

    move_axis(clamp_id, true, 1500.0, 1);
    __HydSimulator_framework_Publish();
    read_axis(&clamp_read, clamp_id, true);
    clamp_pos_before_switch = clamp_read.POS_MM.value;

    move_axis(inject_id, true, 1500.0, 1);
    __HydSimulator_framework_Publish();
    read_axis(&clamp_read, clamp_id, true);
    read_axis(&inject_read, inject_id, true);

    ASSERT_NEAR(clamp_read.POS_MM.value, clamp_pos_before_switch, TOLERANCE,
                "Read should still return clamp feedback even after pump switches to inject");
    ASSERT_TRUE(inject_read.POS_MM.value > 0.0,
                "Read should return inject feedback for inject AXISID");
    ASSERT_TRUE(fabs(clamp_read.POS_MM.value - inject_read.POS_MM.value) > TOLERANCE,
                "Different AXISID values should expose different axis snapshots");
}

/* ==================================================================
 * Test 6: 多轴存在时，每次 publish 只能步进共享 env 一次
 * ================================================================== */
static void test_publish_steps_shared_env_once_per_scan(void) {
    HYD_HydraulicSimFB* clamp_fb;
    HYD_HydraulicSimFB* inject_fb;
    int clamp_id;
    int inject_id;

    __HydSimulator_framework_Init();

    clamp_id = create_axis((unsigned char)SIM_AXIS_CLAMP, 120.0, 0.0, 0.0);
    inject_id = create_axis((unsigned char)SIM_AXIS_INJECT, 120.0, 0.0, 0.0);
    clamp_fb = __MK_GetPublic_HydraulicSimFB(clamp_id);
    inject_fb = __MK_GetPublic_HydraulicSimFB(inject_id);

    ASSERT_TRUE(clamp_fb->_env == inject_fb->_env,
                "Shared env pointer should be identical across PLC-created axis handles");

    move_axis(clamp_id, true, 1500.0, 1);
    __HydSimulator_framework_Publish();
    ASSERT_NEAR(clamp_fb->_env->sim_time_s, CYCLE_PERIOD, TOLERANCE,
                "First publish should advance shared env by exactly one cycle");

    __HydSimulator_framework_Publish();
    ASSERT_NEAR(clamp_fb->_env->sim_time_s, 2.0 * CYCLE_PERIOD, TOLERANCE,
                "Second publish should advance shared env by exactly one more cycle");
}

/* ==================================================================
 * Test 7: 每轴故障注入必须独立
 * ================================================================== */
static void test_fault_injection_is_isolated_per_axis(void) {
    HYD_HydraulicSimFB* clamp_fb;
    HYD_READSIMAXIS clamp_read;
    HYD_READSIMAXIS inject_read;
    int clamp_id;
    int inject_id;

    __HydSimulator_framework_Init();

    clamp_id = create_axis((unsigned char)SIM_AXIS_CLAMP, 120.0, 0.0, 0.0);
    inject_id = create_axis((unsigned char)SIM_AXIS_INJECT, 120.0, 0.0, 0.0);
    clamp_fb = __MK_GetPublic_HydraulicSimFB(clamp_id);

    ASSERT_TRUE(clamp_fb != NULL, "Clamp handle should exist before applying fault injection");

    HydraulicSim_SetAxisMotionStall(clamp_fb->_env, clamp_id, true);

    move_axis(clamp_id, true, 1500.0, 1);
    __HydSimulator_framework_Publish();
    read_axis(&clamp_read, clamp_id, true);

    ASSERT_NEAR(clamp_read.VEL_MM_S.value, 0.0, TOLERANCE,
                "Clamp motion stall should stop only the clamp axis velocity");

    move_axis(inject_id, true, 1500.0, 1);
    __HydSimulator_framework_Publish();
    read_axis(&inject_read, inject_id, true);

    ASSERT_TRUE(inject_read.POS_MM.value > 0.0,
                "Inject axis should still move even when clamp axis is stalled");
}

/* ==================================================================
 * Test 8: 非法 AXISID 调用必须安全且不污染其他轴
 * ================================================================== */
static void test_invalid_axisid_is_safe_and_does_not_pollute_other_axes(void) {
    HYD_READSIMAXIS clamp_read;
    double clamp_pos_before_invalid_ops;
    int clamp_id;

    __HydSimulator_framework_Init();

    clamp_id = create_axis((unsigned char)SIM_AXIS_CLAMP, 120.0, 0.0, 0.0);
    move_axis(clamp_id, true, 1500.0, 1);
    __HydSimulator_framework_Publish();
    read_axis(&clamp_read, clamp_id, true);
    clamp_pos_before_invalid_ops = clamp_read.POS_MM.value;

    move_axis(INVALID_AXIS_ID, true, 2000.0, -1);
    read_axis(&clamp_read, INVALID_AXIS_ID, true);
    __HydSimulator_framework_Publish();
    read_axis(&clamp_read, clamp_id, true);

    ASSERT_TRUE(clamp_read.POS_MM.value > clamp_pos_before_invalid_ops,
                "Invalid AXISID operations should not stop or corrupt a valid moving axis");
}

/* ==================================================================
 * Test 9: PressureModel FB 必须跨拍保持状态，并在 disable 时复位
 * ================================================================== */
static void test_pressure_model_fb_persists_state_and_resets_on_disable(void) {
    HYD_PRESSUREMODEL cmd;
    double first_step_rpm;
    double first_step_real_pressure;

    reset_pressure_model_cmd(&cmd);

    cmd.ENABLE.value = true;
    cmd.MOTOR_RPM.value = 1000.0;
    cmd.TIME_S.value = 0.000;
    __mcl_cmd_updatePressureModel(&cmd);

    first_step_rpm = cmd.ACTUAL_MOTOR_RPM.value;
    first_step_real_pressure = cmd.REAL_PRESSURE_BAR.value;

    ASSERT_TRUE(cmd.ACTIVE.value, "PressureModel FB should become active when enabled");
    ASSERT_TRUE(first_step_rpm > 0.0, "First enabled step should accelerate the motor");

    cmd.TIME_S.value = 0.001;
    __mcl_cmd_updatePressureModel(&cmd);

    ASSERT_TRUE(cmd.ACTUAL_MOTOR_RPM.value > first_step_rpm,
                "Second enabled step should continue from prior motor state");
    ASSERT_TRUE(cmd.REAL_PRESSURE_BAR.value >= first_step_real_pressure,
                "Second enabled step should not restart the pressure state");

    cmd.ENABLE.value = false;
    __mcl_cmd_updatePressureModel(&cmd);

    ASSERT_TRUE(!cmd.ACTIVE.value, "Disabling PressureModel FB should clear ACTIVE");
    ASSERT_NEAR(cmd.REAL_PRESSURE_BAR.value, 0.0, TOLERANCE,
                "Disabling PressureModel FB should reset real pressure output");
    ASSERT_NEAR(cmd.MEASURED_PRESSURE_BAR.value, 0.0, TOLERANCE,
                "Disabling PressureModel FB should reset measured pressure output");
    ASSERT_NEAR(cmd.ACTUAL_MOTOR_RPM.value, 0.0, TOLERANCE,
                "Disabling PressureModel FB should reset actual motor rpm output");

    cmd.ENABLE.value = true;
    cmd.MOTOR_RPM.value = 1000.0;
    cmd.TIME_S.value = 0.000;
    __mcl_cmd_updatePressureModel(&cmd);

    ASSERT_NEAR(cmd.ACTUAL_MOTOR_RPM.value, first_step_rpm, 1e-6,
                "Re-enable after reset should replay the same first-step motor rpm");
    ASSERT_NEAR(cmd.REAL_PRESSURE_BAR.value, first_step_real_pressure, 1e-6,
                "Re-enable after reset should replay the same first-step real pressure");
}

static void test_pressure_model_fb_exposes_first_order_inputs(void) {
    HYD_PRESSUREMODEL cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.MODEL_TYPE.value = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    cmd.K_NUM.value = 0.25;
    cmd.TTAU.value = 0.2;
    cmd.DELAYTIME.value = 0.01;

    ASSERT_TRUE(cmd.MODEL_TYPE.value == PRESSURE_MODEL_TYPE_FIRST_ORDER,
                "PressureModel FB should expose MODEL_TYPE");
    ASSERT_NEAR(cmd.K_NUM.value, 0.25, TOLERANCE,
                "PressureModel FB should expose K_NUM");
    ASSERT_NEAR(cmd.TTAU.value, 0.2, TOLERANCE,
                "PressureModel FB should expose TTAU");
    ASSERT_NEAR(cmd.DELAYTIME.value, 0.01, TOLERANCE,
                "PressureModel FB should expose DELAYTIME");
}

static void test_pressure_model_fb_negative_speed_depressurizes_without_reset(void) {
    HYD_PRESSUREMODEL reverse_cmd;
    HYD_PRESSUREMODEL leak_cmd;
    double reverse_charged_pressure;
    double reverse_pressure;
    double leak_charged_pressure;
    double leak_pressure;
    int i;

    memset(&reverse_cmd, 0, sizeof(reverse_cmd));
    reverse_cmd.ENABLE.value = false;
    reverse_cmd.TIME_S.value = 0.0;
    __mcl_cmd_updatePressureModel(&reverse_cmd);

    for (i = 0; i < 12000; ++i) {
        reverse_cmd.ENABLE.value = true;
        reverse_cmd.MOTOR_RPM.value = 40.0;
        reverse_cmd.TIME_S.value = 0.001 * (double)i;
        __mcl_cmd_updatePressureModel(&reverse_cmd);
    }

    reverse_charged_pressure = reverse_cmd.REAL_PRESSURE_BAR.value;
    ASSERT_TRUE(reverse_charged_pressure > 0.0,
                "Positive-speed pressure build should charge the FB before reverse command");

    reverse_cmd.MOTOR_RPM.value = -40.0;
    for (i = 12000; i < 12100; ++i) {
        reverse_cmd.TIME_S.value = 0.001 * (double)i;
        __mcl_cmd_updatePressureModel(&reverse_cmd);
    }
    reverse_pressure = reverse_cmd.REAL_PRESSURE_BAR.value;

    ASSERT_TRUE(reverse_cmd.ACTIVE.value,
                "Negative-speed depressurization should keep the PressureModel FB active");
    ASSERT_TRUE(reverse_pressure < reverse_charged_pressure,
                "Negative-speed command should reduce the real pressure without a reset");
    ASSERT_TRUE(reverse_pressure > 0.0,
                "Negative-speed command should not reset the FB pressure state to zero");

    memset(&leak_cmd, 0, sizeof(leak_cmd));
    leak_cmd.ENABLE.value = false;
    leak_cmd.TIME_S.value = 0.0;
    __mcl_cmd_updatePressureModel(&leak_cmd);

    for (i = 0; i < 12000; ++i) {
        leak_cmd.ENABLE.value = true;
        leak_cmd.MOTOR_RPM.value = 40.0;
        leak_cmd.TIME_S.value = 0.001 * (double)i;
        __mcl_cmd_updatePressureModel(&leak_cmd);
    }

    leak_charged_pressure = leak_cmd.REAL_PRESSURE_BAR.value;
    ASSERT_NEAR(leak_charged_pressure, reverse_charged_pressure, TOLERANCE,
                "Reset FB should reproduce the same charged pressure before comparison");

    leak_cmd.MOTOR_RPM.value = 0.0;
    for (i = 12000; i < 12100; ++i) {
        leak_cmd.TIME_S.value = 0.001 * (double)i;
        __mcl_cmd_updatePressureModel(&leak_cmd);
    }
    leak_pressure = leak_cmd.REAL_PRESSURE_BAR.value;

    ASSERT_TRUE(reverse_pressure < leak_pressure,
                "Negative-speed command should depressurize faster than passive leak at the FB layer");
    ASSERT_TRUE(reverse_pressure >= 0.0,
                "Negative-speed command must not drive real pressure below zero");
}

static void test_pressure_model_fb_first_order_mode_outputs_equal_real_and_measured(void) {
    HYD_PRESSUREMODEL cmd;
    double expected_pressure;
    int i;

    reset_pressure_model_cmd(&cmd);
    cmd.ENABLE.value = false;
    cmd.TIME_S.value = 0.0;
    __mcl_cmd_updatePressureModel(&cmd);

    for (i = 0; i < 300; ++i) {
        cmd.ENABLE.value = true;
        cmd.MODEL_TYPE.value = PRESSURE_MODEL_TYPE_FIRST_ORDER;
        cmd.K_NUM.value = 0.20;
        cmd.TTAU.value = 0.0;
        cmd.DELAYTIME.value = 0.0;
        cmd.MOTOR_RPM.value = 120.0;
        cmd.TIME_S.value = 0.001 * (double)i;
        __mcl_cmd_updatePressureModel(&cmd);
    }

    ASSERT_TRUE(cmd.ACTIVE.value,
                "First-order PressureModel FB should stay active while enabled");
    expected_pressure = 0.20 * cmd.ACTUAL_MOTOR_RPM.value;
    ASSERT_TRUE(cmd.REAL_PRESSURE_BAR.value > 0.0,
                "First-order PressureModel FB should build pressure for positive rpm");
    ASSERT_NEAR(cmd.REAL_PRESSURE_BAR.value, expected_pressure, 1e-4,
                "First-order tau=0 output should equal K_NUM times actual motor rpm");
    ASSERT_NEAR(cmd.REAL_PRESSURE_BAR.value, cmd.MEASURED_PRESSURE_BAR.value, TOLERANCE,
                "First-order PressureModel FB should report equal real and measured pressure");
}

static void test_pressure_model_fb_online_model_switch_preserves_pressure(void) {
    HYD_PRESSUREMODEL cmd;
    double charged_pressure;
    int i;

    reset_pressure_model_cmd(&cmd);
    cmd.ENABLE.value = false;
    cmd.TIME_S.value = 0.0;
    __mcl_cmd_updatePressureModel(&cmd);

    for (i = 0; i < 12000; ++i) {
        cmd.ENABLE.value = true;
        cmd.MODEL_TYPE.value = PRESSURE_MODEL_TYPE_PHYSICAL;
        cmd.MOTOR_RPM.value = 40.0;
        cmd.TIME_S.value = 0.001 * (double)i;
        __mcl_cmd_updatePressureModel(&cmd);
    }
    charged_pressure = cmd.REAL_PRESSURE_BAR.value;

    cmd.MODEL_TYPE.value = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    cmd.K_NUM.value = 0.0;
    cmd.TTAU.value = 0.0;
    cmd.DELAYTIME.value = 0.0;
    cmd.TIME_S.value = 12.000;
    __mcl_cmd_updatePressureModel(&cmd);

    ASSERT_TRUE(cmd.ACTIVE.value,
                "Switching model type online should keep the PressureModel FB active");
    ASSERT_NEAR(cmd.REAL_PRESSURE_BAR.value, charged_pressure, 1e-6,
                "Switching to first-order mode should preserve the current real pressure");
    ASSERT_NEAR(cmd.MEASURED_PRESSURE_BAR.value, charged_pressure, 1e-6,
                "Switching to first-order mode should preserve the current measured pressure");

    cmd.TIME_S.value = 12.001;
    __mcl_cmd_updatePressureModel(&cmd);

    ASSERT_NEAR(cmd.REAL_PRESSURE_BAR.value, 0.0, 1e-6,
                "The scan after switching to zero-gain first-order mode should use first-order dynamics");
    ASSERT_NEAR(cmd.MEASURED_PRESSURE_BAR.value, 0.0, 1e-6,
                "Measured pressure should follow the zero-gain first-order output after the switch");
}

static void test_pressure_model_fb_hot_updates_first_order_parameters(void) {
    HYD_PRESSUREMODEL cmd;
    double low_gain_pressure;
    double expected_low_gain_pressure;
    double expected_high_gain_pressure;
    int i;

    reset_pressure_model_cmd(&cmd);
    cmd.ENABLE.value = false;
    cmd.TIME_S.value = 0.0;
    __mcl_cmd_updatePressureModel(&cmd);

    for (i = 0; i < 500; ++i) {
        cmd.ENABLE.value = true;
        cmd.MODEL_TYPE.value = PRESSURE_MODEL_TYPE_FIRST_ORDER;
        cmd.K_NUM.value = 0.05;
        cmd.TTAU.value = 0.0;
        cmd.DELAYTIME.value = 0.0;
        cmd.MOTOR_RPM.value = 200.0;
        cmd.TIME_S.value = 0.001 * (double)i;
        __mcl_cmd_updatePressureModel(&cmd);
    }
    low_gain_pressure = cmd.REAL_PRESSURE_BAR.value;
    expected_low_gain_pressure = 0.05 * cmd.ACTUAL_MOTOR_RPM.value;
    ASSERT_NEAR(low_gain_pressure, expected_low_gain_pressure, 1e-4,
                "Low-gain first-order pressure should match K_NUM times actual motor rpm");

    for (i = 500; i < 503; ++i) {
        cmd.K_NUM.value = 0.10;
        cmd.TIME_S.value = 0.001 * (double)i;
        __mcl_cmd_updatePressureModel(&cmd);
    }
    expected_high_gain_pressure = 0.10 * cmd.ACTUAL_MOTOR_RPM.value;

    ASSERT_TRUE(cmd.REAL_PRESSURE_BAR.value > low_gain_pressure + 5.0,
                "Increasing K_NUM online should raise the first-order pressure output");
    ASSERT_NEAR(cmd.REAL_PRESSURE_BAR.value, expected_high_gain_pressure, 1e-4,
                "Hot-updated first-order pressure should match the new K_NUM times actual motor rpm");
}

int main(void) {
    printf("=== HydraulicSimFB PLC Adapter Tests ===\n\n");

    test_framework_init_resets_allocator_and_env();
    test_create_two_axes_assigns_unique_ids_and_keeps_config();
    test_create_rejects_invalid_axis_type();
    test_move_only_target_axis_under_single_pump_owner();
    test_read_uses_axisid_not_current_pump_owner();
    test_publish_steps_shared_env_once_per_scan();
    test_fault_injection_is_isolated_per_axis();
    test_invalid_axisid_is_safe_and_does_not_pollute_other_axes();
    test_pressure_model_fb_persists_state_and_resets_on_disable();
    test_pressure_model_fb_exposes_first_order_inputs();
    test_pressure_model_fb_negative_speed_depressurizes_without_reset();
    test_pressure_model_fb_first_order_mode_outputs_equal_real_and_measured();
    test_pressure_model_fb_online_model_switch_preserves_pressure();
    test_pressure_model_fb_hot_updates_first_order_parameters();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
