#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "hydro_sim_fb.h"
#include "motion_control.h"
#include "motion_interface.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)

#define MAX_SIM_STEPS 5000
#define GUARD_SCANS_AFTER_SETTLE 20

#define SCAN_PERIOD_S 0.001f
#define TARGET_POSITION_MM 80.0f
#define TARGET_VELOCITY_MM_S 30.0f
#define TARGET_END_VELOCITY_MM_S 0.0f
#define TARGET_ACCEL_MM_S2 1000.0f
#define TARGET_DECEL_MM_S2 1000.0f

#define SIM_AXIS_TYPE_INJECT 1U
#define SIM_AXIS_MAX_VEL_MM_S 500.0f
#define SIM_AXIS_MAX_ACC_MM_S2 2000.0f
#define SIM_AXIS_MAX_DEC_MM_S2 2000.0f

#define PUMP_DISPLACEMENT_ML_REV 28.0f
#define PUMP_VOLUMETRIC_EFFICIENCY 0.95f
#define PUMP_MAX_SPEED_RPM 1800.0f
#define CYLINDER_AREA_EXTEND_MM2 8000.0f
#define CYLINDER_AREA_RETRACT_MM2 4500.0f
#define CYLINDER_STROKE_MM 300.0f
#define INJECT_BASE_FRICTION_N 800.0f
#define INJECT_MELT_STIFFNESS_N_MM 150.0f

#define POSITION_TOLERANCE_MM 0.1f
#define VELOCITY_MATCH_TOLERANCE_MM_S 0.02f
#define PRESSURE_MATCH_TOLERANCE_BAR 0.05f
#define CONTINUITY_MARGIN_MM_S 0.05f
#define ZERO_VELOCITY_TOLERANCE_MM_S 0.02f
#define CONTINUITY_JUMP_LIMIT_MM_S ((TARGET_DECEL_MM_S2 * SCAN_PERIOD_S * 2.0f) + CONTINUITY_MARGIN_MM_S)

enum {
    RUN_TIMEOUT = -1,
    RUN_ERROR = -2,
    RUN_ABORTED = -3
};

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { \
        tests_passed++; \
    } else { \
        tests_failed++; \
        printf("  FAIL [line %d]: %s\n", __LINE__, msg); \
    } \
} while (0)

#define ASSERT_NEAR(actual, expected, tol, msg) do { \
    tests_run++; \
    if (fabs((double)((actual) - (expected))) <= (double)(tol)) { \
        tests_passed++; \
    } else { \
        tests_failed++; \
        printf("  FAIL [line %d]: %s (got %.6f, expected %.6f, tol %.6f)\n", \
               __LINE__, msg, (double)(actual), (double)(expected), (double)(tol)); \
    } \
} while (0)

typedef struct {
    HYD_REAL pump_speed_rpm[MAX_SIM_STEPS];
    HYD_REAL pre_pos_mm[MAX_SIM_STEPS];
    HYD_REAL pre_vel_mm_s[MAX_SIM_STEPS];
    HYD_REAL pre_pressure_bar[MAX_SIM_STEPS];
    HYD_REAL post_pos_mm[MAX_SIM_STEPS];
    HYD_REAL post_vel_mm_s[MAX_SIM_STEPS];
    HYD_REAL post_pressure_bar[MAX_SIM_STEPS];
} MotionTrace;

static HYD_REAL expected_velocity_to_flow_gain(HYD_REAL area_mm2)
{
    return area_mm2 * 6.0e-5f;
}

static HYD_REAL area_for_direction(int direction)
{
    return (direction < 0) ? CYLINDER_AREA_RETRACT_MM2 : CYLINDER_AREA_EXTEND_MM2;
}

static HYD_REAL expected_velocity_from_pump_speed(HYD_REAL pump_speed_rpm, int direction)
{
    HYD_REAL area_mm2 = area_for_direction(direction);
    HYD_REAL flow_lpm;

    if (pump_speed_rpm <= 0.0f || direction == 0 || area_mm2 <= 0.0f) {
        return 0.0f;
    }

    flow_lpm = (PUMP_DISPLACEMENT_ML_REV * pump_speed_rpm / 1000.0f) *
               PUMP_VOLUMETRIC_EFFICIENCY;
    return ((direction < 0) ? -1.0f : 1.0f) *
           (flow_lpm / expected_velocity_to_flow_gain(area_mm2));
}

static HYD_REAL expected_flow_from_velocity(HYD_REAL velocity_mm_s)
{
    HYD_REAL area_mm2 = (velocity_mm_s < 0.0f) ? CYLINDER_AREA_RETRACT_MM2 : CYLINDER_AREA_EXTEND_MM2;
    return fabsf(velocity_mm_s) * expected_velocity_to_flow_gain(area_mm2);
}

static HYD_REAL expected_pressure_bar(HYD_REAL position_mm)
{
    return ((INJECT_BASE_FRICTION_N + (INJECT_MELT_STIFFNESS_N_MM * position_mm)) /
            CYLINDER_AREA_EXTEND_MM2) * 10.0f;
}

static int create_external_sim_axis(void)
{
    HYD_CREATESIMAXIS create;

    memset(&create, 0, sizeof(create));
    IEC_VAL(create.EN) = true;
    IEC_VAL(create.AXISTYPE) = SIM_AXIS_TYPE_INJECT;
    IEC_VAL(create.MAXVEL) = SIM_AXIS_MAX_VEL_MM_S;
    IEC_VAL(create.MAXACC) = SIM_AXIS_MAX_ACC_MM_S2;
    IEC_VAL(create.MAXDEC) = SIM_AXIS_MAX_DEC_MM_S2;
    __mcl_cmd_createSimAxis(&create);

    return (int)IEC_VAL(create.AXISID);
}

static int create_motion_axis(void)
{
    HYD_CREATEMOTION create;

    memset(&create, 0, sizeof(create));
    IEC_VAL(create.EN) = true;
    IEC_VAL(create.USE_RECIPE) = false;
    IEC_VAL(create.FLOW_TO_PUMPSPEED) = 20.0f;
    IEC_VAL(create.PUMPSPEED_LIMIT) = PUMP_MAX_SPEED_RPM;
    IEC_VAL(create.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&create);

    return (int)IEC_VAL(create.AXISID);
}

static bool write_parameter_once(int axis_id, int parameter_number, HYD_REAL value)
{
    HYD_WRITEPARAMETER write_cmd;

    memset(&write_cmd, 0, sizeof(write_cmd));
    IEC_VAL(write_cmd.EN) = true;
    IEC_VAL(write_cmd.AXISID) = axis_id;
    IEC_VAL(write_cmd.PARAMETERNUMBER) = parameter_number;
    IEC_VAL(write_cmd.VALUE) = value;
    IEC_VAL(write_cmd.EXECUTE) = true;
    __mcl_cmd_WriteParameter(&write_cmd);

    return IEC_VAL(write_cmd.DONE) == true && IEC_VAL(write_cmd.ERROR) == false;
}

static bool configure_motion_axis_physical_model(int axis_id)
{
    const HYD_REAL velocity_to_flow_gain =
        expected_velocity_to_flow_gain(CYLINDER_AREA_EXTEND_MM2);

    return write_parameter_once(axis_id, HYD_PARAM_VELOCITY_TO_FLOW_GAIN, velocity_to_flow_gain) &&
           write_parameter_once(axis_id, HYD_PARAM_PUMP_DISPLACEMENT, PUMP_DISPLACEMENT_ML_REV) &&
           write_parameter_once(axis_id, HYD_PARAM_PUMP_VOLUMETRIC_EFF, PUMP_VOLUMETRIC_EFFICIENCY) &&
           write_parameter_once(axis_id, HYD_PARAM_PUMP_MAX_SPEED, PUMP_MAX_SPEED_RPM) &&
           write_parameter_once(axis_id, HYD_PARAM_CYLINDER_AREA_EXTEND, CYLINDER_AREA_EXTEND_MM2) &&
           write_parameter_once(axis_id, HYD_PARAM_CYLINDER_AREA_RETRACT, CYLINDER_AREA_RETRACT_MM2) &&
           write_parameter_once(axis_id, HYD_PARAM_CYLINDER_STROKE, CYLINDER_STROKE_MM);
}

static bool get_pump_request(HYD_REAL* pump_speed_rpm)
{
    HYD_GETPUMPREQUEST request;

    memset(&request, 0, sizeof(request));
    IEC_VAL(request.EN) = true;
    IEC_VAL(request.ENABLE) = true;
    IEC_VAL(request.ALLOW_NEGATIVE) = true;
    __mcl_cmd_GetPumpRequest(&request);

    if (IEC_VAL(request.ERROR)) {
        return false;
    }

    if (pump_speed_rpm != NULL) {
        *pump_speed_rpm = IEC_VAL(request.PUMPSPEED);
    }
    return true;
}

static bool move_sim_axis_once(int axis_id, HYD_REAL pump_speed_rpm)
{
    HYD_MOVESIMAXIS move;
    int direction = 0;
    HYD_REAL command_rpm = fabsf(pump_speed_rpm);

    if (pump_speed_rpm > 0.0f) {
        direction = 1;
    } else if (pump_speed_rpm < 0.0f) {
        direction = -1;
    }

    memset(&move, 0, sizeof(move));
    IEC_VAL(move.EN) = true;
    IEC_VAL(move.ENABLE) = (command_rpm > 0.0f);
    IEC_VAL(move.AXISID) = axis_id;
    IEC_VAL(move.CMD_RPM) = command_rpm;
    IEC_VAL(move.DIRECTION) = direction;
    __mcl_cmd_moveSimAxis(&move);

    return true;
}

static bool read_sim_axis_once(int axis_id,
                               HYD_REAL* position_mm,
                               HYD_REAL* velocity_mm_s,
                               HYD_REAL* pressure_bar)
{
    HYD_READSIMAXIS read;

    memset(&read, 0, sizeof(read));
    IEC_VAL(read.EN) = true;
    IEC_VAL(read.ENABLE) = true;
    IEC_VAL(read.AXISID) = axis_id;
    __mcl_cmd_readSimAxis(&read);

    if (position_mm != NULL) {
        *position_mm = IEC_VAL(read.POS_MM);
    }
    if (velocity_mm_s != NULL) {
        *velocity_mm_s = IEC_VAL(read.VEL_MM_S);
    }
    if (pressure_bar != NULL) {
        *pressure_bar = IEC_VAL(read.PRESSURE_BAR);
    }
    return true;
}

static bool set_axis_feedback_once(int axis_id,
                                   HYD_REAL position_mm,
                                   HYD_REAL velocity_mm_s,
                                   HYD_REAL pressure_bar,
                                   HYD_REAL timestamp_s)
{
    HYD_SETAXISFEEDBACK feedback;

    memset(&feedback, 0, sizeof(feedback));
    IEC_VAL(feedback.EN) = true;
    IEC_VAL(feedback.ENABLE) = true;
    IEC_VAL(feedback.AXISID) = axis_id;
    IEC_VAL(feedback.ACT_POSITION) = position_mm;
    IEC_VAL(feedback.ACT_VELOCITY) = velocity_mm_s;
    IEC_VAL(feedback.ACT_FLOW) = expected_flow_from_velocity(velocity_mm_s);
    IEC_VAL(feedback.ACT_PRESSURE) = pressure_bar;
    IEC_VAL(feedback.TIMESTAMP) = timestamp_s;
    __mcl_cmd_SetAxisFeedback(&feedback);

    return IEC_VAL(feedback.DONE) == true && IEC_VAL(feedback.ERROR) == false;
}

static void init_movecontinuousabsolute(HYD_MOVECONTINUOUSABSOLUTE* move,
                                        int axis_id)
{
    memset(move, 0, sizeof(*move));
    IEC_VAL(move->EN) = true;
    IEC_VAL(move->AXISID) = axis_id;
    IEC_VAL(move->POSITION) = TARGET_POSITION_MM;
    IEC_VAL(move->VELOCITY) = TARGET_VELOCITY_MM_S;
    IEC_VAL(move->ENDVELOCITY) = TARGET_END_VELOCITY_MM_S;
    IEC_VAL(move->ENDVELOCITYDIRECTION) = HYD_DIRECTION_POSITIVE;
    IEC_VAL(move->ACCELERATION) = TARGET_ACCEL_MM_S2;
    IEC_VAL(move->DECELERATION) = TARGET_DECEL_MM_S2;
    IEC_VAL(move->DIRECTION) = HYD_DIRECTION_POSITIVE;
    IEC_VAL(move->ADAPTENDVELTOAVOIDOVERSHOOT) = false;
}

static void execute_movecontinuousabsolute_scan(HYD_MOVECONTINUOUSABSOLUTE* move,
                                                bool rising_edge)
{
    IEC_VAL(move->EXECUTE) = true;
    if (rising_edge) {
        move->EXECUTE0.value = false;
    }
    __mcl_cmd_MoveContinuousAbsolute(move);
}

static void test_movecontinuousabsolute_external_sim_axis_tracks_velocity_and_position(void)
{
    HYD_MOVECONTINUOUSABSOLUTE move;
    HYD_MotionControlFB* core;
    MotionTrace trace;
    int sim_axis_id;
    int motion_axis_id;
    int position_reached_step = RUN_TIMEOUT;
    int in_end_velocity_step = RUN_TIMEOUT;
    int settle_guard_remaining = -1;
    HYD_REAL max_observed_velocity = 0.0f;
    HYD_REAL max_velocity_jump = 0.0f;
    HYD_REAL timestamp_s = 0.0f;
    HYD_REAL previous_post_pos = 0.0f;
    HYD_REAL previous_post_vel = 0.0f;
    HYD_REAL final_pump_speed_rpm = 0.0f;
    bool saw_cruise_velocity = false;
    bool saw_positive_pressure = false;
    bool finished = false;

    memset(&trace, 0, sizeof(trace));

    __HydMotion_framework_Init();
    __HydSimulator_framework_Init();

    sim_axis_id = create_external_sim_axis();
    ASSERT_TRUE(sim_axis_id >= 0, "External simulator axis should be created");
    if (sim_axis_id < 0) {
        return;
    }

    motion_axis_id = create_motion_axis();
    ASSERT_TRUE(motion_axis_id >= 0, "Motion control axis should be created");
    if (motion_axis_id < 0) {
        return;
    }

    ASSERT_TRUE(configure_motion_axis_physical_model(motion_axis_id),
                "Motion axis should accept the external pump/cylinder physical model");
    core = __MK_GetPublic_MotionControlFB(motion_axis_id);
    ASSERT_TRUE(core != NULL, "Public motion FB should be available");
    if (core == NULL) {
        return;
    }

    ASSERT_NEAR(core->pumpConfig.displacementMlRev, PUMP_DISPLACEMENT_ML_REV, 1e-6f,
                "Pump displacement should match the configured external simulator model");
    ASSERT_NEAR(core->pumpConfig.volumetricEfficiency, PUMP_VOLUMETRIC_EFFICIENCY, 1e-6f,
                "Pump volumetric efficiency should match the configured external simulator model");
    ASSERT_NEAR(core->pumpConfig.maxSpeedRpm, PUMP_MAX_SPEED_RPM, 1e-6f,
                "Pump max speed should be populated so pumpConfig remains usable");
    ASSERT_NEAR(core->cylinderConfig.areaExtendMm2, CYLINDER_AREA_EXTEND_MM2, 1e-6f,
                "Cylinder extend area should match the external simulator model");
    ASSERT_NEAR(core->cylinderConfig.areaRetractMm2, CYLINDER_AREA_RETRACT_MM2, 1e-6f,
                "Cylinder retract area should match the external simulator model");
    ASSERT_NEAR(core->cylinderConfig.strokeMm, CYLINDER_STROKE_MM, 1e-6f,
                "Cylinder stroke should match the external simulator model");

    ASSERT_TRUE(read_sim_axis_once(sim_axis_id,
                                   &trace.pre_pos_mm[0],
                                   &trace.pre_vel_mm_s[0],
                                   &trace.pre_pressure_bar[0]),
                "Initial external simulator snapshot should be readable");
    ASSERT_TRUE(set_axis_feedback_once(motion_axis_id,
                                       trace.pre_pos_mm[0],
                                       trace.pre_vel_mm_s[0],
                                       trace.pre_pressure_bar[0],
                                       timestamp_s),
                "Initial external simulator snapshot should seed the motion axis");

    init_movecontinuousabsolute(&move, motion_axis_id);

    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        HYD_REAL expected_post_velocity;
        HYD_REAL pump_speed_rpm;
        int pump_direction = 0;

        execute_movecontinuousabsolute_scan(&move, step == 0);

        ASSERT_TRUE(get_pump_request(&pump_speed_rpm),
                    "GetPumpRequest should succeed for the active axis");
        trace.pump_speed_rpm[step] = pump_speed_rpm;
        if (pump_speed_rpm > 0.0f) {
            pump_direction = 1;
        } else if (pump_speed_rpm < 0.0f) {
            pump_direction = -1;
        }

        ASSERT_TRUE(move_sim_axis_once(sim_axis_id, pump_speed_rpm),
                    "MoveSimAxis should accept the commanded pump speed");
        ASSERT_TRUE(read_sim_axis_once(sim_axis_id,
                                       &trace.pre_pos_mm[step],
                                       &trace.pre_vel_mm_s[step],
                                       &trace.pre_pressure_bar[step]),
                    "Pre-publish external simulator snapshot should be readable");
        ASSERT_TRUE(set_axis_feedback_once(motion_axis_id,
                                           trace.pre_pos_mm[step],
                                           trace.pre_vel_mm_s[step],
                                           trace.pre_pressure_bar[step],
                                           timestamp_s),
                    "SetAxisFeedback should accept the external simulator snapshot");

        __HydMotion_framework_Publish();
        __HydSimulator_framework_Publish();

        ASSERT_TRUE(read_sim_axis_once(sim_axis_id,
                                       &trace.post_pos_mm[step],
                                       &trace.post_vel_mm_s[step],
                                       &trace.post_pressure_bar[step]),
                    "Post-publish external simulator snapshot should be readable");

        ASSERT_TRUE(IEC_VAL(move.ERROR) == false,
                    "MoveContinuousAbsolute should not raise ERROR in the external simulator scenario");
        ASSERT_TRUE(IEC_VAL(move.COMMANDABORTED) == false,
                    "MoveContinuousAbsolute should not be aborted in the external simulator scenario");
        ASSERT_TRUE(core->DIAGNOSTIC.code == HYD_DIAG_CODE_NONE,
                    "The motion FB should not publish diagnostics in the nominal external simulator scenario");

        expected_post_velocity = expected_velocity_from_pump_speed(fabsf(pump_speed_rpm), pump_direction);
        ASSERT_NEAR(trace.post_vel_mm_s[step], expected_post_velocity, VELOCITY_MATCH_TOLERANCE_MM_S,
                    "Post-publish simulator velocity should match the same-scan theoretical velocity");

        if (step > 0) {
            HYD_REAL velocity_jump = fabsf(trace.post_vel_mm_s[step] - previous_post_vel);
            ASSERT_NEAR(trace.pre_pos_mm[step], trace.post_pos_mm[step - 1], 1e-6f,
                        "Shared simulator pre-publish position should equal the previous post-publish snapshot");
            ASSERT_NEAR(trace.pre_vel_mm_s[step], trace.post_vel_mm_s[step - 1], 1e-6f,
                        "Shared simulator pre-publish velocity should equal the previous post-publish snapshot");
            ASSERT_TRUE(trace.post_pos_mm[step] + 1e-6f >= previous_post_pos,
                        "External simulator position should stay monotonic nondecreasing");
            ASSERT_TRUE(velocity_jump <= CONTINUITY_JUMP_LIMIT_MM_S,
                        "External simulator velocity should evolve continuously scan to scan");
            if (velocity_jump > max_velocity_jump) {
                max_velocity_jump = velocity_jump;
            }
        } else {
            ASSERT_NEAR(trace.post_pos_mm[step], 0.0f, 1e-6f,
                        "The first post-publish snapshot should remain at the origin during startup");
            ASSERT_NEAR(trace.post_vel_mm_s[step], 0.0f, 1e-6f,
                        "The first post-publish snapshot should remain still during startup");
        }

        if (pump_direction > 0 && fabsf(pump_speed_rpm) > 0.0f) {
            ASSERT_NEAR(trace.post_pressure_bar[step],
                        expected_pressure_bar(trace.post_pos_mm[step]),
                        PRESSURE_MATCH_TOLERANCE_BAR,
                        "Pressurized external simulator pressure should match the inject-axis load model");
            if (trace.post_pressure_bar[step] > 0.0f) {
                saw_positive_pressure = true;
            }
        }

        if (trace.post_vel_mm_s[step] > max_observed_velocity) {
            max_observed_velocity = trace.post_vel_mm_s[step];
        }
        if (fabsf(trace.post_vel_mm_s[step] - TARGET_VELOCITY_MM_S) <= 0.1f) {
            saw_cruise_velocity = true;
        }

        if (position_reached_step < 0 && IEC_VAL(move.POSITIONREACHED)) {
            position_reached_step = step + 1;
        }
        if (in_end_velocity_step < 0 && IEC_VAL(move.INENDVELOCITY)) {
            in_end_velocity_step = step + 1;
            settle_guard_remaining = GUARD_SCANS_AFTER_SETTLE;
        }

        if (settle_guard_remaining >= 0) {
            settle_guard_remaining--;
            if (settle_guard_remaining == 0) {
                finished = true;
                previous_post_pos = trace.post_pos_mm[step];
                previous_post_vel = trace.post_vel_mm_s[step];
                final_pump_speed_rpm = trace.pump_speed_rpm[step];
                break;
            }
        }

        previous_post_pos = trace.post_pos_mm[step];
        previous_post_vel = trace.post_vel_mm_s[step];
        timestamp_s += SCAN_PERIOD_S;
    }

    ASSERT_TRUE(finished,
                "The external simulator scenario should settle within the allowed scan budget");
    ASSERT_TRUE(position_reached_step > 0,
                "MoveContinuousAbsolute should reach POSITIONREACHED against the external simulator axis");
    ASSERT_TRUE(in_end_velocity_step > 0,
                "MoveContinuousAbsolute should reach INENDVELOCITY against the external simulator axis");
    if (position_reached_step > 0 && in_end_velocity_step > 0) {
        ASSERT_TRUE(position_reached_step == in_end_velocity_step,
                    "With zero end velocity and no pressure limiting, POSITIONREACHED and INENDVELOCITY should latch together");
    }

    ASSERT_TRUE(saw_cruise_velocity,
                "The external simulator velocity trace should reach the commanded 30 mm/s cruise plateau");
    ASSERT_TRUE(max_observed_velocity >= TARGET_VELOCITY_MM_S - 0.1f,
                "The external simulator should reach the commanded target velocity");
    ASSERT_TRUE(max_observed_velocity <= TARGET_VELOCITY_MM_S + VELOCITY_MATCH_TOLERANCE_MM_S,
                "The external simulator should not exceed the commanded target velocity in the nominal case");
    ASSERT_TRUE(saw_positive_pressure,
                "The inject-axis load model should produce positive pressure while the pump is driving the axis");
    ASSERT_NEAR(previous_post_pos, TARGET_POSITION_MM, POSITION_TOLERANCE_MM,
                "The external simulator position should settle at the target position");
    ASSERT_NEAR(previous_post_vel, 0.0f, ZERO_VELOCITY_TOLERANCE_MM_S,
                "The external simulator velocity should settle to zero for endvel=0");
    ASSERT_NEAR(final_pump_speed_rpm,
                0.0f, 1e-6f,
                "The pump request should drop to zero within the post-settle guard window");
    ASSERT_TRUE(max_velocity_jump > 0.0f,
                "The trace should include nonzero velocity changes while still remaining continuous");
}

int main(void)
{
    printf("=== MoveContinuousAbsolute external simulator integration ===\n\n");

    test_movecontinuousabsolute_external_sim_axis_tracks_velocity_and_position();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_failed == 0) ? 0 : 1;
}
