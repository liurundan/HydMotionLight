#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "motion_control.h"
#include "motion_interface.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)
#define MAX_SIM_STEPS 20000
#define SAMPLE_COUNT 6

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { \
        tests_passed++; \
    } else { \
        printf("  FAIL: %s\n", msg); \
    } \
} while (0)

static int create_sim_axis_with_gain(HYD_REAL flow_to_pump_speed_gain) {
    HYD_CREATEMOTION cm;

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = flow_to_pump_speed_gain;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = true;
    __mcl_cmd_CreateMotion(&cm);
    return (int)IEC_VAL(cm.AXISID);
}

static void hold_movevelocity_scan(HYD_MOVEVELOCITY* mv) {
    IEC_VAL(mv->EXECUTE) = true;
    mv->EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(mv);
}

static void hold_stop_scan(HYD_STOP* stop) {
    IEC_VAL(stop->EXECUTE) = true;
    stop->EXECUTE0.value = true;
    __mcl_cmd_Stop(stop);
}

static int drive_stop_until_done(HYD_MOVEVELOCITY* mv,
                                 HYD_STOP* stop,
                                 int max_steps) {
    for (int step = 0; step < max_steps; ++step) {
        __HydMotion_framework_Publish();
        hold_movevelocity_scan(mv);
        hold_stop_scan(stop);
        if (IEC_VAL(stop->DONE)) {
            return step + 1;
        }
        if (IEC_VAL(stop->ERROR)) {
            return -1;
        }
    }

    return 0;
}

static void reset_movevelocity_fb(HYD_MOVEVELOCITY* mv) {
    IEC_VAL(mv->EXECUTE) = false;
    mv->EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(mv);
}

static void assert_monotonic_nonnegative_samples(const HYD_REAL* samples,
                                                 int sample_count,
                                                 const char* message) {
    for (int step = 1; step < sample_count; ++step) {
        ASSERT_TRUE(samples[step] + 1e-3f >= samples[step - 1], message);
    }
}

static void test_reverse_restart_after_stop_starts_from_zero(void) {
    HYD_MOVEVELOCITY mv;
    HYD_STOP stop;
    HYD_MotionControlFB* fb;
    HYD_REAL pump_speed_samples[SAMPLE_COUNT];
    HYD_REAL planned_velocity_samples[SAMPLE_COUNT];
    int axis_id;
    int stop_done_step;

    printf("--- Test: MoveVelocity stop then reverse restart should ramp from zero ---\n");

    __HydMotion_framework_Init();
    axis_id = create_sim_axis_with_gain(100.0f);
    ASSERT_TRUE(axis_id >= 0, "CreateMotion should allocate a simulation axis");
    if (axis_id < 0) {
        return;
    }

    fb = __MK_GetPublic_MotionControlFB(axis_id);
    ASSERT_TRUE(fb != NULL, "Allocated simulation axis should expose its FB");
    if (fb == NULL) {
        return;
    }

    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.AXISID) = axis_id;
    IEC_VAL(mv.VELOCITY) = 5.0f;
    IEC_VAL(mv.ACCELERATION) = 50.0f;
    IEC_VAL(mv.DECELERATION) = 50.0f;
    IEC_VAL(mv.DIRECTION) = 1; /* forward */
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    __mcl_cmd_MoveVelocity(&mv);

    for (int step = 0; step < 140; ++step) {
        __HydMotion_framework_Publish();
        hold_movevelocity_scan(&mv);
    }

    ASSERT_TRUE(fb->PUMP_SPEED > 90.0f,
                "Forward MoveVelocity should reach its steady pump-speed magnitude before Stop");

    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.AXISID) = axis_id;
    IEC_VAL(stop.DECELERATION) = 50.0f;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    __mcl_cmd_Stop(&stop);

    stop_done_step = drive_stop_until_done(&mv, &stop, MAX_SIM_STEPS);
    ASSERT_TRUE(stop_done_step > 1,
                "Stop should take multiple cycles before reporting DONE");
    ASSERT_TRUE(IEC_VAL(stop.DONE) == true,
                "Stop should report DONE before the reverse restart begins");
    ASSERT_TRUE(fabs(fb->PUMP_SPEED) < 0.01f,
                "Pump speed should be zero at Stop.DONE");
    ASSERT_TRUE(fabs(fb->STATE.plannedVelocity) < 0.01f,
                "Planned velocity should be zero at Stop.DONE");
    printf("  At Stop.DONE: planner.lastVel=%.3f planner.lastFlow=%.3f\n",
           (double)fb->_plannerState.lastTargetVelocity,
           (double)fb->_plannerState.lastTargetFlow);

    reset_movevelocity_fb(&mv);

    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.AXISID) = axis_id;
    IEC_VAL(mv.VELOCITY) = 5.0f;
    IEC_VAL(mv.ACCELERATION) = 50.0f;
    IEC_VAL(mv.DECELERATION) = 50.0f;
    IEC_VAL(mv.DIRECTION) = 2; /* reverse */
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    __mcl_cmd_MoveVelocity(&mv);
    printf("  After reverse execRising: planner.lastVel=%.3f planner.lastFlow=%.3f fbState=%d\n",
           (double)fb->_plannerState.lastTargetVelocity,
           (double)fb->_plannerState.lastTargetFlow,
           (int)fb->FB_STATE);

    printf("  Reverse restart samples:");
    for (int step = 0; step < SAMPLE_COUNT; ++step) {
        __HydMotion_framework_Publish();
        hold_movevelocity_scan(&mv);
        pump_speed_samples[step] = fb->PUMP_SPEED;
        planned_velocity_samples[step] = fb->STATE.plannedVelocity;
        printf(" [%d] rpm=%.2f vel=%.3f",
               step,
               (double)pump_speed_samples[step],
               (double)planned_velocity_samples[step]);
    }
    printf("\n");

    ASSERT_TRUE(planned_velocity_samples[0] <= 0.0f,
                "Reverse restart should command negative mechanism velocity on the first running cycle");
    ASSERT_TRUE(fabs(planned_velocity_samples[0]) < 0.5f,
                "Reverse restart should begin near zero velocity after a completed Stop");
    ASSERT_TRUE(pump_speed_samples[0] < 10.0f,
                "Reverse restart should not jump to the previous steady pump-speed magnitude");

    assert_monotonic_nonnegative_samples(
        pump_speed_samples,
        SAMPLE_COUNT,
        "Reverse restart pump-speed magnitude should ramp upward monotonically from zero");
}

static void test_continuousupdate_direction_flip_restarts_from_zero(void) {
    HYD_MOVEVELOCITY mv;
    HYD_MotionControlFB* fb;
    HYD_REAL pump_speed_samples[SAMPLE_COUNT];
    HYD_REAL planned_velocity_samples[SAMPLE_COUNT];
    int axis_id;

    printf("--- Test: MoveVelocity continuous-update direction flip should restart from zero ---\n");

    __HydMotion_framework_Init();
    axis_id = create_sim_axis_with_gain(100.0f);
    ASSERT_TRUE(axis_id >= 0, "CreateMotion should allocate a simulation axis for continuous-update");
    if (axis_id < 0) {
        return;
    }

    fb = __MK_GetPublic_MotionControlFB(axis_id);
    ASSERT_TRUE(fb != NULL, "Continuous-update direction-flip test should resolve its FB");
    if (fb == NULL) {
        return;
    }

    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.AXISID) = axis_id;
    IEC_VAL(mv.CONTINUOUSUPDATE) = true;
    IEC_VAL(mv.VELOCITY) = 5.0f;
    IEC_VAL(mv.ACCELERATION) = 50.0f;
    IEC_VAL(mv.DECELERATION) = 50.0f;
    IEC_VAL(mv.DIRECTION) = 1; /* forward */
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    __mcl_cmd_MoveVelocity(&mv);

    for (int step = 0; step < 140; ++step) {
        __HydMotion_framework_Publish();
        hold_movevelocity_scan(&mv);
    }

    ASSERT_TRUE(fb->PUMP_SPEED > 90.0f,
                "Continuous-update test should build the forward steady-state pump speed before flipping direction");

    IEC_VAL(mv.DIRECTION) = 2; /* reverse while still executing */
    __mcl_cmd_MoveVelocity(&mv);

    printf("  After active direction flip request: planner.lastVel=%.3f planner.lastFlow=%.3f fbState=%d\n",
           (double)fb->_plannerState.lastTargetVelocity,
           (double)fb->_plannerState.lastTargetFlow,
           (int)fb->FB_STATE);

    printf("  Continuous-update reverse samples:");
    for (int step = 0; step < SAMPLE_COUNT; ++step) {
        __HydMotion_framework_Publish();
        hold_movevelocity_scan(&mv);
        pump_speed_samples[step] = fb->PUMP_SPEED;
        planned_velocity_samples[step] = fb->STATE.plannedVelocity;
        printf(" [%d] rpm=%.2f vel=%.3f",
               step,
               (double)pump_speed_samples[step],
               (double)planned_velocity_samples[step]);
    }
    printf("\n");

    ASSERT_TRUE(planned_velocity_samples[0] <= 0.0f,
                "Continuous-update direction flip should command negative mechanism velocity on the first running cycle");
    ASSERT_TRUE(fabs(planned_velocity_samples[0]) < 0.5f,
                "Continuous-update direction flip should restart near zero velocity");
    ASSERT_TRUE(pump_speed_samples[0] < 10.0f,
                "Continuous-update direction flip should not jump to the previous steady pump-speed magnitude");
    assert_monotonic_nonnegative_samples(
        pump_speed_samples,
        SAMPLE_COUNT,
        "Continuous-update direction flip pump-speed magnitude should ramp upward monotonically from zero");
}

static void test_pressure_limit_derates_output_and_faults(void) {
    HYD_MOVEVELOCITY mv;
    HYD_MotionControlFB* fb;
    HYD_REAL unrestricted_pump_speed;
    HYD_REAL limited_pump_speed;
    HYD_REAL axis_limited_pump_speed;
    int axis_id;

    printf("--- Test: MoveVelocity pressure limit should derate then fault ---\n");

    __HydMotion_framework_Init();
    axis_id = create_sim_axis_with_gain(100.0f);
    ASSERT_TRUE(axis_id >= 0, "CreateMotion should allocate a pressure-limit simulation axis");
    if (axis_id < 0) {
        return;
    }

    fb = __MK_GetPublic_MotionControlFB(axis_id);
    ASSERT_TRUE(fb != NULL, "Pressure-limit test should resolve its FB");
    if (fb == NULL) {
        return;
    }

    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.AXISID) = axis_id;
    IEC_VAL(mv.VELOCITY) = 5.0f;
    IEC_VAL(mv.ACCELERATION) = 100.0f;
    IEC_VAL(mv.DECELERATION) = 100.0f;
    IEC_VAL(mv.DIRECTION) = 1;
    IEC_VAL(mv.PRESSURELIMIT) = 100.0f;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    __mcl_cmd_MoveVelocity(&mv);

    for (int step = 0; step < 100; ++step) {
        fb->AXIS_REF.pressure = 0.0f;
        __HydMotion_framework_Publish();
        hold_movevelocity_scan(&mv);
    }
    unrestricted_pump_speed = fb->PUMP_SPEED;

    fb->AXIS_REF.pressure = 110.0f;
    __HydMotion_framework_Publish();
    hold_movevelocity_scan(&mv);
    limited_pump_speed = fb->PUMP_SPEED;

    ASSERT_TRUE(unrestricted_pump_speed > 90.0f,
                "MoveVelocity should reach steady unrestricted pump speed before pressure limiting");
    ASSERT_TRUE(limited_pump_speed > unrestricted_pump_speed * 0.68f &&
                limited_pump_speed < unrestricted_pump_speed * 0.72f,
                "A 10 percent pressure excess should reduce pump speed to about 70 percent");
    ASSERT_TRUE(IEC_VAL(mv.BUSY) == true && IEC_VAL(mv.ACTIVE) == true,
                "Pressure derating should keep MoveVelocity busy and active before fault escalation");

    fb->PRESSURE_LIMIT = 80.0f;
    fb->AXIS_REF.pressure = 88.0f;
    __HydMotion_framework_Publish();
    hold_movevelocity_scan(&mv);
    axis_limited_pump_speed = fb->PUMP_SPEED;

    ASSERT_TRUE(axis_limited_pump_speed > unrestricted_pump_speed * 0.68f &&
                axis_limited_pump_speed < unrestricted_pump_speed * 0.72f,
                "The lower axis pressure limit should override the command pressure limit");

    for (int step = 0; step < 2000 && !IEC_VAL(mv.ERROR); ++step) {
        fb->AXIS_REF.pressure = 110.0f;
        __HydMotion_framework_Publish();
        hold_movevelocity_scan(&mv);
    }

    ASSERT_TRUE(IEC_VAL(mv.ERROR) == true,
                "Sustained pressure limiting should surface ERROR on MoveVelocity");
    ASSERT_TRUE(IEC_VAL(mv.ERRORID) == HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT,
                "Sustained pressure limiting should report the pressure-limit fault code");
    ASSERT_TRUE(IEC_VAL(mv.BUSY) == false && IEC_VAL(mv.ACTIVE) == false,
                "Pressure-limit fault should clear MoveVelocity busy and active outputs");
    ASSERT_TRUE(IEC_VAL(mv.COMMANDABORTED) == false,
                "Pressure-limit fault should not be reported as command preemption");
}

int main(void) {
    printf("=== MoveVelocity Stop/Reverse Restart Regression ===\n\n");

    test_reverse_restart_after_stop_starts_from_zero();
    test_continuousupdate_direction_flip_restarts_from_zero();
    test_pressure_limit_derates_output_and_faults();

    printf("\nTests passed: %d/%d\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
