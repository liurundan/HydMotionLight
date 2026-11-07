#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motion_planner.h"

static int failures;
static void check(int condition, const char *message)
{
    if (!condition) {
        ++failures;
        printf("FAIL: %s\n", message);
    }
}

int main(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL previous = 0.0;

    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_POSITIVE;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 100.0;
    segment.velocityToFlowGain = 1.0;

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;

    for (int cycle = 0; cycle < 100; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        check(output.targetVelocity >= previous - 1e-6,
              "speed ramp should not reverse or jump backward while accelerating");
        check(output.targetVelocity - previous <= 0.21,
              "100 ms acceleration time should bound each 1 ms velocity step");
        previous = output.targetVelocity;
    }
    check(output.targetVelocity > 19.0 && output.targetVelocity <= 20.0,
          "speed ramp should reach target after its transition time");

    segment.maxVelocity = 10.0;
    input.elapsedTime = 0.101;
    HYD_MotionPlanner_Execute(&input, &output);
    check(output.targetVelocity <= previous + 1e-6,
          "online target reduction should not jump above the previous output");
    check(output.targetVelocity >= 9.7,
          "online target reduction should smoothly approach the new target");

    printf("motion planner time ramp: %s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
