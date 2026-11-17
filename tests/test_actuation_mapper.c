#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "actuation_mapper.h"

static void assert_near(HYD_REAL actual, HYD_REAL expected,
                        HYD_REAL tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

static HYD_TogglePreparedConfig validated_default(void)
{
    HYD_ToggleGeometryConfig raw = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    assert(HYD_ToggleKinematics_ValidateBlocking(&raw, &prepared, &error));
    return prepared;
}

static void test_mechanism_ordinals_and_direct_identity(void)
{
    HYD_CylinderConfig cylinder = {2000.0f, 1000.0f, 0.0f, 0.0f, 0.0f};
    HYD_ActuationMapperInput input = {
        HYD_MECHANISM_DIRECT, NULL, 12.5f, 5.0f, &cylinder, 0.25f, 0.0f
    };
    HYD_ActuationMapperOutput output;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    assert(HYD_MECHANISM_DIRECT == 0);
    assert(HYD_MECHANISM_FIVE_POINT_TOGGLE == 1);
    assert(HYD_ActuationMapper_MapVelocity(&input, &output, &error));
    assert(error == HYD_TOGGLE_ERROR_NONE);
    assert_near(output.actuatorPosition, 12.5f, 0.0f);
    assert_near(output.velocityRatio, 1.0f, 0.0f);
    assert_near(output.actuatorVelocity, 5.0f, 0.0f);
    assert(output.actuatorDirection == HYD_DIRECTION_EXTEND);
    assert_near(output.effectiveCylinderGain, 0.12f, 1.0e-7f);
    assert_near(output.requestedFlow, 0.6f, 1.0e-6f);

    input.fallbackCylinderVelocityToFlowGain = NAN;
    assert(HYD_ActuationMapper_MapVelocity(&input, &output, &error));
    assert_near(output.effectiveCylinderGain, 0.12f, 1.0e-7f);
}

static void test_toggle_mapping_uses_actuator_direction_area(void)
{
    HYD_TogglePreparedConfig prepared = validated_default();
    HYD_CylinderConfig cylinder = {2000.0f, 1000.0f, 0.0f, 0.0f, 0.0f};
    HYD_ActuationMapperInput input = {
        HYD_MECHANISM_FIVE_POINT_TOGGLE, &prepared,
        101.0f, 10.0f, &cylinder, 0.25f, 0.0f
    };
    HYD_ActuationMapperOutput output;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    assert(HYD_ActuationMapper_MapVelocity(&input, &output, &error));
    assert_near(output.actuatorPosition, 63.808094f, 2.0e-3f);
    assert_near(output.velocityRatio, 0.8083796f, 2.0e-4f);
    assert_near(output.actuatorVelocity, 8.083796f, 2.0e-3f);
    assert(output.actuatorDirection == HYD_DIRECTION_EXTEND);
    assert_near(output.effectiveCylinderGain, 0.12f, 1.0e-7f);
    assert_near(output.requestedFlow, 0.9700555f, 3.0e-4f);

    input.templateVelocity = -10.0f;
    assert(HYD_ActuationMapper_MapVelocity(&input, &output, &error));
    assert(output.actuatorDirection == HYD_DIRECTION_RETRACT);
    assert_near(output.effectiveCylinderGain, 0.06f, 1.0e-7f);
    assert_near(output.requestedFlow, 0.485028f, 2.0e-4f);
}

static void test_fallback_gain_and_hardware_flow_limit(void)
{
    HYD_TogglePreparedConfig prepared = validated_default();
    HYD_CylinderConfig cylinder = {2000.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    HYD_ActuationMapperInput input = {
        HYD_MECHANISM_FIVE_POINT_TOGGLE, &prepared,
        101.0f, 10.0f, &cylinder, 0.2f, 0.3f
    };
    HYD_ActuationMapperOutput output;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    assert(HYD_ActuationMapper_MapVelocity(&input, &output, &error));
    assert(output.actuatorDirection == HYD_DIRECTION_EXTEND);
    assert_near(output.effectiveCylinderGain, 0.12f, 0.0f);
    assert_near(output.requestedFlow, 0.3f, 0.0f);
    assert_near(output.unlimitedRequestedFlow, 0.9700555f, 4.0e-4f);
    assert_near(output.maxTemplateVelocity, 3.092607f, 4.0e-4f);
    assert(output.flowLimitActive);

    input.maxFlow = 0.0f;
    assert(HYD_ActuationMapper_MapVelocity(&input, &output, &error));
    assert_near(output.requestedFlow, 0.9700555f, 4.0e-4f);
    assert_near(output.unlimitedRequestedFlow, output.requestedFlow, 0.0f);
    assert(output.maxTemplateVelocity == 0.0f);
    assert(!output.flowLimitActive);
}

static void test_flow_to_template_velocity_uses_dynamic_gain(void)
{
    HYD_TogglePreparedConfig prepared = validated_default();
    HYD_CylinderConfig cylinder = {2000.0f, 1000.0f, 0.0f, 0.0f, 0.0f};
    HYD_ActuationMapperInput input = {
        HYD_MECHANISM_FIVE_POINT_TOGGLE, &prepared,
        101.0f, 0.0f, &cylinder, 0.25f, 0.0f
    };
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL template_velocity = 1234.0f;

    assert(HYD_ActuationMapper_FlowToTemplateVelocity(
        &input, 0.9700555f, &template_velocity, &error));
    assert_near(template_velocity, 10.0f, 3.0e-3f);

    assert(HYD_ActuationMapper_FlowToTemplateVelocity(
        &input, -0.4850278f, &template_velocity, &error));
    assert_near(template_velocity, -10.0f, 3.0e-3f);

    input.mechanismType = HYD_MECHANISM_DIRECT;
    input.toggleConfig = NULL;
    input.cylinderConfig = NULL;
    input.fallbackCylinderVelocityToFlowGain = 0.25f;
    assert(HYD_ActuationMapper_FlowToTemplateVelocity(
        &input, -1.0f, &template_velocity, &error));
    assert_near(template_velocity, -4.0f, 0.0f);
}

static void test_mapping_failures_preserve_outputs(void)
{
    HYD_ActuationMapperInput input = {
        HYD_MECHANISM_FIVE_POINT_TOGGLE, NULL,
        101.0f, 10.0f, NULL, 0.0f, 0.0f
    };
    HYD_ActuationMapperOutput output = {
        1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f,
        HYD_DIRECTION_HOLD, true
    };
    HYD_ActuationMapperOutput before = output;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL velocity = 77.0f;

    assert(!HYD_ActuationMapper_MapVelocity(&input, &output, &error));
    assert(error == HYD_TOGGLE_ERROR_NULL_ARGUMENT);
    assert(memcmp(&output, &before, sizeof(output)) == 0);

    input.mechanismType = HYD_MECHANISM_DIRECT;
    assert(!HYD_ActuationMapper_FlowToTemplateVelocity(
        &input, 1.0f, &velocity, &error));
    assert(error == HYD_TOGGLE_ERROR_VELOCITY_RATIO_UNSAFE);
    assert_near(velocity, 77.0f, 0.0f);

    input.mechanismType = (HYD_MechanismType)99;
    input.fallbackCylinderVelocityToFlowGain = 0.2f;
    assert(!HYD_ActuationMapper_MapVelocity(&input, &output, &error));
    assert(error == HYD_TOGGLE_ERROR_INVALID_BRANCH);
    assert(memcmp(&output, &before, sizeof(output)) == 0);

    input.mechanismType = HYD_MECHANISM_DIRECT;
    input.templateVelocity = 10.0f;
    input.fallbackCylinderVelocityToFlowGain = 0.0f;
    assert(!HYD_ActuationMapper_MapVelocity(&input, &output, &error));
    assert(error == HYD_TOGGLE_ERROR_VELOCITY_RATIO_UNSAFE);
    assert(memcmp(&output, &before, sizeof(output)) == 0);
}

static void test_reverse_ignores_unused_fields_and_zero_needs_no_area(void)
{
    HYD_ActuationMapperInput input = {
        HYD_MECHANISM_DIRECT, NULL, 5.0f, NAN, NULL, 0.25f, NAN
    };
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL velocity = 99.0f;

    assert(HYD_ActuationMapper_FlowToTemplateVelocity(
        &input, 1.0f, &velocity, &error));
    assert_near(velocity, 4.0f, 0.0f);

    input.fallbackCylinderVelocityToFlowGain = 0.0f;
    assert(HYD_ActuationMapper_FlowToTemplateVelocity(
        &input, 0.0f, &velocity, &error));
    assert_near(velocity, 0.0f, 0.0f);

    input.templateVelocity = 0.0f;
    input.maxFlow = 0.0f;
    {
        HYD_ActuationMapperOutput output;
        assert(HYD_ActuationMapper_MapVelocity(&input, &output, &error));
        assert(output.actuatorDirection == HYD_DIRECTION_HOLD);
        assert_near(output.effectiveCylinderGain, 0.0f, 0.0f);
        assert_near(output.requestedFlow, 0.0f, 0.0f);
    }
}

/* Machine 105 geometry after the on-site LF correction (LF = 323 mm). */
static HYD_TogglePreparedConfig validated_105(void)
{
    HYD_ToggleGeometryConfig raw = {
        260.0f, 323.0f, 190.0f, 100.0f, 125.0f,
        200.0f, 170.0f, 580.0f, 450.0f,
        0.0f, (int8_t)-1, (int8_t)-1, (int8_t)-1, (int8_t)-1
    };
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    assert(HYD_ToggleKinematics_ValidateBlocking(&raw, &prepared, &error));
    return prepared;
}

/* Effective clamp areas: open (extend) 3350 mm^2, close (retract) 4800 mm^2;
 * pump flow cap 80.75 L/min = 1700 rpm * 50 cc * 0.95. */
static void test_range_max_template_velocity_uses_worst_ratio(void)
{
    HYD_TogglePreparedConfig prepared = validated_105();
    HYD_CylinderConfig cylinder = {3350.0f, 4800.0f, 0.0f, 0.0f, 0.0f};
    HYD_ActuationMapperInput input = {
        HYD_MECHANISM_FIVE_POINT_TOGGLE, &prepared,
        0.0f, 0.0f, &cylinder, 0.2f, 80.75f
    };
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL velocity = 0.0f;
    HYD_REAL ratio = 0.0f;

    /* Close 150 -> 10: worst ratio at the die-side end, R(10) = 2.6461. */
    assert(HYD_ActuationMapper_MaxTemplateVelocityOverRange(
        &input, 150.0f, 10.0f, HYD_DIRECTION_RETRACT,
        &velocity, &ratio, &error));
    assert(error == HYD_TOGGLE_ERROR_NONE);
    assert_near(ratio, 2.6461f, 0.002f);
    assert_near(velocity, 105.96f, 0.2f);

    /* Open 200 -> 300 uses the extend area: R(300) = 0.6139. */
    assert(HYD_ActuationMapper_MaxTemplateVelocityOverRange(
        &input, 200.0f, 300.0f, HYD_DIRECTION_EXTEND,
        &velocity, &ratio, &error));
    assert_near(ratio, 0.6139f, 0.002f);
    assert_near(velocity, 654.4f, 1.3f);

    /* Same range, retract direction must switch to the 4800 mm^2 area. */
    assert(HYD_ActuationMapper_MaxTemplateVelocityOverRange(
        &input, 200.0f, 300.0f, HYD_DIRECTION_RETRACT,
        &velocity, &ratio, &error));
    assert_near(velocity, 456.7f, 0.9f);

    /* 300 -> 450 peaks inside the range (|R| = 0.6731 near x = 386.5);
     * the endpoints alone (0.6139 / 0.2451) would overstate the speed. */
    assert(HYD_ActuationMapper_MaxTemplateVelocityOverRange(
        &input, 300.0f, 450.0f, HYD_DIRECTION_EXTEND,
        &velocity, &ratio, &error));
    assert_near(ratio, 0.6731f, 0.0013f);
    assert_near(velocity, 596.9f, 1.2f);

    /* Unknown direction takes the slower (larger-area) side. */
    assert(HYD_ActuationMapper_MaxTemplateVelocityOverRange(
        &input, 200.0f, 300.0f, HYD_DIRECTION_HOLD,
        &velocity, &ratio, &error));
    assert_near(velocity, 456.7f, 0.9f);
}

static void test_range_max_template_velocity_clips_and_rejects(void)
{
    HYD_TogglePreparedConfig prepared = validated_105();
    HYD_CylinderConfig cylinder = {3350.0f, 4800.0f, 0.0f, 0.0f, 0.0f};
    HYD_ActuationMapperInput input = {
        HYD_MECHANISM_FIVE_POINT_TOGGLE, &prepared,
        0.0f, 0.0f, &cylinder, 0.2f, 80.75f
    };
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL velocity = 0.0f;
    HYD_REAL ratio = 0.0f;

    /* Below the online envelope is clipped to x = 0, R(0) = 10.0203. */
    assert(HYD_ActuationMapper_MaxTemplateVelocityOverRange(
        &input, -5.0f, 10.0f, HYD_DIRECTION_EXTEND,
        &velocity, &ratio, &error));
    assert_near(ratio, 10.0203f, 0.01f);
    assert_near(velocity, 40.09f, 0.1f);

    velocity = 7.0f;
    ratio = 8.0f;
    assert(!HYD_ActuationMapper_MaxTemplateVelocityOverRange(
        &input, 460.0f, 500.0f, HYD_DIRECTION_EXTEND,
        &velocity, &ratio, &error));
    assert(error == HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE);
    assert_near(velocity, 7.0f, 0.0f);
    assert_near(ratio, 8.0f, 0.0f);

    assert(!HYD_ActuationMapper_MaxTemplateVelocityOverRange(
        &input, NAN, 10.0f, HYD_DIRECTION_EXTEND,
        &velocity, &ratio, &error));
    assert(error == HYD_TOGGLE_ERROR_NONFINITE_PARAMETER);

    input.maxFlow = 0.0f;
    assert(!HYD_ActuationMapper_MaxTemplateVelocityOverRange(
        &input, 10.0f, 150.0f, HYD_DIRECTION_EXTEND,
        &velocity, &ratio, &error));
    assert(error == HYD_TOGGLE_ERROR_NONFINITE_PARAMETER);
    assert_near(velocity, 7.0f, 0.0f);

    /* Direct axes use ratio 1 over any range: 80.75 / (10000 * 6e-5). */
    input.mechanismType = HYD_MECHANISM_DIRECT;
    input.toggleConfig = NULL;
    input.maxFlow = 80.75f;
    cylinder.areaExtendMm2 = 10000.0f;
    assert(HYD_ActuationMapper_MaxTemplateVelocityOverRange(
        &input, -50.0f, 900.0f, HYD_DIRECTION_EXTEND,
        &velocity, &ratio, &error));
    assert_near(ratio, 1.0f, 0.0f);
    assert_near(velocity, 134.58f, 0.01f);
}

int main(void)
{
    test_mechanism_ordinals_and_direct_identity();
    test_toggle_mapping_uses_actuator_direction_area();
    test_fallback_gain_and_hardware_flow_limit();
    test_flow_to_template_velocity_uses_dynamic_gain();
    test_mapping_failures_preserve_outputs();
    test_reverse_ignores_unused_fields_and_zero_needs_no_area();
    test_range_max_template_velocity_uses_worst_ratio();
    test_range_max_template_velocity_clips_and_rejects();
    printf("actuation mapper tests passed\n");
    return 0;
}
