#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "pressure_model.h"

/*
 * This target is a legacy first-order compatibility regression only.  It is
 * deliberately not evidence for physical-model ripple suppression.
 */
static void test_first_order_hold_profile_has_no_physical_torque_packet(void) {
    PressureModelParams params;
    PressureModelState state;
    PressureModelOutput out;
    int i;

    PressureModel_InitParams(&params);
    params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    params.first_order_k_bar_per_rpm = 0.5f;
    params.first_order_tau_s = 0.050f;
    PressureModel_Reset(&state, 0x5a5a5a5au);
    for (i = 0; i < 1000; ++i) {
        PressureModel_Step(&params, &state, 100.0f, 0.001f, &out);
    }
    assert(isfinite(out.real_pressure_bar));
    assert(out.real_pressure_bar > 0.0f);
    assert(out.measured_pressure_bar == out.real_pressure_bar);
    assert(out.pump_flow_m3_s == 0.0f);
    assert(out.net_flow_m3_s == 0.0f);
    assert(!HYD_PumpFeedback_HasValid(out.pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TORQUE));
}

int main(void) {
    test_first_order_hold_profile_has_no_physical_torque_packet();
    puts("pressure hold diagnosis compatibility test passed");
    return 0;
}
