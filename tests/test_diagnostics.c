/* tests/test_diagnostics.c
 *
 * Unit tests for the diagnostics module — locks in the spec-table lookup
 * driven by HYD_Diagnostics_SetEvent, the Clear/flag helpers, and the
 * snapshot/history retention API. Note: the public API exposes the spec
 * table indirectly through SetEvent, which applies severity / source /
 * recovery / protectionAction from the static HYD_DIAGNOSTIC_SPECS array.
 * There is no HYD_Diagnostics_PopulateFromCode — SetEvent is the canonical
 * entry point.
 */
#include "diagnostics.h"
#include "common_types.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_set_event_timeout_applies_fault_severity_and_stop_action(void) {
    HYD_DiagnosticInfo info;

    memset(&info, 0, sizeof(info));
    HYD_Diagnostics_SetEvent(&info, HYD_DIAG_CODE_TIMEOUT, HYD_DIAG_SEVERITY_NONE);

    /* TIMEOUT spec: severity=FAULT, source=EXECUTION, protectionAction=STOP.
     * Passing severity=NONE lets ApplySpec fall back to the spec's default. */
    assert(info.code == HYD_DIAG_CODE_TIMEOUT);
    assert(info.severity == HYD_DIAG_SEVERITY_FAULT);
    assert(info.source == HYD_DIAG_SOURCE_EXECUTION);
    assert(info.recovery == HYD_DIAG_RECOVERY_RESTART_SEGMENT);
    assert(info.protectionAction == HYD_PROTECTION_ACTION_STOP);
    printf("test_set_event_timeout_applies_fault_severity_and_stop_action PASSED\n");
}

static void test_set_event_position_deviation_applies_warning_severity(void) {
    HYD_DiagnosticInfo info;

    memset(&info, 0, sizeof(info));
    HYD_Diagnostics_SetEvent(&info, HYD_DIAG_CODE_POSITION_DEVIATION, HYD_DIAG_SEVERITY_NONE);

    /* POSITION_DEVIATION spec: severity=WARNING, source=EXECUTION,
     * protectionAction=WARNING (not STOP). */
    assert(info.code == HYD_DIAG_CODE_POSITION_DEVIATION);
    assert(info.severity == HYD_DIAG_SEVERITY_WARNING);
    assert(info.source == HYD_DIAG_SOURCE_EXECUTION);
    assert(info.protectionAction == HYD_PROTECTION_ACTION_WARNING);
    printf("test_set_event_position_deviation_applies_warning_severity PASSED\n");
}

static void test_set_event_over_pressure_uses_derate_protection(void) {
    HYD_DiagnosticInfo info;

    memset(&info, 0, sizeof(info));
    HYD_Diagnostics_SetEvent(&info, HYD_DIAG_CODE_OVER_PRESSURE, HYD_DIAG_SEVERITY_NONE);

    /* OVER_PRESSURE is severity=WARNING with protectionAction=DERATE — the
     * runtime keeps executing but throttles. This guards against accidental
     * "promote to FAULT/STOP" regressions. */
    assert(info.code == HYD_DIAG_CODE_OVER_PRESSURE);
    assert(info.severity == HYD_DIAG_SEVERITY_WARNING);
    assert(info.protectionAction == HYD_PROTECTION_ACTION_DERATE);
    printf("test_set_event_over_pressure_uses_derate_protection PASSED\n");
}

static void test_set_event_severity_override_takes_precedence(void) {
    HYD_DiagnosticInfo info;

    memset(&info, 0, sizeof(info));
    /* Explicit severity overrides the spec default — confirms the
     * (severityOverride == NONE) gating logic in ApplySpec. */
    HYD_Diagnostics_SetEvent(&info, HYD_DIAG_CODE_OVER_PRESSURE, HYD_DIAG_SEVERITY_FAULT);

    assert(info.code == HYD_DIAG_CODE_OVER_PRESSURE);
    assert(info.severity == HYD_DIAG_SEVERITY_FAULT);
    /* Source/recovery/protectionAction still come from the spec table. */
    assert(info.source == HYD_DIAG_SOURCE_EXECUTION);
    assert(info.protectionAction == HYD_PROTECTION_ACTION_DERATE);
    printf("test_set_event_severity_override_takes_precedence PASSED\n");
}

static void test_clear_zeros_all_fields(void) {
    HYD_DiagnosticInfo info;

    /* Fill with non-zero garbage so any miss in Clear is visible. */
    memset(&info, 0xAA, sizeof(info));
    HYD_Diagnostics_Clear(&info);

    assert(info.code == HYD_DIAG_CODE_NONE);
    assert(info.severity == HYD_DIAG_SEVERITY_NONE);
    assert(info.source == HYD_DIAG_SOURCE_NONE);
    assert(info.recovery == HYD_DIAG_RECOVERY_NONE);
    assert(info.protectionAction == HYD_PROTECTION_ACTION_NONE);
    assert(!info.overPressure);
    assert(!info.underPressure);
    assert(!info.flowDeviation);
    assert(!info.positionDeviation);
    assert(!info.velocityDeviation);
    assert(!info.timeout);
    assert(!info.sensorFault);
    assert(!info.timestampRollback);
    printf("test_clear_zeros_all_fields PASSED\n");
}

static void test_capture_snapshot_marks_valid_when_code_is_set(void) {
    HYD_DiagnosticInfo info;
    HYD_DiagnosticSnapshot snapshot;
    HYD_AxisRef axis;
    HYD_ExecutionReference refs;

    memset(&info, 0, sizeof(info));
    memset(&axis, 0, sizeof(axis));
    memset(&refs, 0, sizeof(refs));
    axis.position = 42.0;
    axis.timestamp = 1.5;

    HYD_Diagnostics_SetEvent(&info, HYD_DIAG_CODE_SENSOR_FAULT, HYD_DIAG_SEVERITY_NONE);
    HYD_Diagnostics_CaptureSnapshot(&snapshot,
                                    &info,
                                    &axis,
                                    &refs,
                                    /*eventTimestamp*/ 2.0,
                                    /*segmentIndex*/ 3U,
                                    /*segmentTag*/ 5U,
                                    HYD_STATUS_FAULT,
                                    /*active*/ false,
                                    /*finished*/ false,
                                    /*fault*/ true);

    assert(snapshot.valid);
    assert(snapshot.diagnostic.code == HYD_DIAG_CODE_SENSOR_FAULT);
    assert(snapshot.diagnostic.severity == HYD_DIAG_SEVERITY_FAULT);
    assert(snapshot.segmentIndex == 3U);
    assert(snapshot.segmentTag == 5U);
    assert(snapshot.status == HYD_STATUS_FAULT);
    assert(snapshot.fault);
    assert(snapshot.eventTimestamp == 2.0);
    assert(snapshot.axisRef.position == 42.0);
    printf("test_capture_snapshot_marks_valid_when_code_is_set PASSED\n");
}

static void test_capture_snapshot_marks_invalid_for_no_code(void) {
    HYD_DiagnosticInfo info;
    HYD_DiagnosticSnapshot snapshot;

    memset(&info, 0, sizeof(info));
    /* code stays NONE → snapshot.valid must be false even though we capture. */
    HYD_Diagnostics_CaptureSnapshot(&snapshot,
                                    &info,
                                    NULL,
                                    NULL,
                                    /*eventTimestamp*/ 0.0,
                                    /*segmentIndex*/ 0U,
                                    /*segmentTag*/ 0U,
                                    HYD_STATUS_IDLE,
                                    /*active*/ false,
                                    /*finished*/ false,
                                    /*fault*/ false);

    assert(!snapshot.valid);
    assert(snapshot.diagnostic.code == HYD_DIAG_CODE_NONE);
    printf("test_capture_snapshot_marks_invalid_for_no_code PASSED\n");
}

static void test_history_push_and_get_latest(void) {
    HYD_DiagnosticHistory history;
    HYD_DiagnosticInfo info;
    HYD_DiagnosticSnapshot snapshot;
    HYD_DiagnosticSnapshot retrieved;

    HYD_DiagnosticsHistory_Clear(&history);
    assert(!history.hasRecord);
    assert(history.totalRecorded == 0U);

    memset(&info, 0, sizeof(info));
    HYD_Diagnostics_SetEvent(&info, HYD_DIAG_CODE_OVER_PRESSURE, HYD_DIAG_SEVERITY_NONE);
    HYD_Diagnostics_CaptureSnapshot(&snapshot, &info, NULL, NULL,
                                    1.0, 0U, 0U,
                                    HYD_STATUS_DEGRADED, true, false, false);
    assert(snapshot.valid);

    HYD_DiagnosticsHistory_Push(&history, &snapshot);
    assert(history.hasRecord);
    assert(history.totalRecorded == 1U);

    assert(HYD_DiagnosticsHistory_GetLatest(&history, &retrieved));
    assert(retrieved.diagnostic.code == HYD_DIAG_CODE_OVER_PRESSURE);
    /* Invalid snapshots must be rejected — Push() short-circuits when
     * !snapshot.valid (regression guard). */
    HYD_Diagnostics_ClearSnapshot(&snapshot);
    HYD_DiagnosticsHistory_Push(&history, &snapshot);
    assert(history.totalRecorded == 1U);
    printf("test_history_push_and_get_latest PASSED\n");
}

static void test_get_flag_mask_reflects_active_flags(void) {
    HYD_DiagnosticInfo info;
    HYD_DiagnosticFlags mask;

    memset(&info, 0, sizeof(info));
    info.overPressure = true;
    info.timeout = true;
    mask = HYD_Diagnostics_GetFlagMask(&info);

    assert((mask & HYD_DIAG_FLAG_OVER_PRESSURE) != 0U);
    assert((mask & HYD_DIAG_FLAG_TIMEOUT) != 0U);
    assert((mask & HYD_DIAG_FLAG_FLOW_DEVIATION) == 0U);
    assert(HYD_Diagnostics_HasFlag(&info, HYD_DIAG_FLAG_OVER_PRESSURE));
    assert(!HYD_Diagnostics_HasFlag(&info, HYD_DIAG_FLAG_SENSOR_FAULT));
    /* HasFlag with NONE must return false to avoid masking-all defaults. */
    assert(!HYD_Diagnostics_HasFlag(&info, HYD_DIAG_FLAG_NONE));
    printf("test_get_flag_mask_reflects_active_flags PASSED\n");
}

static void test_diag_spec_returns_warning_for_pressure_ceiling_exceeded(void) {
    HYD_DiagnosticInfo info;
    memset(&info, 0, sizeof(info));
    HYD_Diagnostics_SetEvent(&info, HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED, HYD_DIAG_SEVERITY_NONE);
    assert(info.code == HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED);
    assert(info.severity == HYD_DIAG_SEVERITY_WARNING);
    assert(info.protectionAction == HYD_PROTECTION_ACTION_DERATE);
    printf("test_diag_spec_returns_warning_for_pressure_ceiling_exceeded PASSED\n");
}

static void test_diag_spec_returns_fault_for_pressure_ceiling_violated(void) {
    HYD_DiagnosticInfo info;
    memset(&info, 0, sizeof(info));
    HYD_Diagnostics_SetEvent(&info, HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED, HYD_DIAG_SEVERITY_NONE);
    assert(info.code == HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED);
    assert(info.severity == HYD_DIAG_SEVERITY_FAULT);
    assert(info.protectionAction == HYD_PROTECTION_ACTION_STOP);
    printf("test_diag_spec_returns_fault_for_pressure_ceiling_violated PASSED\n");
}

static void test_ceiling_flag_mask_round_trip(void) {
    HYD_DiagnosticInfo info;
    memset(&info, 0, sizeof(info));
    info.pressureCeilingExceeded = true;
    info.flags = HYD_Diagnostics_GetFlagMask(&info);
    assert((info.flags & HYD_DIAG_FLAG_PRESSURE_CEILING_EXCEEDED) != 0U);
    assert((info.flags & HYD_DIAG_FLAG_PRESSURE_CEILING_VIOLATED) == 0U);

    info.pressureCeilingViolated = true;
    info.flags = HYD_Diagnostics_GetFlagMask(&info);
    assert((info.flags & HYD_DIAG_FLAG_PRESSURE_CEILING_VIOLATED) != 0U);
    printf("test_ceiling_flag_mask_round_trip PASSED\n");
}

static void test_mechanism_diagnostic_recovery_contract(void) {
    HYD_DiagnosticInfo info;

    memset(&info, 0, sizeof(info));
    HYD_Diagnostics_SetEvent(&info,
                             HYD_DIAG_CODE_MECHANISM_CONFIG_INVALID,
                             HYD_DIAG_SEVERITY_NONE);
    assert(info.severity == HYD_DIAG_SEVERITY_WARNING);
    assert(info.recovery == HYD_DIAG_RECOVERY_CHECK_COMMAND);

    HYD_Diagnostics_SetEvent(&info,
                             HYD_DIAG_CODE_KINEMATICS_RUNTIME_INVALID,
                             HYD_DIAG_SEVERITY_NONE);
    assert(info.severity == HYD_DIAG_SEVERITY_FAULT);
    assert(info.recovery == HYD_DIAG_RECOVERY_RESET_CONTROLLER);
    assert(info.protectionAction == HYD_PROTECTION_ACTION_STOP);
    assert(strcmp(HYD_Diagnostics_CodeToString(
                      HYD_DIAG_CODE_KINEMATICS_RUNTIME_INVALID),
                  "KINEMATICS_RUNTIME_INVALID") == 0);
    printf("test_mechanism_diagnostic_recovery_contract PASSED\n");
}

int main(void) {
    test_set_event_timeout_applies_fault_severity_and_stop_action();
    test_set_event_position_deviation_applies_warning_severity();
    test_set_event_over_pressure_uses_derate_protection();
    test_set_event_severity_override_takes_precedence();
    test_clear_zeros_all_fields();
    test_capture_snapshot_marks_valid_when_code_is_set();
    test_capture_snapshot_marks_invalid_for_no_code();
    test_history_push_and_get_latest();
    test_get_flag_mask_reflects_active_flags();
    test_diag_spec_returns_warning_for_pressure_ceiling_exceeded();
    test_diag_spec_returns_fault_for_pressure_ceiling_violated();
    test_ceiling_flag_mask_round_trip();
    test_mechanism_diagnostic_recovery_contract();
    return 0;
}
