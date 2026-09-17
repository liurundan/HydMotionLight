#include "pressure_model.h"

#include <math.h>
#include <string.h>

#define PRESSURE_MODEL_DT_S 0.001f
#define PRESSURE_MODEL_PI 3.14159265358979323846f
#define PRESSURE_MODEL_PA_PER_BAR 1.0e5f
#define PRESSURE_MODEL_EPS_FLOW 1.0e-12f

static float pressure_model_absf(float value) {
    return value < 0.0f ? -value : value;
}

static float pressure_model_clampf(float value, float lower, float upper) {
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

static float pressure_model_maxf(float a, float b) {
    return a > b ? a : b;
}

static float pressure_model_signf(float value) {
    return value < 0.0f ? -1.0f : (value > 0.0f ? 1.0f : 0.0f);
}

static float pressure_model_wrap_unit(float phase) {
    if (!isfinite(phase)) return NAN;
    phase = fmodf(phase, 1.0f);
    return phase < 0.0f ? phase + 1.0f : phase;
}

static uint32_t pressure_model_seed(uint32_t seed) {
    return seed == 0u ? 0xa341316cu : seed;
}

static uint32_t pressure_model_next_u32(PressureModelState *state) {
    uint32_t value = pressure_model_seed(state->rng_state);

    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    state->rng_state = pressure_model_seed(value);
    return state->rng_state;
}

static float pressure_model_uniform01(PressureModelState *state) {
    return (pressure_model_next_u32(state) & 0x00ffffffu) / 16777216.0f;
}

static float pressure_model_gaussian(PressureModelState *state, float stddev) {
    float u1;
    float u2;
    float magnitude;

    if (stddev <= 0.0f) return 0.0f;
    if (state->has_spare_gauss) {
        state->has_spare_gauss = 0;
        return stddev * state->spare_gauss;
    }
    u1 = pressure_model_uniform01(state);
    u2 = pressure_model_uniform01(state);
    if (u1 < 1.0e-7f) u1 = 1.0e-7f;
    magnitude = sqrtf(-2.0f * logf(u1));
    state->spare_gauss = magnitude * sinf(2.0f * PRESSURE_MODEL_PI * u2);
    state->has_spare_gauss = 1;
    return stddev * magnitude * cosf(2.0f * PRESSURE_MODEL_PI * u2);
}

static int pressure_model_is_finite_nonnegative(float value) {
    return isfinite(value) && value >= 0.0f;
}

static int pressure_model_is_finite_positive(float value) {
    return isfinite(value) && value > 0.0f;
}

static int pressure_model_physical_state_is_finite(const PressureModelState *state) {
    int i;

    if (!isfinite(state->motor_rpm) || !isfinite(state->pressure_pa) ||
        !isfinite(state->pump_phase_rev) || !isfinite(state->timestamp_s) ||
        !isfinite(state->spare_gauss) || !isfinite(state->outlet_pressure_pa) ||
        !isfinite(state->line_flow_m3_s) || !isfinite(state->motor_accel_rpm_s) ||
        state->motor_delay_index >= PRESSURE_MODEL_PHYSICAL_DELAY_SLOTS ||
        state->sensor_delay_index >= PRESSURE_MODEL_PHYSICAL_DELAY_SLOTS) {
        return 0;
    }
    for (i = 0; i < PRESSURE_MODEL_PHYSICAL_DELAY_SLOTS; ++i) {
        if (!isfinite(state->motor_delay_ring[i]) ||
            !isfinite(state->sensor_delay_ring[i])) {
            return 0;
        }
    }
    return 1;
}

static int pressure_model_physical_values_are_finite(const PressureModelPhysicalParams *p) {
    return isfinite(p->atmospheric_pressure_pa) &&
           isfinite(p->suction_pressure_pa) &&
           isfinite(p->outlet_volume_m3) &&
           isfinite(p->chamber_volume_m3) &&
           isfinite(p->line_inertance_pa_s2_per_m3) &&
           isfinite(p->line_resistance_pa_s_per_m3) &&
           isfinite(p->line_quadratic_resistance_pa_s2_per_m6) &&
           isfinite(p->beta_oil_pa) &&
           isfinite(p->gas_fraction) &&
           isfinite(p->gas_transition_pa) &&
           isfinite(p->beta_min_pa) &&
           isfinite(p->pump_leak_c0_m3_pa_s) &&
           isfinite(p->pump_leak_speed_m3_pa_s_per_rpm) &&
           isfinite(p->outlet_leak_m3_pa_s) &&
           isfinite(p->cylinder_leak_m3_pa_s) &&
           isfinite(p->eta_v_min) &&
           isfinite(p->eta_m_nominal) &&
           isfinite(p->eta_m_pressure_loss_per_pa) &&
           isfinite(p->eta_m_speed_loss_per_rpm) &&
           isfinite(p->eta_m_min) &&
           isfinite(p->rated_motor_torque_nm) &&
           isfinite(p->torque_ripple13_peak) &&
           isfinite(p->torque_ripple13_phase_rad) &&
           isfinite(p->ripple13_peak) &&
           isfinite(p->ripple26_peak) &&
           isfinite(p->ripple39_peak) &&
           isfinite(p->ripple13_phase_rad) &&
           isfinite(p->ripple26_phase_rad) &&
           isfinite(p->ripple39_phase_rad) &&
           isfinite(p->motor_natural_freq_hz) &&
           isfinite(p->motor_damping) &&
           isfinite(p->motor_delay_s) &&
           isfinite(p->motor_accel_limit_rpm_s) &&
           isfinite(p->motor_torque_limit_permille) &&
           isfinite(p->relief_set_pa) &&
           isfinite(p->relief_deadband_pa) &&
           isfinite(p->relief_orifice_coeff_m3_s_sqrt_pa) &&
           isfinite(p->relief_hysteresis_pa) &&
           isfinite(p->sensor_delay_s) &&
           isfinite(p->sensor_quantization_bar);
}

static unsigned char pressure_model_normalize_type(unsigned char model_type) {
    return model_type == PRESSURE_MODEL_TYPE_FIRST_ORDER
               ? PRESSURE_MODEL_TYPE_FIRST_ORDER
               : PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED;
}

static int pressure_model_delay_steps(float delay_s) {
    if (!isfinite(delay_s) || delay_s < 0.0f ||
        delay_s > PRESSURE_MODEL_PHYSICAL_MAX_DELAY_STEPS * PRESSURE_MODEL_DT_S) {
        return 0;
    }
    int steps = (int)(delay_s / PRESSURE_MODEL_DT_S + 0.5f);

    if (steps < 0) return 0;
    if (steps > PRESSURE_MODEL_PHYSICAL_MAX_DELAY_STEPS) {
        return PRESSURE_MODEL_PHYSICAL_MAX_DELAY_STEPS;
    }
    return steps;
}

static float pressure_model_delay_write_read(float ring[PRESSURE_MODEL_PHYSICAL_DELAY_SLOTS],
                                             unsigned char *index,
                                             int delay_steps,
                                             float value) {
    int read_index;

    if (*index >= PRESSURE_MODEL_PHYSICAL_DELAY_SLOTS) return value;
    ring[*index] = value;
    read_index = ((int)*index + PRESSURE_MODEL_PHYSICAL_DELAY_SLOTS - delay_steps) %
                 PRESSURE_MODEL_PHYSICAL_DELAY_SLOTS;
    value = ring[read_index];
    *index = (unsigned char)(((int)*index + 1) % PRESSURE_MODEL_PHYSICAL_DELAY_SLOTS);
    return value;
}

static void pressure_model_fill_feedback(const PressureModelState *state,
                                         PressureModelOutput *out,
                                         int torque_valid,
                                         float torque_permille) {
    float angle = NAN;

    memset(&out->pumpFeedback, 0, sizeof(out->pumpFeedback));
    if (isfinite(out->actual_motor_rpm)) {
        out->pumpFeedback.rpm = out->actual_motor_rpm;
        out->pumpFeedback.validFlags |= HYD_PUMP_FEEDBACK_VALID_RPM;
    }
    if (isfinite(state->pump_phase_rev)) {
        angle = fmodf(state->pump_phase_rev * 360.0f, 360.0f);
        if (angle < 0.0f) angle += 360.0f;
        if (isfinite(angle)) {
            out->pumpFeedback.angleDeg = angle;
            out->pumpFeedback.validFlags |= HYD_PUMP_FEEDBACK_VALID_ANGLE;
        }
    }
    if (isfinite(state->timestamp_s)) {
        out->pumpFeedback.timestamp = state->timestamp_s;
        out->pumpFeedback.validFlags |= HYD_PUMP_FEEDBACK_VALID_TIMESTAMP;
    }
    if (torque_valid && isfinite(torque_permille)) {
        out->pumpFeedback.torquePermille = torque_permille;
        out->pumpFeedback.validFlags |= HYD_PUMP_FEEDBACK_VALID_TORQUE;
    }
}

float PressureModel_EffectiveBulkModulusPa(const PressureModelPhysicalParams *params,
                                           float absolute_pressure_pa) {
    float gas_alpha;
    float inverse_beta;

    if (params == NULL || !isfinite(absolute_pressure_pa)) return NAN;
    gas_alpha = params->gas_fraction * params->gas_transition_pa /
                pressure_model_maxf(absolute_pressure_pa, params->gas_transition_pa);
    gas_alpha = pressure_model_clampf(gas_alpha, 0.0f, params->gas_fraction);
    inverse_beta = (1.0f - gas_alpha) / params->beta_oil_pa +
                   gas_alpha / pressure_model_maxf(absolute_pressure_pa, 1.0f);
    return pressure_model_clampf(1.0f / inverse_beta,
                                 params->beta_min_pa,
                                 params->beta_oil_pa);
}

int PressureModel_ValidatePhysicalParams(const PressureModelPhysicalParams *params) {
    float stiffness_ratio;

    if (params == NULL || !pressure_model_physical_values_are_finite(params) ||
        !pressure_model_is_finite_positive(params->atmospheric_pressure_pa) ||
        !pressure_model_is_finite_positive(params->suction_pressure_pa) ||
        !pressure_model_is_finite_positive(params->outlet_volume_m3) ||
        !pressure_model_is_finite_positive(params->chamber_volume_m3) ||
        !pressure_model_is_finite_positive(params->line_inertance_pa_s2_per_m3) ||
        !pressure_model_is_finite_positive(params->beta_oil_pa) ||
        !pressure_model_is_finite_positive(params->beta_min_pa) ||
        !pressure_model_is_finite_positive(params->gas_transition_pa) ||
        !pressure_model_is_finite_positive(params->rated_motor_torque_nm) ||
        !pressure_model_is_finite_positive(params->motor_natural_freq_hz) ||
        params->motor_natural_freq_hz >
            PRESSURE_MODEL_PHYSICAL_MAX_MOTOR_NATURAL_FREQ_HZ ||
        !pressure_model_is_finite_positive(params->motor_accel_limit_rpm_s)) {
        return 0;
    }
    if (params->beta_min_pa > params->beta_oil_pa ||
        !pressure_model_is_finite_nonnegative(params->gas_fraction) ||
        params->gas_fraction > 1.0f ||
        !pressure_model_is_finite_nonnegative(params->line_resistance_pa_s_per_m3) ||
        !pressure_model_is_finite_nonnegative(params->line_quadratic_resistance_pa_s2_per_m6) ||
        !pressure_model_is_finite_nonnegative(params->pump_leak_c0_m3_pa_s) ||
        !pressure_model_is_finite_nonnegative(params->pump_leak_speed_m3_pa_s_per_rpm) ||
        !pressure_model_is_finite_nonnegative(params->outlet_leak_m3_pa_s) ||
        !pressure_model_is_finite_nonnegative(params->cylinder_leak_m3_pa_s) ||
        !pressure_model_is_finite_nonnegative(params->relief_set_pa) ||
        !pressure_model_is_finite_nonnegative(params->relief_deadband_pa) ||
        !pressure_model_is_finite_nonnegative(params->relief_orifice_coeff_m3_s_sqrt_pa) ||
        !pressure_model_is_finite_nonnegative(params->relief_hysteresis_pa) ||
        !pressure_model_is_finite_nonnegative(params->sensor_quantization_bar) ||
        !pressure_model_is_finite_nonnegative(params->motor_damping) ||
        params->motor_damping > PRESSURE_MODEL_PHYSICAL_MAX_MOTOR_DAMPING ||
        !pressure_model_is_finite_nonnegative(params->torque_ripple13_peak) ||
        params->torque_ripple13_peak > 1.0f ||
        !pressure_model_is_finite_nonnegative(params->ripple13_peak) ||
        !pressure_model_is_finite_nonnegative(params->ripple26_peak) ||
        !pressure_model_is_finite_nonnegative(params->ripple39_peak) ||
        params->eta_v_min <= 0.0f || params->eta_v_min > 1.0f ||
        params->eta_m_nominal <= 0.0f || params->eta_m_nominal > 1.0f ||
        params->eta_m_min <= 0.0f || params->eta_m_min > params->eta_m_nominal ||
        params->motor_delay_s < 0.0f || params->motor_delay_s > 0.064f ||
        params->sensor_delay_s < 0.0f || params->sensor_delay_s > 0.064f ||
        params->motor_accel_limit_rpm_s >
            PRESSURE_MODEL_PHYSICAL_MAX_MOTOR_ACCEL_RPM_S ||
        params->motor_torque_limit_permille < 0.0f ||
        params->eta_m_pressure_loss_per_pa < 0.0f ||
        params->eta_m_speed_loss_per_rpm < 0.0f) {
        return 0;
    }
    stiffness_ratio = params->beta_oil_pa * PRESSURE_MODEL_DT_S * PRESSURE_MODEL_DT_S /
                      (params->line_inertance_pa_s2_per_m3 *
                       (params->outlet_volume_m3 < params->chamber_volume_m3
                            ? params->outlet_volume_m3
                            : params->chamber_volume_m3));
    return isfinite(stiffness_ratio) && stiffness_ratio <= 0.25f;
}

static int pressure_model_physical_pressure_increment_is_safe(
    const PressureModelParams *params) {
    float max_abs_rpm;
    float min_volume_m3;
    float peak_pump_flow_m3_s;
    float max_net_flow_m3_s;
    float pressure_increment_pa;

    max_abs_rpm = pressure_model_maxf(pressure_model_absf(params->min_rpm),
                                      pressure_model_absf(params->max_rpm));
    min_volume_m3 = params->physical.outlet_volume_m3 < params->physical.chamber_volume_m3
                        ? params->physical.outlet_volume_m3
                        : params->physical.chamber_volume_m3;
    peak_pump_flow_m3_s = params->pump_displacement_m3_rev * max_abs_rpm / 60.0f *
                          PRESSURE_MODEL_PHYSICAL_MAX_FLOW_RIPPLE_FACTOR;
    max_net_flow_m3_s = peak_pump_flow_m3_s + PRESSURE_MODEL_PHYSICAL_MAX_FLOW_M3_S;
    pressure_increment_pa = params->physical.beta_oil_pa / min_volume_m3 *
                            max_net_flow_m3_s * PRESSURE_MODEL_DT_S;
    return isfinite(pressure_increment_pa) &&
           pressure_increment_pa <= PRESSURE_MODEL_PHYSICAL_MAX_PRESSURE_INCREMENT_PA;
}

static float pressure_model_physical_max_net_flow(const PressureModelParams *params) {
    float max_abs_rpm = pressure_model_maxf(pressure_model_absf(params->min_rpm),
                                            pressure_model_absf(params->max_rpm));

    return params->pump_displacement_m3_rev * max_abs_rpm / 60.0f *
               PRESSURE_MODEL_PHYSICAL_MAX_FLOW_RIPPLE_FACTOR +
           PRESSURE_MODEL_PHYSICAL_MAX_FLOW_M3_S;
}

static int pressure_model_physical_state_is_admissible(
    const PressureModelParams *params,
    const PressureModelState *state) {
    return state->motor_rpm >= params->min_rpm &&
           state->motor_rpm <= params->max_rpm &&
           state->pressure_pa >= 0.0f &&
           state->outlet_pressure_pa >= 0.0f &&
           pressure_model_absf(state->motor_accel_rpm_s) <=
               params->physical.motor_accel_limit_rpm_s &&
           state->pump_phase_rev >= 0.0f && state->pump_phase_rev < 1.0f &&
           state->motor_delay_index < PRESSURE_MODEL_PHYSICAL_DELAY_SLOTS &&
           state->sensor_delay_index < PRESSURE_MODEL_PHYSICAL_DELAY_SLOTS &&
           pressure_model_absf(state->line_flow_m3_s) <=
               pressure_model_physical_max_net_flow(params);
}

static int pressure_model_validate_runtime_params(const PressureModelParams *params) {
    return params != NULL &&
           PressureModel_ValidatePhysicalParams(&params->physical) &&
           pressure_model_is_finite_positive(params->pump_displacement_m3_rev) &&
           params->pump_displacement_m3_rev <=
               PRESSURE_MODEL_PHYSICAL_MAX_PUMP_DISPLACEMENT_M3_REV &&
           isfinite(params->min_rpm) &&
           isfinite(params->max_rpm) &&
           params->min_rpm >= -PRESSURE_MODEL_PHYSICAL_MAX_ABS_RPM &&
           params->max_rpm <= PRESSURE_MODEL_PHYSICAL_MAX_ABS_RPM &&
           params->min_rpm <= params->max_rpm &&
           params->min_rpm <= 0.0f &&
           params->max_rpm >= 0.0f &&
           pressure_model_is_finite_positive(params->sensor_range_bar) &&
           pressure_model_is_finite_nonnegative(params->sensor_noise_std_bar) &&
           isfinite(params->sensor_bias_bar) &&
           pressure_model_physical_pressure_increment_is_safe(params);
}

int PressureModel_ValidateParams(const PressureModelParams *params) {
    return pressure_model_validate_runtime_params(params);
}

int PressureModel_ValidateInput(const PressureModelParams *params,
                                const PressureModelInput *input) {
    return PressureModel_ValidateParams(params) && input != NULL &&
           isfinite(input->target_rpm) && isfinite(input->load_flow_m3_s) &&
           input->target_rpm >= params->min_rpm &&
           input->target_rpm <= params->max_rpm &&
           pressure_model_absf(input->load_flow_m3_s) <=
               PRESSURE_MODEL_PHYSICAL_MAX_FLOW_M3_S;
}

void PressureModel_InitParams(PressureModelParams *params) {
    PressureModelPhysicalParams *physical;

    if (params == NULL) return;
    memset(params, 0, sizeof(*params));
    /* Servo pump: 25 cm^3/rev, rated 1700 rpm (user-confirmed 2026-09-16).
     * Rated flow = 25e-6 * 1700 / 60 = 7.083e-4 m^3/s = 42.5 L/min. */
    params->pump_displacement_m3_rev = 25.0e-6f;
    params->bulk_modulus_pa = 1.6e9f;
    params->chamber_volume_m3 = 5.0e-4f;
    params->leak_coeff_m3_pa_s = 8.333333e-13f;
    params->relief_set_pa = 25.0e6f;
    params->relief_coeff_m3_pa_s = 1.2e-9f;
    params->sensor_range_bar = 250.0f;
    params->sensor_noise_std_bar = 0.4f;
    params->motor_tau_s = 0.06f;
    params->motor_noise_std_rpm = 2.0f;
    params->flow_ripple_ratio = 0.08f;
    params->tooth_drop_depth_ratio = 0.10f;
    params->tooth_drop_width_ratio = 0.20f;
    params->min_rpm = -100.0f;
    params->max_rpm = 1700.0f;   /* 额定转速；模型校验上限仍为 2000 rpm */
    params->enable_sensor_noise = 1u;
    params->enable_motor_noise = 1u;

    /* ------------------------------------------------------------------
     * Plant model selection (校准 2026-09-16).
     *
     * The physical-calibrated branch is the DEFAULT: the first-order lag was
     * only ever valid for the pressure-hold case, while the library must also
     * serve charge / load-clamped segments. Production code therefore runs
     * dP/dt = (beta_e/V) * (Q_pump(n,P) - Q_load - Q_leak(P)) with no load
     * clamp on the plant itself.
     *
     * PRESSURE_MODEL_TYPE_FIRST_ORDER is kept as an explicit opt-in for the
     * legacy regression suite (tests set params.model_type themselves); it is
     * never selected implicitly.
     * ------------------------------------------------------------------ */
    params->model_type = PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED;

    /* Retained for the opt-in first-order branch. Set to the same Ksys as the
     * calibration below so the two branches cannot disagree about the machine. */
    params->first_order_k_bar_per_rpm = 5.0f;
    params->first_order_tau_s = 1.0f;
    params->first_order_delay_s = 0.0f;

    physical = &params->physical;
    physical->atmospheric_pressure_pa = 101325.0f;
    physical->suction_pressure_pa = 101325.0f;
    physical->outlet_volume_m3 = 5.0e-4f;
    physical->chamber_volume_m3 = 5.0e-4f;
    /* ------------------------------------------------------------------
     * Pressure line calibration -- L = 3.5 m (user-supplied estimate
     * 2026-09-16: "管道长度未知，需实际测量，大概 3 米或 4 米"; 3.5 m is the
     * midpoint and will be replaced once measured).
     *
     * Derived from geometry, NOT hand-tuned:
     *   inertance  I = rho * L / A                 [Pa*s^2/m^3]
     *   laminar R  = 128 * mu * L / (pi * d^4)     [Pa*s/m^3]
     * with ISO VG46 at 40 degC (rho = 870 kg/m^3, mu = 0.041 Pa*s) and an
     * assumed bore d = 12 mm (the pump outlet port size; the only assumption
     * left, and the one real measurement will fix):
     *   A = 1.131e-4 m^2
     *   I = 870 * 3.5 / 1.131e-4        = 2.69e7  Pa*s^2/m^3
     *   R = 128*0.041*3.5/(pi*0.012^4)  = 2.82e8  Pa*s/m^3
     *
     * Sensitivity on the bore (L = 3.5 m fixed):
     *   d=10 mm -> I=3.88e7, R=5.85e8, line resonance 56 Hz
     *   d=12 mm -> I=2.69e7, R=2.82e8, line resonance 67 Hz   <-- adopted
     *   d=16 mm -> I=1.51e7, R=8.92e7, line resonance 90 Hz
     * The line's LC frequency is 1/(2*pi*sqrt(I*C_series)); with the pump-side
     * and chamber capacitances in series C_series = 2.08e-13 m^3/Pa this lands
     * at 56..90 Hz, i.e. exactly the classic quarter-wave pipe resonance
     * a/(4L) = 1200/(4*3.5) = 86 Hz. Both the physics and the numbers agree.
     *
     * Why NOT the old quasi-static values (I=1.0e8, R=2.0e11):
     * the old pair was picked so dt*R = 2e8 > I = 1e8 made the line algebraic,
     * a numerical convenience rather than a physical statement. R = 2.0e11
     * implies a 1417 bar drop at the rated 42.5 L/min -- impossible -- and it
     * made the line carry no dynamics at all. Closed-loop measurement
     * (tests/test_boost_flow_limit + the 3.5 m sweep) shows the physical pair
     * is also the BETTER behaved one: residual overshoot 1.11% -> 0.13% and
     * hold-band pk-pk 1.43 -> 0.63 bar, with a bounded 0.63 bar ring.
     *
     * Numerical safety (both guards are in PressureModel_ValidatePhysicalParams):
     *   stiffness_ratio = beta*dt^2/(I*min(V)) = 2.4e6/I = 0.089 <= 0.25  OK
     *   zeta = R/(2*sqrt(I/C_series)) = 0.0124 (lightly damped, as real lines
     *   are); the explicit-Euler step is stable (omega*dt = 0.42 < 2) and the
     *   observed ring stays bounded. If measurement later gives a different L,
     *   re-run the two formulas above -- the guards will flag an unsafe pair.
     * ------------------------------------------------------------------ */
    physical->line_inertance_pa_s2_per_m3 = 2.69e7f;
    physical->line_resistance_pa_s_per_m3 = 2.82e8f;
    physical->line_quadratic_resistance_pa_s2_per_m6 = 0.0f;
    physical->beta_oil_pa = 1.2e9f;
    physical->gas_fraction = 0.002f;
    physical->gas_transition_pa = 1.0e6f;
    physical->beta_min_pa = 5.0e7f;

    /* ------------------------------------------------------------------
     * Leakage calibration -> open-loop DC gain Ksys = 5.0 bar/rpm.
     *
     * Machine under calibration (user-confirmed 2026-09-16):
     *   pump displacement 25 cm^3/rev, rated speed 1700 rpm
     *   -> rated flow 25e-6*1700/60 = 7.083e-4 m^3/s (42.5 L/min)
     *
     * Ksys is the machine constant the process layer already knows:
     *   P_chamber[bar] = Ksys * n[rpm]   (valid up to the relief valve)
     * and it is the direct inverse of the lumped leakage characteristic
     * ("K = 稳态压力/流量"), which is exactly the K the RBF-PID feedforward
     * P_set/K consumes. Calibrating the plant to it makes the simulated
     * steady state and the controller's own model agree by construction.
     *
     * Do NOT hand-pick the coefficients: the line resistance makes the pump
     * outlet pressure higher than the chamber pressure, which scales BOTH the
     * pump leak and the outlet leak. Ignoring the R term under-estimates the
     * gain (it cost ~29% back when R was the unphysical 2.0e11, where
     * c_cyl*R = 4.2e-2; with the physical R = 2.82e8 it is only 5.6e-5, but the
     * closed form below stays exact either way and re-derives itself when L,
     * d or Ksys change).
     *
     * Steady state, zero load flow:
     *   Q_line   = C_cyl * P_c
     *   P_out    = P_c + Q_line * R = P_c * (1 + C_cyl*R)
     *   Q_pump   = Q_line + C_out * P_out
     *   Q_pump   = D*n/60 - C_pump * P_out          (volumetric loss)
     * =>
     *   D/60 = P_c * [ C_cyl + (C_pump + C_out) * (1 + C_cyl*R) ]
     * => with P_c = Ksys*1e5*n the bracket is fixed; C_pump is set from a
     *    stated volumetric efficiency on that same line, and C_out is solved.
     *
     * eta_v on the DC line = 1 - C_pump*P_out/(D*n/60). Sizing C_pump from
     * eta_v_dc_target keeps it comfortably above eta_v_min = 0.60; if the floor
     * were reached the realised gain would silently drop (e.g. to 3.0 bar/rpm)
     * while the code still looked calibrated.
     *
     * pump_leak_speed is zeroed on purpose: a speed-dependent term would make
     * Ksys drift with rpm and break the constant-gain calibration.
     * ------------------------------------------------------------------ */
    {
        const float ksys_bar_per_rpm = 5.0f;
        const float eta_v_dc_target = 0.75f;   /* volumetric eff. on the DC line */
        const float c_cyl = 2.0e-13f;
        const float r_line = physical->line_resistance_pa_s_per_m3;
        const float q_per_rpm = params->pump_displacement_m3_rev / 60.0f;
        const float p_c_per_rpm_pa = ksys_bar_per_rpm * PRESSURE_MODEL_PA_PER_BAR;
        const float p_o_per_rpm_pa = p_c_per_rpm_pa * (1.0f + c_cyl * r_line);
        /* Bracket in the derivation above; C_out is what is left to solve. */
        const float bracket = q_per_rpm / p_c_per_rpm_pa;
        const float c_pump = (1.0f - eta_v_dc_target) * q_per_rpm / p_o_per_rpm_pa;

        physical->pump_leak_c0_m3_pa_s = c_pump;
        physical->cylinder_leak_m3_pa_s = c_cyl;
        physical->outlet_leak_m3_pa_s =
            (bracket - c_cyl) / (1.0f + c_cyl * r_line) - c_pump;
    }
    physical->pump_leak_speed_m3_pa_s_per_rpm = 0.0f;
    physical->eta_v_min = 0.60f;
    physical->eta_m_nominal = 0.90f;
    physical->eta_m_pressure_loss_per_pa = 1.0e-9f;
    physical->eta_m_speed_loss_per_rpm = 1.0e-5f;
    physical->eta_m_min = 0.50f;
    physical->rated_motor_torque_nm = 150.0f;
    physical->torque_ripple13_peak = 0.02f;
    physical->ripple13_peak = 0.08f;
    physical->ripple26_peak = 0.03f;
    physical->ripple39_peak = 0.01f;
    physical->motor_natural_freq_hz = 12.0f;
    physical->motor_damping = 1.0f;
    physical->motor_accel_limit_rpm_s = 20000.0f;
    physical->motor_torque_limit_permille = 1000.0f;
    physical->relief_set_pa = 25.0e6f;
    physical->relief_deadband_pa = 0.2e6f;
    physical->relief_orifice_coeff_m3_s_sqrt_pa = 1.0e-8f;
    physical->relief_hysteresis_pa = 0.5e6f;
}

void PressureModel_Reset(PressureModelState *state, uint32_t seed) {
    if (state == NULL) return;
    memset(state, 0, sizeof(*state));
    state->rng_state = pressure_model_seed(seed);
    state->active_model_type = PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED;
}

static void pressure_model_fill_first_order_history(PressureModelState *state,
                                                    float pressure_bar) {
    int i;
    for (i = 0; i < PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS; ++i) {
        state->first_order_delay_buffer[i] = pressure_bar;
    }
    state->first_order_buffer_index = 0;
}

static void pressure_model_step_first_order(const PressureModelParams *params,
                                            PressureModelState *state,
                                            float target_rpm,
                                            float dt_s,
                                            PressureModelOutput *out) {
    float tau = pressure_model_maxf(params->first_order_tau_s, 0.0f);
    float gain = pressure_model_maxf(params->first_order_k_bar_per_rpm, 0.0f);
    float undelayed;
    float delayed;
    int delay_steps;

    state->motor_rpm = isfinite(target_rpm)
                           ? pressure_model_clampf(target_rpm, params->min_rpm, params->max_rpm)
                           : 0.0f;
    if (tau > 0.0f) {
        undelayed = (gain * state->motor_rpm * dt_s +
                     tau * state->first_order_prev_pressure_bar) / (tau + dt_s);
    } else {
        undelayed = gain * state->motor_rpm;
    }
    undelayed = pressure_model_clampf(undelayed, 0.0f, 250.0f);
    delay_steps = (int)(pressure_model_clampf(params->first_order_delay_s, 0.0f, 1.0f) /
                        dt_s);
    if (delay_steps >= PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS) {
        delay_steps = PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS - 1;
    }
    state->first_order_delay_buffer[state->first_order_buffer_index] = undelayed;
    delayed = delay_steps == 0
                  ? undelayed
                  : state->first_order_delay_buffer[
                        (state->first_order_buffer_index +
                         PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS - delay_steps) %
                        PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS];
    state->first_order_buffer_index =
        (state->first_order_buffer_index + 1) % PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS;
    state->first_order_prev_pressure_bar = undelayed;
    state->pressure_pa = delayed * PRESSURE_MODEL_PA_PER_BAR;
    out->actual_motor_rpm = state->motor_rpm;
    out->real_pressure_bar = delayed;
    out->measured_pressure_bar = delayed;
    out->pump_flow_m3_s = 0.0f;
    out->net_flow_m3_s = 0.0f;
    out->relief_flow_m3_s = 0.0f;
    out->relief_active = undelayed >= 250.0f;
    out->estimated_torque_trend = 0.0f;
    pressure_model_fill_feedback(state, out, 0, 0.0f);
}

static void pressure_model_write_hold_output(const PressureModelState *state,
                                             PressureModelOutput *out) {
    float pressure_bar = isfinite(state->pressure_pa) ? state->pressure_pa / PRESSURE_MODEL_PA_PER_BAR
                                                       : 0.0f;
    out->actual_motor_rpm = isfinite(state->motor_rpm) ? state->motor_rpm : 0.0f;
    out->real_pressure_bar = pressure_bar;
    out->measured_pressure_bar = pressure_bar;
    out->pump_flow_m3_s = 0.0f;
    out->net_flow_m3_s = 0.0f;
    out->relief_flow_m3_s = 0.0f;
    out->relief_active = 0;
    out->estimated_torque_trend = 0.0f;
    pressure_model_fill_feedback(state, out, 0, 0.0f);
}

static float pressure_model_order_waveform(float phase) {
    /*
     * A zero-mean, unit-peak sinusoid keeps each enabled term at exactly its
     * named order.  Asymmetric Fourier content requires separately gated
     * harmonic terms and is deferred until those calibration parameters exist.
     */
    return sinf(phase);
}

static int pressure_model_step_physical_substep(const PressureModelParams *params,
                                                 PressureModelState *state,
                                                 float target_rpm,
                                                 float load_flow_m3_s,
                                                 PressureModelOutput *out) {
    const PressureModelPhysicalParams *p = &params->physical;
    float delayed_target;
    float wn;
    float accel_derivative;
    float acceleration;
    float rpm;
    float abs_rpm;
    float phase13;
    float phase26;
    float phase39;
    float ripple;
    float p_out_abs;
    float p_chamber_abs;
    float delta_p_pump;
    float q_theoretical;
    float q_pump_leak;
    float eta_v;
    float q_pump;
    float q_outlet_leak;
    float q_cylinder_leak;
    float q_relief = 0.0f;
    float line_denominator;
    float beta_outlet;
    float beta_chamber;
    float q_outlet_net;
    float q_chamber_net;
    float eta_m;
    float eta_m_unclamped;
    float torque_nm;
    float torque_raw_permille;
    float torque_permille;
    float line_numerator;
    float outlet_pressure_delta;
    float chamber_pressure_delta;
    float next_outlet_pressure;
    float next_pressure;
    int torque_valid;
    int order13_active;
    int order26_active;
    int order39_active;

    delayed_target = pressure_model_delay_write_read(
        state->motor_delay_ring, &state->motor_delay_index,
        pressure_model_delay_steps(p->motor_delay_s),
        pressure_model_clampf(target_rpm, params->min_rpm, params->max_rpm));
    wn = 2.0f * PRESSURE_MODEL_PI * p->motor_natural_freq_hz;
    accel_derivative = wn * wn * (delayed_target - state->motor_rpm) -
                       2.0f * p->motor_damping * wn * state->motor_accel_rpm_s;
    acceleration = state->motor_accel_rpm_s + PRESSURE_MODEL_DT_S * accel_derivative;
    acceleration = pressure_model_clampf(acceleration,
                                         -p->motor_accel_limit_rpm_s,
                                         p->motor_accel_limit_rpm_s);
    state->motor_accel_rpm_s = acceleration;
    state->motor_rpm += acceleration * PRESSURE_MODEL_DT_S;
    state->motor_rpm = pressure_model_clampf(state->motor_rpm,
                                              params->min_rpm, params->max_rpm);
    state->pump_phase_rev = pressure_model_wrap_unit(
        state->pump_phase_rev + state->motor_rpm * PRESSURE_MODEL_DT_S / 60.0f);
    rpm = state->motor_rpm;
    abs_rpm = pressure_model_absf(rpm);
    phase13 = 2.0f * PRESSURE_MODEL_PI * 13.0f * state->pump_phase_rev;
    phase26 = 2.0f * PRESSURE_MODEL_PI * 26.0f * state->pump_phase_rev;
    phase39 = 2.0f * PRESSURE_MODEL_PI * 39.0f * state->pump_phase_rev;
    order13_active = 13.0f * abs_rpm / 60.0f < 0.45f / PRESSURE_MODEL_DT_S;
    order26_active = 26.0f * abs_rpm / 60.0f < 0.45f / PRESSURE_MODEL_DT_S;
    order39_active = 39.0f * abs_rpm / 60.0f < 0.45f / PRESSURE_MODEL_DT_S;
    ripple = 1.0f;
    if (order13_active) ripple += p->ripple13_peak *
        pressure_model_order_waveform(phase13 + p->ripple13_phase_rad);
    if (order26_active) ripple += p->ripple26_peak *
        pressure_model_order_waveform(phase26 + p->ripple26_phase_rad);
    if (order39_active) ripple += p->ripple39_peak *
        pressure_model_order_waveform(phase39 + p->ripple39_phase_rad);
    ripple = pressure_model_clampf(ripple, 0.20f,
                                   PRESSURE_MODEL_PHYSICAL_MAX_FLOW_RIPPLE_FACTOR);

    p_out_abs = p->atmospheric_pressure_pa + pressure_model_maxf(state->outlet_pressure_pa, 0.0f);
    p_chamber_abs = p->atmospheric_pressure_pa + pressure_model_maxf(state->pressure_pa, 0.0f);
    delta_p_pump = pressure_model_maxf(p_out_abs - p->suction_pressure_pa, 0.0f);
    q_theoretical = params->pump_displacement_m3_rev * abs_rpm / 60.0f;
    q_pump_leak = (p->pump_leak_c0_m3_pa_s +
                   p->pump_leak_speed_m3_pa_s_per_rpm * abs_rpm) * delta_p_pump;
    eta_v = pressure_model_clampf(1.0f - q_pump_leak /
                                  pressure_model_maxf(q_theoretical, PRESSURE_MODEL_EPS_FLOW),
                                  p->eta_v_min, 1.0f);
    q_pump = pressure_model_signf(rpm) * q_theoretical * eta_v * ripple;
    q_outlet_leak = p->outlet_leak_m3_pa_s * delta_p_pump;
    q_cylinder_leak = p->cylinder_leak_m3_pa_s *
                      pressure_model_maxf(p_chamber_abs - p->suction_pressure_pa, 0.0f);
    if (!isfinite(p_out_abs) || !isfinite(p_chamber_abs) ||
        !isfinite(delta_p_pump) || !isfinite(q_theoretical) ||
        !isfinite(q_pump_leak) || !isfinite(eta_v) || !isfinite(q_pump) ||
        !isfinite(q_outlet_leak) || !isfinite(q_cylinder_leak)) {
        return 0;
    }

    if (!state->relief_latched &&
        state->outlet_pressure_pa >= p->relief_set_pa + p->relief_deadband_pa) {
        state->relief_latched = 1u;
    } else if (state->relief_latched &&
               state->outlet_pressure_pa <= p->relief_set_pa - p->relief_hysteresis_pa) {
        state->relief_latched = 0u;
    }
    if (state->relief_latched) {
        q_relief = p->relief_orifice_coeff_m3_s_sqrt_pa *
                   sqrtf(pressure_model_maxf(state->outlet_pressure_pa -
                                              (p->relief_set_pa + p->relief_deadband_pa),
                                              0.0f));
    }
    if (!isfinite(q_relief)) return 0;
    line_denominator = p->line_inertance_pa_s2_per_m3 + PRESSURE_MODEL_DT_S *
                       (p->line_resistance_pa_s_per_m3 +
                        p->line_quadratic_resistance_pa_s2_per_m6 *
                            pressure_model_absf(state->line_flow_m3_s));
    line_numerator = p->line_inertance_pa_s2_per_m3 * state->line_flow_m3_s +
                     PRESSURE_MODEL_DT_S *
                         (state->outlet_pressure_pa - state->pressure_pa);
    if (!isfinite(line_denominator) || line_denominator <= 0.0f ||
        !isfinite(line_numerator)) {
        return 0;
    }
    state->line_flow_m3_s = line_numerator / line_denominator;
    if (!isfinite(state->line_flow_m3_s)) return 0;
    if (!pressure_model_physical_state_is_admissible(params, state)) return 0;
    beta_outlet = PressureModel_EffectiveBulkModulusPa(p, p_out_abs);
    q_outlet_net = q_pump - state->line_flow_m3_s - q_outlet_leak - q_relief;
    if (!isfinite(beta_outlet) || !isfinite(q_outlet_net)) return 0;
    outlet_pressure_delta = beta_outlet / p->outlet_volume_m3 *
                            q_outlet_net * PRESSURE_MODEL_DT_S;
    next_outlet_pressure = state->outlet_pressure_pa + outlet_pressure_delta;
    if (!isfinite(outlet_pressure_delta) || !isfinite(next_outlet_pressure) ||
        pressure_model_absf(outlet_pressure_delta) >
            PRESSURE_MODEL_PHYSICAL_MAX_PRESSURE_INCREMENT_PA) {
        return 0;
    }
    state->outlet_pressure_pa = pressure_model_maxf(0.0f, next_outlet_pressure);
    p_chamber_abs = p->atmospheric_pressure_pa + pressure_model_maxf(state->pressure_pa, 0.0f);
    beta_chamber = PressureModel_EffectiveBulkModulusPa(p, p_chamber_abs);
    q_chamber_net = state->line_flow_m3_s - load_flow_m3_s - q_cylinder_leak;
    if (!isfinite(p_chamber_abs) || !isfinite(beta_chamber) ||
        !isfinite(q_chamber_net)) {
        return 0;
    }
    chamber_pressure_delta = beta_chamber / p->chamber_volume_m3 *
                             q_chamber_net * PRESSURE_MODEL_DT_S;
    next_pressure = state->pressure_pa + chamber_pressure_delta;
    if (!isfinite(chamber_pressure_delta) || !isfinite(next_pressure) ||
        pressure_model_absf(chamber_pressure_delta) >
            PRESSURE_MODEL_PHYSICAL_MAX_PRESSURE_INCREMENT_PA) {
        return 0;
    }
    state->pressure_pa = pressure_model_maxf(0.0f, next_pressure);

    out->actual_motor_rpm = rpm;
    out->real_pressure_bar = state->pressure_pa / PRESSURE_MODEL_PA_PER_BAR;
    out->measured_pressure_bar = pressure_model_delay_write_read(
        state->sensor_delay_ring, &state->sensor_delay_index,
        pressure_model_delay_steps(p->sensor_delay_s), out->real_pressure_bar);
    if (p->sensor_quantization_bar > 0.0f) {
        out->measured_pressure_bar = roundf(out->measured_pressure_bar /
                                            p->sensor_quantization_bar) *
                                     p->sensor_quantization_bar;
    }
    out->measured_pressure_bar += params->sensor_bias_bar;
    if (params->enable_sensor_noise) {
        out->measured_pressure_bar += pressure_model_gaussian(state, params->sensor_noise_std_bar);
    }
    out->measured_pressure_bar = pressure_model_clampf(out->measured_pressure_bar,
                                                        0.0f, params->sensor_range_bar);
    out->pump_flow_m3_s = q_pump;
    out->net_flow_m3_s = q_chamber_net;
    out->relief_flow_m3_s = q_relief;
    out->active_order_mask = 0u;
    if (order13_active) out->active_order_mask |= PRESSURE_MODEL_ORDER_13_ACTIVE;
    if (order26_active) out->active_order_mask |= PRESSURE_MODEL_ORDER_26_ACTIVE;
    if (order39_active) out->active_order_mask |= PRESSURE_MODEL_ORDER_39_ACTIVE;
    out->relief_active = state->relief_latched != 0u;
    out->estimated_torque_trend = 0.0f;

    eta_m_unclamped = p->eta_m_nominal -
                      p->eta_m_pressure_loss_per_pa * delta_p_pump -
                      p->eta_m_speed_loss_per_rpm * abs_rpm;
    if (!isfinite(eta_m_unclamped)) return 0;
    eta_m = pressure_model_clampf(eta_m_unclamped, p->eta_m_min, 1.0f);
    torque_nm = pressure_model_signf(rpm) * delta_p_pump * params->pump_displacement_m3_rev /
                (2.0f * PRESSURE_MODEL_PI * eta_m);
    torque_nm *= 1.0f + p->torque_ripple13_peak *
                 sinf(phase13 + p->torque_ripple13_phase_rad);
    torque_raw_permille = 1000.0f * torque_nm / p->rated_motor_torque_nm;
    if (!isfinite(torque_nm) || !isfinite(torque_raw_permille)) return 0;
    torque_permille = pressure_model_clampf(torque_raw_permille,
                                            -p->motor_torque_limit_permille,
                                            p->motor_torque_limit_permille);
    torque_valid = isfinite(torque_permille) && isfinite(phase13) &&
                   isfinite(delta_p_pump) && isfinite(eta_m) && p->rated_motor_torque_nm > 0.0f;
    pressure_model_fill_feedback(state, out, torque_valid, torque_permille);
    return 1;
}

void PressureModel_StepInput(const PressureModelParams *params,
                             PressureModelState *state,
                             const PressureModelInput *input,
                             PressureModelOutput *out) {
    unsigned char requested_type;
    float dt_s;
    float pressure_bar;
    PressureModelState call_start;
    PressureModelInput effective;
    int substeps;
    int i;

    if (params == NULL || state == NULL || input == NULL || out == NULL) return;
    memset(out, 0, sizeof(*out));
    dt_s = input->dt_s;
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.004f ||
        fabsf(dt_s / PRESSURE_MODEL_DT_S - roundf(dt_s / PRESSURE_MODEL_DT_S)) > 1.0e-4f) {
        substeps = 1;
    } else {
        substeps = (int)roundf(dt_s / PRESSURE_MODEL_DT_S);
    }
    requested_type = pressure_model_normalize_type(params->model_type);
    if (!isfinite(input->target_rpm) || !isfinite(input->load_flow_m3_s) ||
        (requested_type == PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED &&
         !PressureModel_ValidateParams(params))) {
        state->timestamp_s += substeps * PRESSURE_MODEL_DT_S;
        pressure_model_write_hold_output(state, out);
        return;
    }

    /* ------------------------------------------------------------------
     * Saturation, not rejection: the drive clamps an out-of-envelope command.
     *
     * A target speed outside [min_rpm, max_rpm] means the commanded speed
     * exceeds the pump/ servo-drive envelope -- typically a reverse-flow
     * command larger than the drive's reverse limit. The documented design
     * clamps the target (see
     * docs/superpowers/plans/2026-06-12-pressure-model-closed-loop.md:
     * `clamped_target = clampf(target_rpm, min_rpm, max_rpm)`), and the
     * physical substep already clamps too.
     *
     * Rejecting it here instead held the ENTIRE plant state, which silently
     * FROZE the pressure: the motor kept ramping (its update precedes the
     * rejection point inside the substep) while pressure / line flow stopped
     * moving. Observed on the 25cc / 1700 rpm machine with
     * flowToPumpSpeedGain = 40 rpm/(L/min): Q_cmd = -5 L/min -> -200 rpm, below
     * min_rpm = -100, so the plant held at 151.58 bar with the pump commanded
     * to full reverse and never recovered.
     *
     * Clamping is both the documented behaviour and what a real servo drive
     * does. `out->command_saturated` reports it so the process layer can tell
     * "drive at its limit" apart from "pump/line fault".
     * ------------------------------------------------------------------ */
    effective = *input;
    if (requested_type == PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED) {
        const float max_load_flow = PRESSURE_MODEL_PHYSICAL_MAX_FLOW_M3_S;

        if (effective.target_rpm < params->min_rpm) {
            effective.target_rpm = params->min_rpm;
            out->command_saturated = 1;
        } else if (effective.target_rpm > params->max_rpm) {
            effective.target_rpm = params->max_rpm;
            out->command_saturated = 1;
        }
        if (effective.load_flow_m3_s > max_load_flow) {
            effective.load_flow_m3_s = max_load_flow;
            out->command_saturated = 1;
        } else if (effective.load_flow_m3_s < -max_load_flow) {
            effective.load_flow_m3_s = -max_load_flow;
            out->command_saturated = 1;
        }
    }

    if (requested_type == PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED &&
        (!pressure_model_physical_state_is_finite(state) ||
         !pressure_model_physical_state_is_admissible(params, state))) {
        state->timestamp_s += substeps * PRESSURE_MODEL_DT_S;
        pressure_model_write_hold_output(state, out);
        return;
    }
    if (requested_type == PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED) {
        call_start = *state;
    }
    pressure_bar = isfinite(state->pressure_pa) ? state->pressure_pa / PRESSURE_MODEL_PA_PER_BAR : 0.0f;
    if (requested_type != state->active_model_type) {
        if (pressure_bar <= 0.0f) {
            if (requested_type == PRESSURE_MODEL_TYPE_FIRST_ORDER) {
                state->first_order_prev_pressure_bar = pressure_bar;
                pressure_model_fill_first_order_history(state, pressure_bar);
            }
            state->active_model_type = requested_type;
        } else if (requested_type == PRESSURE_MODEL_TYPE_FIRST_ORDER) {
            state->first_order_prev_pressure_bar = pressure_bar;
            pressure_model_fill_first_order_history(state, pressure_bar);
            state->active_model_type = requested_type;
            state->timestamp_s += substeps * PRESSURE_MODEL_DT_S;
            pressure_model_write_hold_output(state, out);
            return;
        } else {
            state->outlet_pressure_pa = pressure_model_maxf(0.0f, state->outlet_pressure_pa);
            state->line_flow_m3_s = 0.0f;
            state->motor_accel_rpm_s = 0.0f;
            state->relief_latched = 0u;
            state->active_model_type = requested_type;
            state->timestamp_s += substeps * PRESSURE_MODEL_DT_S;
            pressure_model_write_hold_output(state, out);
            return;
        }
    }

    if (requested_type == PRESSURE_MODEL_TYPE_FIRST_ORDER) {
        state->timestamp_s += substeps * PRESSURE_MODEL_DT_S;
        pressure_model_step_first_order(params, state, effective.target_rpm,
                                        substeps * PRESSURE_MODEL_DT_S, out);
        return;
    }
    for (i = 0; i < substeps; ++i) {
        if (!pressure_model_physical_state_is_finite(state) ||
            !pressure_model_physical_state_is_admissible(params, state)) {
            *state = call_start;
            state->timestamp_s += substeps * PRESSURE_MODEL_DT_S;
            pressure_model_write_hold_output(state, out);
            return;
        }
        if (!pressure_model_step_physical_substep(params, state, effective.target_rpm,
                                                  effective.load_flow_m3_s,
                                                  out) ||
            !pressure_model_physical_state_is_finite(state)) {
            *state = call_start;
            state->timestamp_s += substeps * PRESSURE_MODEL_DT_S;
            pressure_model_write_hold_output(state, out);
            return;
        }
        state->timestamp_s += PRESSURE_MODEL_DT_S;
        pressure_model_fill_feedback(state, out,
            HYD_PumpFeedback_HasValid(out->pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TORQUE),
            out->pumpFeedback.torquePermille);
    }
}

void PressureModel_Step(const PressureModelParams *params,
                        PressureModelState *state,
                        float target_rpm,
                        float dt_s,
                        PressureModelOutput *out) {
    PressureModelInput input;

    input.target_rpm = target_rpm;
    input.load_flow_m3_s = 0.0f;
    input.dt_s = dt_s;
    PressureModel_StepInput(params, state, &input, out);
}

float pressure_update(float target_rpm,
                      float t,
                      float *P_state,
                      float *real_P,
                      float *actual_motor_rpm) {
    static PressureModelParams params;
    static PressureModelState state;
    static float last_t;
    static int initialized;
    PressureModelOutput out;
    float dt_s;

    if (!initialized) {
        PressureModel_InitParams(&params);
        PressureModel_Reset(&state, 0x2468ace1u);
        initialized = 1;
    }
    if (P_state != NULL) state.pressure_pa = *P_state;
    dt_s = t > last_t ? t - last_t : PRESSURE_MODEL_DT_S;
    last_t = t;
    PressureModel_Step(&params, &state, target_rpm, dt_s, &out);
    if (P_state != NULL) *P_state = state.pressure_pa;
    if (real_P != NULL) *real_P = out.real_pressure_bar;
    if (actual_motor_rpm != NULL) *actual_motor_rpm = out.actual_motor_rpm;
    return out.measured_pressure_bar;
}
