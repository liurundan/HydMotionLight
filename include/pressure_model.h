#ifndef PRESSURE_MODEL_H
#define PRESSURE_MODEL_H

#include <stdint.h>

#include "common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS 1000
#define PRESSURE_MODEL_PHYSICAL_MAX_DELAY_STEPS 64
#define PRESSURE_MODEL_PHYSICAL_DELAY_SLOTS (PRESSURE_MODEL_PHYSICAL_MAX_DELAY_STEPS + 1)

enum {
    PRESSURE_MODEL_ORDER_13_ACTIVE = 1u << 0,
    PRESSURE_MODEL_ORDER_26_ACTIVE = 1u << 1,
    PRESSURE_MODEL_ORDER_39_ACTIVE = 1u << 2
};

typedef enum {
    PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED = 0u,
    PRESSURE_MODEL_TYPE_PHYSICAL = PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED,
    PRESSURE_MODEL_TYPE_FIRST_ORDER = 1u
} PressureModelType;

typedef struct {
    float atmospheric_pressure_pa;
    float suction_pressure_pa;
    float outlet_volume_m3;
    float chamber_volume_m3;
    float line_inertance_pa_s2_per_m3;
    float line_resistance_pa_s_per_m3;
    float line_quadratic_resistance_pa_s2_per_m6;
    float beta_oil_pa;
    float gas_fraction;
    float gas_transition_pa;
    float beta_min_pa;
    float pump_leak_c0_m3_pa_s;
    float pump_leak_speed_m3_pa_s_per_rpm;
    float outlet_leak_m3_pa_s;
    float cylinder_leak_m3_pa_s;
    float eta_v_min;
    float eta_m_nominal;
    float eta_m_pressure_loss_per_pa;
    float eta_m_speed_loss_per_rpm;
    float eta_m_min;
    float rated_motor_torque_nm;
    float torque_ripple13_peak;
    float torque_ripple13_phase_rad;
    float ripple13_peak;
    float ripple26_peak;
    float ripple39_peak;
    float ripple13_phase_rad;
    float ripple26_phase_rad;
    float ripple39_phase_rad;
    float motor_natural_freq_hz;
    float motor_damping;
    float motor_delay_s;
    float motor_accel_limit_rpm_s;
    float motor_torque_limit_permille;
    float relief_set_pa;
    float relief_deadband_pa;
    float relief_orifice_coeff_m3_s_sqrt_pa;
    float relief_hysteresis_pa;
    float sensor_delay_s;
    float sensor_quantization_bar;
} PressureModelPhysicalParams;

typedef struct {
    float target_rpm;
    float load_flow_m3_s;
    float dt_s;
} PressureModelInput;

typedef struct {
    /* Compatibility fields retained for existing first-order callers. */
    float pump_displacement_m3_rev;
    float bulk_modulus_pa;
    float chamber_volume_m3;
    float leak_coeff_m3_pa_s;
    float relief_set_pa;
    float relief_coeff_m3_pa_s;
    float sensor_range_bar;
    float sensor_noise_std_bar;
    float sensor_bias_bar;
    float motor_tau_s;
    float motor_noise_std_rpm;
    float process_noise_std_m3_s;
    float flow_ripple_ratio;
    float tooth_drop_depth_ratio;
    float tooth_drop_width_ratio;
    float min_rpm;
    float max_rpm;
    unsigned char enable_sensor_noise;
    unsigned char enable_motor_noise;
    unsigned char enable_process_noise;
    float veff_base_m3;
    float leak_base_m3_pa_s;
    float tooth_drop_depth_base;
    float tooth_drop_phase_base;
    float veff_speed_scale[3];
    float leak_speed_scale[3];
    float drop_depth_scale[3];
    float drop_phase_offset[3];
    float torque_bias;
    float torque_from_pressure_gain;
    float torque_from_speed_gain;
    unsigned char model_type;
    float first_order_k_bar_per_rpm;
    float first_order_tau_s;
    float first_order_delay_s;
    PressureModelPhysicalParams physical;
} PressureModelParams;

typedef struct {
    float motor_rpm;
    float pressure_pa;
    float pump_phase_rev;
    float timestamp_s;
    uint32_t rng_state;
    int has_spare_gauss;
    float spare_gauss;
    unsigned char active_model_type;
    float first_order_prev_pressure_bar;
    int first_order_buffer_index;
    float first_order_delay_buffer[PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS];
    float outlet_pressure_pa;
    float line_flow_m3_s;
    float motor_accel_rpm_s;
    unsigned char relief_latched;
    unsigned char motor_delay_index;
    unsigned char sensor_delay_index;
    float motor_delay_ring[PRESSURE_MODEL_PHYSICAL_DELAY_SLOTS];
    float sensor_delay_ring[PRESSURE_MODEL_PHYSICAL_DELAY_SLOTS];
} PressureModelState;

typedef struct {
    float measured_pressure_bar;
    float real_pressure_bar;
    float actual_motor_rpm;
    float pump_flow_m3_s;
    float net_flow_m3_s;
    int relief_active;
    float estimated_torque_trend;
    HYD_PumpFeedback pumpFeedback;
    float relief_flow_m3_s;
    uint8_t active_order_mask;
} PressureModelOutput;

void PressureModel_InitParams(PressureModelParams *params);
void PressureModel_Reset(PressureModelState *state, uint32_t seed);
int PressureModel_ValidatePhysicalParams(const PressureModelPhysicalParams *params);
float PressureModel_EffectiveBulkModulusPa(const PressureModelPhysicalParams *params,
                                           float absolute_pressure_pa);
void PressureModel_StepInput(const PressureModelParams *params,
                             PressureModelState *state,
                             const PressureModelInput *input,
                             PressureModelOutput *out);
void PressureModel_Step(const PressureModelParams *params,
                        PressureModelState *state,
                        float target_rpm,
                        float dt_s,
                        PressureModelOutput *out);
float pressure_update(float target_rpm,
                      float t,
                      float *P_state,
                      float *real_P,
                      float *actual_motor_rpm);

#ifdef __cplusplus
}
#endif

#endif /* PRESSURE_MODEL_H */
