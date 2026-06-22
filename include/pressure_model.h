#ifndef PRESSURE_MODEL_H
#define PRESSURE_MODEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS 1000

typedef enum {
    PRESSURE_MODEL_TYPE_PHYSICAL = 0u,
    PRESSURE_MODEL_TYPE_FIRST_ORDER = 1u
} PressureModelType;

typedef struct {
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
} PressureModelParams;

typedef struct {
    float motor_rpm;
    float pressure_pa;
    float pump_phase_rev;
    uint32_t rng_state;
    int has_spare_gauss;
    float spare_gauss;
    unsigned char active_model_type;
    float first_order_prev_pressure_bar;
    int first_order_buffer_index;
    float first_order_delay_buffer[PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS];
} PressureModelState;

typedef struct {
    float measured_pressure_bar;
    float real_pressure_bar;
    float actual_motor_rpm;
    float pump_flow_m3_s;
    float net_flow_m3_s;
    int relief_active;
    float estimated_torque_trend;
} PressureModelOutput;

void PressureModel_InitParams(PressureModelParams *params);
void PressureModel_Reset(PressureModelState *state, uint32_t seed);
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
