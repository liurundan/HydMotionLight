#ifndef HYD_PRESSURE_RIPPLE_TABLE_H
#define HYD_PRESSURE_RIPPLE_TABLE_H

#ifndef HYD_PRESSURE_RIPPLE_ENTRY_DEFINED
#define HYD_PRESSURE_RIPPLE_ENTRY_DEFINED
typedef struct {
    float rpm;
    float amp13_rpm;
    float phase13_rad;
    float amp26_rpm;
    float phase26_rad;
} HYD_PressureRippleEntry;
#endif

/*
 * The current physical artifact is intentionally uncalibrated. This
 * placeholder keeps the production include stable without injecting data.
 */
#define HYD_PRESSURE_RIPPLE_TABLE_CALIBRATED 0
#define HYD_PRESSURE_RIPPLE_TABLE_COUNT 0
static const HYD_PressureRippleEntry HYD_PRESSURE_RIPPLE_TABLE[1] = {{0}};

#endif /* HYD_PRESSURE_RIPPLE_TABLE_H */
