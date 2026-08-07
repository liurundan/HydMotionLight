#include <errno.h>
#include <ctype.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pressure_model.h"

typedef struct {
    uint32_t state[8];
    uint64_t bit_count;
    unsigned char buffer[64];
    size_t buffer_len;
} Sha256;

static uint32_t sha_rotr(uint32_t x, unsigned n) { return (x >> n) | (x << (32u - n)); }
static void sha_transform(Sha256 *sha, const unsigned char *data) {
    static const uint32_t k[64] = {
        0x428a2f98u,0x71374491u,0xb5c0fbcfu,0xe9b5dba5u,0x3956c25bu,0x59f111f1u,
        0x923f82a4u,0xab1c5ed5u,0xd807aa98u,0x12835b01u,0x243185beu,0x550c7dc3u,
        0x72be5d74u,0x80deb1feu,0x9bdc06a7u,0xc19bf174u,0xe49b69c1u,0xefbe4786u,
        0x0fc19dc6u,0x240ca1ccu,0x2de92c6fu,0x4a7484aau,0x5cb0a9dcu,0x76f988dau,
        0x983e5152u,0xa831c66du,0xb00327c8u,0xbf597fc7u,0xc6e00bf3u,0xd5a79147u,
        0x06ca6351u,0x14292967u,0x27b70a85u,0x2e1b2138u,0x4d2c6dfcu,0x53380d13u,
        0x650a7354u,0x766a0abbu,0x81c2c92eu,0x92722c85u,0xa2bfe8a1u,0xa81a664bu,
        0xc24b8b70u,0xc76c51a3u,0xd192e819u,0xd6990624u,0xf40e3585u,0x106aa070u,
        0x19a4c116u,0x1e376c08u,0x2748774cu,0x34b0bcb5u,0x391c0cb3u,0x4ed8aa4au,
        0x5b9cca4fu,0x682e6ff3u,0x748f82eeu,0x78a5636fu,0x84c87814u,0x8cc70208u,
        0x90befffau,0xa4506cebu,0xbef9a3f7u,0xc67178f2u
    };
    uint32_t w[64], a, b, c, d, e, f, g, h, t1, t2;
    unsigned i;
    for (i = 0; i < 16; ++i) {
        w[i] = ((uint32_t)data[i * 4] << 24) | ((uint32_t)data[i * 4 + 1] << 16) |
               ((uint32_t)data[i * 4 + 2] << 8) | data[i * 4 + 3];
    }
    for (i = 16; i < 64; ++i) {
        uint32_t s0 = sha_rotr(w[i - 15], 7) ^ sha_rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = sha_rotr(w[i - 2], 17) ^ sha_rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    a=sha->state[0]; b=sha->state[1]; c=sha->state[2]; d=sha->state[3];
    e=sha->state[4]; f=sha->state[5]; g=sha->state[6]; h=sha->state[7];
    for (i = 0; i < 64; ++i) {
        uint32_t s1 = sha_rotr(e, 6) ^ sha_rotr(e, 11) ^ sha_rotr(e, 25);
        uint32_t ch = (e & f) ^ ((~e) & g);
        uint32_t s0 = sha_rotr(a, 2) ^ sha_rotr(a, 13) ^ sha_rotr(a, 22);
        uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        t1 = h + s1 + ch + k[i] + w[i]; t2 = s0 + maj;
        h=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
    }
    sha->state[0]+=a; sha->state[1]+=b; sha->state[2]+=c; sha->state[3]+=d;
    sha->state[4]+=e; sha->state[5]+=f; sha->state[6]+=g; sha->state[7]+=h;
}
static void sha_init(Sha256 *sha) {
    static const uint32_t init[8] = {0x6a09e667u,0xbb67ae85u,0x3c6ef372u,0xa54ff53au,
                                     0x510e527fu,0x9b05688cu,0x1f83d9abu,0x5be0cd19u};
    memset(sha, 0, sizeof(*sha)); memcpy(sha->state, init, sizeof(init));
}
static void sha_update(Sha256 *sha, const unsigned char *data, size_t len) {
    while (len != 0u) {
        size_t take = 64u - sha->buffer_len;
        if (take > len) take = len;
        memcpy(sha->buffer + sha->buffer_len, data, take);
        sha->buffer_len += take; data += take; len -= take; sha->bit_count += (uint64_t)take * 8u;
        if (sha->buffer_len == 64u) { sha_transform(sha, sha->buffer); sha->buffer_len = 0u; }
    }
}
static void sha_final(Sha256 *sha, unsigned char digest[32]) {
    unsigned char pad[128] = {0x80};
    unsigned i; uint64_t bits = sha->bit_count;
    size_t pad_len = sha->buffer_len < 56u ? 56u - sha->buffer_len : 120u - sha->buffer_len;
    sha_update(sha, pad, pad_len);
    for (i = 0; i < 8; ++i) pad[7u - i] = (unsigned char)(bits >> (i * 8u));
    sha_update(sha, pad, 8u);
    for (i = 0; i < 8; ++i) {
        digest[i*4] = (unsigned char)(sha->state[i] >> 24);
        digest[i*4+1] = (unsigned char)(sha->state[i] >> 16);
        digest[i*4+2] = (unsigned char)(sha->state[i] >> 8);
        digest[i*4+3] = (unsigned char)sha->state[i];
    }
}

static int sha_update_kv_line(Sha256 *sha, const char *key, const char *value) {
    char normalized[256];
    int written = snprintf(normalized, sizeof(normalized), "%s=%s\n", key, value);
    if (written < 0 || (size_t)written >= sizeof(normalized)) return 0;
    sha_update(sha, (const unsigned char *)normalized, (size_t)written);
    return 1;
}

static int kv_load(const char *path, PressureModelParams *params, char *calibration_id) {
    FILE *file = fopen(path, "rb");
    char line[256], previous_key[96] = "";
    unsigned seen = 0u, status_ok = 0u, field_count = 0u;
    uint64_t fields_seen = 0u;
    Sha256 sha; unsigned char digest[32]; char computed[65]; size_t i;
    if (file == NULL) return 0;
    sha_init(&sha);
    while (fgets(line, sizeof(line), file) != NULL) {
        char *eq = strchr(line, '='), *value, *end;
        if (strchr(line, '\n') == NULL && !feof(file)) { fclose(file); return 0; }
        if (eq == NULL || strchr(eq + 1, '=') != NULL) { fclose(file); return 0; }
        *eq = '\0';
        value = eq + 1; end = value + strlen(value);
        while (end > value && (end[-1] == '\n' || end[-1] == '\r')) --end;
        *end = '\0';
        if (strcmp(line, "schema_version") == 0) {
            if (strcmp(value, "1") != 0) { fclose(file); return 0; }
            if (seen & 1u || field_count != 0u) { fclose(file); return 0; }
            if (!sha_update_kv_line(&sha, line, value)) { fclose(file); return 0; }
            seen |= 1u; continue;
        }
        if (strcmp(line, "calibration_id") == 0) {
            if (seen & 2u || strlen(value) != 64u) { fclose(file); return 0; }
            for (i = 0; i < 64u; ++i) if (!isxdigit((unsigned char)value[i])) { fclose(file); return 0; }
            memcpy(calibration_id, value, 64); calibration_id[64] = '\0'; seen |= 2u; continue;
        }
        if (strcmp(line, "calibration_status") == 0) {
            if (seen & 4u) { fclose(file); return 0; }
            seen |= 4u;
            status_ok = strcmp(value, "calibrated") == 0; continue;
        }
        {
            float parsed;
            errno = 0; parsed = strtof(value, &end);
            if (errno != 0 || end == value || *end != '\0' || !isfinite(parsed)) { fclose(file); return 0; }
#define KV_FIELD(index, name) \
            if (strcmp(line, #name) == 0) { \
                if ((fields_seen & (UINT64_C(1) << (index))) != 0u) { fclose(file); return 0; } \
                if (previous_key[0] != '\0' && strcmp(previous_key, line) >= 0) { fclose(file); return 0; } \
                fields_seen |= UINT64_C(1) << (index); ++field_count; params->physical.name = parsed; \
                if (!sha_update_kv_line(&sha, line, value)) { fclose(file); return 0; } \
                strncpy(previous_key, line, sizeof(previous_key) - 1u); previous_key[sizeof(previous_key) - 1u] = '\0'; \
                continue; \
            }
            KV_FIELD(0, atmospheric_pressure_pa) KV_FIELD(1, suction_pressure_pa)
            KV_FIELD(2, outlet_volume_m3) KV_FIELD(3, chamber_volume_m3)
            KV_FIELD(4, line_inertance_pa_s2_per_m3) KV_FIELD(5, line_resistance_pa_s_per_m3)
            KV_FIELD(6, line_quadratic_resistance_pa_s2_per_m6) KV_FIELD(7, beta_oil_pa)
            KV_FIELD(8, gas_fraction) KV_FIELD(9, gas_transition_pa) KV_FIELD(10, beta_min_pa)
            KV_FIELD(11, pump_leak_c0_m3_pa_s) KV_FIELD(12, pump_leak_speed_m3_pa_s_per_rpm)
            KV_FIELD(13, outlet_leak_m3_pa_s) KV_FIELD(14, cylinder_leak_m3_pa_s)
            KV_FIELD(15, eta_v_min) KV_FIELD(16, eta_m_nominal)
            KV_FIELD(17, eta_m_pressure_loss_per_pa) KV_FIELD(18, eta_m_speed_loss_per_rpm)
            KV_FIELD(19, eta_m_min) KV_FIELD(20, rated_motor_torque_nm)
            KV_FIELD(21, torque_ripple13_peak) KV_FIELD(22, torque_ripple13_phase_rad)
            KV_FIELD(23, ripple13_peak) KV_FIELD(24, ripple26_peak) KV_FIELD(25, ripple39_peak)
            KV_FIELD(26, ripple13_phase_rad) KV_FIELD(27, ripple26_phase_rad) KV_FIELD(28, ripple39_phase_rad)
            KV_FIELD(29, motor_natural_freq_hz) KV_FIELD(30, motor_damping) KV_FIELD(31, motor_delay_s)
            KV_FIELD(32, motor_accel_limit_rpm_s) KV_FIELD(33, motor_torque_limit_permille)
            KV_FIELD(34, relief_set_pa) KV_FIELD(35, relief_deadband_pa)
            KV_FIELD(36, relief_orifice_coeff_m3_s_sqrt_pa) KV_FIELD(37, relief_hysteresis_pa)
            KV_FIELD(38, sensor_delay_s) KV_FIELD(39, sensor_quantization_bar)
#undef KV_FIELD
            fclose(file); return 0;
        }
    }
    fclose(file);
    if ((seen & 7u) != 7u || !status_ok || field_count != 40u) return 0;
    sha_final(&sha, digest);
    for (i = 0; i < sizeof(digest); ++i) sprintf(computed + i * 2u, "%02x", digest[i]);
    computed[64] = '\0';
    return strcmp(computed, calibration_id) == 0;
}

static int parse_float(const char *text, float *value) {
    char *end = NULL;

    errno = 0;
    *value = strtof(text, &end);
    return errno == 0 && end != text && *end == '\0' && isfinite(*value);
}

static int parse_count(const char *text, int *value) {
    char *end = NULL;
    long parsed;

    errno = 0;
    parsed = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' ||
        parsed < 1 || parsed > INT_MAX) {
        return 0;
    }
    *value = (int)parsed;
    return 1;
}

int main(int argc, char **argv) {
    PressureModelParams params;
    PressureModelState state;
    PressureModelOutput out;
    float rpm;
    int samples;
    int i;
    char calibration_id[65] = "uncalibrated";

    if (argc != 4 && argc != 5) {
        fprintf(stderr,
                "usage: %s <first_order|physical> <rpm> <samples> [identified_params.kv]\n",
                argv[0]);
        return 2;
    }
    if (!parse_float(argv[2], &rpm) || !parse_count(argv[3], &samples)) {
        fprintf(stderr, "invalid replay RPM or sample count\n");
        return 2;
    }
    PressureModel_InitParams(&params);
    if (strcmp(argv[1], "first_order") == 0) {
        params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    } else if (strcmp(argv[1], "physical") == 0) {
        params.model_type = PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED;
        params.enable_sensor_noise = 0u;
        params.enable_motor_noise = 0u;
        params.enable_process_noise = 0u;
    } else {
        fprintf(stderr, "profile must be first_order or physical\n");
        return 2;
    }
    if (argc == 5) {
        PressureModelInput input;
        input.target_rpm = rpm;
        input.load_flow_m3_s = 0.0f;
        input.dt_s = 0.001f;
        if (params.model_type != PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED ||
            !kv_load(argv[4], &params, calibration_id) ||
            !PressureModel_ValidateParams(&params) ||
            !PressureModel_ValidateInput(&params, &input)) {
            fprintf(stderr, "model not calibrated: invalid or incomplete identified_params.kv\n");
            return 2;
        }
    }
    PressureModel_Reset(&state, 0x13572468u);
    printf("# calibration_id=%s\n", calibration_id);
    if (argc == 5) {
        puts("# calibration_status=calibrated");
    } else {
        puts("# calibration_status=uncalibrated");
    }
    puts("# active_order_mask: bit0=13th, bit1=26th, bit2=39th; clear bits are disabled");
    puts("sample,actual_rpm,real_pressure_bar,measured_pressure_bar,angle_deg,"
         "torque_permille,timestamp_s,valid_flags,active_order_mask");
    for (i = 0; i < samples; ++i) {
        PressureModel_Step(&params, &state, rpm, 0.001f, &out);
        printf("%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%" PRIu32 ",%u\n",
               i,
               out.actual_motor_rpm,
               out.real_pressure_bar,
               out.measured_pressure_bar,
               out.pumpFeedback.angleDeg,
               out.pumpFeedback.torquePermille,
               out.pumpFeedback.timestamp,
               (uint32_t)out.pumpFeedback.validFlags,
               (unsigned int)out.active_order_mask);
    }
    return 0;
}
