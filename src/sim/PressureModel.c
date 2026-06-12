/*
 * PressureModel.c
 *
 *  Created on: 2026年6月12日
 *      Author: Administrator
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

// ================= 物理参数定义 =================
#define PI          3.14159265358979323846

// --- 油泵参数 ---
#define PUMP_DISP_CC_REV   20.0             // 排量 cc/rev
#define PUMP_DISP          (PUMP_DISP_CC_REV * 1e-6) // 排量 m^3/rev
#define PUMP_TEETH         13               // 齿轮齿数

// --- 液压系统参数 ---
#define BETA               1.6e9            // 油液体积弹性模量 Pa
#define VOLUME             5e-4             // 封闭容腔总容积 m^3
#define LEAK_COEFF         8.33333e-13      // 泄漏系数 m^3/(Pa·s) (由10rpm->40bar标定)

// --- 溢流阀参数 ---
#define RELIEF_SET_PA      250.0e5          // 开启压力 250 bar (Pa)
#define RELIEF_COEFF       1.2e-9           // 溢流阀系数 m^3/(Pa·s)

// --- 压力传感器 ---
#define SENSOR_RANGE_BAR   250.0
#define SENSOR_NOISE_STD   0.4              // 高斯噪声标准差 bar

// --- 电机参数 ---
#define MOTOR_TAU          0.05             // 电机转速动态时间常数 s
#define MOTOR_NOISE_STD    2.0              // 转速波动噪声标准差 rpm

// --- 压力脉动与跌落 ---
#define FLOW_RIPPLE_RATIO  0.10             // 流量脉动幅度 (10%)
#define DROP_DEPTH         0.05             // 压力跌落深度 (5%)
#define DROP_WIDTH_RATIO   0.05             // 跌落宽度相对齿周期的比例

// --- 仿真参数 ---
#define DT                 0.001           // 仿真步长 s (0.1ms)
#define MAX_RPM            2000.0
#define MIN_RPM            -100.0

// ================= 函数声明 =================
double gauss_noise(double std_dev);
double motor_dynamics(double target_rpm, double *motor_state, double dt);


// ================= 噪声生成（Box-Muller） =================
double gauss_noise(double std_dev) {
    static int has_spare = 0;
    static double spare;
    if (has_spare) {
        has_spare = 0;
        return std_dev * spare;
    }
    has_spare = 1;
    double u, v, s;
    do {
        u = (rand() / (double)RAND_MAX) * 2.0 - 1.0;
        v = (rand() / (double)RAND_MAX) * 2.0 - 1.0;
        s = u * u + v * v;
    } while (s >= 1.0 || s == 0.0);
    s = sqrt(-2.0 * log(s) / s);
    spare = v * s;
    return std_dev * (u * s);
}

// ================= 电机动态模型（一阶低通+噪声） =================
double motor_dynamics(double target_rpm, double *motor_state, double dt) {
    // 限幅（含负值）
    if (target_rpm > MAX_RPM) target_rpm = MAX_RPM;
    if (target_rpm < MIN_RPM) target_rpm = MIN_RPM;

    double alpha = dt / (MOTOR_TAU + dt);
    double rpm = *motor_state + alpha * (target_rpm - *motor_state);
    rpm += gauss_noise(MOTOR_NOISE_STD);

    // 限幅
    if (rpm > MAX_RPM) rpm = MAX_RPM;
    if (rpm < MIN_RPM) rpm = MIN_RPM;
    *motor_state = rpm;
    return rpm;
}

// ================= 压力模型更新 =================
/*
 * 状态变量：
 *   real_P  - 封闭容腔内的真实压力 (Pa)，由微分方程积分得到
 * 返回值：
 *   传感器测量压力 (bar)，已叠加跌落效应、传感器噪声
 * 参数：
 *   actual_rps - 电机实际转速 (转/秒)
 *   t          - 当前仿真时间 (秒)
 *   P_state    - 指向真实压力的指针 (Pa)，需在外部保留
 */
float pressure_update(float target_rpm, float t, float *P_state, float *real_P, float* actual_motor_rpm) {
    // 电机动态，得到实际转速
    double motor_state = 0.0;    // 电机实际转速 rpm
    double actual_rpm = motor_dynamics(target_rpm, &motor_state, DT);
    double actual_rps = actual_rpm / 60.0;   // 转为 rps
    *actual_motor_rpm = actual_rpm; // 输出实际转速供观察
    double P = *P_state;
    double n = actual_rps;

    // 1. 理论流量（正转含13齿脉动，反转无脉动）
    double Q_theoretical = 0.0;
    if (n > 0.01) {
        double freq = PUMP_TEETH * n;
        double ripple = 1.0 + FLOW_RIPPLE_RATIO * sin(2.0 * PI * freq * t);
        Q_theoretical = PUMP_DISP * n * ripple;
    } else if (n < -0.01) {
        Q_theoretical = PUMP_DISP * n;   // 反转直接负流量
    }
    // 转速过零附近 Q_theoretical = 0

    // 2. 泄漏（始终为正，加速卸压）
    double Q_leak = LEAK_COEFF * P;

    // 3. 溢流阀
    double Q_relief = 0.0;
    if (P > RELIEF_SET_PA) {
        Q_relief = RELIEF_COEFF * (P - RELIEF_SET_PA);
    }

    double Q_net = Q_theoretical - Q_leak - Q_relief;

    // 4. 压力积分（防负压）
    double dP = (BETA / VOLUME) * Q_net * DT;
    if (P <= 0.0 && dP < 0.0) dP = 0.0;   // 不可低于 0
    P += dP;
    if (P < 0.0) P = 0.0;
    *P_state = P;

    // 5. 13齿压力跌落（仅正转时）
    double P_display = P;
    if (n > 0.01) {
        double tooth_period = 1.0 / (PUMP_TEETH * n);
        double phase = fmod(t, tooth_period);
        double drop_width = DROP_WIDTH_RATIO * tooth_period;
        if (phase < drop_width) {
            double depth_factor = 1.0 - DROP_DEPTH * 0.5 * (1.0 + cos(2.0 * PI * phase / drop_width));
            P_display *= depth_factor;
        }
    }

    if (real_P != 0) *real_P = P_display* 1e-5; // 转为 bar 供外部观察

    // 6. 传感器噪声与限幅
    double P_bar = P_display * 1e-5;
    P_bar += gauss_noise(SENSOR_NOISE_STD);
    if (P_bar < 0.0) P_bar = 0.0;
    if (P_bar > SENSOR_RANGE_BAR) P_bar = SENSOR_RANGE_BAR;
    return P_bar;
}


