#include <stdio.h>
#include "rbf_pid.h"

int main(void) {
    RBF_PID_Handle pid;
    RBF_PID_Init(&pid, 0.01f, 1500.0f, 1.0f);
    pid.enable = true;
    RBF_PID_SetParamLimits(&pid, 0.01f, 0.05f, 0.005f, 0.02f, 0.1f, 0.2f);

    float setpoint = 100.0f;
    float feedback = 0.0f;

    printf("RBF PID test start\n");
    for (int i = 0; i < 20; ++i) {
        float output = RBF_PID_Update(&pid, setpoint, feedback);
        printf("step=%2d set=%.1f fb=%.1f out=%.4f kp=%.4f ki=%.4f kd=%.4f n_out=%.1f\n",
               i, setpoint, feedback, output, pid.KP, pid.KI, pid.KD, pid.n_out);
        feedback += output * 5.0f;
        if (feedback > setpoint) {
            feedback = setpoint;
        }
    }
    return 0;
}
