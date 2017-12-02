#include "PID.h"

void PID_init(PID *pid, double kp, double ki, double kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    pid->errors[0] = 0.0;
    pid->errors[1] = 0.0;
    pid->prev_correction = 0.0;
}

double PID_step(PID *pid, double cur_error) {
    pid->prev_correction += pid->kp*(cur_error - pid->errors[0]);
    pid->prev_correction += pid->ki*cur_error;
    pid->prev_correction += pid->kd*(cur_error - 2*pid->errors[0] + pid->errors[1]);

    pid->errors[1] = pid->errors[0];
    pid->errors[0] = cur_error;

    return pid->prev_correction;
}
