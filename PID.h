#ifndef __PID_H_
#define __PID_H_

// Copied from https://stackoverflow.com/questions/14769603/how-can-i-write-a-clamp-clip-bound-macro-for-returning-a-value-in-a-gi/14770282#14770282
#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

typedef struct {
    double kp;
    double ki;
    double kd;
    
    double errors[2];
    double prev_correction;
} PID;

/*
 * Initializes the PID struct with the given k values
 */
void PID_init(PID *pid, double kp, double ki, double kd);

/*
 * Takes the current error value.
 *
 * Returns the correction value. It is unbounded.
 */
double PID_step(PID *pid, double cur_error);

#endif /* __PID_H_ */
