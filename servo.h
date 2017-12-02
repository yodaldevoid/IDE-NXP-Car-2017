#ifndef __SERVO_H_
#define __SERVO_H_

#define SERVO_DUTY_LEFT     6.75
#define SERVO_DUTY_CENTER   7.675
#define SERVO_DUTY_RIGHT    9.25

void set_servo_duty(double dutyCycle);
void init_servo(void);

#endif /* SERVO_H_ */
