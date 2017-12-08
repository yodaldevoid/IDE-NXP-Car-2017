#ifndef __MOTOR_H_
#define __MOTOR_H_

void set_motor_duty(unsigned int dutyCycle);
void set_motor_sep_duty(unsigned int leftDutyCycle, unsigned int rightDutyCycle);
void init_motor(void);

#endif /* __MOTOR_H_ */
