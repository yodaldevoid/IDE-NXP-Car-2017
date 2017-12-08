/*
 * Main method for testing the PWM DC motor control and servo control.
 * PWM signal connected to output pins PC3 and PC4
 * Servo connected to pin PB18
 *
 * Author: Unknown
 * Modified by: John Cowan <jrc9071@rit.edu>
 *              Gabriel Smith <gas6030@rit.edu>
 * Created: Unknown
 * Modified: 10/20/17
 */

#include <stdio.h>
#include <stdbool.h>

#include "MK64F12.h"
#include "util.h"
#include "uart.h"
#include "motor.h"
#include "servo.h"
#include "camera.h"
#include "PID.h"

#define FILTER_THRESHOLD 3250 //3500

#define FAST_SPEED 45
#define SLOW_SPEED 35
#define LEFT_TURN 66 //66 is good for testing
#define RIGHT_TURN 61 //61 is good for testing
void initialize(void);

void filter_camera(uint16_t raw[], int filtered[]);
double find_center(int filtered[], int old_center, int *leftEdge, int *rightEdge);

int main(void) {
    int filtered[128];
    int center;
    PID servoPID;
    PID speedPID;
    double servo_correction;
    double speed_correction;
    bool is_turning;
	bool turning_left;
    int wanted, assumed;
    char str[100];
    int left, right;

#ifdef DEBUG_CAM_DATA
    int i;
#endif

    // Initialize UART and PWM
    initialize();

    PID_init(&servoPID, 0.5, 0.1, 0.25);
    PID_init(&speedPID, 0.5, 0.1, 0.25);
    set_servo_duty(SERVO_DUTY_CENTER);

    center = 63;

    is_turning = false;
	turning_left = false;
    assumed = 0;

    // Wait for switch to be pressed to go
    while(!SW3_IN) {};

    while(1) {
        if(camReadDone) {
            camReadDone = 0;
            filter_camera(line, filtered);
            // Zero out the edges after filtering?
            center = find_center(filtered, center, &left, &right);

            servo_correction = PID_step(&servoPID, 63.5 - center);

#ifndef DEBUG_CAM_DATA
            //sprintf(str, "%i %i %i\r\n", center, left, right);
            //sprintf(str, "%d\n\r", (int) (100* servo_correction));
            //uart_put(str);
#endif

#if 0
            double clamped_correction = CLAMP(servo_correction/10.0, -1.0, 1.0);
            if(clamped_correction < -0.05) {
                // center too far to right, turn left
                PTB->PCOR |= (1 << LED_BLUE);
                PTB->PSOR |= (1 << LED_RED);
                is_turning = true;
            } else if(clamped_correction > 0.05) {
                // center too far to left, turn right
                PTB->PCOR |= (1 << LED_RED);
                PTB->PSOR |= (1 << LED_BLUE);
                is_turning = true;
            } else {
                // just go straight
                PTB->PSOR |= (1 << LED_RED);
                PTB->PSOR |= (1 << LED_RED);
                is_turning = false;
            }
            set_servo_duty(SERVO_DUTY_CENTER + clamped_correction);
#else
            if(center > LEFT_TURN) {
                // center too far to right, turn left
                set_servo_duty(SERVO_DUTY_CENTER - 1.0);
                PTB->PCOR |= (1 << LED_BLUE);
                PTB->PSOR |= (1 << LED_RED);
                is_turning = true;
				turning_left = true;
            } else if(center < RIGHT_TURN) {
                // center too far to left, turn right
                set_servo_duty(SERVO_DUTY_CENTER + 1.0);
                PTB->PCOR |= (1 << LED_RED);
                PTB->PSOR |= (1 << LED_BLUE);
                is_turning = true;
				turning_left = false;
            } else {
                // just go straight
                set_servo_duty(SERVO_DUTY_CENTER);
                PTB->PSOR |= (1 << LED_RED);
                PTB->PSOR |= (1 << LED_RED);
                is_turning = false;
            }
#endif

            if(is_turning) {
                wanted = SLOW_SPEED;
            } else {
                wanted = FAST_SPEED;
            }
#if 1
            speed_correction = PID_step(&speedPID, wanted - assumed);
            assumed = CLAMP(assumed + speed_correction, 0, 100);
            sprintf(str, "\n SpeedCorrection:%d Assumed: %d,\n\r", (int) (speed_correction*100), assumed);
            uart_put(str);
			if(is_turning && turning_left){
				set_motor_sep_duty(assumed - 2, assumed);
			}
			else if(is_turning && !turning_left){
				set_motor_sep_duty(assumed, assumed - 2);
			}
			else{
				set_motor_duty(assumed);            
			}
#else
            set_motor_duty(wanted);
#endif
        }

#ifdef DEBUG_CAM_DATA
        // Every 2 seconds
        if(capCnt >= (2/INTEGRATION_TIME)) {
            PTE->PCOR |= (1 << LED_GREEN);
            // send the array over uart
            sprintf(str,"%i\n\r",-1); // start value
            uart_put(str);
            for(i = 0; i < 127; i++) {
#if 0
                sprintf(str,"%i\n", line[i]);
#else
                sprintf(str,"%i\n", filtered[i]);
#endif
                uart_put(str);
            }
            sprintf(str,"%i\n\r",-2); // end value
            uart_put(str);
            capCnt = 0;
            PTE->PSOR |= (1 << LED_GREEN);
        }
#endif
    }
}

void filter_camera(uint16_t raw[], int filtered[]) {
    int i, j;
    int val;

    filtered[0] = raw[0];
    filtered[1] = raw[0]/2 + raw[1]/2;
    filtered[2] = raw[0]/3 + raw[1]/3 + raw[2]/3;
    filtered[3] = raw[0]/4 + raw[1]/4 + raw[2]/4 + raw[3]/4;
    for(i = 4; i < 128; i++) {
        filtered[i] = 0;
        for(j = 0; j < 5; j++) {
            filtered[i] += raw[i - j]/5;
        }
    }

    for(i = 1; i < 128; i++) {
        val = filtered[i] - filtered[i - 1];
        if(val > FILTER_THRESHOLD || val < -FILTER_THRESHOLD) {
            filtered[i - 1] = 1;
        } else {
            filtered[i - 1] = 0;
        }
    }
    filtered[0] = 0;
}

double find_center(int filtered[], int old_center, int *left, int *right) {
    int leftEdge, rightEdge, i;

    leftEdge = 0;
    rightEdge = 0;

    for(i = 0; (!leftEdge || !rightEdge) && i < 128; i++) {
        if((old_center - i >= 0) && !leftEdge && filtered[old_center - i]) {
            leftEdge = old_center - i;
        }
        if((old_center + i < 128) && !rightEdge && filtered[old_center + i]) {
            rightEdge = old_center + i;
        }
    }

    if(!rightEdge) {
        rightEdge = 127;
    }

    if(left) {
        *left = leftEdge;
    }

    if(right) {
        *right = rightEdge;
    }

    return (leftEdge + rightEdge)/2.0;
}

void initialize(void) {
    // Initialize UART
    init_uart();
    init_LEDs();
    init_switches();
    init_motor();
    init_servo();
    init_camera();
}
