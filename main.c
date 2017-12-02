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

#include "MK64F12.h"
#include "util.h"
#include "uart.h"
#include "motor.h"
#include "servo.h"
#include "camera.h"
#include "PID.h"

#define FILTER_THRESHOLD 3500

void initialize(void);

void filter_camera(uint16_t raw[], int filtered[]);
double find_center(int filtered[], int old_center, int *leftEdge, int *rightEdge);

int main(void) {
    int filtered[128];
    int center;
    PID servoPID;
    double servoCorrection;
    char str[100];

    int left, right;

#ifdef DEBUG_CAM_DATA
    int i;
#endif

    // Initialize UART and PWM
    initialize();

    PID_init(&servoPID, 0.5, 0.1, 0.25);

    set_motor_duty(35);
    set_servo_duty(SERVO_DUTY_CENTER);

    center = 63;
    
    while(1) {
        if(camReadDone) {
            camReadDone = 0;
            filter_camera(line, filtered);
            // Zero out the edges after filtering?
            center = find_center(filtered, center, &left, &right);
            servoCorrection = PID_step(&servoPID, 63.5 - center);

#ifndef DEBUG_CAM_DATA
            sprintf(str, "%i %i %i\r\n", center, left, right);
            //sprintf(str, "%i\n\r", (int32_t) (servoCorrection*100.0));
            uart_put(str);
#endif

#if 0
            set_servo_duty(SERVO_DUTY_CENTER + CLAMP(servoCorrection/10.0, -1.0, 1.0));
#else
            if(center > 66) {
                // center too far to right, turn left
                set_servo_duty(SERVO_DUTY_CENTER - 1.0);
                PTB->PCOR |= (1 << LED_BLUE);
                PTB->PSOR |= (1 << LED_RED);
            } else if(center < 61) {
                // center too far to left, turn right
                set_servo_duty(SERVO_DUTY_CENTER + 1.0);
                PTB->PCOR |= (1 << LED_RED);
                PTB->PSOR |= (1 << LED_BLUE);
            } else {
                // just go straight
                set_servo_duty(SERVO_DUTY_CENTER);
                PTB->PSOR |= (1 << LED_RED);
                PTB->PSOR |= (1 << LED_RED);
            }
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
