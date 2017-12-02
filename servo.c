/*
 * Pulse-Width-Modulation Code for K64
 * Modified specifically for servo control using FTM2
 * PWM signal can be connected to output pin PB18
 *
 * Author: Brent Dimmig <bnd8678@rit.edu>
 * Modified by: Gabriel Smith <gas6030@rit.edu>
 *              John Cowan <jrc9071@rit.edu>
 * Created: 2/20/2014
 * Modified: 10/20/2017
 */
#include "MK64F12.h"
#include "servo.h"
#include "util.h"

#define SERVO_FREQ 50

// System clock divided by prescaler
#define CLOCK (DEFAULT_SYSTEM_CLOCK/8)
#define FTM2_DEFAULT_VALUE (CLOCK/SERVO_FREQ)

// 6.25 is far left
// 7.75 is straight ahead
// 9.25 is far right
void set_servo_duty(double dutyCycle) {
    // Calculate the new cutoff value
    uint16_t mod = (uint16_t) (((CLOCK/SERVO_FREQ) * dutyCycle) / 100.0);

    FTM2_C0V = mod;
}

/*
 * Initialize the FlexTimer for PWM
 */
void init_servo() {
    // 12.2.13 Enable the Clock to the FTM1 Module
    SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

    // Enable clock on PORT B so it can output
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; // | SIM_SCGC5_PORTA_MASK;

    // 11.4.1 Route the output of FTM channel 2 to the pins
    // Use drive strength enable flag to high drive strength
    PORTB_PCR18 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK; // Ch0

    // 39.3.10 Disable Write Protection
    FTM2_MODE |= FTM_MODE_WPDIS_MASK;

    // 39.3.4 FTM Counter Value
    // Initialize the CNT to 0 before writing to MOD
    FTM2_CNT = 0;

    // 39.3.8 Set the Counter Initial Value to 0
    FTM2_CNTIN = 0;

    // 39.3.5 Set the Modulo resister
    FTM2_MOD = FTM2_DEFAULT_VALUE;

    // 39.3.6 Set the Status and Control of both channels
    // Used to configure mode, edge and level selection
    // See Table 39-67, Edge-aligned PWM, High-true pulses (clear out on match)
    FTM2_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM2_C0SC &= ~FTM_CnSC_ELSA_MASK;

    // 39.3.3 FTM Setup
    // Set prescale value to 8
    // Chose system clock source
    // Timer Overflow Interrupt Enable
    FTM2_SC = FTM_SC_PS(3) | FTM_SC_CLKS(1);
}
