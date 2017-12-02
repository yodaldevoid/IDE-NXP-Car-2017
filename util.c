#include "MK64F12.h"
#include "util.h"

/**
 * Waits for a delay (in milliseconds)
 *
 * millis - The delay in milliseconds
 */
void delay(int millis) {
    int i;
    for(i = 0; i < millis*DEFAULT_SYSTEM_CLOCK/1000; i++) {}
}

void init_LEDs(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK |
                  SIM_SCGC5_PORTE_MASK;

    // Configure mux for Outputs
    PORTB->PCR[LED_BLUE] = PORT_PCR_MUX(1);
    PORTB->PCR[LED_RED] = PORT_PCR_MUX(1);
    PORTE->PCR[LED_GREEN] = PORT_PCR_MUX(1);

    // Switch to output mode
    PTB->PDDR |= (1 << LED_BLUE);
    PTB->PDDR |= (1 << LED_RED);
    PTE->PDDR |= (1 << LED_GREEN);

    // Turn off the LEDs
    LED_disable();
}

// Pull all LED gpios high. LEDs are enabled low.
void LED_disable(void) {
    PTB->PSOR |= (1 << LED_BLUE);
    PTB->PSOR |= (1 << LED_RED);
    PTE->PSOR |= (1 << LED_GREEN);
}
