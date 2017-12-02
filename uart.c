/*
 * File:    uart.c
 * Purpose: Provide UART routines for serial IO
 *
 * Notes:
 *
 */

#include "MK64F12.h"
#include "util.h"

#include "uart.h"

#include <stdio.h>

#define BAUD_RATE 9600     // default baud rate
#define SYS_CLOCK DEFAULT_SYSTEM_CLOCK

void init_uart(void) {
    // define variables for baud rate and baud rate fine adjust
    uint16_t ubd, brfa;

    // Enable clock for UART
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

    // Configure the port control register to alternative 3 (which is UART mode for K64)
    PORTB->PCR[16] = PORT_PCR_MUX(3);
    PORTB->PCR[17] = PORT_PCR_MUX(3);

    // Configure the UART for establishing serial communication //
    // Disable transmitter and receiver until proper settings are chosen for the UART module
    UART0->C2 &= ~(UART_C2_RE_MASK | UART_C2_TE_MASK);

    // Select default transmission/reception settings for serial communication of UART by clearing the control register 1
    UART0->C1 = 0;

    // UART Baud rate is calculated by: baud rate = UART module clock / (16 × (SBR[12:0] + BRFD))
    // 13 bits of SBR are shared by the 8 bits of UART3_BDL and the lower 5 bits of UART3_BDH
    // BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
    // BRFA is defined by the lower 5 bits of control register, UART0_C4

    // calculate baud rate settings: ubd = UART module clock/16* baud rate
    ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));

    // clear SBR bits of BDH
    UART0->BDH &= ~(UART_BDH_SBR_MASK);

    // distribute this ubd in BDH and BDL
    UART0->BDH |= (ubd >> 8) & UART_BDH_SBR_MASK;
    UART0->BDL = ubd & UART_BDL_SBR_MASK;

    // BRFD = (1/32)*BRFA
    // make the baud rate closer to the desired value by using BRFA
    brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));

    // write the value of brfa in UART0_C4
    UART0->C4 &= ~(UART_C4_BRFA_MASK);
    UART0->C4 |= brfa & UART_C4_BRFA_MASK;

    // Enable transmitter and receiver of UART
    UART0->C2 |= UART_C2_RE_MASK | UART_C2_TE_MASK;
}

uint8_t uart_getchar(void) {
    /* Wait until there is space for more data in the receiver buffer */
    while(!(UART0->S1 & UART_S1_RDRF_MASK)) {}
    /* Return the 8-bit data from the receiver */
    return UART0->D;
}

void uart_putchar(char ch) {
    /* Wait until transmission of previous bit is complete */
    while(!(UART0->S1 & UART_S1_TDRE_MASK)) {}
    /* Send the character */
    UART0_D = ch;
    while(!(UART0->S1 & UART_S1_TC_MASK)) {}
}

void uart_put(char *ptr_str) {
    /* use putchar to print string */
#if 1
    /*
     * Copied wholecloth from the provided put function in main.c of exercise 2.
     * This was done as the previous method was less efficient than the one provided.
     */
    while(*ptr_str)
        uart_putchar(*ptr_str++);
#else
    uint32_t i = 0;
    while(ptr_str[i] != 0)
        uart_putchar(ptr_str[i++]);
#endif
}

void uart_putnumU(unsigned int i) {
    char buf[11];

    sprintf(buf, "%i", i);
    uart_put(buf);
}
