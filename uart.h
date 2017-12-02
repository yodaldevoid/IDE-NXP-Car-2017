#ifndef __UART_H
#define __UART_H

void init_uart(void);

void uart_put(char *ptr_str);
void uart_putnumU(unsigned int i);

uint8_t uart_getchar(void);
void uart_putchar(char ch);

#endif /* __UART_H */
