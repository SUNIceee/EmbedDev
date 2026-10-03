/* USART1 host model with interrupt-style RXNE handling and TX log. */

#include "6_generated_code.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

static CircArray msg = {0};
static volatile bool host_usart_rxne = false;
static volatile uint8_t host_usart_dr = 0;

void init_usart1(int baud)
{
    if (baud == 0) {
        baud = UART_BAUD;
        fprintf(stderr, "init_usart1: baud=0, defaulting to 9600\n");
    }

    usart1_baud_used = (uint32_t)baud;
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    host_usart_rxne = false;
    host_usart_dr = 0;
}

void USART1_IRQHandler(void)
{
    if (!host_usart_rxne) {
        return;
    }

    char c = (char)host_usart_dr;
    (void)buf_putbyte(&msg, c);
    host_usart_rxne = false;
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne)
{
    host_usart_dr = value;
    host_usart_rxne = rxne;
    USART1_IRQHandler();
}

void usart1_send(volatile char *s)
{
    if (s == NULL) {
        return;
    }

    while (*s != '\0') {
        /* Host TXE is always ready. */
        if (usart1_tx_length >= (sizeof(usart1_tx_log) - 1u)) {
            break;
        }

        usart1_tx_log[usart1_tx_length++] = *s;
        usart1_tx_log[usart1_tx_length] = '\0';
        s++;
    }
}

uint8_t usart1_read(void)
{
    return (uint8_t)buf_getbyte(&msg);
}

signed char usart1_readc(void)
{
    uint8_t byte = (uint8_t)buf_getbyte(&msg);

    if (byte <= 0x7Fu) {
        return (signed char)byte;
    }

    /* For 0x80..0xFF, use arithmetic conversion to two's complement negative
       range without relying on implementation-defined signed conversion. */
    return (signed char)((int)byte - 0x100);
}

uint32_t usart1_available(void)
{
    return (uint32_t)buf_available(&msg);
}

