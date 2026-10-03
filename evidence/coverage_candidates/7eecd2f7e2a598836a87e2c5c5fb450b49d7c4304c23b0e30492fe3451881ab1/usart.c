/* USART1 serial communication, interrupt handling and RX/TX operations */

#include "6_generated_code.h"
#include <string.h>

static CircArray msg = {0};

void init_usart1(int baud) {
    usart1_baud_used = baud == 0 ? 9600 : (uint32_t)baud;
    if (!msg.enabled) {
        initCircArray(&msg, 200);
    }
}

void USART1_IRQHandler(void) {
    /* Interrupt simulation logic */
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    if (rxne) {
        buf_putbyte(&msg, (char)value);
    }
}

void usart1_send(volatile char *s) {
    while (s && *s != '\0') {
        if (usart1_tx_length < sizeof(usart1_tx_log) - 1) {
            usart1_tx_log[usart1_tx_length++] = *s;
        }
        s++;
    }
    usart1_tx_log[usart1_tx_length] = '\0';
}

uint8_t usart1_read(void) {
    return (uint8_t)buf_getbyte(&msg);
}

char usart1_readc(void) {
    return buf_getbyte(&msg);
}

uint32_t usart1_available(void) {
    return (uint32_t)buf_available(&msg);
}
