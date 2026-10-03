/* Circular buffer array and USART1 implementation for DiscoBot */
#include "6_generated_code.h"

static CircArray msg = {NULL, 0, false, 0, 0};

uint32_t usart1_baud_used = 9600;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;

static uint8_t host_usart1_dr = 0;
static bool host_usart1_rxne = false;

void initCircArray(CircArray *arr, int size) {
    if (!arr || size <= 0) return;
    if (arr->enabled) {
        printf("CircArray already enabled\n");
        return;
    }
    arr->buf = (char *)calloc((size_t)size, 1);
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

bool buf_empty(CircArray *arr) {
    if (!arr) return true;
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (!arr || !arr->enabled || arr->size == 0) return false;
    if (buf_empty(arr)) return false;
    return (arr->n_r % arr->size) == (arr->n_w % arr->size);
}

int buf_available(CircArray *arr) {
    if (!arr) return 0;
    return (int)(arr->n_w - arr->n_r);
}

int buf_putbyte(CircArray *arr, char c) {
    if (!arr || !arr->enabled || !arr->buf) return 0;
    if (buf_full(arr)) return 0;
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (!arr || !arr->enabled || !arr->buf) return 0;
    if (buf_empty(arr)) return 0;
    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

bool buf_resize(CircArray *arr, int newSize) {
    if (!arr || newSize <= 0) return false;
    char *newbuf = (char *)realloc(arr->buf, (size_t)newSize);
    if (!newbuf) return false;
    arr->buf = newbuf;
    arr->size = (uint32_t)newSize;
    return true;
}

void buf_clear(CircArray *arr) {
    if (!arr || !arr->buf || arr->size == 0) return;
    memset(arr->buf, 0, arr->size);
}

bool buf_delete(CircArray *arr) {
    if (!arr) return false;
    buf_clear(arr);
    if (arr->buf) {
        free(arr->buf);
        arr->buf = NULL;
    }
    arr->size = 0;
    arr->n_r = 0;
    arr->n_w = 0;
    arr->enabled = false;
    return true;
}

void init_usart1(int baud) {
    if (baud == 0) {
        usart1_baud_used = 9600;
        printf("Warning: Baud rate 0 requested, default 9600 used\n");
    } else {
        usart1_baud_used = (uint32_t)baud;
    }
    initCircArray(&msg, 200);
}

void USART1_IRQHandler(void) {
    if (host_usart1_rxne) {
        buf_putbyte(&msg, (char)host_usart1_dr);
        host_usart1_rxne = false;
    }
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    host_usart1_dr = value;
    host_usart1_rxne = rxne;
    USART1_IRQHandler();
}

void usart1_send(volatile char *s) {
    if (!s) return;
    while (*s) {
        if (usart1_tx_length < sizeof(usart1_tx_log) - 1) {
            usart1_tx_log[usart1_tx_length++] = *s;
            usart1_tx_log[usart1_tx_length] = '\0';
        }
        s++;
    }
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
