/* Implementation of DiscoBot peripheral emulation, GPIO, ADC, RNG, CircArray, and USART1 */
#include "6_generated_code.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 168000000;
uint32_t SysTick_reload = 168000;
uint32_t usart1_baud_used = 9600;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";

static int16_t accel_raw_x = 0;
static int16_t accel_raw_y = 0;
static int16_t accel_raw_z = 0;
static uint16_t adc_raw_temp = 943;
static bool rng_ready_flag = true;
static uint32_t rng_raw_value = 0x12345678;

static uint8_t simulated_usart1_dr = 0;
static bool simulated_usart1_rxne = false;
static CircArray usart1_rx_ring = {0};

void SystemInit(void) {
    SystemCoreClock = 168000000;
}

void init_systick(void) {
    SystemCoreClock = 168000000;
    SysTick_reload = SystemCoreClock / 1000;
    msTicks = 0;
}

void init_button(void) {
    GPIOA_IDR &= ~(1U << 0);
    b_i = 0;
    buttstate = ButtonIsReleased;
}

void discobot_set_button_level(bool high) {
    if (high) {
        GPIOA_IDR |= (1U << 0);
    } else {
        GPIOA_IDR &= ~(1U << 0);
    }
}

int read_buttonc(int i) {
    if (i > 3) return -1;
    return (int)((GPIOA_IDR >> i) & 1U);
}

void SysTick_Handler(void) {
    msTicks++;
    if (read_buttonc(0) == 1) {
        b_i++;
        if (b_i >= 250) {
            buttstate = ButtonIsPressed;
        }
    } else {
        b_i = 0;
        buttstate = ButtonIsReleased;
    }
}

void checkbutton(void) {
    static ButtonState laststate = ButtonIsReleased;
    if (buttstate != laststate) {
        laststate = buttstate;
    }
}

void init_GPIO_A1A2A3A4_output(void) {
    GPIOA_output[1] = 0; GPIOA_output[2] = 0;
    GPIOA_output[3] = 0; GPIOA_output[4] = 0;
}

void init_LED_pins(void) {
    GPIOD_output[12] = 0; GPIOD_output[13] = 0;
    GPIOD_output[14] = 0; GPIOD_output[15] = 0;
}

void LED_On(int i) {
    if (i >= 0 && i < 4) { GPIOD_output[12 + i] = 1; }
}

void LED_Off(int i) {
    if (i >= 0 && i < 4) { GPIOD_output[12 + i] = 0; }
}

int init_accelerometers(void) {
    accel_raw_x = 0; accel_raw_y = 0; accel_raw_z = 0;
    return 0;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    accel_raw_x = x_mg; accel_raw_y = y_mg; accel_raw_z = z_mg;
}

void read_accelerometers(float b[3]) {
    if (!b) return;
    b[0] = (float)accel_raw_x / 1000.0f;
    b[1] = (float)accel_raw_y / 1000.0f;
    b[2] = (float)accel_raw_z / 1000.0f;
}

void init_temperature_sensor(void) {
    adc_raw_temp = 943;
}

void discobot_set_adc_raw(uint16_t raw) {
    adc_raw_temp = raw;
}

float read_temperature_sensor(void) {
    float temp = (float)adc_raw_temp;
    temp = (temp / 4095.0f) * 3.3f;
    temp = ((temp - 0.760f) / 0.0025f) + 25.0f;
    return temp;
}

void init_rng(void) {
    rng_ready_flag = true;
    rng_raw_value = 0x12345678;
}

void discobot_set_rng(bool ready, uint32_t value) {
    rng_ready_flag = ready;
    rng_raw_value = value;
}

uint32_t get_random_number(void) {
    while (!rng_ready_flag) { }
    return rng_raw_value;
}

void initCircArray(CircArray *arr, int size) {
    if (!arr || size <= 0) return;
    if (arr->enabled) {
        printf("Error: CircArray already enabled.\n");
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
    if (!arr || arr->size == 0) return false;
    return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_available(CircArray *arr) {
    if (!arr) return 0;
    return (int)(arr->n_w - arr->n_r);
}

int buf_putbyte(CircArray *arr, char c) {
    if (!arr || !arr->buf || arr->size == 0 || buf_full(arr)) return 0;
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (!arr || !arr->buf || arr->size == 0 || buf_empty(arr)) return 0;
    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

void buf_clear(CircArray *arr) {
    if (!arr || !arr->buf) return;
    memset(arr->buf, 0, arr->size);
}

bool buf_resize(CircArray *arr, int newSize) {
    if (!arr || newSize <= 0) return false;
    char *new_buf = (char *)realloc(arr->buf, (size_t)newSize);
    if (!new_buf) return false;
    arr->buf = new_buf;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr) {
    if (!arr) return false;
    buf_clear(arr);
    free(arr->buf);
    arr->buf = NULL;
    arr->size = 0;
    arr->n_r = 0;
    arr->n_w = 0;
    arr->enabled = false;
    return true;
}

void init_usart1(int baud) {
    if (usart1_rx_ring.enabled && usart1_rx_ring.buf != NULL) {
        buf_delete(&usart1_rx_ring);
    }
    usart1_baud_used = (baud == 0) ? 9600 : (uint32_t)baud;
    initCircArray(&usart1_rx_ring, 200);
}

void USART1_IRQHandler(void) {
    if (simulated_usart1_rxne) {
        buf_putbyte(&usart1_rx_ring, (char)simulated_usart1_dr);
        simulated_usart1_rxne = false;
    }
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    simulated_usart1_dr = value;
    simulated_usart1_rxne = rxne;
    USART1_IRQHandler();
}

void usart1_send(volatile char *s) {
    if (!s) return;
    while (*s && usart1_tx_length < (sizeof(usart1_tx_log) - 1)) {
        usart1_tx_log[usart1_tx_length++] = *s++;
    }
    usart1_tx_log[usart1_tx_length] = '\0';
}

uint8_t usart1_read(void) {
    return (uint8_t)buf_getbyte(&usart1_rx_ring);
}

char usart1_readc(void) {
    return buf_getbyte(&usart1_rx_ring);
}

uint32_t usart1_available(void) {
    return (uint32_t)buf_available(&usart1_rx_ring);
}

void init_system(void) {
    SystemInit();
    init_systick();
    init_LED_pins();
    init_button();
    init_accelerometers();
    init_rng();
    init_temperature_sensor();
    init_GPIO_A1A2A3A4_output();
    init_usart1(9600);
    usart1_send("UART1 Initialized. @9600bps\r\n");
}

void main_loop_iteration(void) {
    if (usart1_available() > 0) {
        char c = usart1_readc();
        dispatch_uart_command((int)c);
    }
    checkbutton();
    execute_tasks();
}
