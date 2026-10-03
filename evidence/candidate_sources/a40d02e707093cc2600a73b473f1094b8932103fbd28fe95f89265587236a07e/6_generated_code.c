#include "6_generated_code.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int host_SysTick_Config(uint32_t ticks) {
    SysTick_reload = ticks;
    return 0;
}
#define SysTick_Config host_SysTick_Config
static void SystemCoreClockUpdate(void) {
    SystemCoreClock = 168000000u;
}

volatile uint32_t msTicks = 0u;
volatile uint32_t b_i = 0u;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = STOPCAR;
motor_command_fn callme = 0;
motor_command_fn flookup[9] = {
    move_forward, move_backward, move_forward_soft_left, move_forward_soft_right,
    move_backward_soft_left, move_backward_soft_right, move_spin_right, move_spin_left, stop
};

uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0u;
uint32_t SystemCoreClock = 168000000u;
uint32_t SysTick_reload = 0u;
uint32_t usart1_baud_used = 0u;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0u;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS] = {{0}};

static CircArray msg;
static int16_t accel_raw[3] = {0, 0, 0};
static uint16_t adc_raw = 0u;
static volatile bool adc_eoc = true;
static volatile bool rng_ready = false;
static uint32_t rng_value = 0u;
static volatile bool mock_usart_rxne = false;
static uint8_t mock_usart_rx_data = 0u;
static ButtonState last_buttstate = ButtonIsReleased;

void SystemInit(void) {
    SystemCoreClock = 168000000u;
}

void init_systick(void) {
    SystemCoreClockUpdate();
    uint32_t ticks = SystemCoreClock / 1000u;
    if (SysTick_Config(ticks) != 0) {
        while (1) {}
    }
}

void init_LED_pins(void) {
    GPIOD_output[12] = 0u;
    GPIOD_output[13] = 0u;
    GPIOD_output[14] = 0u;
    GPIOD_output[15] = 0u;
}

void init_button(void) {
    GPIOA_IDR &= ~1u;
    b_i = 0u;
    buttstate = ButtonIsReleased;
    last_buttstate = ButtonIsReleased;
}

int init_accelerometers(void) {
    return 0;
}

void init_rng(void) {
}

void init_temperature_sensor(void) {
    adc_eoc = true;
}

void init_GPIO_A1A2A3A4_output(void) {
    GPIOA_output[1] = 0u;
    GPIOA_output[2] = 0u;
    GPIOA_output[3] = 0u;
    GPIOA_output[4] = 0u;
}

void init_usart1(int baud) {
    if (baud == 0) {
        baud = UART_BAUD;
        fprintf(stderr, "Warning: baud 0 set to 9600\n");
    }
    usart1_baud_used = (uint32_t)baud;
    usart1_tx_length = 0u;
    usart1_tx_log[0] = '\0';
    buf_delete(&msg);
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
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
    init_usart1(UART_BAUD);
    usart1_send((volatile char *)discobot_startup_banner);
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    int a1 = 0;
    int a2 = 0;
    switch (direc) {
        case FORWARD:
            a1 = 1;
            a2 = 0;
            break;
        case BACKWARD:
            a1 = 0;
            a2 = 1;
            break;
        case STOP:
        default:
            a1 = 0;
            a2 = 0;
            break;
    }
    GPIOA_output[1] = (uint8_t)a1;
    GPIOA_output[2] = (uint8_t)a2;
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    int a3 = 0;
    int a4 = 0;
    switch (direc) {
        case FORWARD:
            a3 = 1;
            a4 = 0;
            break;
        case BACKWARD:
            a3 = 0;
            a4 = 1;
            break;
        case STOP:
        default:
            a3 = 0;
            a4 = 0;
            break;
    }
    GPIOA_output[3] = (uint8_t)a3;
    GPIOA_output[4] = (uint8_t)a4;
}

void move_forward(void) {
    GPIOA_output[1] = 1u; GPIOA_output[2] = 0u;
    GPIOA_output[3] = 1u; GPIOA_output[4] = 0u;
}

void move_backward(void) {
    GPIOA_output[1] = 0u; GPIOA_output[2] = 1u;
    GPIOA_output[3] = 0u; GPIOA_output[4] = 1u;
}

void move_forward_soft_left(void) {
    GPIOA_output[1] = 0u; GPIOA_output[2] = 0u;
    GPIOA_output[3] = 1u; GPIOA_output[4] = 0u;
}

void move_forward_soft_right(void) {
    GPIOA_output[1] = 1u; GPIOA_output[2] = 0u;
    GPIOA_output[3] = 0u; GPIOA_output[4] = 0u;
}

void move_backward_soft_left(void) {
    GPIOA_output[1] = 0u; GPIOA_output[2] = 0u;
    GPIOA_output[3] = 0u; GPIOA_output[4] = 1u;
}

void move_backward_soft_right(void) {
    GPIOA_output[1] = 0u; GPIOA_output[2] = 1u;
    GPIOA_output[3] = 0u; GPIOA_output[4] = 0u;
}

void move_spin_right(void) {
    GPIOA_output[1] = 1u; GPIOA_output[2] = 0u;
    GPIOA_output[3] = 0u; GPIOA_output[4] = 1u;
}

void move_spin_left(void) {
    GPIOA_output[1] = 0u; GPIOA_output[2] = 1u;
    GPIOA_output[3] = 1u; GPIOA_output[4] = 0u;
}

void stop(void) {
    GPIOA_output[1] = 0u; GPIOA_output[2] = 0u;
    GPIOA_output[3] = 0u; GPIOA_output[4] = 0u;
}

bool dispatch_uart_command(int command) {
    if (command >= 0 && command <= 8) {
        msgid = (_ID)command;
        callme = flookup[command];
        if (callme != 0) {
            callme();
        }
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    static uint32_t last_1000ms = 0u;
    if (usart1_available() > 0u) {
        int c = (int)(signed char)usart1_readc();
        (void)dispatch_uart_command(c);
    }
    checkbutton();
    execute_tasks();
    if ((msTicks - last_1000ms) > 1000u) {
        last_1000ms = msTicks;
    }
}

int read_buttonc(int i) {
    if (i < 0 || i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> i) & 1u);
}

void SysTick_Handler(void) {
    msTicks++;
    if (read_buttonc(0) == 1) {
        b_i++;
        if (b_i >= BUTTON_DEBOUNCE_MS) {
            buttstate = ButtonIsPressed;
        }
    } else {
        b_i = 0u;
        buttstate = ButtonIsReleased;
    }
}

void checkbutton(void) {
    if (last_buttstate != buttstate) {
        switch (buttstate) {
            case ButtonIsReleased:
                break;
            case ButtonIsPressed:
                break;
            default:
                break;
        }
        last_buttstate = buttstate;
    }
}

void discobot_set_button_level(bool high) {
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    const float pi = 3.14159265358979323846f;
    float root = sqrtf(acc_y * acc_y + acc_z * acc_z);
    if (roll != 0) {
        *roll = atan2f(acc_y, acc_z) * 180.0f / pi;
    }
    if (pitch != 0) {
        *pitch = atan2f(-acc_x, root) * 180.0f / pi;
    }
}

void LED_On(int i) {
    if (i < 0 || i >= 4) {
        return;
    }
    GPIOD_output[12 + i] = 1u;
}

void LED_Off(int i) {
    if (i < 0 || i >= 4) {
        return;
    }
    GPIOD_output[12 + i] = 0u;
}

void read_accelerometers(float b[3]) {
    if (b == 0) {
        return;
    }
    b[0] = accel_raw[0] / 1000.0f;
    b[1] = accel_raw[1] / 1000.0f;
    b[2] = accel_raw[2] / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    accel_raw[0] = x_mg;
    accel_raw[1] = y_mg;
    accel_raw[2] = z_mg;
}

float read_temperature_sensor(void) {
    while (!adc_eoc) {}
    float temp = (float)adc_raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    adc_raw = raw;
    adc_eoc = true;
}

uint32_t get_random_number(void) {
    while (!rng_ready) {}
    return rng_value;
}

void discobot_set_rng(bool ready, uint32_t value) {
    rng_ready = ready;
    rng_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    long ms = (long)(interval_sec * 1000.0f);
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == 0) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)ms;
            timed_tasks[i].last_called = 0u;
            timed_tasks[i].numcalls = 0u;
            return;
        }
    }
}

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        TimedTask *t = &timed_tasks[i];
        if (t->task != 0) {
            if (msTicks >= (t->last_called + t->msinterval)) {
                t->task();
                t->last_called = msTicks;
                t->numcalls++;
            }
        }
    }
}

void printtimes(void) {
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task != 0) {
            printf("t%d=%u", i, (unsigned)timed_tasks[i].numcalls);
        }
    }
}

void initCircArray(CircArray *arr, int size) {
    if (arr == 0) {
        return;
    }
    if (arr->enabled) {
        fprintf(stderr, "CircArray already enabled\n");
        return;
    }
    if (size <= 0) {
        size = 1;
    }
    arr->buf = (char *)calloc((size_t)size, 1);
    if (arr->buf == 0) {
        arr->size = 0u;
        arr->enabled = false;
        return;
    }
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0u;
    arr->n_w = 0u;
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == 0 || arr->buf == 0 || arr->size == 0u) {
        return 0;
    }
    if (!buf_full(arr)) {
        arr->buf[arr->n_w % arr->size] = c;
        arr->n_w++;
        return 1;
    }
    return 0;
}

char buf_getbyte(CircArray *arr) {
    if (arr == 0 || arr->buf == 0 || arr->size == 0u || buf_empty(arr)) {
        return 0;
    }
    char value = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return value;
}

bool buf_empty(CircArray *arr) {
    if (arr == 0) {
        return true;
    }
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (arr == 0 || arr->size == 0u) {
        return false;
    }
    return !buf_empty(arr) && (arr->n_r % arr->size) == (arr->n_w % arr->size);
}

int buf_available(CircArray *arr) {
    if (arr == 0) {
        return 0;
    }
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == 0 || newSize <= 0) {
        return false;
    }
    char *newBuf = (char *)realloc(arr->buf, (size_t)newSize);
    if (newBuf == 0) {
        return false;
    }
    arr->buf = newBuf;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr) {
    if (arr == 0) {
        return false;
    }
    buf_clear(arr);
    free(arr->buf);
    arr->buf = 0;
    arr->size = 0u;
    arr->enabled = false;
    arr->n_r = 0u;
    arr->n_w = 0u;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == 0 || arr->buf == 0) {
        return;
    }
    memset(arr->buf, 0, arr->size);
}

void USART1_IRQHandler(void) {
    if (mock_usart_rxne) {
        (void)buf_putbyte(&msg, (char)mock_usart_rx_data);
        mock_usart_rxne = false;
    }
}

void usart1_send(volatile char *s) {
    if (s == 0) {
        return;
    }
    while (*s) {
        if (usart1_tx_length < sizeof(usart1_tx_log) - 1u) {
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
    return (char)(signed char)usart1_read();
}

uint32_t usart1_available(void) {
    return (uint32_t)buf_available(&msg);
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    mock_usart_rx_data = value;
    mock_usart_rxne = rxne;
    USART1_IRQHandler();
}

int func2(int R0) {
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0) {
    return func2(R0);
}