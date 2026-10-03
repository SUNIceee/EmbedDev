#include "generated_code.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ------------------------------------------------------------------------- */
/* Host-observable state                                                     */
/* ------------------------------------------------------------------------- */
uint8_t GPIOA_output[5]   = {0};
uint8_t GPIOD_output[16]  = {0};
uint32_t GPIOA_IDR        = 0;
uint32_t SysTick_reload   = 0;
volatile uint32_t msTicks = 0;
CircArray msg             = {0};

char *usart1_tx_log       = NULL;
size_t usart1_tx_length   = 0;
size_t usart1_tx_capacity = 0;
uint32_t usart1_baud      = UART_BAUD;
volatile bool usart1_rxne_pending = false;
uint8_t usart1_rx_data    = 0;

motor_command_fn flookup[9] = {
    move_forward,
    move_backward,
    move_forward_soft_left,
    move_forward_soft_right,
    move_backward_soft_left,
    move_backward_soft_right,
    move_spin_right,
    move_spin_left,
    stop
};

timed_task_t timed_tasks[MAXNUMTASKS] = {0};

int buttstate    = ButtonIsReleased;
int laststate    = ButtonIsReleased;
uint32_t b_i     = 0;
uint32_t t_prev  = 0;

int32_t discobot_accel_raw[3] = {0};
uint16_t discobot_adc_raw     = 0;
uint32_t discobot_rng_value   = 0;
bool discobot_rng_ready       = false;

/* ------------------------------------------------------------------------- */
/* Car / motors                                                              */
/* ------------------------------------------------------------------------- */
void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) {
        GPIOA_output[1] = 1;
        GPIOA_output[2] = 0;
    } else if (direc == BACKWARD) {
        GPIOA_output[1] = 0;
        GPIOA_output[2] = 1;
    } else {
        GPIOA_output[1] = 0;
        GPIOA_output[2] = 0;
    }
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) {
        GPIOA_output[3] = 1;
        GPIOA_output[4] = 0;
    } else if (direc == BACKWARD) {
        GPIOA_output[3] = 0;
        GPIOA_output[4] = 1;
    } else {
        GPIOA_output[3] = 0;
        GPIOA_output[4] = 0;
    }
}

void move_forward(void)           { set_left_motor_direc(FORWARD, 0);   set_right_motor_direc(FORWARD, 0);   }
void move_backward(void)          { set_left_motor_direc(BACKWARD, 0);  set_right_motor_direc(BACKWARD, 0);  }
void move_forward_soft_left(void)  { set_left_motor_direc(STOP, 0);      set_right_motor_direc(FORWARD, 0);   }
void move_forward_soft_right(void) { set_left_motor_direc(FORWARD, 0);   set_right_motor_direc(STOP, 0);      }
void move_backward_soft_left(void) { set_left_motor_direc(STOP, 0);      set_right_motor_direc(BACKWARD, 0);  }
void move_backward_soft_right(void){ set_left_motor_direc(BACKWARD, 0);  set_right_motor_direc(STOP, 0);      }
void move_spin_right(void)         { set_left_motor_direc(FORWARD, 0);   set_right_motor_direc(BACKWARD, 0);  }
void move_spin_left(void)          { set_left_motor_direc(BACKWARD, 0);  set_right_motor_direc(FORWARD, 0);   }
void stop(void)                    { set_left_motor_direc(STOP, 0);      set_right_motor_direc(STOP, 0);      }

void init_GPIO_A1A2A3A4_output(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

/* ------------------------------------------------------------------------- */
/* LED                                                                       */
/* ------------------------------------------------------------------------- */
void init_LED_pins(void) {
    GPIOD_output[12] = 0;
    GPIOD_output[13] = 0;
    GPIOD_output[14] = 0;
    GPIOD_output[15] = 0;
}

void LED_On(int i) {
    if (i >= 0 && i <= 3) {
        GPIOD_output[12 + i] = 1;
    }
}

void LED_Off(int i) {
    if (i >= 0 && i <= 3) {
        GPIOD_output[12 + i] = 0;
    }
}

/* ------------------------------------------------------------------------- */
/* Button                                                                    */
/* ------------------------------------------------------------------------- */
void init_button(void) {
    GPIOA_IDR &= ~(1u << 0);
    b_i = 0;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
}

int read_buttonc(int i) {
    if (i < 0 || i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> i) & 1u);
}

void checkbutton(void) {
    if (laststate != buttstate) {
        switch (buttstate) {
            case ButtonIsPressed:
                break;
            case ButtonIsReleased:
                break;
            default:
                break;
        }
        laststate = buttstate;
    }
}

void SysTick_Handler(void) {
    msTicks++;

    if (read_buttonc(0) == 1) {
        if (b_i < UINT32_MAX) {
            b_i++;
        }
        if (b_i >= BUTTON_DEBOUNCE_MS) {
            buttstate = ButtonIsPressed;
        }
    } else {
        b_i = 0;
        buttstate = ButtonIsReleased;
    }

    checkbutton();
}

/* ------------------------------------------------------------------------- */
/* Accelerometer                                                             */
/* ------------------------------------------------------------------------- */
int init_accelerometers(void) {
    discobot_accel_raw[0] = 0;
    discobot_accel_raw[1] = 0;
    discobot_accel_raw[2] = 0;
    return 0;
}

void read_accelerometers(float *acc[3]) {
    if (acc == NULL) {
        return;
    }
    if (acc[0] != NULL) {
        *acc[0] = discobot_accel_raw[0] / 1000.0f;
    }
    if (acc[1] != NULL) {
        *acc[1] = discobot_accel_raw[1] / 1000.0f;
    }
    if (acc[2] != NULL) {
        *acc[2] = discobot_accel_raw[2] / 1000.0f;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z,
                     float *pitch, float *roll) {
    if (roll != NULL) {
        *roll = atan2f(acc_y, acc_z) * 180.0f / (float)M_PI;
    }
    if (pitch != NULL) {
        *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) *
                 180.0f / (float)M_PI;
    }
}

/* ------------------------------------------------------------------------- */
/* Temperature sensor                                                        */
/* ------------------------------------------------------------------------- */
void init_temperature_sensor(void) {
    discobot_adc_raw = 0;
}

float read_temperature_sensor(void) {
    float temp = (float)discobot_adc_raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

/* ------------------------------------------------------------------------- */
/* RNG                                                                       */
/* ------------------------------------------------------------------------- */
void init_rng(void) {
    discobot_rng_ready = false;
    discobot_rng_value = 0;
}

uint32_t get_random_number(void) {
    while (!discobot_rng_ready) {
        /* Wait for DRDY. Host tests must set ready before calling. */
    }
    return discobot_rng_value;
}

/* ------------------------------------------------------------------------- */
/* CircArray                                                                 */
/* ------------------------------------------------------------------------- */
void initCircArray(CircArray *arr, int size) {
    if (arr == NULL) {
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
    if (arr->buf == NULL) {
        return;
    }
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

bool buf_empty(CircArray *arr) {
    if (arr == NULL) {
        return true;
    }
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (arr == NULL || arr->size == 0) {
        return false;
    }
    if (buf_empty(arr)) {
        return false;
    }
    return (arr->n_r % arr->size) == (arr->n_w % arr->size);
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0) {
        return 0;
    }
    if (buf_full(arr)) {
        return 0;
    }
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0) {
        return 0;
    }
    if (buf_empty(arr)) {
        return 0;
    }
    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

int buf_available(CircArray *arr) {
    if (arr == NULL) {
        return 0;
    }
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == NULL || newSize <= 0) {
        return false;
    }
    char *newbuf = (char *)realloc(arr->buf, (size_t)newSize);
    if (newbuf == NULL) {
        return false;
    }
    arr->buf = newbuf;
    arr->size = (uint32_t)newSize;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == NULL) {
        return;
    }
    if (arr->buf != NULL && arr->size > 0) {
        memset(arr->buf, 0, arr->size);
    }
}

bool buf_delete(CircArray *arr) {
    if (arr == NULL) {
        return false;
    }
    buf_clear(arr);
    if (arr->buf != NULL) {
        free(arr->buf);
    }
    arr->buf = NULL;
    arr->size = 0;
    arr->enabled = false;
    arr->n_r = 0;
    arr->n_w = 0;
    return true;
}

/* ------------------------------------------------------------------------- */
/* USART                                                                     */
/* ------------------------------------------------------------------------- */
static char *ensure_tx_capacity(size_t needed) {
    if (usart1_tx_log == NULL) {
        size_t initial = needed + 1;
        if (initial < 256) {
            initial = 256;
        }
        usart1_tx_log = (char *)calloc(initial, 1);
        if (usart1_tx_log == NULL) {
            return NULL;
        }
        usart1_tx_capacity = initial;
        usart1_tx_length = 0;
    }

    while (usart1_tx_length + needed >= usart1_tx_capacity) {
        size_t newcap = usart1_tx_capacity * 2;
        char *p = (char *)realloc(usart1_tx_log, newcap);
        if (p == NULL) {
            return NULL;
        }
        usart1_tx_log = p;
        usart1_tx_capacity = newcap;
    }
    return usart1_tx_log;
}

void usart1_send(volatile char *s) {
    if (s == NULL) {
        return;
    }
    while (*s != '\0') {
        if (ensure_tx_capacity(1) == NULL) {
            return;
        }
        usart1_tx_log[usart1_tx_length++] = *s;
        s++;
    }
    if (usart1_tx_log != NULL && usart1_tx_length < usart1_tx_capacity) {
        usart1_tx_log[usart1_tx_length] = '\0';
    }
}

void init_usart1(int baud) {
    usart1_baud = (baud == 0) ? UART_BAUD : (uint32_t)baud;
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    usart1_rxne_pending = false;
    usart1_rx_data = 0;
}

void USART1_IRQHandler(void) {
    if (usart1_rxne_pending) {
        usart1_rxne_pending = false;
        (void)buf_putbyte(&msg, (char)usart1_rx_data);
    }
}

uint8_t usart1_read(void) {
    return (uint8_t)buf_getbyte(&msg);
}

char usart1_readc(void) {
    if (buf_empty(&msg)) {
        return -1;
    }
    uint8_t b = (uint8_t)buf_getbyte(&msg);
    return (char)(signed char)b;
}

uint32_t usart1_available(void) {
    return (uint32_t)buf_available(&msg);
}

/* ------------------------------------------------------------------------- */
/* Timed task scheduler                                                      */
/* ------------------------------------------------------------------------- */
void add_timed_task(void (*myfunc)(void), float interval_sec) {
    if (myfunc == NULL) {
        return;
    }
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (long)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            return;
        }
    }
    /* All 25 slots full: silently fail. */
}

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL) {
            uint32_t due = timed_tasks[i].last_called +
                           (uint32_t)timed_tasks[i].msinterval;
            if (msTicks >= due) {
                timed_tasks[i].task();
                timed_tasks[i].last_called = msTicks;
                timed_tasks[i].numcalls++;
            }
        }
    }
}

void printtimes(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL) {
            printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* System / main-loop host model                                             */
/* ------------------------------------------------------------------------- */
void SystemInit(void) {
    /* Host no-op. */
}

void init_systick(void) {
    SysTick_reload = SYSTICK_PERIOD_TICKS;
    msTicks = 0;
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

void dispatch_uart_command(void) {
    int c = (int)(signed char)usart1_readc();
    if (c >= 0 && c <= 8) {
        if (flookup[c] != NULL) {
            flookup[c]();
        }
    }
}

void main_loop_iteration(void) {
    if ((msTicks - t_prev) > 1000) {
        t_prev = msTicks;
    }
    dispatch_uart_command();
}

/* ------------------------------------------------------------------------- */
/* Test injection helpers                                                    */
/* ------------------------------------------------------------------------- */
void discobot_set_button_level(int level) {
    if (level) {
        GPIOA_IDR |= (1u << 0);
    } else {
        GPIOA_IDR &= ~(1u << 0);
    }
}

void discobot_set_accelerometer_raw(int x, int y, int z) {
    discobot_accel_raw[0] = x;
    discobot_accel_raw[1] = y;
    discobot_accel_raw[2] = z;
}

void discobot_set_adc_raw(uint16_t raw) {
    discobot_adc_raw = raw;
}

void discobot_set_rng(uint32_t value, bool ready) {
    discobot_rng_value = value;
    discobot_rng_ready = ready;
}

void discobot_usart1_inject_rx(uint8_t byte) {
    usart1_rx_data = byte;
    usart1_rxne_pending = true;
    USART1_IRQHandler();
}

/* ------------------------------------------------------------------------- */
/* ARM assembly equivalents                                                  */
/* ------------------------------------------------------------------------- */
int func2(int R0) {
    uint32_t x = (uint32_t)R0;
    x += 1u;
    return (int)x;
}

int func1(int R0) {
    return func2(R0);
}
