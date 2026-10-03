#include "generated_code.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

uint32_t SystemCoreClock = 168000000u;
uint32_t SysTick_reload = 0u;
uint32_t msTicks = 0u;
uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0u;

int gpioa_clock_enabled = 0;
int gpiob_clock_enabled = 0;
int gpiod_clock_enabled = 0;
int rng_clock_enabled = 0;
int adc1_clock_enabled = 0;
int usart1_enabled = 0;
int usart1_baud = 0;
int usart1_rxne_interrupt_enabled = 0;
int nvic_usart1_preempt_priority = -1;
int nvic_usart1_sub_priority = -1;
int systick_failed = 0;
int systick_config_result = 0;

int button_pin_initialized = 0;
int led_pins_initialized = 0;
int motor_pins_initialized = 0;
int accelerometer_initialized = 0;
int rng_initialized = 0;
int temperature_sensor_initialized = 0;

uint32_t b_i = 0u;
ButtonState buttstate = ButtonIsReleased;
ButtonState laststate = ButtonIsReleased;

CircArray msg = {0};
timed_task timed_tasks[MAXNUMTASKS] = {0};

USART_TypeDef USART1_instance = {USART_SR_TXE, 0u};
USART_TypeDef *USART1 = &USART1_instance;

char usart1_tx_log[DISCOBOT_USART_TX_LOG_CAPACITY] = {0};
uint32_t usart1_tx_length = 0u;
int usart1_tx_overflow = 0;

int16_t discobot_accel_raw_x = 0;
int16_t discobot_accel_raw_y = 0;
int16_t discobot_accel_raw_z = 0;
uint16_t discobot_adc_raw = 943u;
bool discobot_adc_eoc = true;
bool discobot_rng_ready = true;
uint32_t discobot_rng_value = 0u;
bool discobot_rng_blocked = false;

int init_sequence_log[16] = {0};
uint32_t init_sequence_count = 0u;

motor_command_fn flookup[MAXIDSIZE] = {
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

static void log_init_step(int step) {
    if (init_sequence_count < (uint32_t)(sizeof(init_sequence_log) / sizeof(init_sequence_log[0]))) {
        init_sequence_log[init_sequence_count++] = step;
    }
}

void SystemInit(void) {
    SystemCoreClock = 168000000u;
    log_init_step(1);
}

void SystemCoreClockUpdate(void) {
    SystemCoreClock = 168000000u;
}

int SysTick_Config(uint32_t ticks) {
    SysTick_reload = ticks;
    return systick_config_result;
}

void init_systick(void) {
    log_init_step(2);
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000u) != 0) {
        systick_failed = 1;
    }
}

static void set_gpioa_pin(int pin, int value) {
    if (pin >= 0 && pin < 16) {
        GPIOA_output[pin] = value ? 1u : 0u;
    }
}

static void set_gpiod_pin(int pin, int value) {
    if (pin >= 0 && pin < 16) {
        GPIOD_output[pin] = value ? 1u : 0u;
    }
}

void init_GPIO_A1A2A3A4_output(void) {
    log_init_step(8);
    gpioa_clock_enabled = 1;
    motor_pins_initialized = 1;
    set_gpioa_pin(1, 0);
    set_gpioa_pin(2, 0);
    set_gpioa_pin(3, 0);
    set_gpioa_pin(4, 0);
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) {
        set_gpioa_pin(1, 1);
        set_gpioa_pin(2, 0);
    } else if (direc == BACKWARD) {
        set_gpioa_pin(1, 0);
        set_gpioa_pin(2, 1);
    } else {
        set_gpioa_pin(1, 0);
        set_gpioa_pin(2, 0);
    }
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) {
        set_gpioa_pin(3, 1);
        set_gpioa_pin(4, 0);
    } else if (direc == BACKWARD) {
        set_gpioa_pin(3, 0);
        set_gpioa_pin(4, 1);
    } else {
        set_gpioa_pin(3, 0);
        set_gpioa_pin(4, 0);
    }
}

void move_forward(void) {
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void move_backward(void) {
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_forward_soft_left(void) {
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void move_forward_soft_right(void) {
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void move_backward_soft_left(void) {
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_backward_soft_right(void) {
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void move_spin_right(void) {
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_spin_left(void) {
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void stop(void) {
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

static void init_button(void) {
    log_init_step(4);
    gpioa_clock_enabled = 1;
    button_pin_initialized = 1;
    GPIOA_IDR &= ~1u;
    b_i = 0u;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
}

int read_buttonc(int i) {
    if (i > 3 || i < 0) {
        return -1;
    }
    return (int)((GPIOA_IDR >> (uint32_t)i) & 1u);
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

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    if (roll != NULL) {
        *roll = atan2f(acc_y, acc_z) * 180.0f / PI;
    }
    if (pitch != NULL) {
        *pitch = atan2f(-acc_x, sqrtf((acc_y * acc_y) + (acc_z * acc_z))) * 180.0f / PI;
    }
}

void init_LED_pins(void) {
    log_init_step(3);
    gpiod_clock_enabled = 1;
    led_pins_initialized = 1;
    for (int pin = 12; pin <= 15; pin++) {
        set_gpiod_pin(pin, 0);
    }
}

void LED_On(int i) {
    if (i >= 0 && i < 4) {
        set_gpiod_pin(12 + i, 1);
    }
}

void LED_Off(int i) {
    if (i >= 0 && i < 4) {
        set_gpiod_pin(12 + i, 0);
    }
}

int init_accelerometers(void) {
    log_init_step(5);
    accelerometer_initialized = 1;
    return 0;
}

void read_accelerometers(float *acc[3]) {
    if (acc != NULL) {
        if (acc[0] != NULL) {
            *acc[0] = (float)discobot_accel_raw_x / 1000.0f;
        }
        if (acc[1] != NULL) {
            *acc[1] = (float)discobot_accel_raw_y / 1000.0f;
        }
        if (acc[2] != NULL) {
            *acc[2] = (float)discobot_accel_raw_z / 1000.0f;
        }
    }
}

void discobot_set_accelerometer_raw(int16_t x, int16_t y, int16_t z) {
    discobot_accel_raw_x = x;
    discobot_accel_raw_y = y;
    discobot_accel_raw_z = z;
}

void init_temperature_sensor(void) {
    log_init_step(7);
    adc1_clock_enabled = 1;
    temperature_sensor_initialized = 1;
    discobot_adc_eoc = true;
}

float read_temperature_sensor(void) {
    discobot_adc_eoc = true;
    while (!discobot_adc_eoc) {
    }
    float temp = (float)discobot_adc_raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    discobot_adc_raw = (uint16_t)(raw & 0x0FFFu);
    discobot_adc_eoc = true;
}

void init_rng(void) {
    log_init_step(6);
    rng_clock_enabled = 1;
    rng_initialized = 1;
}

uint32_t get_random_number(void) {
    discobot_rng_blocked = false;
    if (!discobot_rng_ready) {
        discobot_rng_blocked = true;
        return 0u;
    }
    return discobot_rng_value;
}

void discobot_set_rng(bool ready, uint32_t value) {
    discobot_rng_ready = ready;
    discobot_rng_value = value;
    discobot_rng_blocked = false;
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (long)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0u;
            timed_tasks[i].numcalls = 0u;
            return;
        }
    }
}

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL &&
            msTicks >= timed_tasks[i].last_called + (uint32_t)timed_tasks[i].msinterval) {
            timed_tasks[i].task();
            timed_tasks[i].last_called = msTicks;
            timed_tasks[i].numcalls++;
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

void initCircArray(CircArray *arr, int size) {
    if (arr == NULL || size <= 0) {
        return;
    }
    if (arr->enabled) {
        printf("Error: CircArray already enabled\n");
        return;
    }
    arr->buf = (char *)calloc((size_t)size, 1u);
    if (arr->buf == NULL) {
        arr->size = 0u;
        arr->enabled = false;
        arr->n_r = 0u;
        arr->n_w = 0u;
        return;
    }
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0u;
    arr->n_w = 0u;
}

bool buf_empty(CircArray *arr) {
    return arr == NULL || arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (arr == NULL || arr->size == 0u || buf_empty(arr)) {
        return false;
    }
    return (arr->n_r % arr->size) == (arr->n_w % arr->size);
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0u || buf_full(arr)) {
        return 0;
    }
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0u || buf_empty(arr)) {
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
    char *newBuf = (char *)realloc(arr->buf, (size_t)newSize);
    if (newBuf == NULL) {
        return false;
    }
    arr->buf = newBuf;
    arr->size = (uint32_t)newSize;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr != NULL && arr->buf != NULL && arr->size > 0u) {
        memset(arr->buf, 0, arr->size);
    }
}

bool buf_delete(CircArray *arr) {
    if (arr == NULL) {
        return false;
    }
    buf_clear(arr);
    free(arr->buf);
    arr->buf = NULL;
    arr->size = 0u;
    arr->enabled = false;
    arr->n_r = 0u;
    arr->n_w = 0u;
    return true;
}

void init_usart1(int baud) {
    log_init_step(9);
    if (baud == 0) {
        baud = UART_BAUD;
        printf("Warning: baud was 0, using 9600\n");
    }
    gpiob_clock_enabled = 1;
    usart1_baud = baud;
    usart1_rxne_interrupt_enabled = 1;
    nvic_usart1_preempt_priority = 0;
    nvic_usart1_sub_priority = 0;
    if (!msg.enabled) {
        initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    }
    USART1->SR |= USART_SR_TXE;
    usart1_enabled = 1;
}

void USART1_IRQHandler(void) {
    if ((USART1->SR & USART_SR_RXNE) != 0u) {
        char c = (char)(USART1->DR & 0xFFu);
        (void)buf_putbyte(&msg, c);
        USART1->SR &= ~USART_SR_RXNE;
    }
}

void usart1_send(volatile char *s) {
    if (s == NULL) {
        return;
    }
    while (*s) {
        while ((USART1->SR & USART_SR_TXE) == 0u) {
        }
        if (usart1_tx_length + 1u < DISCOBOT_USART_TX_LOG_CAPACITY) {
            usart1_tx_log[usart1_tx_length++] = (char)*s;
            usart1_tx_log[usart1_tx_length] = '\0';
        } else {
            usart1_tx_overflow = 1;
        }
        USART1->DR = (uint32_t)(uint8_t)*s;
        s++;
    }
}

uint8_t usart1_read(void) {
    return (uint8_t)buf_getbyte(&msg);
}

char usart1_readc(void) {
    return (char)buf_getbyte(&msg);
}

uint32_t usart1_available(void) {
    return (uint32_t)buf_available(&msg);
}

void discobot_usart1_inject_rx(uint8_t c) {
    USART1->DR = (uint32_t)c;
    USART1->SR |= USART_SR_RXNE;
    USART1_IRQHandler();
}

void discobot_clear_usart1_tx_log(void) {
    memset(usart1_tx_log, 0, sizeof(usart1_tx_log));
    usart1_tx_length = 0u;
    usart1_tx_overflow = 0;
}

void dispatch_uart_command(void) {
    if (usart1_available() == 0u) {
        return;
    }
    char c = usart1_readc();
    if (c >= 0 && c < MAXIDSIZE) {
        motor_command_fn callme = flookup[(int)c];
        if (callme != NULL) {
            callme();
        }
    }
}

void main_loop_iteration(void) {
    static uint32_t t_prev = 0u;
    if (msTicks - t_prev > 1000u) {
        t_prev = msTicks;
    }
    dispatch_uart_command();
}

void init_system(void) {
    SystemInit();
    init_systick();
    if (systick_failed) {
        return;
    }
    init_LED_pins();
    init_button();
    (void)init_accelerometers();
    init_rng();
    init_temperature_sensor();
    init_GPIO_A1A2A3A4_output();
    init_usart1(UART_BAUD);
    usart1_send((volatile char *)"UART1 Initialized. @9600bps\r\n");
}

void discobot_set_button_level(int high) {
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

void discobot_reset_host_state(void) {
    buf_delete(&msg);
    memset(GPIOA_output, 0, sizeof(GPIOA_output));
    memset(GPIOD_output, 0, sizeof(GPIOD_output));
    memset(timed_tasks, 0, sizeof(timed_tasks));
    memset(init_sequence_log, 0, sizeof(init_sequence_log));
    init_sequence_count = 0u;
    GPIOA_IDR = 0u;
    SystemCoreClock = 168000000u;
    SysTick_reload = 0u;
    msTicks = 0u;
    gpioa_clock_enabled = 0;
    gpiob_clock_enabled = 0;
    gpiod_clock_enabled = 0;
    rng_clock_enabled = 0;
    adc1_clock_enabled = 0;
    usart1_enabled = 0;
    usart1_baud = 0;
    usart1_rxne_interrupt_enabled = 0;
    nvic_usart1_preempt_priority = -1;
    nvic_usart1_sub_priority = -1;
    systick_failed = 0;
    systick_config_result = 0;
    button_pin_initialized = 0;
    led_pins_initialized = 0;
    motor_pins_initialized = 0;
    accelerometer_initialized = 0;
    rng_initialized = 0;
    temperature_sensor_initialized = 0;
    b_i = 0u;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
    USART1_instance.SR = USART_SR_TXE;
    USART1_instance.DR = 0u;
    discobot_accel_raw_x = 0;
    discobot_accel_raw_y = 0;
    discobot_accel_raw_z = 0;
    discobot_adc_raw = 943u;
    discobot_adc_eoc = true;
    discobot_rng_ready = true;
    discobot_rng_value = 0u;
    discobot_rng_blocked = false;
    discobot_clear_usart1_tx_log();
}

int func2(int R0) {
    return R0 + 1;
}

int func1(int R0) {
    return func2(R0);
}
