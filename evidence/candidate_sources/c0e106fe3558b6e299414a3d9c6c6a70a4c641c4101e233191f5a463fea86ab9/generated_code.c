#include "generated_code.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

uint32_t SystemCoreClock = 168000000UL;
volatile uint32_t msTicks = 0;
uint32_t SysTick_reload = 0;
int SysTick_configured = 0;
int SysTick_config_failed = 0;
int discobot_force_systick_fail = 0;

int GPIOA_output[16] = {0};
int GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;

volatile ButtonState buttstate = ButtonIsReleased;
volatile ButtonState laststate = ButtonIsReleased;
volatile int b_i = 0;

CircArray msg = {0};
timed_task timed_tasks[MAXNUMTASKS] = {{0}};

USART_TypeDef USART1_instance = {0x40U, 0U};

char usart1_tx_log[DISCOBOT_TX_LOG_CAPACITY] = {0};
uint32_t usart1_tx_length = 0;
int usart1_baud_configured = 0;
int usart1_enabled = 0;
int usart1_rxne_interrupt_enabled = 0;
int usart1_nvic_preempt_priority = 0;
int usart1_nvic_sub_priority = 0;

int accelerometer_initialized = 0;
int32_t discobot_acc_raw_x = 0;
int32_t discobot_acc_raw_y = 0;
int32_t discobot_acc_raw_z = 0;

int temperature_sensor_initialized = 0;
uint16_t discobot_adc_raw = 943;
int discobot_adc_eoc = SET;

int rng_initialized = 0;
bool discobot_rng_ready = false;
uint32_t discobot_rng_value = 0;
int discobot_rng_wait_blocked = 0;

int led_initialized = 0;
int button_initialized = 0;
int motor_gpio_initialized = 0;

int discobot_init_sequence[16] = {0};
int discobot_init_sequence_length = 0;

static uint32_t main_loop_t_prev = 0;

static void record_init_step(int step) {
    if (discobot_init_sequence_length < (int)(sizeof(discobot_init_sequence) / sizeof(discobot_init_sequence[0]))) {
        discobot_init_sequence[discobot_init_sequence_length++] = step;
    }
}

static void set_gpioa_pin(int pin, int value) {
    if (pin >= 0 && pin < 16) {
        GPIOA_output[pin] = value ? 1 : 0;
    }
}

static void set_gpiod_pin(int pin, int value) {
    if (pin >= 0 && pin < 16) {
        GPIOD_output[pin] = value ? 1 : 0;
    }
}

void SystemInit(void) {
    SystemCoreClock = 168000000UL;
    record_init_step(1);
}

void SystemCoreClockUpdate(void) {
    SystemCoreClock = 168000000UL;
}

int SysTick_Config(uint32_t ticks) {
    SysTick_reload = ticks;
    if (discobot_force_systick_fail) {
        SysTick_configured = 0;
        SysTick_config_failed = 1;
        return 1;
    }
    SysTick_configured = 1;
    SysTick_config_failed = 0;
    return 0;
}

void init_systick(void) {
    record_init_step(2);
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000UL) != 0) {
        SysTick_config_failed = 1;
        return;
    }
}

void SysTick_Handler(void) {
    msTicks++;
    if (read_buttonc(0) == 1) {
        b_i++;
        if (b_i >= BUTTON_DEBOUNCE_MS) {
            buttstate = ButtonIsPressed;
        }
    } else {
        b_i = 0;
        buttstate = ButtonIsReleased;
    }
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

void init_GPIO_A1A2A3A4_output(void) {
    record_init_step(8);
    motor_gpio_initialized = 1;
    set_gpioa_pin(1, 0);
    set_gpioa_pin(2, 0);
    set_gpioa_pin(3, 0);
    set_gpioa_pin(4, 0);
}

int read_buttonc(int i) {
    if (i > 3 || i < 0) {
        return -1;
    }
    return (int)((GPIOA_IDR >> i) & 1U);
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

void init_button(void) {
    record_init_step(4);
    button_initialized = 1;
    GPIOA_IDR &= ~1U;
    b_i = 0;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
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
    record_init_step(3);
    led_initialized = 1;
    for (int i = 12; i <= 15; ++i) {
        set_gpiod_pin(i, 0);
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
    record_init_step(5);
    accelerometer_initialized = 1;
    return 0;
}

void read_accelerometers(float *acc[3]) {
    if (acc != NULL) {
        if (acc[0] != NULL) {
            *(acc[0]) = ((float)discobot_acc_raw_x) / 1000.0f;
        }
        if (acc[1] != NULL) {
            *(acc[1]) = ((float)discobot_acc_raw_y) / 1000.0f;
        }
        if (acc[2] != NULL) {
            *(acc[2]) = ((float)discobot_acc_raw_z) / 1000.0f;
        }
    }
}

void discobot_set_accelerometer_raw(int32_t x_mg, int32_t y_mg, int32_t z_mg) {
    discobot_acc_raw_x = x_mg;
    discobot_acc_raw_y = y_mg;
    discobot_acc_raw_z = z_mg;
}

void init_temperature_sensor(void) {
    record_init_step(7);
    temperature_sensor_initialized = 1;
    discobot_adc_eoc = SET;
}

float read_temperature_sensor(void) {
    discobot_adc_eoc = SET;
    while (discobot_adc_eoc == RESET) {
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
    discobot_adc_raw = (uint16_t)(raw & 0x0FFFU);
    discobot_adc_eoc = SET;
}

void init_rng(void) {
    record_init_step(6);
    rng_initialized = 1;
}

uint32_t get_random_number(void) {
    discobot_rng_wait_blocked = 0;
    if (!discobot_rng_ready) {
        discobot_rng_wait_blocked = 1;
        return 0U;
    }
    return discobot_rng_value;
}

void discobot_set_rng(bool ready, uint32_t value) {
    discobot_rng_ready = ready;
    discobot_rng_value = value;
    discobot_rng_wait_blocked = 0;
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (long)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            return;
        }
    }
}

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task != NULL) {
            uint32_t due = timed_tasks[i].last_called + (uint32_t)timed_tasks[i].msinterval;
            if (msTicks >= due) {
                timed_tasks[i].task();
                timed_tasks[i].last_called = msTicks;
                timed_tasks[i].numcalls++;
            }
        }
    }
}

void printtimes(void) {
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task != NULL) {
            printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
        }
    }
}

void init_usart1(int baud) {
    record_init_step(9);
    if (baud == 0) {
        baud = UART_BAUD;
        fprintf(stderr, "Warning: USART1 baud 0, using 9600\n");
    }
    usart1_baud_configured = baud;
    usart1_enabled = 1;
    usart1_rxne_interrupt_enabled = 1;
    usart1_nvic_preempt_priority = 0;
    usart1_nvic_sub_priority = 0;
    USART1->SR |= 0x40U;
    if (!msg.enabled) {
        initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    }
}

void USART1_IRQHandler(void) {
    if ((USART1->SR & 0x20U) != 0U) {
        char c = (char)(USART1->DR & 0xFFU);
        (void)buf_putbyte(&msg, c);
        USART1->SR &= ~0x20U;
    }
}

void USART_SendData(USART_TypeDef *usart, uint16_t data) {
    if (usart != NULL) {
        usart->DR = (uint32_t)(data & 0xFFU);
        usart->SR |= 0x40U;
    }
    if (usart1_tx_length + 1U < DISCOBOT_TX_LOG_CAPACITY) {
        usart1_tx_log[usart1_tx_length++] = (char)(data & 0xFFU);
        usart1_tx_log[usart1_tx_length] = '\0';
    }
}

void usart1_send(volatile char *s) {
    if (s == NULL) {
        return;
    }
    while (*s) {
        while ((USART1->SR & 0x40U) == 0U) {
        }
        USART_SendData(USART1, (uint16_t)(uint8_t)(*s));
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
    USART1->SR |= 0x20U;
    USART1_IRQHandler();
}

void discobot_clear_usart1_tx_log(void) {
    memset(usart1_tx_log, 0, sizeof(usart1_tx_log));
    usart1_tx_length = 0;
}

void initCircArray(CircArray *arr, int size) {
    if (arr == NULL || size <= 0) {
        return;
    }
    if (arr->enabled) {
        printf("CircArray already enabled\n");
        return;
    }
    arr->buf = (char *)calloc((size_t)size, 1U);
    if (arr->buf == NULL) {
        arr->size = 0;
        arr->enabled = false;
        arr->n_r = 0;
        arr->n_w = 0;
        return;
    }
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0U || buf_full(arr)) {
        return 0;
    }
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0U || buf_empty(arr)) {
        return 0;
    }
    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

bool buf_empty(CircArray *arr) {
    if (arr == NULL) {
        return true;
    }
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (arr == NULL || arr->size == 0U) {
        return false;
    }
    return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
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
    void *p = realloc(arr->buf, (size_t)newSize);
    if (p == NULL) {
        return false;
    }
    arr->buf = (char *)p;
    arr->size = (uint32_t)newSize;
    if (arr->n_r > arr->n_w) {
        arr->n_r = arr->n_w;
    }
    return true;
}

bool buf_delete(CircArray *arr) {
    if (arr == NULL) {
        return false;
    }
    buf_clear(arr);
    free(arr->buf);
    arr->buf = NULL;
    arr->size = 0;
    arr->enabled = false;
    arr->n_r = 0;
    arr->n_w = 0;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0U) {
        return;
    }
    memset(arr->buf, 0, arr->size);
}

void dispatch_uart_command(void) {
    if (usart1_available() == 0U) {
        return;
    }
    char c = usart1_readc();
    int command = (int)c;
    if (command >= 0 && command < MAXIDSIZE) {
        motor_command_fn callme = flookup[command];
        if (callme != NULL) {
            callme();
        }
    }
}

void main_loop_iteration(void) {
    if ((uint32_t)(msTicks - main_loop_t_prev) > 1000U) {
        main_loop_t_prev = msTicks;
    }
    dispatch_uart_command();
}

void init_system(void) {
    discobot_init_sequence_length = 0;
    SystemInit();
    init_systick();
    init_LED_pins();
    init_button();
    (void)init_accelerometers();
    init_rng();
    init_temperature_sensor();
    init_GPIO_A1A2A3A4_output();
    init_usart1(UART_BAUD);
    usart1_send((volatile char *)"UART1 Initialized. @9600bps\r\n");
}

void discobot_set_button_level(int level) {
    if (level) {
        GPIOA_IDR |= 1U;
    } else {
        GPIOA_IDR &= ~1U;
    }
}

void discobot_reset_host_state(void) {
    memset(GPIOA_output, 0, sizeof(GPIOA_output));
    memset(GPIOD_output, 0, sizeof(GPIOD_output));
    GPIOA_IDR = 0;
    msTicks = 0;
    SysTick_reload = 0;
    SysTick_configured = 0;
    SysTick_config_failed = 0;
    discobot_force_systick_fail = 0;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
    b_i = 0;
    main_loop_t_prev = 0;
    memset(timed_tasks, 0, sizeof(timed_tasks));
    discobot_clear_usart1_tx_log();
    usart1_baud_configured = 0;
    usart1_enabled = 0;
    usart1_rxne_interrupt_enabled = 0;
    USART1_instance.SR = 0x40U;
    USART1_instance.DR = 0U;
    accelerometer_initialized = 0;
    discobot_acc_raw_x = 0;
    discobot_acc_raw_y = 0;
    discobot_acc_raw_z = 0;
    temperature_sensor_initialized = 0;
    discobot_adc_raw = 943;
    discobot_adc_eoc = SET;
    rng_initialized = 0;
    discobot_rng_ready = false;
    discobot_rng_value = 0;
    discobot_rng_wait_blocked = 0;
    led_initialized = 0;
    button_initialized = 0;
    motor_gpio_initialized = 0;
    discobot_init_sequence_length = 0;
    if (msg.enabled) {
        (void)buf_delete(&msg);
    } else {
        msg.buf = NULL;
        msg.size = 0;
        msg.n_r = 0;
        msg.n_w = 0;
        msg.enabled = false;
    }
}

int func2(int R0) {
    return R0 + 1;
}

int func1(int R0) {
    return func2(R0);
}
