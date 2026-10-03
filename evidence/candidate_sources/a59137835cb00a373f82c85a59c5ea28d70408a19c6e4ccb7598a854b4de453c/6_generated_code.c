#include "6_generated_code.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile uint32_t msTicks = 0U;
volatile uint32_t b_i = 0U;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = STOPCAR;
motor_command_fn callme = NULL;
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

uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0U;
uint32_t SystemCoreClock = 168000000U;
uint32_t SysTick_reload = 0U;
uint32_t usart1_baud_used = 0U;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0U;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";

TimedTask timed_tasks[MAXNUMTASKS];
CircArray msg;

volatile ButtonState laststate = ButtonIsReleased;
volatile int SysTick_init_error = 0;

static int16_t host_accel_x_mg = 0;
static int16_t host_accel_y_mg = 0;
static int16_t host_accel_z_mg = 0;

static uint16_t host_adc_raw_value = 0U;
static volatile uint32_t host_adc_eoc = 1U;

static bool rng_ready_flag = false;
static uint32_t rng_value_state = 0U;

static volatile uint8_t usart1_dr_byte = 0U;
static volatile bool usart1_rxne_pending = false;
static volatile bool usart1_enabled = false;
static volatile uint32_t usart1_sr = 0U;

static uint32_t t_prev = 0U;

#define ENABLE 1
#define DISABLE 0
#define RESET 0
#define ADC1 ((void*)0)
#define ADC_FLAG_EOC ((uint32_t)0x00000002U)
#define RCC_AHB2Periph_RNG ((uint32_t)0x00000001U)
#define RNG_FLAG_DRDY ((uint32_t)0x00000001U)
#define USART1 ((void*)0)
#define USART_IT_RXNE ((uint32_t)0x00000001U)
#define TM_LIS3DSH_Sensitivity_2G 0
#define TM_LIS3DSH_Filter_50Hz 0

static void SystemCoreClockUpdate(void);
static int SysTick_Config(uint32_t ticks);
static int TM_LIS302DL_LIS3DSH_Init(int sensitivity, int filter);
static uint32_t ADC_GetFlagStatus(void *adc, uint32_t flag);
static uint16_t ADC_GetConversionValue(void *adc);
static void ADC_SoftwareStartConv(void *adc);
static void RCC_AHB2PeriphClockCmd(uint32_t periph, uint8_t state);
static void RNG_Cmd(uint8_t state);
static uint32_t RNG_GetFlagStatus(uint32_t flag);
static uint32_t RNG_GetRandomNumber(void);
static void USART_ITConfig(void *uart, uint32_t it, uint8_t state);
static void USART_Cmd(void *uart, uint8_t state);
static uint32_t USART_GetITStatus(void *uart, uint32_t flag);
static uint8_t USART_ReceiveData(void *uart);
static void USART_SendData(void *uart, uint8_t data);

typedef struct {
    int16_t X;
    int16_t Y;
    int16_t Z;
} LIS3DSH_Axes;

static LIS3DSH_Axes TM_LIS302DL_LIS3DSH_ReadAxes(void);

static void SystemCoreClockUpdate(void) {
    SystemCoreClock = 168000000U;
}

static int SysTick_Config(uint32_t ticks) {
    if (ticks == 0U) {
        return 1;
    }
    SysTick_reload = ticks;
    return 0;
}

static int TM_LIS302DL_LIS3DSH_Init(int sensitivity, int filter) {
    (void)sensitivity;
    (void)filter;
    return 0;
}

static LIS3DSH_Axes TM_LIS302DL_LIS3DSH_ReadAxes(void) {
    LIS3DSH_Axes axes;
    axes.X = host_accel_x_mg;
    axes.Y = host_accel_y_mg;
    axes.Z = host_accel_z_mg;
    return axes;
}

static uint32_t ADC_GetFlagStatus(void *adc, uint32_t flag) {
    (void)adc;
    (void)flag;
    return host_adc_eoc;
}

static uint16_t ADC_GetConversionValue(void *adc) {
    (void)adc;
    return host_adc_raw_value;
}

static void ADC_SoftwareStartConv(void *adc) {
    (void)adc;
    host_adc_eoc = 1U;
}

static void RCC_AHB2PeriphClockCmd(uint32_t periph, uint8_t state) {
    (void)periph;
    (void)state;
}

static void RNG_Cmd(uint8_t state) {
    (void)state;
}

static uint32_t RNG_GetFlagStatus(uint32_t flag) {
    (void)flag;
    return rng_ready_flag ? 1U : 0U;
}

static uint32_t RNG_GetRandomNumber(void) {
    return rng_value_state;
}

static void USART_ITConfig(void *uart, uint32_t it, uint8_t state) {
    (void)uart;
    (void)it;
    (void)state;
}

static void USART_Cmd(void *uart, uint8_t state) {
    (void)uart;
    usart1_enabled = (state != 0U);
    if (usart1_enabled) {
        usart1_sr |= 0x40U;
    } else {
        usart1_sr &= ~0x40U;
    }
}

static uint32_t USART_GetITStatus(void *uart, uint32_t flag) {
    (void)uart;
    if (flag == USART_IT_RXNE) {
        return usart1_rxne_pending ? 1U : 0U;
    }
    return 0U;
}

static uint8_t USART_ReceiveData(void *uart) {
    (void)uart;
    return usart1_dr_byte;
}

static void USART_SendData(void *uart, uint8_t data) {
    (void)uart;
    (void)data;
}

void init_system(void) {
    SystemInit();
    init_systick();
    init_LED_pins();
    init_button();
    (void)init_accelerometers();
    init_rng();
    init_temperature_sensor();
    init_GPIO_A1A2A3A4_output();
    init_usart1(UART_BAUD);
    usart1_send("UART1 Initialized. @9600bps\r\n");
}

void SystemInit(void) {
    SystemCoreClock = 168000000U;
}

void init_systick(void) {
    SystemCoreClockUpdate();
    msTicks = 0U;
    SysTick_reload = SystemCoreClock / 1000U;
    if (SysTick_Config(SysTick_reload) != 0) {
        SysTick_init_error = 1;
        while (1) {
        }
    }
}

void init_button(void) {
    GPIOA_IDR &= ~1U;
    b_i = 0U;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
}

bool dispatch_uart_command(int command) {
    if (command >= 0 && command <= 8) {
        msgid = (_ID)command;
        callme = flookup[command];
        callme();
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    if (usart1_available() > 0U) {
        char c = usart1_readc();
        (void)dispatch_uart_command((int)c);
    }

    checkbutton();
    execute_tasks();

    if ((msTicks - t_prev) > 1000U) {
        t_prev = msTicks;
    }
}

void init_GPIO_A1A2A3A4_output(void) {
    GPIOA_output[1] = 0U;
    GPIOA_output[2] = 0U;
    GPIOA_output[3] = 0U;
    GPIOA_output[4] = 0U;
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    switch (direc) {
        case FORWARD:
            GPIOA_output[1] = 1U;
            GPIOA_output[2] = 0U;
            break;
        case BACKWARD:
            GPIOA_output[1] = 0U;
            GPIOA_output[2] = 1U;
            break;
        case STOP:
        default:
            GPIOA_output[1] = 0U;
            GPIOA_output[2] = 0U;
            break;
    }
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    switch (direc) {
        case FORWARD:
            GPIOA_output[3] = 1U;
            GPIOA_output[4] = 0U;
            break;
        case BACKWARD:
            GPIOA_output[3] = 0U;
            GPIOA_output[4] = 1U;
            break;
        case STOP:
        default:
            GPIOA_output[3] = 0U;
            GPIOA_output[4] = 0U;
            break;
    }
}

void move_forward(void) {
    GPIOA_output[1] = 1U;
    GPIOA_output[2] = 0U;
    GPIOA_output[3] = 1U;
    GPIOA_output[4] = 0U;
}

void move_backward(void) {
    GPIOA_output[1] = 0U;
    GPIOA_output[2] = 1U;
    GPIOA_output[3] = 0U;
    GPIOA_output[4] = 1U;
}

void move_forward_soft_left(void) {
    GPIOA_output[1] = 0U;
    GPIOA_output[2] = 0U;
    GPIOA_output[3] = 1U;
    GPIOA_output[4] = 0U;
}

void move_forward_soft_right(void) {
    GPIOA_output[1] = 1U;
    GPIOA_output[2] = 0U;
    GPIOA_output[3] = 0U;
    GPIOA_output[4] = 0U;
}

void move_backward_soft_left(void) {
    GPIOA_output[1] = 0U;
    GPIOA_output[2] = 0U;
    GPIOA_output[3] = 0U;
    GPIOA_output[4] = 1U;
}

void move_backward_soft_right(void) {
    GPIOA_output[1] = 0U;
    GPIOA_output[2] = 1U;
    GPIOA_output[3] = 0U;
    GPIOA_output[4] = 0U;
}

void move_spin_right(void) {
    GPIOA_output[1] = 1U;
    GPIOA_output[2] = 0U;
    GPIOA_output[3] = 0U;
    GPIOA_output[4] = 1U;
}

void move_spin_left(void) {
    GPIOA_output[1] = 0U;
    GPIOA_output[2] = 1U;
    GPIOA_output[3] = 1U;
    GPIOA_output[4] = 0U;
}

void stop(void) {
    GPIOA_output[1] = 0U;
    GPIOA_output[2] = 0U;
    GPIOA_output[3] = 0U;
    GPIOA_output[4] = 0U;
}

int read_buttonc(int i) {
    if (i < 0 || i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> (uint32_t)i) & 1U);
}

void SysTick_Handler(void) {
    msTicks++;

    if (read_buttonc(0) == 1) {
        b_i++;
        if (b_i >= BUTTON_DEBOUNCE_MS) {
            buttstate = ButtonIsPressed;
        }
    } else {
        b_i = 0U;
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
    const float pi = 3.14159265358979323846f;

    if (pitch == NULL || roll == NULL) {
        return;
    }

    *roll = atan2f(acc_y, acc_z) * 180.0f / pi;
    *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / pi;
}

void discobot_set_button_level(bool high) {
    GPIOA_IDR = (GPIOA_IDR & ~1U) | (high ? 1U : 0U);
}

void init_LED_pins(void) {
    GPIOD_output[12] = 0U;
    GPIOD_output[13] = 0U;
    GPIOD_output[14] = 0U;
    GPIOD_output[15] = 0U;
}

void LED_On(int i) {
    if (i < 0 || i > 3) {
        return;
    }
    GPIOD_output[12 + i] = 1U;
}

void LED_Off(int i) {
    if (i < 0 || i > 3) {
        return;
    }
    GPIOD_output[12 + i] = 0U;
}

int init_accelerometers(void) {
    return TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_50Hz);
}

void read_accelerometers(float b[3]) {
    LIS3DSH_Axes raw;

    if (b == NULL) {
        return;
    }

    raw = TM_LIS302DL_LIS3DSH_ReadAxes();
    b[0] = raw.X / 1000.0f;
    b[1] = raw.Y / 1000.0f;
    b[2] = raw.Z / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    host_accel_x_mg = x_mg;
    host_accel_y_mg = y_mg;
    host_accel_z_mg = z_mg;
}

void init_temperature_sensor(void) {
    host_adc_eoc = 1U;
}

float read_temperature_sensor(void) {
    uint16_t raw;
    float temp;

    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
    }

    raw = ADC_GetConversionValue(ADC1);
    temp = (float)raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;

    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    host_adc_raw_value = raw;
}

void init_rng(void) {
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
}

uint32_t get_random_number(void) {
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {
    }
    return RNG_GetRandomNumber();
}

void discobot_set_rng(bool ready, uint32_t value) {
    rng_ready_flag = ready;
    rng_value_state = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    int i;

    for (i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0U;
            timed_tasks[i].numcalls = 0U;
            return;
        }
    }
}

void execute_tasks(void) {
    int i;

    for (i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL &&
            msTicks >= (timed_tasks[i].last_called + timed_tasks[i].msinterval)) {
            timed_tasks[i].task();
            timed_tasks[i].last_called = msTicks;
            timed_tasks[i].numcalls++;
        }
    }
}

void printtimes(void) {
    int i;

    for (i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL) {
            printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
        }
    }
}

void initCircArray(CircArray *arr, int size) {
    char *p;

    if (arr == NULL) {
        return;
    }

    if (arr->enabled) {
        fprintf(stderr, "initCircArray: already enabled\n");
        return;
    }

    if (size <= 0) {
        return;
    }

    p = (char *)calloc((size_t)size, 1U);
    if (p == NULL) {
        return;
    }

    arr->buf = p;
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0U;
    arr->n_w = 0U;
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->size == 0U) {
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
    uint32_t idx;
    char c;

    if (arr == NULL || arr->size == 0U) {
        return 0;
    }

    if (buf_empty(arr)) {
        return 0;
    }

    idx = arr->n_r % arr->size;
    c = arr->buf[idx];
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

    return !buf_empty(arr) &&
           (arr->n_r % arr->size) == (arr->n_w % arr->size);
}

int buf_available(CircArray *arr) {
    if (arr == NULL) {
        return 0;
    }
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize) {
    char *p;

    if (arr == NULL || newSize <= 0) {
        return false;
    }

    p = (char *)realloc(arr->buf, (size_t)newSize);
    if (p == NULL) {
        return false;
    }

    arr->buf = p;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr) {
    if (arr == NULL) {
        return false;
    }

    buf_clear(arr);
    free(arr->buf);
    arr->buf = NULL;
    arr->size = 0U;
    arr->enabled = false;
    arr->n_r = 0U;
    arr->n_w = 0U;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL) {
        return;
    }
    memset(arr->buf, 0, arr->size);
}

void init_usart1(int baud) {
    if (baud == 0) {
        baud = UART_BAUD;
        fprintf(stderr, "warning: USART baud was 0, defaulting to %d\n", UART_BAUD);
    }

    usart1_baud_used = (uint32_t)baud;
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
    usart1_rxne_pending = false;
    usart1_dr_byte = 0U;
}

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t c = USART_ReceiveData(USART1);
        usart1_rxne_pending = false;
        (void)buf_putbyte(&msg, (char)c);
    }
}

void usart1_send(volatile char *s) {
    if (s == NULL) {
        return;
    }

    while (*s != '\0') {
        while ((usart1_sr & 0x40U) == 0U) {
        }

        {
            char ch = (char)*s;
            USART_SendData(USART1, (uint8_t)*s);
            s++;

            if (usart1_tx_length < (sizeof(usart1_tx_log) - 1U)) {
                usart1_tx_log[usart1_tx_length++] = ch;
                usart1_tx_log[usart1_tx_length] = '\0';
            }
        }
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

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    if (rxne) {
        usart1_dr_byte = value;
        usart1_rxne_pending = true;
        USART1_IRQHandler();
    }
}

int func1(int R0) {
    return func2(R0);
}

int func2(int R0) {
    return (int)((uint32_t)R0 + 1U);
}