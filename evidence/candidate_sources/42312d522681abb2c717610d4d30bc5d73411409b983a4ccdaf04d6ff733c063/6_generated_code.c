#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef RESET
#define RESET 0
#endif
#ifndef SET
#define SET 1
#endif
#ifndef ENABLE
#define ENABLE 1
#endif

#define USART_IT_RXNE 5u
#define ADC_FLAG_EOC 0x02u
#define RCC_AHB2Periph_RNG 0x40u
#define TM_LIS3DSH_Sensitivity_2G 0
#define TM_LIS3DSH_Filter_50Hz 0

struct HostUSARTRegs {
    volatile uint32_t SR;
    volatile uint32_t DR;
};

struct Host_LIS3DSH_Axes {
    int16_t X;
    int16_t Y;
    int16_t Z;
};

static struct HostUSARTRegs host_usart1;
static volatile uint8_t host_usart_rxne = 0;
static CircArray msg;
static int16_t accel_raw_mg[3] = {0, 0, 0};
static ButtonState laststate = ButtonIsReleased;
static uint32_t t_prev = 0;

bool rng_ready = false;
uint32_t rng_value = 0;
uint16_t temperature_adc_raw = 0;
bool temperature_adc_eoc = false;

#define USART1 (&host_usart1)

static void Host_USART_SendData(struct HostUSARTRegs *u, uint16_t data) {
    u->DR = data;
}
#define USART_SendData(u, d) Host_USART_SendData((u), (d))

static uint8_t Host_USART_ReceiveData(struct HostUSARTRegs *u) {
    uint8_t c = (uint8_t)(u->DR & 0xFFu);
    host_usart_rxne = 0;
    return c;
}
#define USART_ReceiveData(u) Host_USART_ReceiveData((u))

static int Host_USART_GetITStatus(struct HostUSARTRegs *u, uint8_t it) {
    (void)u;
    if (it == USART_IT_RXNE) {
        return (host_usart_rxne != 0) ? SET : RESET;
    }
    return RESET;
}
#define USART_GetITStatus(u, it) Host_USART_GetITStatus((u), (it))

static void Host_USART_Cmd(struct HostUSARTRegs *u, uint8_t enable) {
    (void)u;
    (void)enable;
    host_usart1.SR |= 0x40u;
}
#define USART_Cmd(u, e) Host_USART_Cmd((u), (e))

static void Host_USART_ITConfig(struct HostUSARTRegs *u, uint8_t it, uint8_t enable) {
    (void)u;
    (void)it;
    (void)enable;
}
#define USART_ITConfig(u, it, e) Host_USART_ITConfig((u), (it), (e))

static void Host_ADC_SoftwareStartConv(void *adc) {
    (void)adc;
}
#define ADC1 ((void*)0)
#define ADC_SoftwareStartConv(adc) Host_ADC_SoftwareStartConv((adc))

static int Host_ADC_GetFlagStatus(void *adc, uint8_t flag) {
    (void)adc;
    (void)flag;
    return temperature_adc_eoc ? SET : RESET;
}
#define ADC_GetFlagStatus(adc, flag) Host_ADC_GetFlagStatus((adc), (flag))

static uint16_t Host_ADC_GetConversionValue(void *adc) {
    (void)adc;
    return temperature_adc_raw;
}
#define ADC_GetConversionValue(adc) Host_ADC_GetConversionValue((adc))

static void Host_RCC_AHB2PeriphClockCmd(uint32_t periph, uint8_t enable) {
    (void)periph;
    (void)enable;
}
#define RCC_AHB2PeriphClockCmd(p, e) Host_RCC_AHB2PeriphClockCmd((p), (e))

static void Host_RNG_Cmd(uint8_t enable) {
    (void)enable;
}
#define RNG_Cmd(e) Host_RNG_Cmd((e))

static struct Host_LIS3DSH_Axes Host_TM_LIS302DL_LIS3DSH_ReadAxes(void) {
    struct Host_LIS3DSH_Axes axes;
    axes.X = accel_raw_mg[0];
    axes.Y = accel_raw_mg[1];
    axes.Z = accel_raw_mg[2];
    return axes;
}
#define TM_LIS302DL_LIS3DSH_ReadAxes() Host_TM_LIS302DL_LIS3DSH_ReadAxes()

static int Host_TM_LIS302DL_LIS3DSH_Init(int sens, int filter) {
    (void)sens;
    (void)filter;
    return 0;
}
#define TM_LIS302DL_LIS3DSH_Init(sens, filter) Host_TM_LIS302DL_LIS3DSH_Init((sens), (filter))

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
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
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 0;
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS] = {0};

static void SystemCoreClockUpdate(void) {
    SystemCoreClock = 168000000u;
}

static uint32_t SysTick_Config(uint32_t ticks) {
    if (ticks > 0x00FFFFFFu) {
        return 1u;
    }
    SysTick_reload = ticks;
    return 0u;
}

void SystemInit(void) {
    SystemCoreClock = 168000000u;
}

void init_systick(void) {
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000u) != 0u) {
        while (1) {}
    }
}

void init_button(void) {
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
}

void init_GPIO_A1A2A3A4_output(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

void init_LED_pins(void) {
    GPIOD_output[12] = 0;
    GPIOD_output[13] = 0;
    GPIOD_output[14] = 0;
    GPIOD_output[15] = 0;
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    switch (direc) {
        case FORWARD:
            GPIOA_output[1] = 1;
            GPIOA_output[2] = 0;
            break;
        case BACKWARD:
            GPIOA_output[1] = 0;
            GPIOA_output[2] = 1;
            break;
        case STOP:
        default:
            GPIOA_output[1] = 0;
            GPIOA_output[2] = 0;
            break;
    }
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    switch (direc) {
        case FORWARD:
            GPIOA_output[3] = 1;
            GPIOA_output[4] = 0;
            break;
        case BACKWARD:
            GPIOA_output[3] = 0;
            GPIOA_output[4] = 1;
            break;
        case STOP:
        default:
            GPIOA_output[3] = 0;
            GPIOA_output[4] = 0;
            break;
    }
}

void move_forward(void) {
    GPIOA_output[1] = 1;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 1;
    GPIOA_output[4] = 0;
}

void move_backward(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 1;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 1;
}

void move_forward_soft_left(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 1;
    GPIOA_output[4] = 0;
}

void move_forward_soft_right(void) {
    GPIOA_output[1] = 1;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

void move_backward_soft_left(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 1;
}

void move_backward_soft_right(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 1;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

void move_spin_right(void) {
    GPIOA_output[1] = 1;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 1;
}

void move_spin_left(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 1;
    GPIOA_output[3] = 1;
    GPIOA_output[4] = 0;
}

void stop(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

int read_buttonc(int i) {
    if (i < 0 || i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> (uint32_t)i) & 1u);
}

void SysTick_Handler(void) {
    msTicks++;
    if (read_buttonc(0) == 1) {
        b_i++;
        if (b_i >= 250u) {
            buttstate = ButtonIsPressed;
        }
    } else {
        b_i = 0;
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
    const float rad_to_deg = 180.0f / 3.14159265358979f;
    *roll = atan2f(acc_y, acc_z) * rad_to_deg;
    *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * rad_to_deg;
}

void discobot_set_button_level(bool high) {
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
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

int init_accelerometers(void) {
    return TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_50Hz);
}

void read_accelerometers(float b[3]) {
    struct Host_LIS3DSH_Axes raw = TM_LIS302DL_LIS3DSH_ReadAxes();
    b[0] = raw.X / 1000.0f;
    b[1] = raw.Y / 1000.0f;
    b[2] = raw.Z / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    accel_raw_mg[0] = x_mg;
    accel_raw_mg[1] = y_mg;
    accel_raw_mg[2] = z_mg;
}

void init_temperature_sensor(void) {
    temperature_adc_raw = 0;
    temperature_adc_eoc = false;
}

float read_temperature_sensor(void) {
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
#ifndef DISCOBOT_TARGET
        if (!temperature_adc_eoc) {
            return NAN;
        }
#endif
    }
    uint16_t raw = ADC_GetConversionValue(ADC1);
    float temp = (float)raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    temperature_adc_raw = raw;
    temperature_adc_eoc = true;
}

void init_rng(void) {
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
}

uint32_t get_random_number(void) {
#ifndef DISCOBOT_TARGET
    if (!rng_ready) {
        return 0u;
    }
#endif
    while (!rng_ready) {}
    return rng_value;
}

void discobot_set_rng(bool ready, uint32_t value) {
    rng_ready = ready;
    rng_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    int i;
    for (i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            break;
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
    if (arr == NULL) {
        return;
    }
    if (arr->enabled) {
        fprintf(stderr, "initCircArray: already enabled\n");
        return;
    }
    arr->buf = (char *)calloc((size_t)size, 1);
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->size == 0u) {
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
    if (arr == NULL || arr->size == 0u) {
        return 0;
    }
    if (buf_empty(arr)) {
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
    if (arr == NULL || arr->size == 0u) {
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
    if (arr != NULL && arr->buf != NULL && arr->size > 0u) {
        memset(arr->buf, 0, arr->size);
    }
}

void init_usart1(int baud) {
    if (baud == 0) {
        baud = UART_BAUD;
        fprintf(stderr, "init_usart1: baud==0, defaulting to 9600\n");
    }
    usart1_baud_used = (uint32_t)baud;
    host_usart1.SR = 0x40u;
    host_usart_rxne = 0;
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

static void usart1_rx_service(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t c = USART_ReceiveData(USART1);
        (void)buf_putbyte(&msg, (char)c);
    }
}

void USART1_IRQHandler(void) {
    usart1_rx_service();
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    host_usart1.DR = value;
    host_usart_rxne = rxne ? 1 : 0;
    usart1_rx_service();
}

void usart1_send(volatile char *s) {
    if (s == NULL) {
        return;
    }
    while (*s) {
        while (!(USART1->SR & 0x40u)) {}
        char c = (char)*s++;
        USART_SendData(USART1, (uint16_t)(uint8_t)c);
        if (usart1_tx_length < (sizeof(usart1_tx_log) - 1u)) {
            usart1_tx_log[usart1_tx_length] = c;
            usart1_tx_length++;
            usart1_tx_log[usart1_tx_length] = '\0';
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

bool dispatch_uart_command(int command) {
    if (command < 0 || command > MAXIDSIZE - 1) {
        return false;
    }
    callme = flookup[command];
    callme();
    return true;
}

void main_loop_iteration(void) {
    if (usart1_available() > 0u) {
        int c = (int)usart1_readc();
        (void)dispatch_uart_command(c);
    }
    checkbutton();
    execute_tasks();
    if (msTicks - t_prev > 1000u) {
        t_prev = msTicks;
    }
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
    usart1_send((volatile char *)discobot_startup_banner);
}

int func2(int R0) {
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0) {
    return func2(R0);
}