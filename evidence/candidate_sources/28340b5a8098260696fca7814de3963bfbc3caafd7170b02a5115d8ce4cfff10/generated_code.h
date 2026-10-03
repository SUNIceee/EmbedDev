#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define START_BYTE 0x24
#define STOP_BYTE  0x2A
#define UART_BAUD 9600
#define BUTTON_DEBOUNCE_MS 250
#define SYSTICK_PERIOD_TICKS 168000UL
#define CIRC_BUFFER_MIN_SIZE 200
#define MAXNUMTASKS 25
#define MAXIDSIZE 9
#define DISCOBOT_TX_LOG_CAPACITY 4096
#define PI 3.14159265358979323846f

#define RESET 0
#define SET 1
#define ENABLE 1
#define DISABLE 0

typedef enum { FORWARD = 0, BACKWARD = 1, STOP = 2 } Directions;

typedef enum _ID {
    FWD = 0,
    BWD = 1,
    FWDLEFT = 2,
    FWDRIGHT = 3,
    BCKLEFT = 4,
    BCKRIGHT = 5,
    SPINRIGHT = 6,
    SPINLEFT = 7,
    STOPCAR = 8
} _ID;

typedef enum {
    ButtonIsReleased = 0,
    ButtonIsPressed = 1
} ButtonState;

typedef void (*motor_command_fn)(void);

typedef struct CircArray {
    char *buf;
    uint32_t size;
    bool enabled;
    uint32_t n_r;
    uint32_t n_w;
} CircArray;

typedef struct timed_task {
    void (*task)(void);
    long msinterval;
    uint32_t last_called;
    uint32_t numcalls;
} timed_task;

typedef struct USART_TypeDef {
    uint32_t SR;
    uint32_t DR;
} USART_TypeDef;

extern uint32_t SystemCoreClock;
extern volatile uint32_t msTicks;
extern uint32_t SysTick_reload;
extern int SysTick_configured;
extern int SysTick_config_failed;
extern int discobot_force_systick_fail;

extern int GPIOA_output[16];
extern int GPIOD_output[16];
extern uint32_t GPIOA_IDR;

extern volatile ButtonState buttstate;
extern volatile ButtonState laststate;
extern volatile int b_i;

extern CircArray msg;
extern timed_task timed_tasks[MAXNUMTASKS];
extern motor_command_fn flookup[MAXIDSIZE];

extern USART_TypeDef USART1_instance;
#define USART1 (&USART1_instance)

extern char usart1_tx_log[DISCOBOT_TX_LOG_CAPACITY];
extern uint32_t usart1_tx_length;
extern int usart1_baud_configured;
extern int usart1_enabled;
extern int usart1_rxne_interrupt_enabled;
extern int usart1_nvic_preempt_priority;
extern int usart1_nvic_sub_priority;

extern int accelerometer_initialized;
extern int32_t discobot_acc_raw_x;
extern int32_t discobot_acc_raw_y;
extern int32_t discobot_acc_raw_z;

extern int temperature_sensor_initialized;
extern uint16_t discobot_adc_raw;
extern int discobot_adc_eoc;

extern int rng_initialized;
extern bool discobot_rng_ready;
extern uint32_t discobot_rng_value;
extern int discobot_rng_wait_blocked;

extern int led_initialized;
extern int button_initialized;
extern int motor_gpio_initialized;

extern int discobot_init_sequence[16];
extern int discobot_init_sequence_length;

void SystemInit(void);
void SystemCoreClockUpdate(void);
int SysTick_Config(uint32_t ticks);
void init_systick(void);
void SysTick_Handler(void);

void set_left_motor_direc(int direc, float speed);
void set_right_motor_direc(int direc, float speed);
void move_forward(void);
void move_backward(void);
void move_forward_soft_left(void);
void move_forward_soft_right(void);
void move_backward_soft_left(void);
void move_backward_soft_right(void);
void move_spin_right(void);
void move_spin_left(void);
void stop(void);
void init_GPIO_A1A2A3A4_output(void);

int read_buttonc(int i);
void checkbutton(void);
void init_button(void);

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll);

void init_LED_pins(void);
void LED_On(int i);
void LED_Off(int i);

int init_accelerometers(void);
void read_accelerometers(float *acc[3]);
void discobot_set_accelerometer_raw(int32_t x_mg, int32_t y_mg, int32_t z_mg);

void init_temperature_sensor(void);
float read_temperature_sensor(void);
void discobot_set_adc_raw(uint16_t raw);

void init_rng(void);
uint32_t get_random_number(void);
void discobot_set_rng(bool ready, uint32_t value);

void add_timed_task(void (*myfunc)(void), float interval_sec);
void execute_tasks(void);
void printtimes(void);

void init_usart1(int baud);
void USART1_IRQHandler(void);
void usart1_send(volatile char *s);
uint8_t usart1_read(void);
char usart1_readc(void);
uint32_t usart1_available(void);
void USART_SendData(USART_TypeDef *usart, uint16_t data);
void discobot_usart1_inject_rx(uint8_t c);
void discobot_clear_usart1_tx_log(void);

void initCircArray(CircArray *arr, int size);
int buf_putbyte(CircArray *arr, char c);
char buf_getbyte(CircArray *arr);
bool buf_empty(CircArray *arr);
bool buf_full(CircArray *arr);
int buf_available(CircArray *arr);
bool buf_resize(CircArray *arr, int newSize);
bool buf_delete(CircArray *arr);
void buf_clear(CircArray *arr);

void dispatch_uart_command(void);
void main_loop_iteration(void);
void init_system(void);

void discobot_set_button_level(int level);
void discobot_reset_host_state(void);

int func1(int R0);
int func2(int R0);

#ifdef __cplusplus
}
#endif

#endif
