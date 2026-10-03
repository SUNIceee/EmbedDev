#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#define START_BYTE 0x24
#define STOP_BYTE 0x2A
#define UART_BAUD 9600
#define BUTTON_DEBOUNCE_MS 250
#define SYSTICK_PERIOD_TICKS 168000
#define CIRC_BUFFER_MIN_SIZE 200
#define MAXNUMTASKS 25
#define MAXIDSIZE 9

#define USART_SR_RXNE 0x20u
#define USART_SR_TXE  0x40u

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

typedef enum ButtonState {
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

typedef struct USART_Model {
    uint32_t SR;
    uint32_t DR;
    int enabled;
    int baud;
} USART_Model;

extern uint32_t SystemCoreClock;
extern volatile uint32_t msTicks;
extern uint32_t SysTick_reload;
extern int SysTick_config_should_fail;
extern int SysTick_config_failed;

extern int GPIOA_output[16];
extern int GPIOD_output[16];
extern uint32_t GPIOA_IDR;

extern volatile uint32_t b_i;
extern volatile ButtonState buttstate;
extern volatile ButtonState laststate;

extern CircArray msg;
extern timed_task timed_tasks[MAXNUMTASKS];
extern motor_command_fn flookup[MAXIDSIZE];

extern USART_Model USART1_instance;
#define USART1 (&USART1_instance)

extern char usart1_tx_log[4096];
extern uint32_t usart1_tx_length;

extern int discobot_acc_raw_x;
extern int discobot_acc_raw_y;
extern int discobot_acc_raw_z;
extern uint16_t discobot_adc_raw;
extern bool discobot_adc_ready;
extern bool discobot_adc_wait_blocked;
extern bool discobot_rng_ready;
extern uint32_t discobot_rng_value;
extern bool discobot_rng_wait_blocked;

extern int discobot_init_sequence[16];
extern int discobot_init_sequence_len;

void SystemInit(void);
void SystemCoreClockUpdate(void);
uint32_t SysTick_Config(uint32_t ticks);
int init_systick(void);
void SysTick_Handler(void);

void init_GPIO_A1A2A3A4_output(void);
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

void init_button(void);
int read_buttonc(int i);
void checkbutton(void);

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll);

void init_LED_pins(void);
void LED_On(int i);
void LED_Off(int i);

int init_accelerometers(void);
void read_accelerometers(float *acc[3]);
void discobot_set_accelerometer_raw(int x_mg, int y_mg, int z_mg);

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
void discobot_usart1_inject_rx(uint8_t c);

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

int func2(int R0);
int func1(int R0);

#endif
