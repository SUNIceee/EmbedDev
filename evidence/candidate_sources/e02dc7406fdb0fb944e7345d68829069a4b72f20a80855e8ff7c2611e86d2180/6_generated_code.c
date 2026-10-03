#include "6_generated_code.h"

// Global Variables
volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid;
motor_command_fn callme;
uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 168000000;
uint32_t SysTick_reload = 168000;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
TimedTask timed_tasks[MAXNUMTASKS] = {0};
static CircArray msg;
static bool rng_ready = false;
static uint32_t rng_val = 0;
static uint16_t adc_val = 0;

motor_command_fn flookup[9] = {
    move_forward, move_backward, move_forward_soft_left, 
    move_forward_soft_right, move_backward_soft_left, 
    move_backward_soft_right, move_spin_right, move_spin_left, stop
};

void move_forward(void) { GPIOA_output[1]=1; GPIOA_output[2]=0; GPIOA_output[3]=1; GPIOA_output[4]=0; }
void move_backward(void) { GPIOA_output[1]=0; GPIOA_output[2]=1; GPIOA_output[3]=0; GPIOA_output[4]=1; }
void move_forward_soft_left(void) { GPIOA_output[1]=0; GPIOA_output[2]=0; GPIOA_output[3]=1; GPIOA_output[4]=0; }
void move_forward_soft_right(void) { GPIOA_output[1]=1; GPIOA_output[2]=0; GPIOA_output[3]=0; GPIOA_output[4]=0; }
void move_backward_soft_left(void) { GPIOA_output[1]=0; GPIOA_output[2]=0; GPIOA_output[3]=0; GPIOA_output[4]=1; }
void move_backward_soft_right(void) { GPIOA_output[1]=0; GPIOA_output[2]=1; GPIOA_output[3]=0; GPIOA_output[4]=0; }
void move_spin_right(void) { GPIOA_output[1]=1; GPIOA_output[2]=0; GPIOA_output[3]=0; GPIOA_output[4]=1; }
void move_spin_left(void) { GPIOA_output[1]=0; GPIOA_output[2]=1; GPIOA_output[3]=1; GPIOA_output[4]=0; }
void stop(void) { GPIOA_output[1]=0; GPIOA_output[2]=0; GPIOA_output[3]=0; GPIOA_output[4]=0; }

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

void SysTick_Handler(void) {
    msTicks++;
    if (read_buttonc(0) == 1) {
        if (b_i < 250) b_i++;
        if (b_i >= 250) buttstate = ButtonIsPressed;
    } else {
        b_i = 0;
        buttstate = ButtonIsReleased;
    }
}

void initCircArray(CircArray *arr, int size) {
    arr->buf = (char*)calloc(size, 1);
    arr->size = size; arr->enabled = true; arr->n_r = arr->n_w = 0;
}

int buf_putbyte(CircArray *arr, char c) {
    if ((arr->n_w - arr->n_r) >= arr->size) return 0;
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (arr->n_r == arr->n_w) return 0;
    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

bool dispatch_uart_command(int command) {
    if (command >= 0 && command <= 8) {
        flookup[command]();
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    if (usart1_available() > 0) {
        char c = usart1_readc();
        dispatch_uart_command((int)c);
    }
    checkbutton();
    execute_tasks();
}

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL && (msTicks >= timed_tasks[i].last_called + timed_tasks[i].msinterval)) {
            timed_tasks[i].task();
            timed_tasks[i].last_called = msTicks;
            timed_tasks[i].numcalls++;
        }
    }
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(interval_sec * 1000);
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            break;
        }
    }
}

float read_temperature_sensor(void) {
    float temp = ((adc_val / 4095.0f) * 3.3f);
    temp = (temp - 0.760f) / 0.0025f + 25.0f;
    return temp;
}

void usart1_send(volatile char *s) {
    while (*s) usart1_tx_log[usart1_tx_length++] = *s++;
}

void discobot_set_button_level(bool high) { GPIOA_IDR = high ? (GPIOA_IDR | 1) : (GPIOA_IDR & ~1); }
int read_buttonc(int i) { return (i > 3) ? -1 : ((GPIOA_IDR >> i) & 1); }
void init_button(void) { b_i = 0; buttstate = ButtonIsReleased; }
void init_GPIO_A1A2A3A4_output(void) { stop(); }
void init_usart1(int baud) { initCircArray(&msg, 200); }
uint32_t usart1_available(void) { return msg.n_w - msg.n_r; }
char usart1_readc(void) { return buf_getbyte(&msg); }
void init_systick(void) {} 
void init_LED_pins(void) {}
void init_accelerometers(void) {}
void init_rng(void) {}
void init_temperature_sensor(void) {}
void SystemInit(void) {}
void checkbutton(void) {}
void calc_pitch_roll(float ax, float ay, float az, float *p, float *r) {
    *r = atan2(ay, az) * 180.0f / 3.14159f;
    *p = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / 3.14159f;
}
void discobot_set_adc_raw(uint16_t raw) { adc_val = raw; }
int func2(int R0) { return R0 + 1; }
int func1(int R0) { return func2(R0); }
void discobot_usart1_inject_rx(uint8_t v, bool r) { if(r) buf_putbyte(&msg, (char)v); }
