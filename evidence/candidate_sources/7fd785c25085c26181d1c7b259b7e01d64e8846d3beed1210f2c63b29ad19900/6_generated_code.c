#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#ifdef DISCOBOT_TARGET
#define ENABLE 1
#define RESET 0
#define GPIO_Mode_OUT 1U
#define GPIO_Mode_IN 0U
#define GPIO_Mode_AF 2U
#define GPIO_OType_PP 0U
#define GPIO_PuPd_UP 1U
#define GPIO_PuPd_NOPULL 0U
#define GPIO_Speed_25MHz 25U
#define GPIO_Speed_100MHz 100U
#define RCC_AHB1Periph_GPIOA 0x00000001U
#define RCC_AHB1Periph_GPIOB 0x00000002U
#define RCC_AHB1Periph_GPIOD 0x00000008U
#define RCC_AHB2Periph_RNG 0x40U
#define RCC_APB2Periph_USART1 0x00000010U
#define RCC_APB2Periph_ADC1 0x00000100U
#define RNG_FLAG_DRDY 1U
#define USART_IT_RXNE 0x20U
#define USART_Mode_Tx 0x08U
#define USART_Mode_Rx 0x04U
#define USART_WordLength_8b 0U
#define USART_StopBits_1 0U
#define USART_Parity_No 0U
#define USART_HardwareFlowControl_None 0U
#define USART1_IRQn 37
#define ADC_FLAG_EOC 1U
#define ADC_Channel_TempSensor 16U
#define ADC_SampleTime_144Cycles 144U
typedef struct { uint32_t GPIO_Pin; uint32_t GPIO_Mode; uint32_t GPIO_Speed; uint32_t GPIO_OType; uint32_t GPIO_PuPd; } GPIO_InitTypeDef;
typedef struct { uint32_t USART_BaudRate; uint32_t USART_WordLength; uint32_t USART_StopBits; uint32_t USART_Parity; uint32_t USART_HardwareFlowControl; uint32_t USART_Mode; } USART_InitTypeDef;
typedef struct { int NVIC_IRQChannel; int NVIC_IRQChannelPreemptionPriority; int NVIC_IRQChannelSubPriority; int NVIC_IRQChannelCmd; } NVIC_InitTypeDef;
typedef struct { volatile uint32_t SR; volatile uint32_t DR; } USART_TypeDef;
extern USART_TypeDef *USART1;
extern void *GPIOA; extern void *GPIOB; extern void *GPIOD; extern void *ADC1;
extern void RCC_AHB1PeriphClockCmd(uint32_t, int);
extern void RCC_AHB2PeriphClockCmd(uint32_t, int);
extern void RCC_APB2PeriphClockCmd(uint32_t, int);
extern void GPIO_Init(void *, GPIO_InitTypeDef *);
extern void GPIO_PinAFConfig(void *, uint16_t, uint8_t);
extern void USART_Init(USART_TypeDef *, USART_InitTypeDef *);
extern void USART_ITConfig(USART_TypeDef *, uint16_t, int);
extern void USART_Cmd(USART_TypeDef *, int);
extern void USART_SendData(USART_TypeDef *, uint16_t);
extern void NVIC_Init(NVIC_InitTypeDef *);
extern int TM_LIS302DL_LIS3DSH_Init(int, int);
extern void ADC_SoftwareStartConv(void *);
extern int ADC_GetFlagStatus(void *, uint32_t);
extern uint16_t ADC_GetConversionValue(void *);
extern void ADC_TempSensorVrefintCmd(int);
extern void ADC_Cmd(void *, int);
extern void RNG_Cmd(int);
extern int RNG_GetFlagStatus(uint32_t);
extern uint32_t RNG_GetRandomNumber(void);
#endif

volatile uint32_t msTicks = 0U;
volatile uint32_t b_i = 0U;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
motor_command_fn callme = NULL;
uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0U;
uint32_t SystemCoreClock = 168000000U;
uint32_t SysTick_reload = SYSTICK_PERIOD_TICKS;
uint32_t usart1_baud_used = 0U;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0U;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS] = {{0}};
static CircArray msg = {0};
static ButtonState laststate = ButtonIsReleased;
static uint32_t main_t_prev = 0U;
static int16_t accel_raw_x = 0, accel_raw_y = 0, accel_raw_z = 0;
static uint16_t adc_raw_temperature = 0U;
static volatile bool rng_ready = false;
static uint32_t rng_value = 0U;
static uint8_t usart1_dr = 0U;
static bool usart1_rxne = false;
static bool accel_initialized = false;
static bool adc_initialized = false;
static bool rng_initialized = false;
static bool host_rng_blocked = false;

motor_command_fn flookup[9] = { move_forward, move_backward, move_forward_soft_left, move_forward_soft_right, move_backward_soft_left, move_backward_soft_right, move_spin_right, move_spin_left, stop };

static void SystemCoreClockUpdate(void) { SystemCoreClock = 168000000U; }
static int SysTick_Config(uint32_t ticks) { SysTick_reload = ticks; return ticks == 0U ? 1 : 0; }
static void write_motor_pair(int a, int b, int direc) { if (direc == FORWARD) { GPIOA_output[a]=1U; GPIOA_output[b]=0U; } else if (direc == BACKWARD) { GPIOA_output[a]=0U; GPIOA_output[b]=1U; } else { GPIOA_output[a]=0U; GPIOA_output[b]=0U; } }

void SystemInit(void) { SystemCoreClock = 168000000U; }
void init_systick(void) { SystemCoreClockUpdate(); if (SysTick_Config(SystemCoreClock / 1000U) != 0) { for (;;) { } } }

void init_LED_pins(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); GPIO_InitTypeDef g = {0xF000U, GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_UP}; GPIO_Init(GPIOD, &g);
#endif
    GPIOD_output[12]=0U; GPIOD_output[13]=0U; GPIOD_output[14]=0U; GPIOD_output[15]=0U;
}
void init_button(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); GPIO_InitTypeDef g = {0x0001U, GPIO_Mode_IN, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL}; GPIO_Init(GPIOA, &g);
#endif
    GPIOA_IDR &= ~1U; b_i = 0U; buttstate = ButtonIsReleased; laststate = ButtonIsReleased;
}
int init_accelerometers(void) {
#ifdef DISCOBOT_TARGET
    (void)TM_LIS302DL_LIS3DSH_Init(2, 50);
#endif
    accel_initialized = true; return 0;
}
void init_rng(void) { #ifdef DISCOBOT_TARGET
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE); RNG_Cmd(ENABLE);
#endif
    rng_initialized = true; }
void init_temperature_sensor(void) {
#ifdef DISCOBOT_TARGET
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); ADC_TempSensorVrefintCmd(ENABLE); ADC_Cmd(ADC1, ENABLE);
#endif
    adc_initialized = true;
}
void init_GPIO_A1A2A3A4_output(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); GPIO_InitTypeDef g = {0x001EU, GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_UP}; GPIO_Init(GPIOA, &g);
#endif
    GPIOA_output[1]=0U; GPIOA_output[2]=0U; GPIOA_output[3]=0U; GPIOA_output[4]=0U;
}
void init_system(void) { SystemInit(); init_systick(); init_LED_pins(); init_button(); init_accelerometers(); init_rng(); init_temperature_sensor(); init_GPIO_A1A2A3A4_output(); init_usart1(9600); usart1_send((volatile char *)"UART1 Initialized. @9600bps\r\n"); }

void set_left_motor_direc(int direc, float speed) { (void)speed; write_motor_pair(1,2,direc); }
void set_right_motor_direc(int direc, float speed) { (void)speed; write_motor_pair(3,4,direc); }
void move_forward(void) { set_left_motor_direc(FORWARD,1.0f); set_right_motor_direc(FORWARD,1.0f); }
void move_backward(void) { set_left_motor_direc(BACKWARD,1.0f); set_right_motor_direc(BACKWARD,1.0f); }
void move_forward_soft_left(void) { set_left_motor_direc(STOP,0.0f); set_right_motor_direc(FORWARD,1.0f); }
void move_forward_soft_right(void) { set_left_motor_direc(FORWARD,1.0f); set_right_motor_direc(STOP,0.0f); }
void move_backward_soft_left(void) { set_left_motor_direc(STOP,0.0f); set_right_motor_direc(BACKWARD,1.0f); }
void move_backward_soft_right(void) { set_left_motor_direc(BACKWARD,1.0f); set_right_motor_direc(STOP,0.0f); }
void move_spin_right(void) { set_left_motor_direc(FORWARD,1.0f); set_right_motor_direc(BACKWARD,1.0f); }
void move_spin_left(void) { set_left_motor_direc(BACKWARD,1.0f); set_right_motor_direc(FORWARD,1.0f); }
void stop(void) { set_left_motor_direc(STOP,0.0f); set_right_motor_direc(STOP,0.0f); }

bool dispatch_uart_command(int command) { if (command >= 0 && command < MAXIDSIZE) { callme = flookup[command]; msgid = (_ID)command; if (callme != NULL) { callme(); } return true; } return false; }
void main_loop_iteration(void) { if (msTicks - main_t_prev > 1000U) { main_t_prev = msTicks; } if (usart1_available() > 0U) { char c = usart1_readc(); dispatch_uart_command((int)c); } checkbutton(); execute_tasks(); }
int read_buttonc(int i) { if (i > 3 || i < 0) { return -1; } return (int)((GPIOA_IDR >> (uint32_t)i) & 1U); }
void SysTick_Handler(void) { msTicks++; if (read_buttonc(0)==1) { b_i++; if (b_i >= BUTTON_DEBOUNCE_MS) { buttstate = ButtonIsPressed; } } else { b_i = 0U; buttstate = ButtonIsReleased; } }
void checkbutton(void) { if (laststate != buttstate) { switch (buttstate) { case ButtonIsPressed: break; case ButtonIsReleased: break; default: break; } laststate = buttstate; } }
void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) { if (roll != NULL) { *roll = atan2f(acc_y, acc_z) * 180.0f / PI; } if (pitch != NULL) { *pitch = atan2f(-acc_x, sqrtf(acc_y*acc_y + acc_z*acc_z)) * 180.0f / PI; } }
void discobot_set_button_level(bool high) { if (high) { GPIOA_IDR |= 1U; } else { GPIOA_IDR &= ~1U; } }
void LED_On(int i) { if (i >= 0 && i < 4) { GPIOD_output[12+i] = 1U; } }
void LED_Off(int i) { if (i >= 0 && i < 4) { GPIOD_output[12+i] = 0U; } }
void read_accelerometers(float b[3]) { (void)accel_initialized; if (b == NULL) return; b[0]=(float)accel_raw_x/1000.0f; b[1]=(float)accel_raw_y/1000.0f; b[2]=(float)accel_raw_z/1000.0f; }
void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) { accel_raw_x=x_mg; accel_raw_y=y_mg; accel_raw_z=z_mg; }
float read_temperature_sensor(void) { (void)adc_initialized; #ifdef DISCOBOT_TARGET
    ADC_SoftwareStartConv(ADC1); while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) { } uint16_t raw = ADC_GetConversionValue(ADC1);
#else
    uint16_t raw = adc_raw_temperature;
#endif
    float temp=(float)raw/4095.0f; temp*=3.3f; temp-=0.760f; temp/=0.0025f; temp+=25.0f; return temp; }
void discobot_set_adc_raw(uint16_t raw) { adc_raw_temperature = raw; }
uint32_t get_random_number(void) { (void)rng_initialized; #ifdef DISCOBOT_TARGET
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) { } return RNG_GetRandomNumber();
#else
    if (rng_ready == false) { host_rng_blocked = true; fprintf(stderr, "DiscoBot RNG blocked: DRDY false\n"); abort(); } host_rng_blocked = false; return rng_value;
#endif
}
void discobot_set_rng(bool ready, uint32_t value) { rng_ready=ready; rng_value=value; host_rng_blocked=false; }
void add_timed_task(void (*myfunc)(void), float interval_sec) { for (int i=0;i<MAXNUMTASKS;i++) if (timed_tasks[i].task==NULL) { timed_tasks[i].task=myfunc; timed_tasks[i].msinterval=(uint32_t)((long)(interval_sec*1000.0f)); timed_tasks[i].last_called=0U; timed_tasks[i].numcalls=0U; return; } }
void execute_tasks(void) { for (int i=0;i<MAXNUMTASKS;i++) if (timed_tasks[i].task!=NULL && msTicks >= timed_tasks[i].last_called + timed_tasks[i].msinterval) { timed_tasks[i].task(); timed_tasks[i].last_called=msTicks; timed_tasks[i].numcalls++; } }
void printtimes(void) { for (int i=0;i<MAXNUMTASKS;i++) if (timed_tasks[i].task!=NULL) printf("t%d=%d", i, (int)timed_tasks[i].numcalls); }
void initCircArray(CircArray *arr, int size) { if (arr==NULL || size<=0) return; if (arr->enabled) { printf("ERROR: CircArray already enabled"); return; } arr->buf=(char *)calloc((size_t)size,1U); if (arr->buf==NULL) return; arr->size=(uint32_t)size; arr->enabled=true; arr->n_r=0U; arr->n_w=0U; }
bool buf_empty(CircArray *arr) { return arr==NULL ? true : (arr->n_r == arr->n_w); }
bool buf_full(CircArray *arr) { if (arr==NULL || arr->size==0U) return false; return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size)); }
int buf_putbyte(CircArray *arr, char c) { if (arr==NULL || arr->buf==NULL || arr->size==0U) return 0; if (buf_full(arr)) return 0; arr->buf[arr->n_w % arr->size]=c; arr->n_w++; return 1; }
char buf_getbyte(CircArray *arr) { if (arr==NULL || arr->buf==NULL || arr->size==0U || buf_empty(arr)) return 0; char v=arr->buf[arr->n_r % arr->size]; arr->n_r++; return v; }
int buf_available(CircArray *arr) { if (arr==NULL) return 0; return (int)(arr->n_w - arr->n_r); }
bool buf_resize(CircArray *arr, int newSize) { if (arr==NULL || newSize<=0) return false; void *tmp=realloc(arr->buf,(size_t)newSize); if (tmp==NULL) return false; arr->buf=(char *)tmp; arr->size=(uint32_t)newSize; return true; }
void buf_clear(CircArray *arr) { if (arr==NULL || arr->buf==NULL) return; memset(arr->buf,0,arr->size); }
bool buf_delete(CircArray *arr) { if (arr==NULL) return false; buf_clear(arr); free(arr->buf); arr->buf=NULL; arr->size=0U; arr->n_r=0U; arr->n_w=0U; arr->enabled=false; return true; }
void init_usart1(int baud) { if (baud==0) { baud=9600; printf("Warning: baud 0, using 9600"); } #ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); GPIO_InitTypeDef g={0x00C0U,GPIO_Mode_AF,GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_UP}; GPIO_Init(GPIOB,&g); GPIO_PinAFConfig(GPIOB,6U,7U); GPIO_PinAFConfig(GPIOB,7U,7U); USART_InitTypeDef u={(uint32_t)baud,USART_WordLength_8b,USART_StopBits_1,USART_Parity_No,USART_HardwareFlowControl_None,USART_Mode_Tx|USART_Mode_Rx}; USART_Init(USART1,&u); USART_ITConfig(USART1,USART_IT_RXNE,ENABLE); NVIC_InitTypeDef n={USART1_IRQn,0,0,ENABLE}; NVIC_Init(&n); USART_Cmd(USART1,ENABLE);
#endif
    usart1_baud_used=(uint32_t)baud; if (msg.enabled) (void)buf_delete(&msg); initCircArray(&msg,200); }
void USART1_IRQHandler(void) { #ifdef DISCOBOT_TARGET
    if (USART1 != NULL && (USART1->SR & USART_IT_RXNE) != 0U) { char c=(char)(USART1->DR & 0xFFU); (void)buf_putbyte(&msg,c); }
#else
    if (usart1_rxne) { char c=(char)usart1_dr; (void)buf_putbyte(&msg,c); usart1_rxne=false; }
#endif
}
void usart1_send(volatile char *s) { if (s==NULL) return; while (*s) { #ifdef DISCOBOT_TARGET
        while (!(USART1->SR & 0x40U)) { } USART_SendData(USART1,(uint16_t)(uint8_t)*s);
#endif
        if (usart1_tx_length < sizeof(usart1_tx_log)) { usart1_tx_log[usart1_tx_length]=(char)*s; usart1_tx_length++; } s++; } }
uint8_t usart1_read(void) { return (uint8_t)buf_getbyte(&msg); }
char usart1_readc(void) { signed char v=(signed char)(uint8_t)buf_getbyte(&msg); return (char)v; }
uint32_t usart1_available(void) { return (uint32_t)buf_available(&msg); }
void discobot_usart1_inject_rx(uint8_t value, bool rxne) { usart1_dr=value; usart1_rxne=rxne; if (rxne) USART1_IRQHandler(); }
int func2(int R0) { uint32_t v=(uint32_t)R0; v+=1U; return (int)v; }
int func1(int R0) { return func2(R0); }