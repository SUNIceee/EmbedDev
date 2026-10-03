/* Core application main logic, system initialization, timers, buttons, pitch/roll math and LEDs */

#include "6_generated_code.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static ButtonState lastbuttonstate = ButtonIsReleased;
static uint32_t t_prev = 0;

void SystemInit(void) {
    SystemCoreClock = 168000000;
}

void init_systick(void) {
    SysTick_reload = SystemCoreClock / 1000;
}

void init_button(void) {
    GPIOA_IDR &= ~(1u << 0);
    b_i = 0;
    buttstate = ButtonIsReleased;
    lastbuttonstate = ButtonIsReleased;
}

void discobot_set_button_level(bool high) {
    if (high) {
        GPIOA_IDR |= (1u << 0);
    } else {
        GPIOA_IDR &= ~(1u << 0);
    }
}

int read_buttonc(int i) {
    if (i > 3 || i < 0) {
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
        b_i = 0;
        buttstate = ButtonIsReleased;
    }
}

void checkbutton(void) {
    if (buttstate != lastbuttonstate) {
        /* Transition handling based on requirements */
        if (buttstate == ButtonIsPressed && lastbuttonstate == ButtonIsReleased) {
            /* Button pressed action: perform stop */
            stop();
        }
        lastbuttonstate = buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    if (roll != NULL) {
        *roll = (float)(atan2((double)acc_y, (double)acc_z) * 180.0 / M_PI);
    }
    if (pitch != NULL) {
        double denom = sqrt((double)acc_y * (double)acc_y + (double)acc_z * (double)acc_z);
        *pitch = (float)(atan2(-(double)acc_x, denom) * 180.0 / M_PI);
    }
}

void init_LED_pins(void) {
    for (int i = 12; i <= 15; i++) {
        GPIOD_output[i] = 0;
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

bool dispatch_uart_command(int command) {
    if (command >= 0 && command <= 8) {
        msgid = (_ID)command;
        callme = flookup[command];
        if (callme != NULL) {
            callme();
        }
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    /* 1s timer logic */
    if (msTicks - t_prev > 1000) {
        t_prev = msTicks;
    }
    
    /* UART process */
    if (usart1_available() > 0) {
        char c = usart1_readc();
        if (c >= 0 && c <= 8) {
            dispatch_uart_command((int)c);
        }
    }
    checkbutton();
    execute_tasks();
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
