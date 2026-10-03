"""PlantUML sequence diagram knowledge base — curated correct syntax examples."""

SEQUENCE_DIAGRAM_EXAMPLES = [
    # ── Example 1: Init flow with participants and notes ──
    {
        "id": "init_flow",
        "keywords": [
            "init", "initialization", "HAL_Init", "Clock", "GPIO",
            "timer", "ADC", "peripheral", "boot", "startup"
        ],
        "description": "System initialization sequence with participant declarations and notes",
        "code": (
            "participant \"main()\" as Main\n"
            "participant \"HAL\" as HAL\n"
            "participant \"Clock\" as CLK\n"
            "participant \"GPIO\" as GPIO\n"
            "participant \"TIM_Init\" as TIMI\n"
            "\n"
            "Main -> HAL : HAL_Init()\n"
            "activate HAL\n"
            "HAL --> Main : done\n"
            "deactivate HAL\n"
            "\n"
            "Main -> CLK : SystemClock_Config()\n"
            "activate CLK\n"
            "note over CLK\n"
            "    HSE = 8MHz\n"
            "    PLL = 168MHz (SYSCLK)\n"
            "    APB2 = 84MHz\n"
            "end note\n"
            "CLK --> Main : clocks_ready\n"
            "deactivate CLK\n"
            "\n"
            "Main -> GPIO : MX_GPIO_Init()\n"
            "activate GPIO\n"
            "note over GPIO : PC5(OUT), PC13(OUT)\n"
            "GPIO --> Main : gpio_done\n"
            "deactivate GPIO\n"
            "\n"
            "Main -> TIMI : TIM_Config()\n"
            "activate TIMI\n"
            "note over TIMI\n"
            "    TIM4: PWM 8.4kHz, CH3/CH4\n"
            "    TIM3: PWM 300Hz, CH2/CH3\n"
            "    TIM1/TIM5: Encoder TI12\n"
            "end note\n"
            "TIMI --> Main : timers_done\n"
            "deactivate TIMI"
        ),
    },
    # ── Example 2: Control loop with alt/opt blocks ──
    {
        "id": "control_loop_with_opt_alt",
        "keywords": [
            "control", "loop", "PID", "PWM", "motor", "speed",
            "encoder", "sensor", "read", "write", "compute",
            "SysTick", "tick", "period"
        ],
        "description": "Periodic control loop with opt (conditional) and alt blocks",
        "code": (
            "SysTick -> Main : tick (1ms period)\n"
            "activate Main\n"
            "\n"
            "Main -> Encoder : Get_Speed(&speedL, &speedR, &speedA)\n"
            "activate Encoder\n"
            "note over Encoder\n"
            "    Read TIM1 CNT -> Current_L\n"
            "    Read TIM5 CNT -> Current_R\n"
            "    Delta = Current - Previous\n"
            "end note\n"
            "Encoder --> Main : speed_done\n"
            "deactivate Encoder\n"
            "\n"
            "Main -> ADC : Get_Adc(&adcValue)\n"
            "activate ADC\n"
            "ADC -> ADC : HAL_ADC_Start()\n"
            "ADC -> ADC : HAL_ADC_PollForConversion(10ms)\n"
            "ADC -> ADC : raw = HAL_ADC_GetValue()\n"
            "\n"
            "alt isFirst == true\n"
            "    note over ADC : Adc = raw (skip filter)\n"
            "else isFirst == false\n"
            "    note over ADC : Adc = 0.1 * raw + 0.9 * pre_Adc\n"
            "end\n"
            "\n"
            "note over ADC : pre_Adc = Adc\n"
            "ADC --> Main : adc_done\n"
            "deactivate ADC\n"
            "\n"
            "Main -> PID : PID_Compute(setpoint, feedback)\n"
            "activate PID\n"
            "note over PID\n"
            "    Error = Setpoint - Feedback\n"
            "    P_term = Kp * Error\n"
            "    I_term += Ki * Error * dt\n"
            "    I_term = clamp(I_term, -MAX, MAX)\n"
            "    D_term = Kd * (Error - PrevError) / dt\n"
            "    Output = P_term + I_term + D_term\n"
            "    Output = clamp(Output, -PulseMax, PulseMax)\n"
            "end note\n"
            "PID --> Main : pid_done (pulse1, pulse2)\n"
            "deactivate PID\n"
            "\n"
            "Main -> Motor : Motor_SetPulse1(state, pulse1)\n"
            "activate Motor\n"
            "\n"
            "alt pulse1 >= 0\n"
            "    note over Motor : dir = forward (PC5=LOW)\n"
            "else pulse1 < 0\n"
            "    note over Motor : dir = reverse (PC5=HIGH)\n"
            "end\n"
            "\n"
            "note over Motor : pwm_val = clamp(pwm_val, 0, M1_Pulse_Max)\n"
            "Motor -> TIM4 : HAL_TIM_PWM_Pulse(&TimHandle, CH4, pwm_val)\n"
            "activate TIM4\n"
            "TIM4 --> Motor : done\n"
            "deactivate TIM4\n"
            "Motor --> Main : motor1_done\n"
            "deactivate Motor\n"
            "\n"
            "deactivate Main"
        ),
    },
    # ── Example 3: SD card init with nested alt ──
    {
        "id": "sd_card_init_seq",
        "keywords": [
            "SD", "card", "FatFs", "mount", "file", "f_open",
            "f_write", "f_close", "LED", "blink", "driver", "link"
        ],
        "description": "SD card init with nested alt blocks for success/failure paths",
        "code": (
            "Main -> SD : SD_Init(state)\n"
            "activate SD\n"
            "\n"
            "SD -> FatFs : FATFS_LinkDriver(&SD_Driver, SDPath)\n"
            "activate FatFs\n"
            "\n"
            "alt driver linked\n"
            "    FatFs --> SD : reg_ok\n"
            "    \n"
            "    SD -> FatFs : f_mount(&SDFatFs, SDPath, 1)\n"
            "    activate FatFs\n"
            "    \n"
            "    alt mount success\n"
            "        FatFs --> SD : mount_ok\n"
            "        \n"
            "        SD -> FatFs : f_open(&MyFile, \"STM32.TXT\", FA_CREATE_ALWAYS)\n"
            "        activate FatFs\n"
            "        \n"
            "        alt file opened\n"
            "            FatFs --> SD : file_ok\n"
            "            \n"
            "            SD -> FatFs : f_write(&MyFile, test_str, len, &bw)\n"
            "            activate FatFs\n"
            "            FatFs --> SD : write_ok\n"
            "            deactivate FatFs\n"
            "            \n"
            "            SD -> FatFs : f_close(&MyFile)\n"
            "            activate FatFs\n"
            "            FatFs --> SD : close_ok\n"
            "            deactivate FatFs\n"
            "            \n"
            "            SD -> LED : blink 6x (200ms)\n"
            "            activate LED\n"
            "            LED --> SD : led_done\n"
            "            deactivate LED\n"
            "            \n"
            "        else open fail\n"
            "            FatFs --> SD : open_fail\n"
            "            note over SD : Error_Handler()\n"
            "        end\n"
            "        \n"
            "    else mount fail\n"
            "        FatFs --> SD : mount_fail\n"
            "        note over SD : Error_Handler()\n"
            "    end\n"
            "    \n"
            "else driver link fail\n"
            "    FatFs --> SD : reg_fail\n"
            "    note over SD : skip silently, continue boot\n"
            "end\n"
            "\n"
            "deactivate FatFs\n"
            "deactivate SD\n"
            "SD --> Main : sd_done"
        ),
    },
    # ── Example 4: Encoder speed acquisition detail ──
    {
        "id": "encoder_speed_detail",
        "keywords": [
            "encoder", "speed", "TIM1", "TIM5", "counter", "delta",
            "overflow", "underflow", "yaw", "filter", "IIR"
        ],
        "description": "Encoder speed calculation with overflow handling opt blocks",
        "code": (
            "encoder_diagram ==\n"
            "participant \"TIM1\" as T1\n"
            "participant \"TIM5\" as T5\n"
            "participant \"SpeedCalc\" as SPC\n"
            "\n"
            "SPC -> T1 : Current_L = __HAL_TIM_GET_COUNTER(&TimHandleL)\n"
            "activate T1\n"
            "note over T1 : 16-bit counter (PA8/PA9)\n"
            "T1 --> SPC : Current_L\n"
            "deactivate T1\n"
            "\n"
            "SPC -> T5 : Current_R = __HAL_TIM_GET_COUNTER(&TimHandleR)\n"
            "activate T5\n"
            "note over T5 : 16-bit counter (PA0/PA1)\n"
            "T5 --> SPC : Current_R\n"
            "deactivate T5\n"
            "\n"
            "note over SPC : Delta_L = Current_L - Pre_Speed_L\n"
            "note over SPC : Delta_R = Current_R - Pre_Speed_R\n"
            "\n"
            "opt Delta_L < -20000\n"
            "    note over SPC : underflow: Delta_L += 65535\n"
            "end\n"
            "\n"
            "opt Delta_L > 20000\n"
            "    note over SPC : overflow: Delta_L -= 65535\n"
            "end\n"
            "\n"
            "opt Delta_R < -20000\n"
            "    note over SPC : underflow: Delta_R += 65535\n"
            "end\n"
            "\n"
            "opt Delta_R > 20000\n"
            "    note over SPC : overflow: Delta_R -= 65535\n"
            "end\n"
            "\n"
            "note over SPC : *speedL = -Delta_L\n"
            "note over SPC : *speedR = Delta_R\n"
            "note over SPC : Speed_A = (Delta_R - Delta_L) / 2 * 0.7 + Pre_Speed_A * 0.3\n"
            "note over SPC : Pre_Speed_L = Current_L\n"
            "note over SPC : Pre_Speed_R = Current_R"
        ),
    },
    # ── Example 5: Separate sub-diagrams with sections ──
    {
        "id": "multi_section_diagram",
        "keywords": [
            "section", "separator", "multi", "split", "phase",
            "subsystem", "detail", "flow"
        ],
        "description": "Multiple logical sections separated by == markers",
        "code": (
            "== system_initialization ==\n"
            "participant \"main()\" as Main\n"
            "participant \"HAL\" as HAL\n"
            "\n"
            "Main -> HAL : HAL_Init()\n"
            "activate HAL\n"
            "HAL --> Main : done\n"
            "deactivate HAL\n"
            "\n"
            "== main_control_loop ==\n"
            "participant \"SysTick\" as ST\n"
            "participant \"Motor\" as MTR\n"
            "\n"
            "ST -> Main : tick (1ms)\n"
            "activate Main\n"
            "Main -> MTR : Motor_Control(pulse)\n"
            "activate MTR\n"
            "MTR --> Main : done\n"
            "deactivate MTR\n"
            "deactivate Main\n"
            "\n"
            "== fault_handling ==\n"
            "note over Main : Fault detected\n"
            "Main -> Main : enter degraded mode\n"
            "note over Main : LED error code displayed"
        ),
    },
    # ── Example 6: ISR callback chain ──
    {
        "id": "isr_callback_chain",
        "keywords": [
            "ISR", "interrupt", "callback", "IRQ", "handler",
            "HAL_TIM_PeriodElapsedCallback", "DMA", "EXTI",
            "flag", "clear"
        ],
        "description": "Interrupt service routine callback chain",
        "code": (
            "Timer -> ISR : TIM4_IRQHandler()\n"
            "activate ISR\n"
            "ISR -> HAL : HAL_TIM_IRQHandler(&TimHandle)\n"
            "activate HAL\n"
            "HAL -> CB : HAL_TIM_PeriodElapsedCallback(htim)\n"
            "activate CB\n"
            "note over CB\n"
            "    Read sensors\n"
            "    Update control variables\n"
            "    Set flags for main loop\n"
            "end note\n"
            "CB --> HAL : done\n"
            "deactivate CB\n"
            "HAL --> ISR : done\n"
            "deactivate HAL\n"
            "ISR --> Timer : exit\n"
            "deactivate ISR"
        ),
    },
    # ── Example 7: activate/deactivate pairs ──
    {
        "id": "activate_deactivate_pairs",
        "keywords": [
            "activate", "deactivate", "lifeline", "pair",
            "balance", "self-contained"
        ],
        "description": "Properly paired activate/deactivate within the same section",
        "code": (
            "Main -> Motor : Motor_Control(pulse)\n"
            "activate Motor\n"
            "note over Motor : Processing\n"
            "Motor -> TIM4 : HAL_TIM_PWM_Pulse()\n"
            "activate TIM4\n"
            "TIM4 --> Motor : done\n"
            "deactivate TIM4\n"
            "Motor --> Main : completed\n"
            "deactivate Motor"
        ),
    },
    # ── Example 8: Simple messages + notes ──
    {
        "id": "messages_and_notes",
        "keywords": [
            "message", "arrow", "solid", "dashed", "note over",
            "note right", "simple", "basic"
        ],
        "description": "Basic message arrows with notes",
        "code": (
            "A -> B : solid_message\n"
            "activate B\n"
            "B --> A : dashed_response\n"
            "deactivate B\n"
            "\n"
            "note over A : Single-line note with colon\n"
            "\n"
            "note over A\n"
            "    Multi-line note without colon\n"
            "    Second line of note\n"
            "end note\n"
            "\n"
            "note right of B : Right-side note\n"
            "\n"
            "A -> B : message_with_label"
        ),
    },
]

SEQ_SYNTAX_FIXES = [
    {
        "symptom": "Single-line note followed by bare text that should be part of the note",
        "wrong": "note over X : first line of text\n    second line of text\nX -> Y : go",
        "right": "note over X\n    first line of text\n    second line of text\nend note\nX -> Y : go",
    },
    {
        "symptom": "Bare assignment lines without arrow",
        "wrong": "count = 0\nfield = value",
        "right": "note over A : count = 0\nA -> A : field = value",
    },
    {
        "symptom": "Orphan deactivate without matching activate",
        "wrong": "A -> B : msg\ndeactivate B",
        "right": "A -> B : msg\nactivate B\nB --> A : done\ndeactivate B",
    },
    {
        "symptom": "Using 'return' keyword in message label",
        "wrong": "A --> B : return result",
        "right": "A --> B : result",
    },
    {
        "symptom": "Using <- or <-- arrows",
        "wrong": "A <-- B : message",
        "right": "B --> A : message",
    },
    {
        "symptom": "Cross-section activate/deactivate",
        "wrong": "== sec1 ==\nactivate X\n== sec2 ==\ndeactivate X",
        "right": "== sec1 ==\nactivate X\nX -> Y : work\ndeactivate X\n== sec2 ==\nactivate X\n...\ndeactivate X",
    },
]
