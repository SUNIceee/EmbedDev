"""PlantUML state diagram knowledge base — curated correct syntax examples.

Each entry = {id, keywords, description, code}
"""

STATE_DIAGRAM_EXAMPLES = [
    # ── Example 1: Composite states with notes OUTSIDE ──
    {
        "id": "composite_state_note_outside",
        "keywords": [
            "composite", "state", "note", "note right of", "substate",
            "initialization", "init", "peripheral", "hardware", "STM32",
            "nested", "hierarchical"
        ],
        "description": "Composite state with notes placed OUTSIDE the closing brace",
        "code": (
            "state SystemInit {\n"
            "    [*] --> ClockConfig\n"
            "    ClockConfig --> PeripheralInit : clocks_ready\n"
            "    PeripheralInit --> [*] : periph_done\n\n"
            "    state ClockConfig {\n"
            "        [*] --> SetHSE\n"
            "        SetHSE --> SetPLL : hse_stable\n"
            "        SetPLL --> SetAPB2 : pll_locked\n"
            "        SetAPB2 --> [*] : done\n"
            "    }\n\n"
            "    state PeripheralInit {\n"
            "        [*] --> GPIO_Init\n"
            "        GPIO_Init --> TIM_Init : gpio_done\n"
            "        TIM_Init --> ADC_Init : tim_done\n"
            "        ADC_Init --> [*] : adc_done\n"
            "    }\n"
            "}\n"
            "note right of ClockConfig\n"
            "    HSE = 8MHz, PLL = 168MHz\n"
            "    APB2 = 84MHz (TIM clock)\n"
            "end note\n"
            "note right of PeripheralInit\n"
            "    GPIO: PC5, PC13 outputs\n"
            "    TIM4: PWM 8.4kHz\n"
            "end note"
        ),
    },
    # ── Example 2: Running loop with subsystems ──
    {
        "id": "control_loop_subsystems",
        "keywords": [
            "control", "loop", "running", "periodic", "tick", "PID",
            "sensor", "acquire", "motor", "servo", "ADC", "encoder",
            "speed", "PWM", "subsystem"
        ],
        "description": "Main control loop with sensor/actuator subsystems as nested states",
        "code": (
            "state Running {\n"
            "    [*] --> ControlLoop\n"
            "    ControlLoop --> SpeedAcquire : tick\n"
            "    SpeedAcquire --> ADCAcquire : speed_done\n"
            "    ADCAcquire --> PIDCompute : adc_done\n"
            "    PIDCompute --> MotorOutput : pid_done\n"
            "    MotorOutput --> ServoOutput : motor_done\n"
            "    ServoOutput --> ControlLoop : servo_done\n\n"
            "    state SpeedAcquire {\n"
            "        [*] --> ReadEncoder\n"
            "        ReadEncoder --> CalcDelta : read_done\n"
            "        CalcDelta --> OverflowCheck : delta_ready\n"
            "        OverflowCheck --> [*] : speed_ready\n"
            "    }\n\n"
            "    state PIDCompute {\n"
            "        [*] --> ComputeError\n"
            "        ComputeError --> P_Term : err_ready\n"
            "        P_Term --> I_Term : p_done\n"
            "        I_Term --> D_Term : i_done\n"
            "        D_Term --> SumPID : d_done\n"
            "        SumPID --> ClampOutput : summed\n"
            "        ClampOutput --> [*] : output_ready\n"
            "    }\n\n"
            "    state MotorOutput {\n"
            "        [*] --> CheckPolarity\n"
            "        CheckPolarity --> SetForward : pulse_positive\n"
            "        CheckPolarity --> SetReverse : pulse_negative\n"
            "        SetForward --> WriteCCR : dir_set\n"
            "        SetReverse --> WriteCCR : dir_set\n"
            "        WriteCCR --> [*] : pwm_active\n"
            "    }\n"
            "}\n"
            "note right of SpeedAcquire\n"
            "    TIM1 (left): PA8/PA9, 16-bit counter\n"
            "    Delta = Current - Previous\n"
            "end note\n"
            "note right of MotorOutput\n"
            "    Motor1: PC5 dir, TIM4 CH4\n"
            "    Pulse >= 0: dir=forward\n"
            "    Pulse < 0: dir=reverse\n"
            "end note"
        ),
    },
    # ── Example 3: Fault handling states ──
    {
        "id": "fault_handling",
        "keywords": [
            "fault", "error", "error handler", "Error_Handler", "LED",
            "timeout", "fail", "warning", "degraded", "recovery"
        ],
        "description": "Fault/error states with recovery transitions",
        "code": (
            "state FaultHandling {\n"
            "    [*] --> LogError\n"
            "    LogError --> LEDIndicate : logged\n"
            "    LEDIndicate --> DegradedMode : indicated\n"
            "    DegradedMode --> RecoveryCheck : check\n"
            "    RecoveryCheck --> [*] : recovered\n"
            "    RecoveryCheck --> DegradedMode : still_fault\n"
            "}\n"
            "note right of FaultHandling\n"
            "    Error codes shown via LED blink\n"
            "    Degraded mode: operate with defaults\n"
            "    Recovery: periodic retry every 1s\n"
            "end note"
        ),
    },
    # ── Example 4: Conditional branching with merge ──
    {
        "id": "conditional_branch_merge",
        "keywords": [
            "branch", "merge", "condition", "if", "else", "choice",
            "threshold", "check", "overflow", "underflow", "filter",
            "decimation", "subsampling"
        ],
        "description": "Conditional branches that MERGE back — both paths go to same state",
        "code": (
            "state DecimationLogic {\n"
            "    [*] --> CheckCount\n"
            "    CheckCount --> DoWork : count_met\n"
            "    CheckCount --> SkipWork : count_below\n"
            "    DoWork --> Merge : work_done\n"
            "    SkipWork --> Merge : skipped\n"
            "    Merge --> Increment : merged\n"
            "    Increment --> Interpolate : inc_done\n"
            "    Interpolate --> [*] : output_ready\n"
            "}\n"
            "note right of DecimationLogic\n"
            "    Both branches converge at Merge\n"
            "    count++ and interpolation always execute\n"
            "    regardless of which branch was taken\n"
            "end note"
        ),
    },
    # ── Example 5: Simple flat state with transitions ──
    {
        "id": "simple_flat_states",
        "keywords": [
            "simple", "flat", "idle", "active", "busy", "ready",
            "standby", "off", "on", "status", "mode"
        ],
        "description": "Simple flat states with short label transitions",
        "code": (
            "[*] --> Idle\n"
            "Idle --> Active : start_cmd\n"
            "Active --> Busy : processing\n"
            "Busy --> Idle : done\n"
            "Busy --> Error : timeout\n"
            "Error --> Idle : reset\n"
            "Idle --> Standby : low_power\n"
            "Standby --> Idle : wake\n"
        ),
    },
    # ── Example 6: SD card init flow with skip path ──
    {
        "id": "sd_card_init",
        "keywords": [
            "SD", "card", "FatFs", "mount", "file", "init",
            "f_mount", "f_open", "LED", "blink", "storage"
        ],
        "description": "SD card initialization with skip-on-failure path",
        "code": (
            "state SDCardInit {\n"
            "    [*] --> RegisterDriver\n"
            "    RegisterDriver --> MountFS : reg_ok\n"
            "    RegisterDriver --> SkipSD : reg_fail\n"
            "    MountFS --> CreateFile : mount_ok\n"
            "    MountFS --> ErrorHalt : mount_fail\n"
            "    CreateFile --> WriteVerify : file_ok\n"
            "    CreateFile --> ErrorHalt : open_fail\n"
            "    WriteVerify --> FlashLED : write_ok\n"
            "    FlashLED --> [*] : led_done\n"
            "    SkipSD --> [*] : silent_skip\n"
            "}\n"
            "note right of SDCardInit\n"
            "    Success: LED0 blinks 6x (200ms each)\n"
            "    Driver link fail: skip silently\n"
            "    Mount/open fail: Error_Handler()\n"
            "end note"
        ),
    },
    # ── Example 7: Interrupt-driven state transitions ──
    {
        "id": "interrupt_driven",
        "keywords": [
            "ISR", "interrupt", "callback", "IRQ", "handler",
            "SysTick", "TIM", "DMA", "EXTI", "trigger"
        ],
        "description": "States triggered by hardware interrupts",
        "code": (
            "state ISRHandling {\n"
            "    [*] --> ISR_Entry\n"
            "    ISR_Entry --> ClearFlag : entered\n"
            "    ClearFlag --> ReadSensor : flag_cleared\n"
            "    ReadSensor --> UpdateBuffer : sensor_read\n"
            "    UpdateBuffer --> SetFlag : buffer_updated\n"
            "    SetFlag --> [*] : exit_isr\n"
            "}\n"
            "note right of ISRHandling\n"
            "    Keep ISR short: no blocking ops\n"
            "    Defer heavy processing to main loop\n"
            "    Use volatile flags for ISR->main sync\n"
            "end note"
        ),
    },
    # ── Example 8: Global skinparam and title ──
    {
        "id": "skinparam_and_title",
        "keywords": [
            "skinparam", "title", "font", "style", "format", "color"
        ],
        "description": "Correct skinparam and title declarations",
        "code": (
            "@startuml\n"
            "skinparam DefaultFontSize 11\n"
            "skinparam StateFontStyle bold\n"
            "skinparam DefaultFontName Courier\n"
            "skinparam BackgroundColor #FEFEFE\n"
            "skinparam StateBorderColor #333333\n"
            "skinparam StateBackgroundColor #F0F0F0\n"
            "title STM32F4 Domain State Machine\n"
            "@enduml"
        ),
    },
]

# ── Common mistakes and their fixes (negative -> positive correction pairs) ──
SYNTAX_FIXES = [
    {
        "symptom": "note inside state {} block causes parse error at that line",
        "wrong": "state Foo {\n    [*] --> Bar\n    note right\n        text here\n    end note\n}",
        "right": "state Foo {\n    [*] --> Bar\n}\nnote right of Foo\n    text here\nend note",
    },
    {
        "symptom": "> or < characters in transition labels break PlantUML",
        "wrong": "Check --> DoWork : count < N",
        "right": "Check --> DoWork : count_below_N",
    },
    {
        "symptom": "unclosed note block swallows subsequent content",
        "wrong": "note right of X\n    description text\nStateA --> StateB : go",
        "right": "note right of X\n    description text\nend note\nStateA --> StateB : go",
    },
    {
        "symptom": "mismatched state { } braces",
        "wrong": "state Outer {\n    state Inner {\n        [*] --> Done\n    }\n}  # missing inner }",
        "right": "state Outer {\n    state Inner {\n        [*] --> Done\n    }\n}",
    },
]
