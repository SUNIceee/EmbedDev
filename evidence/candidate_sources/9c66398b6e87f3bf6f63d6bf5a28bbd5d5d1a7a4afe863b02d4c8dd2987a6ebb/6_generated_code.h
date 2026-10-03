#ifndef FSE_GENERATED_CODE_H_
#define FSE_GENERATED_CODE_H_

/*
 * 6_generated_code.h
 *
 * Public API for the Frozen DiscoBot library. This header is generated from
 * the frozen API specification and must remain the single source of truth for
 * external callers.
 *
 * The host-injection functions (DiscoHost_*) are available in all builds and
 * are intended for deterministic test harnesses. When DISCOBOT_TARGET is
 * defined, the same functions can be used to simulate external events.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Task callback signature used by the internal scheduler. */
typedef void (*TaskFunction)(void);

/*
 * Opaque-capable ring/circular buffer public data structure.
 * The fields are intentionally exposed so that host tests can inspect state
 * without breaking the library API.
 */
typedef struct CircArray {
  uint8_t* data;
  size_t capacity;
  size_t head;
  size_t tail;
  size_t count;
} CircArray;

/* System and main loop */
void DiscoBot_SystemInit(void);
void DiscoBot_CommandDispatch(uint8_t cmd);
void main_loop_iteration(void);
void SysTick_Handler(void);

/* Motor controls */
void DiscoBot_CarForward(void);
void DiscoBot_CarBackward(void);
void DiscoBot_CarLeft(void);
void DiscoBot_CarRight(void);
void DiscoBot_CarStop(void);

/* LED controls */
void DiscoBot_LEDInit(void);
void DiscoBot_LEDOn(uint8_t led);
void DiscoBot_LEDOff(uint8_t led);
void DiscoBot_LEDToggle(uint8_t led);

/* Button input */
uint8_t DiscoBot_ButtonGetState(void);
void DiscoBot_ButtonDebounce(void);

/* Accelerometer */
void DiscoBot_AccelInit(void);
void DiscoBot_AccelRead(float* x, float* y, float* z);

/* Temperature sensor */
void DiscoBot_TempInit(void);
float DiscoBot_TempRead(void);

/* Random number generator */
void DiscoBot_RNGInit(void);
bool DiscoBot_RNGIsReady(void);
uint32_t DiscoBot_RNGRead(void);
void DiscoBot_RNGIRQHandler(void);

/* Task scheduler */
void DiscoBot_TaskInit(void);
bool DiscoBot_TaskAdd(TaskFunction func, uint32_t period_ms);
void DiscoBot_TaskExecute(void);

/* USART */
void DiscoBot_USARTInit(void);
void DiscoBot_USARTIRQHandler(void);
bool DiscoBot_USARTReadByte(uint8_t* byte);
void DiscoBot_USARTSendByte(uint8_t byte);
void DiscoBot_USARTSendString(const char* s);

/* Circular array utilities */
bool DiscoBot_CircArrayInit(CircArray* ca, size_t capacity);
bool DiscoBot_CircArrayPush(CircArray* ca, uint8_t value);
bool DiscoBot_CircArrayPop(CircArray* ca, uint8_t* value);
bool DiscoBot_CircArrayIsFull(const CircArray* ca);
bool DiscoBot_CircArrayIsEmpty(const CircArray* ca);
void DiscoBot_CircArrayFree(CircArray* ca);

/* Host injection helpers */
void DiscoHost_InjectButton(uint8_t state);
void DiscoHost_InjectAccel(float x, float y, float z);
void DiscoHost_InjectADC(float voltage);
void DiscoHost_InjectRNG(uint32_t value, bool ready);
void DiscoHost_InjectUARTByte(uint8_t byte);
void DiscoHost_SetFlagReady(uint8_t flag_id, bool ready);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif  /* FSE_GENERATED_CODE_H_ */
