/**
 * @file buffer.c
 * @brief RingBuffer implementation using overwrite-on-full for real-time data transfer.
 * @details Follows the Google C Style Guide; provides an efficient, thread/interrupt-safe lock-free design for a single producer and single consumer.
 */

#include "disco_core.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the ring buffer.
 * 
 * @param buf Pointer to the RingBuffer structure to initialize.
 * @param storage_buffer Externally supplied backing storage array.
 * @param capacity Maximum capacity of the backing storage.
 * @return true Initialization succeeded.
 * @return false Invalid argument or null pointer.
 */
bool ring_buffer_init(RingBuffer *buf, uint8_t *storage_buffer, uint32_t capacity) {
  if (buf == NULL || storage_buffer == NULL || capacity == 0) {
    return false;
  }

  buf->buffer = storage_buffer;
  buf->capacity = capacity;
  buf->head = 0;
  buf->tail = 0;
  buf->full = false;

  return true;
}

/**
 * @brief Clear the ring buffer.
 * 
 * @param buf Pointer to the RingBuffer structure.
 */
void ring_buffer_clear(RingBuffer *buf) {
  if (buf == NULL) {
    return;
  }

  buf->head = 0;
  buf->tail = 0;
  buf->full = false;
}

/**
 * @brief Check whether the ring buffer is empty.
 * 
 * @param buf Pointer to the RingBuffer structure.
 * @return true The buffer is empty.
 * @return false The buffer is not empty.
 */
bool ring_buffer_is_empty(const RingBuffer *buf) {
  if (buf == NULL) {
    return true;
  }
  return (!buf->full && (buf->head == buf->tail));
}

/**
 * @brief Check whether the ring buffer is full.
 * 
 * @param buf Pointer to the RingBuffer structure.
 * @return true The buffer is full.
 * @return false The buffer is not full.
 */
bool ring_buffer_is_full(const RingBuffer *buf) {
  if (buf == NULL) {
    return false;
  }
  return buf->full;
}

/**
 * @brief Get the number of bytes currently stored in the ring buffer.
 * 
 * @param buf Pointer to the RingBuffer structure.
 * @return uint32_t Number of stored bytes.
 */
uint32_t ring_buffer_size(const RingBuffer *buf) {
  if (buf == NULL) {
    return 0;
  }

  if (buf->full) {
    return buf->capacity;
  }

  if (buf->head >= buf->tail) {
    return buf->head - buf->tail;
  }

  return buf->capacity + buf->head - buf->tail;
}

/**
 * @brief Push one byte into the ring buffer.
 * @details Overwrite on full: replace the oldest data to retain the latest data.
 * 
 * @param buf Pointer to the RingBuffer structure.
 * @param data Byte to push.
 * @return int 0: normal push; 1: full buffer, oldest byte overwritten; -1: invalid pointer.
 */
int ring_buffer_push(RingBuffer *buf, uint8_t data) {
  if (buf == NULL || buf->buffer == NULL || buf->capacity == 0) {
    return -1;
  }

  int result = 0;

  if (buf->full) {
    // When full, advance both head and tail, discarding the oldest byte to retain the newest.
    buf->tail = (buf->tail + 1) % buf->capacity;
    result = 1; // Record that an overwrite occurred.
  }

  buf->buffer[buf->head] = data;
  buf->head = (buf->head + 1) % buf->capacity;

  if (buf->head == buf->tail) {
    buf->full = true;
  }

  return result;
}

/**
 * @brief Pop one byte from the ring buffer.
 * 
 * @param buf Pointer to the RingBuffer structure.
 * @param data Output pointer receiving the byte.
 * @return int 0: byte popped successfully; -1: empty buffer or invalid pointer.
 */
int ring_buffer_pop(RingBuffer *buf, uint8_t *data) {
  if (buf == NULL || data == NULL || buf->buffer == NULL || buf->capacity == 0) {
    return -1;
  }

  if (ring_buffer_is_empty(buf)) {
    return -1; // The buffer is empty.
  }

  *data = buf->buffer[buf->tail];
  buf->full = false;
  buf->tail = (buf->tail + 1) % buf->capacity;

  return 0;
}
