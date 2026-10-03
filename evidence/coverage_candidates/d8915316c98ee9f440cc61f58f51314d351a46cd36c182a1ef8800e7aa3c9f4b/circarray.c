/* Circular array buffer management implementation */

#include "6_generated_code.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void initCircArray(CircArray *arr, int size) {
    if (arr == NULL) return;
    if (arr->enabled) {
        return;
    }
    if (size <= 0) size = CIRC_BUFFER_MIN_SIZE;
    arr->buf = (char *)calloc((size_t)size, 1);
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

bool buf_empty(CircArray *arr) {
    return arr ? (arr->n_r == arr->n_w) : true;
}

bool buf_full(CircArray *arr) {
    if (!arr || arr->size == 0) return false;
    return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_available(CircArray *arr) {
    return arr ? (int)(arr->n_w - arr->n_r) : 0;
}

int buf_putbyte(CircArray *arr, char c) {
    if (!arr || !arr->enabled || buf_full(arr)) return 0;
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (!arr || !arr->enabled || buf_empty(arr)) return 0;
    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

bool buf_resize(CircArray *arr, int newSize) {
    if (!arr || !arr->enabled || newSize <= 0) return false;
    char *newBuf = (char *)realloc(arr->buf, (size_t)newSize);
    if (!newBuf) return false;
    arr->buf = newBuf;
    arr->size = (uint32_t)newSize;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr && arr->buf) memset(arr->buf, 0, (size_t)arr->size);
}

bool buf_delete(CircArray *arr) {
    if (!arr) return false;
    buf_clear(arr);
    free(arr->buf);
    arr->buf = NULL;
    arr->size = 0;
    arr->enabled = false;
    return true;
}
