/* CircArray implementation: fixed-size ring buffer with resize/delete support. */

#include "6_generated_code.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void initCircArray(CircArray *arr, int size)
{
    if (arr == NULL || size <= 0) {
        return;
    }

    if (arr->enabled) {
        fprintf(stderr, "CircArray already enabled\n");
        return;
    }

    arr->buf = (char *)calloc((size_t)size, 1);
    if (arr->buf == NULL) {
        arr->size = 0;
        arr->enabled = false;
        arr->n_r = 0;
        arr->n_w = 0;
        return;
    }

    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

int buf_putbyte(CircArray *arr, char c)
{
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0) {
        return 0;
    }

    if (buf_full(arr)) {
        return 0;
    }

    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr)
{
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0) {
        return 0;
    }

    if (buf_empty(arr)) {
        return 0;
    }

    uint32_t index = arr->n_r % arr->size;
    char c = arr->buf[index];
    arr->n_r++;
    return c;
}

bool buf_empty(CircArray *arr)
{
    if (arr == NULL) {
        return true;
    }
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr)
{
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0) {
        return false;
    }
    return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_available(CircArray *arr)
{
    if (arr == NULL) {
        return 0;
    }
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize)
{
    if (arr == NULL || newSize <= 0) {
        return false;
    }

    char *new_buf = (char *)realloc(arr->buf, (size_t)newSize);
    if (new_buf == NULL) {
        return false;
    }

    arr->buf = new_buf;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr)
{
    if (arr == NULL) {
        return false;
    }

    if (arr->buf != NULL) {
        if (arr->size > 0) {
            memset(arr->buf, 0, arr->size);
        }
        free(arr->buf);
        arr->buf = NULL;
    }

    arr->size = 0;
    arr->enabled = false;
    arr->n_r = 0;
    arr->n_w = 0;
    return true;
}

void buf_clear(CircArray *arr)
{
    if (arr == NULL || arr->buf == NULL || arr->size == 0) {
        return;
    }

    memset(arr->buf, 0, arr->size);
}

