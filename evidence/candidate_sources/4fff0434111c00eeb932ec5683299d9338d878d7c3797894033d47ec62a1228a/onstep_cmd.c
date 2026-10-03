/*
 * OnStep LX200 Command Parsing Implementation
 */

#include "onstep_internal.h"
#include <stdio.h>
#include <string.h>

os_error_t os_command_parse(const char *command, size_t length,
                            uint8_t source_channel,
                            char *reply_buffer, size_t reply_buffer_size,
                            size_t *reply_length) {
    if (!command || !reply_buffer || !reply_length) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (source_channel > OS_CHANNEL_ETHERNET) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length < 3 || command[0] != OS_LX200_CMD_PREFIX || command[length - 1] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    *reply_length = 0;
    reply_buffer[0] = '\0';

    if (strncmp(command, ":GVP#", 5) == 0) {
        int n = snprintf(reply_buffer, reply_buffer_size, "OnStep 1.0.0#");
        if (n > 0) *reply_length = (size_t)n;
        return OS_ERR_NONE;
    }

    if (strncmp(command, ":GR#", 4) == 0) {
        os_equatorial_coord_t eq;
        os_query_coordinates(&eq);
        int h = (int)eq.ra_hours;
        int m = (int)((eq.ra_hours - h) * 60.0f);
        int s = (int)((eq.ra_hours - h - m / 60.0f) * 3600.0f);
        int n = snprintf(reply_buffer, reply_buffer_size, "%02d:%02d:%02d#", h, m, s);
        if (n > 0) *reply_length = (size_t)n;
        return OS_ERR_NONE;
    }

    if (strncmp(command, ":GD#", 4) == 0) {
        os_equatorial_coord_t eq;
        os_query_coordinates(&eq);
        char sign = (eq.dec_degrees >= 0) ? '+' : '-';
        float abs_dec = fabsf(eq.dec_degrees);
        int d = (int)abs_dec;
        int m = (int)((abs_dec - d) * 60.0f);
        int s = (int)((abs_dec - d - m / 60.0f) * 3600.0f);
        int n = snprintf(reply_buffer, reply_buffer_size, "%c%02d*%02d'%02d#", sign, d, m, s);
        if (n > 0) *reply_length = (size_t)n;
        return OS_ERR_NONE;
    }

    if (strncmp(command, ":MS#", 4) == 0) {
        os_error_t err = os_goto_equatorial(g_os_ctx.target_equatorial);
        if (err == OS_ERR_NONE) {
            int n = snprintf(reply_buffer, reply_buffer_size, "0#");
            if (n > 0) *reply_length = (size_t)n;
        } else {
            int n = snprintf(reply_buffer, reply_buffer_size, "1Target Error#");
            if (n > 0) *reply_length = (size_t)n;
        }
        return OS_ERR_NONE;
    }

    if (strncmp(command, ":Q#", 3) == 0) {
        os_goto_abort();
        return OS_ERR_NONE;
    }

    if (strncmp(command, ":hP#", 4) == 0) {
        os_park();
        int n = snprintf(reply_buffer, reply_buffer_size, "1#");
        if (n > 0) *reply_length = (size_t)n;
        return OS_ERR_NONE;
    }

    if (strncmp(command, ":hO#", 4) == 0) {
        os_unpark();
        int n = snprintf(reply_buffer, reply_buffer_size, "1#");
        if (n > 0) *reply_length = (size_t)n;
        return OS_ERR_NONE;
    }

    if (strncmp(command, ":Mn#", 4) == 0) {
        os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
        return OS_ERR_NONE;
    }
    if (strncmp(command, ":Ms#", 4) == 0) {
        os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
        return OS_ERR_NONE;
    }
    if (strncmp(command, ":Me#", 4) == 0) {
        os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
        return OS_ERR_NONE;
    }
    if (strncmp(command, ":Mw#", 4) == 0) {
        os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
        return OS_ERR_NONE;
    }

    return OS_ERR_COMMAND_FORMAT;
}
