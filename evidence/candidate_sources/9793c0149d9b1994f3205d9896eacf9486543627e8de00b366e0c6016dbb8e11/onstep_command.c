/* Implementation of LX200 protocol parsing and command execution */

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
    if (source_channel > OS_CHANNEL_ETHERNET || reply_buffer_size == 0) {
        return OS_ERR_INVALID_ARGUMENT;
    }
    if (length < 3 || command[0] != OS_LX200_CMD_PREFIX || command[length - 1] != OS_LX200_CMD_SUFFIX) {
        return OS_ERR_COMMAND_FORMAT;
    }

    reply_buffer[0] = '\0';
    *reply_length = 0;

    /* Parse LX200 command family */
    if (command[1] == 'G') {
        /* Query commands */
        if (command[2] == 'R' && length == 4) {
            /* Get RA: :GR# -> HH:MM:SS# */
            os_equatorial_coord_t eq;
            os_query_coordinates(&eq);
            int hrs = (int)eq.ra_hours;
            int mins = (int)((eq.ra_hours - (float)hrs) * 60.0f);
            int secs = (int)((((eq.ra_hours - (float)hrs) * 60.0f) - (float)mins) * 60.0f);
            snprintf(reply_buffer, reply_buffer_size, "%02d:%02d:%02d#", hrs, mins, secs);
        } else if (command[2] == 'D' && length == 4) {
            /* Get Dec: :GD# -> +DD*MM:SS# */
            os_equatorial_coord_t eq;
            os_query_coordinates(&eq);
            char sign = (eq.dec_degrees >= 0.0f) ? '+' : '-';
            float abs_dec = (eq.dec_degrees >= 0.0f) ? eq.dec_degrees : -eq.dec_degrees;
            int degs = (int)abs_dec;
            int mins = (int)((abs_dec - (float)degs) * 60.0f);
            int secs = (int)((((abs_dec - (float)degs) * 60.0f) - (float)mins) * 60.0f);
            snprintf(reply_buffer, reply_buffer_size, "%c%02d*%02d:%02d#", sign, degs, mins, secs);
        } else if (command[2] == 'V' && command[3] == 'P' && length == 5) {
            /* Get Version: :GVP# */
            snprintf(reply_buffer, reply_buffer_size, "OnStep %d.%d.%d#",
                     OS_FIRMWARE_VERSION_MAJOR, OS_FIRMWARE_VERSION_MINOR, OS_FIRMWARE_VERSION_PATCH);
        } else {
            snprintf(reply_buffer, reply_buffer_size, "0#");
        }
    } else if (command[1] == 'S') {
        /* Set commands */
        if (command[2] == 'r') {
            /* Set RA: :SrHH:MM:SS# */
            int h = 0, m = 0, s = 0;
            if (sscanf(command + 3, "%d:%d:%d", &h, &m, &s) >= 2) {
                g_os_ctx.current_target_eq.ra_hours = (float)h + (float)m / 60.0f + (float)s / 3600.0f;
                snprintf(reply_buffer, reply_buffer_size, "1");
            } else {
                snprintf(reply_buffer, reply_buffer_size, "0");
            }
        } else if (command[2] == 'd') {
            /* Set Dec: :Sd+DD*MM:SS# */
            int d = 0, m = 0, s = 0;
            char sign = '+';
            if (sscanf(command + 3, "%c%d*%d:%d", &sign, &d, &m, &s) >= 2) {
                float dec = (float)d + (float)m / 60.0f + (float)s / 3600.0f;
                if (sign == '-') dec = -dec;
                g_os_ctx.current_target_eq.dec_degrees = dec;
                snprintf(reply_buffer, reply_buffer_size, "1");
            } else {
                snprintf(reply_buffer, reply_buffer_size, "0");
            }
        } else {
            snprintf(reply_buffer, reply_buffer_size, "0");
        }
    } else if (command[1] == 'M') {
        /* Motion commands */
        if (command[2] == 'S' && length == 4) {
            /* Slew Goto: :MS# */
            os_error_t err = os_goto_equatorial(g_os_ctx.current_target_eq);
            if (err == OS_ERR_NONE) {
                snprintf(reply_buffer, reply_buffer_size, "0");
            } else {
                snprintf(reply_buffer, reply_buffer_size, "1");
            }
        } else if (command[2] == 'e') {
            os_move_start(OS_DIRECTION_EAST, OS_SPEED_MEDIUM);
        } else if (command[2] == 'w') {
            os_move_start(OS_DIRECTION_WEST, OS_SPEED_MEDIUM);
        } else if (command[2] == 'n') {
            os_move_start(OS_DIRECTION_NORTH, OS_SPEED_MEDIUM);
        } else if (command[2] == 's') {
            os_move_start(OS_DIRECTION_SOUTH, OS_SPEED_MEDIUM);
        }
    } else if (command[1] == 'Q') {
        /* Stop motion */
        os_goto_abort();
        os_move_stop();
    } else if (command[1] == 'h') {
        if (command[2] == 'P') {
            os_park();
        } else if (command[2] == 'O') {
            os_unpark();
        }
    }

    *reply_length = strlen(reply_buffer);
    return OS_ERR_NONE;
}
