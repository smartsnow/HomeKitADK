/**
 * @file light.c
 * @author Snow Yang (snowyang.iot@outlook.com)
 * @brief
 * @version 0.1
 * @date 2021-02-04
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>

#include "mesh.h"
#include "light.h"

#define SHADOW_CHECK_INTERVAL_MS (10)

typedef enum {
    LIGHT_MODE_WHITE,
    LIGHT_MODE_COLOR,
} light_mode_t;

typedef struct {
    bool on;
    uint16_t hue;
    uint8_t saturation;
    uint8_t value;
    uint8_t brightness;
    uint8_t temperature;
    light_mode_t mode;
} light_shadow_t;

static light_shadow_t light_shadow;
static light_shadow_t light_shadow_last;

/* msleep(): Sleep for the requested number of milliseconds. */
int msleep(long msec) {
    struct timespec ts;
    int res;

    if (msec < 0) {
        errno = EINVAL;
        return -1;
    }

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

    return res;
}

static void light_set_on_internal(uint16_t dst, bool on) {

    uint8_t data[4];

    /* TID */
    data[0] = 0x00;

    /* ONOFF type = 0x0100 */
    data[1] = 0x00;
    data[2] = 0x01;

    /* Value: uint8_t */
    data[3] = on;

    mesh_send(dst, 0xD22209, data, sizeof(data), 1);
}

static void light_set_hsv_internal(uint16_t dst, uint16_t hue, uint8_t saturation, uint8_t value) {

    uint8_t data[7];

    /* TID */
    data[0] = 0x00;

    /* HSV type = 0x0123 */
    data[1] = 0x23;
    data[2] = 0x01;

    /* Hue: uint16_t */
    data[3] = hue & 0xFF;
    data[4] = hue >> 8;

    /* Saturation: uint8_t */
    data[5] = saturation;

    /* Brightness: uint8_t */
    data[6] = value;

    mesh_send(dst, 0xD22209, data, sizeof(data), 1);
}

static void light_set_brightness_internal(uint16_t dst, uint8_t brightness) {

    uint8_t data[5];

    /* TID */
    data[0] = 0x00;

    /* Brightness type = 0x0121 */
    data[1] = 0x21;
    data[2] = 0x01;

    /* Brightness: uint16_t */
    data[3] = brightness;
    data[4] = 0x00;

    mesh_send(dst, 0xD22209, data, sizeof(data), 1);
}

static void light_set_temperature_internal(uint16_t dst, uint8_t temperature) {

    uint8_t data[4];

    /* TID */
    data[0] = 0x00;

    /* ColorTemperature Percent type = 0x01F1 */
    data[1] = 0xF1;
    data[2] = 0x01;

    /* ColorTemperature Percent: uint8_t */

    data[3] = temperature;

    mesh_send(dst, 0xD22209, data, sizeof(data), 1);
}

static void light_set_by_mode(void) {
    bool mode_changed = false;

    light_mode_t mode = light_shadow.mode;
    if (mode != light_shadow_last.mode) {
        mode_changed = true;
        light_shadow_last.mode = mode;
    }

    switch (mode) {
        case LIGHT_MODE_WHITE: {
            uint8_t brightness = light_shadow.brightness;
            uint8_t temperature = light_shadow.temperature;
            if (mode_changed || brightness != light_shadow_last.brightness) {
                light_set_brightness_internal(DEMO_LIGHT_MESH_ADDR, brightness);
                light_shadow_last.brightness = brightness;
            }
            if (mode_changed || temperature != light_shadow_last.temperature) {
                light_set_temperature_internal(DEMO_LIGHT_MESH_ADDR, temperature);
                light_shadow_last.temperature = temperature;
            }
        } break;

        case LIGHT_MODE_COLOR: {
            uint16_t hue = light_shadow.hue;
            uint8_t saturation = light_shadow.saturation;
            uint8_t value = light_shadow.value;
            if (mode_changed || hue != light_shadow_last.hue || saturation != light_shadow_last.saturation ||
                value != light_shadow_last.value) {
                light_set_hsv_internal(DEMO_LIGHT_MESH_ADDR, hue, saturation, value);
                light_shadow_last.hue = hue;
                light_shadow_last.saturation = saturation;
                light_shadow_last.value = value;
            }
        } break;
    }
}

static void light_task(void* arg) {
    while (1) {
        msleep(SHADOW_CHECK_INTERVAL_MS);

        bool on = light_shadow.on;
        if (on != light_shadow_last.on) {
            if (on) {
                light_set_by_mode();
            }
            light_set_on_internal(DEMO_LIGHT_MESH_ADDR, on);
            light_shadow_last.on = on;
        } else {
            if (on) {
                light_set_by_mode();
            }
        }
    }
}

void light_init(void) {
    static pthread_t tid;
    pthread_create(&tid, NULL, &light_task, NULL);
}

void light_color_set_on(uint16_t dst, bool on) {
    light_shadow.on = on;
    light_shadow.mode = LIGHT_MODE_COLOR;
}

void light_set_hue(uint16_t dst, float hue) {
    light_shadow.hue = hue;
}

void light_set_saturation(uint16_t dst, float saturation) {
    light_shadow.saturation = saturation;
}

void light_set_value(uint16_t dst, uint8_t value) {
    light_shadow.value = value;
}

void light_white_set_on(uint16_t dst, bool on) {
    light_shadow.on = on;
    light_shadow.mode = LIGHT_MODE_WHITE;
}

void light_set_brightness(uint16_t dst, uint8_t brightness) {
    light_shadow.brightness = brightness;
}

void light_set_temperature(uint16_t dst, uint32_t temperature) {
    light_shadow.temperature = (300 - temperature) * 100 / 350;
}
