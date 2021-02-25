/**
 * @file switch.c
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

#include "mesh.h"
#include "switch.h"

#define SHADOW_CHECK_INTERVAL_MS (10)

typedef struct {
    bool on;
} switch_shadow_t;

static switch_shadow_t switch_shadow;
static switch_shadow_t switch_shadow_last;

static void switch_set_on_internal(uint16_t dst, bool on) {

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

int msleep(long msec);

static void switch_task(void* arg) {
    while (1) {
        msleep(SHADOW_CHECK_INTERVAL_MS);

        bool on = switch_shadow.on;
        if (on != switch_shadow_last.on) {
            switch_set_on_internal(DEMO_SWITCH_MESH_ADDR, on);
            switch_shadow_last.on = on;
        }
    }
}

void switch_init(void) {
    static pthread_t tid;
    pthread_create(&tid, NULL, &switch_task, NULL);
}

void switch_set_on(uint16_t dst, bool on) {
    switch_shadow.on = on;
}
