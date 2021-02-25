#include <stdint.h>
#include <pthread.h>
#include <stdio.h>

#include "uart.h"

typedef struct {
    uint16_t dst;
    uint32_t opcode;
    uint8_t seg;
    uint8_t len;
} __attribute__((packed)) serial_mesh_msg_tx_t;

typedef struct {
    uint16_t src;
    uint16_t dst;
    uint32_t opcode;
    uint8_t ttl;
    int8_t rssi;
    uint8_t len;
} __attribute__((packed)) serial_mesh_msg_rx_t;

static int fd;
static pthread_t tid;

static void* uart_recv_thread(void* arg) {

    uint8_t data[256];
    serial_mesh_msg_rx_t msg;

    while (1) {
        uart_read(fd, &msg, sizeof(serial_mesh_msg_rx_t));
        if (msg.len > 0)
            uart_read(fd, data, msg.len);
        printf("<-- SRC:%04X, DST:%04X, OpCode:%X, TTL:%u, RSSI:%d\n", msg.src, msg.dst, msg.opcode, msg.ttl, msg.rssi);
    }
}

void mesh_init(void) {
    fd = uart_init("/dev/cu.SLAB_USBtoUART");

    pthread_create(&tid, NULL, &uart_recv_thread, NULL);
}

void mesh_send(uint16_t dst, uint32_t opcode, uint8_t* data, uint16_t len, uint8_t seg) {
    serial_mesh_msg_tx_t msg;
    msg.dst = dst;
    msg.opcode = opcode;
    msg.seg = seg;
    msg.len = len;
    printf("--> DST:%04X, OpCode:%X, SEG: %u\n", msg.dst, msg.opcode, msg.seg);
    uart_write(fd, &msg, sizeof(serial_mesh_msg_tx_t));
    uart_write(fd, data, len);
}