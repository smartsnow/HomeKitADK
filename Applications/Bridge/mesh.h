#include <stdint.h>

void mesh_init(void);
void mesh_send(uint16_t dst, uint32_t opcode, uint8_t* data, uint16_t len, uint8_t seg);