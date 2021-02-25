int uart_init(const char* name);
void uart_write(int fd, const void* data, int size);
void uart_read(int fd, void* data, int size);
