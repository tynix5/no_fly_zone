#ifndef INC_UART1_H_
#define INC_UART1_H_


void uart1_config(uint32_t baud);
void uart1_writebyte(uint8_t byte);
void uart1_writeint(int num);
void uart1_writestr(char * str);


#endif
