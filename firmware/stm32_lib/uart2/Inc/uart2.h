/*
 * uart2.h
 *
 *  Created on: Jul 21, 2025
 *      Author: tynix
 */

#ifndef UART2_H_
#define UART2_H_

#include <stdint.h>

typedef enum {

	USART_STOP_1 = 0,
	USART_STOP_HALF,
	USART_STOP_2
} uart_stop_type_t;

typedef enum {

	USART_DATA_8 = 0,
	USART_DATA_9
} uart_data_type_t;

typedef enum {

	USART_OK = 0,
	USART_INVALID_NSTOP,
	USART_INVALID_NDATA,
	USART_IDLE_ERR,
	USART_OVERRUN_ERR,
	USART_NOISE_ERR,
	USART_FRAME_ERR,
} uart_err_type_t;

void uart2_set_fcpu(unsigned long freq);
uart_err_type_t uart2_config(uint32_t baud, uart_data_type_t ndata, uart_stop_type_t nstop);
uart_err_type_t uart2_dma1_config(uint32_t baud, uart_data_type_t ndata, uart_stop_type_t nstop);
uart_err_type_t uart2_write_byte(uint8_t data);
uart_err_type_t uart2_write(uint8_t * data, uint8_t len);
void uart2_dma1_write(uint16_t n, uint8_t * data);


#endif /* UART2_H_ */
