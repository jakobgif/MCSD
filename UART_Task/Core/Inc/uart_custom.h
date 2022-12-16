/*
 * uart_custom.h
 *
 *  Created on: Dec 1, 2022
 *      Author: Jakob
 */

#ifndef INC_UART_CUSTOM_H_
#define INC_UART_CUSTOM_H_


#include <stdint.h>
#include "main.h"

/*! \brief Function data_decode will decode the incoming 6 byte data and
 * 			will carry out the commands that are given in the data
 *  \param data is a pointer to the incoming data bytes
 *  \param decodedData is a pointer to the variable where the decoded 2 bytes of data should be stored
 *  \param handle is the UART handle required to send the UART response data
 *  \return 0 if no error
 *  		1 if no valid start byte
 *  		2 if no valid data separator
 *  		3 if no valid end byte
 *  		4 if no valid command
 */
uint8_t data_decode(uint8_t *data, uint8_t *decodedData, UART_HandleTypeDef *handle);


#endif /* INC_UART_CUSTOM_H_ */
