/*
 * uart_custom.c
 *
 *  Created on: Dec 1, 2022
 *      Author: Jakob
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "main.h"

//from ASCII table
#define STX 2 //start of text
#define US 31 //unit separator
#define ETX 3 //end of text
#define ACK 6 //acknowledge

#define cmd_gedADC 97 //command for ADC read (Potentiometer Pin)
#define cmd_blueLEDon 98 //command to turn blue LED on
#define cmd_blueLEDoff 99 //command to turn blue LED off
#define cmd_blueLEDtoggle 100 //command to toggle blue LED
#define cmd_blueLEDwrite 101 //command to specify the state of the blue LED

uint8_t buffer[6]; //buffer for incoming data

//for adc
//defined as extern because declaration is in main
extern uint16_t adcValue;
extern bool converted;
extern ADC_HandleTypeDef hadc1;

/*examples:
 * 002 101 031 (any byte) 001 003 turns on blue led
 * 002 101 031 (any byte) 000 003 turns off blue led
 * 002 100 031 (any byte) (any byte) 003 toggles blue led
 * 002 97 031 (any byte) (any byte) 003 gets the ADC value
 */


uint8_t data_decode(uint8_t *data, uint8_t *decodedData, UART_HandleTypeDef *handle){
	if(*data == STX){//check if valid start of text
		if(*(data+2) == US){ //check if valid data separator
			if(*(data+5) == ETX){//check if valid end of text

				//extract the data from buffer
				*decodedData = *(data+3);
				*(decodedData+1) = *(data+4);

				//send the acknowledgment message
				buffer[0] = ACK;
				buffer[1] = *(data+1);
				buffer[2] = US;
				buffer[3] = *decodedData;
				buffer[4] = *(decodedData+1);
				buffer[5] = ETX;
				if (HAL_UART_Transmit(handle, (uint8_t *)buffer, 6, 1000) == HAL_ERROR)
					return 100;

				//handle the commands
				switch(*(data+1)){
					case cmd_gedADC: //adc read command

						HAL_ADC_Start_IT(&hadc1);

						while(!converted){
							//waiting for conversion complete
						}

						//send back 12 bit ADC value
						buffer[0] = STX;
						buffer[1] = cmd_gedADC;
						buffer[2] = US;
						buffer[3] = adcValue >> 8; //higher 8 bits
						buffer[4] = adcValue & 0xff; //lower 8 bits
						buffer[5] = ETX;
						if (HAL_UART_Transmit(handle, (uint8_t *)buffer, 6, 1000) == HAL_ERROR)
							Error_Handler();

						converted = false;

						return 0; //exit without error

					case cmd_blueLEDon: //turn blue LED on
						HAL_GPIO_WritePin(ledBlue_GPIO_Port, ledBlue_Pin, 0); //active low LED

						//give feedback other end
						buffer[0] = STX;
						buffer[1] = cmd_blueLEDon;
						buffer[2] = US;
						buffer[3] = 1;
						buffer[4] = 1;
						buffer[5] = ETX;
						if (HAL_UART_Transmit(handle, (uint8_t *)buffer, 6, 1000) == HAL_ERROR)
							Error_Handler();
						return 0; //exit without error

					case cmd_blueLEDoff: //turn blue LED off
						HAL_GPIO_WritePin(ledBlue_GPIO_Port, ledBlue_Pin, 1); //active low LED

						//give feedback other end
						buffer[0] = STX;
						buffer[1] = cmd_blueLEDoff;
						buffer[2] = US;
						buffer[3] = 1;
						buffer[4] = 0;
						buffer[5] = ETX;
						if (HAL_UART_Transmit(handle, (uint8_t *)buffer, 6, 1000) == HAL_ERROR)
							Error_Handler();
						return 0; //exit without error

					case cmd_blueLEDtoggle: //toggles blue LED
						HAL_GPIO_TogglePin(ledBlue_GPIO_Port, ledBlue_Pin);

						//give feedback other end
						buffer[0] = STX;
						buffer[1] = cmd_blueLEDtoggle;
						buffer[2] = US;
						buffer[3] = 0;
						buffer[4] = 0;
						buffer[5] = ETX;
						if (HAL_UART_Transmit(handle, (uint8_t *)buffer, 6, 1000) == HAL_ERROR)
							Error_Handler();
						return 0; //exit without error

					case cmd_blueLEDwrite:
						HAL_GPIO_WritePin(ledBlue_GPIO_Port, ledBlue_Pin, !(*(decodedData+1) & 0x01));
						buffer[0] = STX;
						buffer[1] = cmd_blueLEDwrite;
						buffer[2] = US;
						buffer[3] = *decodedData;
						buffer[4] = *(decodedData+1);
						buffer[5] = ETX;
						if (HAL_UART_Transmit(handle, (uint8_t *)buffer, 6, 1000) == HAL_ERROR)
							Error_Handler();
						return 0; //exit without error

					default:
						return 4; //no valid command
				}
			}else
				return 3; //no valid end byte
		}else
			return 2; //no valid data separator
	}else
		return 1; //no valid start byte

}

