/*
 * analog_custom.c
 *
 *  Created on: Nov 26, 2022
 *      Author: Jakob
 */

#include <stdint.h>
#include "main.h"


uint8_t _adc_configure(ADC_HandleTypeDef *adcHandle){
	if (HAL_ADC_Start_IT(adcHandle) != HAL_OK) //starts adc
		return 1;

	else
		return 0;
}


uint8_t _adc_getval(uint16_t *pvalue, ADC_HandleTypeDef *adcHandle){
	*pvalue = HAL_ADC_GetValue(adcHandle); //gets the value and stores it at the address given with pvalue
	return 0;
}


uint8_t _dac_setval(uint16_t value, DAC_HandleTypeDef *dacHandle){

	if (HAL_DAC_SetValue(dacHandle, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value) != HAL_OK) //sets the dac value
		return 1;

	if (HAL_DAC_Start(dacHandle, DAC_CHANNEL_1) != HAL_OK) //starts the dac output
		return 2;

	else
		return 0;
}



