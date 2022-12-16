/*
 * analog_custom.h
 *
 *  Created on: Nov 26, 2022
 *      Author: Jakob
 */

#ifndef INC_ANALOG_CUSTOM_H_
#define INC_ANALOG_CUSTOM_H_

#include <stdint.h>
#include "main.h"

/*! \brief Function _adc_configure() starts the ADC given by the ADC Handle
 *  \param adcHandle is the ADC configuration that shall be converted
 *  \returns 0 upon success, >0 on error
 */
uint8_t _adc_configure(ADC_HandleTypeDef *adcHandle);


/*! \brief Function _adc_getval() will write the result of the ADC conversion to the provided address.
 *  \param *pValue is the address where the result will be written to
 *  \param adcHandle is the ADC configuration of which the value should be fetched
 *  \returns 0
 */
uint8_t _adc_getval(uint16_t *pvalue, ADC_HandleTypeDef *adcHandle);


/*! \brief Function _dac_setval() will configure the DAC to output
 *         an analog value at the given handle.
 *  \param value is the digital representation of the value to convert
 *  \param dacHandle is the DAC configuration where to output the value
 *  \return 0 upon success, >0 otherwise
 */
uint8_t _dac_setval(uint16_t value, DAC_HandleTypeDef *dacHandle);


#endif /* INC_ANALOG_CUSTOM_H_ */
