/*
 * ADC.h
 *
 *  Created on: Oct 10, 2023
 *      Author: serfa
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "stm32h7xx_hal.h"

  //LL_ADC::ADC
class ADC {
	private:
		uint16_t data;

		ADC_HandleTypeDef *hadc;
		UART_HandleTypeDef *huart;
	public:
		uint8_t msg[10];
		ADC(ADC_HandleTypeDef *_hadc, UART_HandleTypeDef *_huart);
		void read();
};
 /* namespace LL_ADC */
#endif /* INC_ADC_H_ */
