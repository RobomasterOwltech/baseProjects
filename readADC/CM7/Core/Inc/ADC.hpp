/*
 * ADC.h
 *
 *  Created on: Oct 10, 2023
 *      Author: serfa
 */

#ifndef INC_ADC_HPP_
#define INC_ADC_HPP_

#include "stm32h7xx_hal.h"


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

#endif /* INC_ADC_HPP_ */
