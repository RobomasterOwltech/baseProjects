/*
 * readADC.cpp
 * Programm to read an ADC and display the result in the serial monitor
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "stm32h7xx_hal.h"

namespace LL_ADC {
class ADC {
	private:
		float data;
		char msg[10];
		ADC_HandleTypeDef hadc1;
		UART_HandleTypeDef huart3;
	public:
		ADC();
		void read();
};
} /* namespace LL_ADC */
#endif /* INC_ADC_H_ */

namespace LL_ADC{
	ADC::ADC(ADC_HandleTypeDef &_hadc1, UART_HandleTypeDef &_huart3) {
		hadc1 = _hadc1;
		huart3 = _huart3;
		HAL_ADC_Start(hadc1);
	}
	ADC::~ADC() {}
	void ADC::read(){
    	HAL_ADC_PollForConversion(hadc1, HAL_MAX_DELAY);
    	data = HAL_ADC_GetValue(hadc1);
	    sprintf(msg, "%hu\r\n", data);
	    HAL_UART_Transmit(huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}
} 