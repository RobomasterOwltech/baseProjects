/*
 * readADC.cpp
 * Programm to read an ADC and display the result in the serial monitor
 */


#include "ADC.h"

ADC::ADC(ADC_HandleTypeDef *_hadc1, UART_HandleTypeDef *_huart3) {
	hadc = _hadc1;
	huart = _huart3;
	HAL_ADC_Start(hadc);
}

void ADC::read(){
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	data =  HAL_ADC_GetValue(hadc);
	msg[0] = (data/10000)+48;
	data = data%10000;
	msg[1] = (data/1000)+48;
	data = data%1000;
	msg[2] = (data/100)+48;
	data = data%100;
	msg[3] = (data/10)+48;
	data = data%10;
	msg[4] = data+48;
	msg[5] = 10;
	msg[6] = 13;
	HAL_UART_Transmit(huart, msg, 7, HAL_MAX_DELAY);
}

