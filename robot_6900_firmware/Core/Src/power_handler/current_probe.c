/*
 * current_probe.c
 *
 *  Created on: Jan 4, 2021
 *      Author: valbe
 */
#include "current_probe.h"

ADC_HandleTypeDef* _hadc1;

uint16_t adc_values[4];
uint8_t are_ready = 0;

void ADC_Conversion_INT(ADC_HandleTypeDef* hadc)
{

	adc_values[0] = HAL_ADC_GetValue(_hadc1);
	adc_values[1] = HAL_ADC_GetValue(_hadc1);
	//adc_values[2] = HAL_ADC_GetValue(_hadc1);
	//adc_values[3] = HAL_ADC_GetValue(_hadc1);

	HAL_ADC_Stop_IT(_hadc1);
	are_ready = 1;
}

void init_current_probes(ADC_HandleTypeDef* hadc)
{
	_hadc1 = hadc;

	adc_values[0] = 0;
	adc_values[1] = 0;
	adc_values[2] = 0;
	adc_values[3] = 0;

	HAL_ADC_GetValue(_hadc1);

	are_ready = 0;
}

void current_probes_process(ROBOT6900_HANDLER* h_robot6900)
{
	static uint16_t _delay = 0;

	if(ABS(h_robot6900->robot_state->time_ms, _delay) >= CURRENT_PROBES_READ_DELAY_MS)
	{
		//HAL_ADC_PollForConversion(_hadc1, HAL_MAX_DELAY);
		HAL_ADC_Start_IT(_hadc1);
		_delay = h_robot6900->robot_state->time_ms;
	}
	else
	{
		if(are_ready)
		{
			h_robot6900->robot_state->motor_0_current_mA = TO_CURRENT_MA(adc_values[0]);
			h_robot6900->robot_state->motor_1_current_mA = TO_CURRENT_MA(adc_values[1]);
			are_ready = 0;
		}
	}
}
