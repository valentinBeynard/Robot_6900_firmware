/*
 * current_probe.h
 *
 *  Created on: Jan 4, 2021
 *      Author: valbe
 */
#include "../Src/robot_handler.h"

#ifndef INC_CURRENT_PROBE_H_
#define INC_CURRENT_PROBE_H_

#define ABS(a, b)	(a <= b ? (b-a) : (a-b))

#define CURRENT_PROBES_READ_DELAY_MS	2000

#define CURRENT_PROBE_GAIN	50
#define SHUNT_VALUE	0.004
#define TO_CURRENT_MA(adc_value)	(float)( 1000.0 * ( (3.3 * adc_value) / (1.0 * 0xFFF) ) / ( CURRENT_PROBE_GAIN * SHUNT_VALUE) )

void ADC_Conversion_INT(ADC_HandleTypeDef* hadc);
void init_current_probes(ADC_HandleTypeDef* hadc);
void current_probes_process(ROBOT6900_HANDLER* h_robot6900);

#endif /* INC_CURRENT_PROBE_H_ */
