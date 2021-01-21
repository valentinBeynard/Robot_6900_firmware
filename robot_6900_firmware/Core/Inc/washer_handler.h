/*
 * washer_handler.h
 *
 *  Created on: Jan 19, 2021
 *      Author: valbe
 */
#include "main.h"
#include "../Src/robot_handler.h"


#ifndef INC_WASHER_HANDLER_H_
#define INC_WASHER_HANDLER_H_

#define ABS(a, b)	(a <= b ? (b-a) : (a-b))

void washer_init(ADC_HandleTypeDef* p_hadc1);
void washer_process(ROBOT6900_HANDLER* h_robot6900);

void niveau_eau();
void initialiser_pompe();
void initialiser_brosse();
void demarrer_pompe();
void eteindre_pompe();
void activer_brosse();
void desactiver_brosse();
void enable_UVC();
void disable_UVC();

#endif /* INC_WASHER_HANDLER_H_ */
