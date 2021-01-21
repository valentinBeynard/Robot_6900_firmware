/*
 * washer_handler.c
 *
 *  Created on: Jan 19, 2021
 *      Author: valbe
 */

#include "washer_handler.h"

ADC_HandleTypeDef* _hadc1;

uint16_t humid;

void washer_init(ADC_HandleTypeDef* p_hadc1)
{
	_hadc1 = p_hadc1;

	initialiser_pompe();
	initialiser_brosse();

	//HAL_ADC_Start(_hadc1);
}

void washer_process(ROBOT6900_HANDLER* h_robot6900)
{
	static uint16_t _delay = 0;
	static uint8_t enable = 0;
	static uint8_t _mode = 0;

	if(h_robot6900->robot_state->Etat_Washer != Washer_Stop)
	{
//		if(ABS(h_robot6900->robot_state->time_ms, _delay) > (_mode == 0 ? 2500 : 5000))
//		{
//			if(!enable)
//			{
//				(_mode == 0 ? demarrer_pompe() : activer_brosse());
//				enable = 1;
//			}
//			else
//			{
//				(_mode == 0 ? eteindre_pompe() : desactiver_brosse());
//				enable = 0;
//			}
//			_delay = h_robot6900->robot_state->time_ms;
//		}

		if(h_robot6900->robot_state->Etat_Washer == Washer_Brosse_ON)
		{
			activer_brosse();
		}
		else
		{
			desactiver_brosse();
		}

		if(h_robot6900->robot_state->Etat_Washer == Washer_Pump_ON)
		{
			demarrer_pompe();
		}
		else
		{
			eteindre_pompe();
		}
	}

	if(h_robot6900->robot_state->Etat_Washer == Washer_UVC_ON)
	{
		enable_UVC();
	}
	else
	{
		disable_UVC();
	}
}

void niveau_eau()
{

	HAL_ADC_PollForConversion(_hadc1, HAL_MAX_DELAY);
	humid = HAL_ADC_GetValue(_hadc1);
}

void initialiser_pompe()
{
	HAL_GPIO_WritePin(relais_GPIO_Port , relais_Pin , GPIO_PIN_SET); //relais off initialement
}

void initialiser_brosse()
{
	HAL_GPIO_WritePin(brosse_GPIO_Port , brosse_Pin , GPIO_PIN_RESET); //brosse inactive initialement
}

void demarrer_pompe()
{
  if (humid < 3000.0)
  {
		  //HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,0); //Eteindre LED: reservoir rempli
		  HAL_GPIO_WritePin(relais_GPIO_Port , relais_Pin , GPIO_PIN_RESET); //relais ferme: Allumer pompe
  }
  else
  {
		  //HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,1); //Allumer LED: reservoir vide
  }
}

void eteindre_pompe()
{
	HAL_GPIO_WritePin(relais_GPIO_Port , relais_Pin , GPIO_PIN_SET); //relais ouvert: Eteindre pompe
}

void activer_brosse()
{
	HAL_GPIO_WritePin(brosse_GPIO_Port , brosse_Pin , GPIO_PIN_SET);
}

void desactiver_brosse()
{
	HAL_GPIO_WritePin(brosse_GPIO_Port , brosse_Pin ,GPIO_PIN_RESET);
}

void enable_UVC()
{
	HAL_GPIO_WritePin(UVC_Enable_GPIO_Port , UVC_Enable_Pin , GPIO_PIN_SET);
}

void disable_UVC()
{
	HAL_GPIO_WritePin(UVC_Enable_GPIO_Port , UVC_Enable_Pin ,GPIO_PIN_RESET);
}

