/*
 * parser_handler.h
 *
 *  Created on: 4 oct. 2020
 *      Author: valbe
 */

#ifndef SRC_COMMAND_PARSER_ROBOT_HANDLER_H_
#define SRC_COMMAND_PARSER_ROBOT_HANDLER_H_

#include "stm32f3xx_hal.h"

enum RobotState {Sleeping, Running, Safety_Stop};
enum Movement {Mouvement_non, Avancer, Reculer, Stopper, Rot_90D, Rot_90G, Rot_180D, Rot_180G, Rot_AngD, Rot_AngG, Depl_Coord};
enum Acq_power{NoAcq, Acq_I, Acq_E, Acq_P};
enum Position{Position_non, Init_Position, Demande_Position};
enum LiDAR{LiDAR_Sleep, LiDAR_ACQ_Const};
enum DEBUG_LEDs{DB_LED_OK = 0x00, DB_LED3 = 0x01, DB_LED4 = 0x02, DB_LED5 = 0x04, DB_LED6 = 0x08, DB_LED7 = 0x10, DB_LED8 = 0x20, DB_LED9 = 0x40, DB_LED10 = 0x80};

typedef struct __attribute__((__packed__))
{
	enum DEBUG_LEDs debug_leds;

	uint8_t status_update;

	enum  RobotState Robot_State;    // State of the robot

	enum  Movement Etat_Movement;	// Type of movement asked by the HOST
	uint8_t  Speed;						// Robot Speed
	uint8_t  X_Pos;	                 	// Robot X Coord
	uint8_t  Y_Pos;                  	// Robot Y Coord
	uint16_t   Angle;                // Robot Angle

	enum LiDAR LiDAR_State;			// State of the LiDAR

}ROBOT6900_STATE;

typedef struct __attribute__((__packed__))
{
	IWDG_HandleTypeDef* _hiwdg;

	ROBOT6900_STATE robot_state;

	uint8_t* aux_data;

}ROBOT6900_HANDLER;





#endif /* SRC_COMMAND_PARSER_ROBOT_HANDLER_H_ */
