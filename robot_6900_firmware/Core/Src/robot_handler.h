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
enum Movement {Mouvement_non, Mouvement_Post_Launch, Start, Avancer, Reculer, Stopper, Rot_90D, Rot_90G, Rot_180D, Rot_180G, Rot_AngD, Rot_AngG, Depl_Coord};
enum Washer{Washer_Stop, Washer_Brosse_ON, Washer_Brosse_OFF, Washer_Pump_ON, Washer_Pump_OFF, Washer_UVC_ON, Washer_UVC_OFF};
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

	enum Washer Etat_Washer;

	uint8_t motor_0_duty;
	uint8_t motor_1_duty;

	uint16_t motor_0_current_mA;
	uint16_t motor_1_current_mA;

	uint8_t  Speed;						// Robot Speed
	uint8_t  X_Pos;	                 	// Robot X Coord
	uint8_t  Y_Pos;                  	// Robot Y Coord
	uint16_t   Angle;                // Robot Angle

	uint32_t time_ms;

	enum LiDAR LiDAR_State;			// State of the LiDAR

}ROBOT6900_STATE;

typedef struct __attribute__((__packed__))
{
	uint8_t state;
	uint16_t error_code;
	uint8_t* data;
	uint16_t data_size;
	uint8_t RPlidar_update;
}RPLIDAR_HANDLER;

typedef struct __attribute__((__packed__))
{
	IWDG_HandleTypeDef* _hiwdg;

	ROBOT6900_STATE* robot_state;

	RPLIDAR_HANDLER* RPlidar;

	uint8_t* aux_data;

}ROBOT6900_HANDLER;





#endif /* SRC_COMMAND_PARSER_ROBOT_HANDLER_H_ */
