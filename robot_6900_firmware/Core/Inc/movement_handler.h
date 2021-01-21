/*
 * movement_handler.h
 *
 *  Created on: Jan 14, 2021
 *      Author: valbe
 */
#ifndef INC_MOVEMENT_HANDLER_H_
#define INC_MOVEMENT_HANDLER_H_

#include "../Src/robot_handler.h"

/** FPGA Motor Controler Frames Descriptor **/
#define FPGA_SFD	0xA0

// Commands
#define FPGA_CMD_START_STOP	0x01
#define FPGA_CMD_ROT_DIRECTION	0x02
#define FPGA_CMD_PWM_FREQUENCY	0x04
#define FPGA_CMD_DUTY_CYCLE	0x07

// Motor IDs
#define FPGA_CMD_MOTOR_ID_0	0x00
#define FPGA_CMD_MOTOR_ID_1 0x08

// Values
#define FPGA_CMD_V_START	0x01
#define FPGA_CMD_V_STOP	0x00

#define FPGA_CMD_V_DIR0	0x00
#define FPGA_CMD_V_DIR1	0x01



#define DEFAULT_MOTORS_DIR	FPGA_CMD_V_DIR0

// 7-bits value : 24 = 20 kHz PWM
#define DEFAULT_MOTORS_FREQ_PRESCALER	24

// 7-bits value : 10%
#define DEFAULT_MOTORS_DUTY	5//10
#define DECELERATE_DUTY	2

#define RAMP_INT_MS	5000
#define RAMP_DUTY_OFFSET	5
#define DECELERATE_DUTY_OFFSET	2

#define MOTOR_DELAY_MS	3000

#define ABS(a, b)	(a <= b ? (b-a) : (a-b))

typedef enum
{
	MOTOR0 = FPGA_CMD_MOTOR_ID_0,
	MOTOR1 = FPGA_CMD_MOTOR_ID_1,
	MOTOR_BOTH = (0x01 | FPGA_CMD_MOTOR_ID_1)
}MOTOR_ID;

typedef enum
{
	MVT_HDL_WAIT,
	MVT_HDL_INIT,
	MVT_HDL_ARMED,
	MVT_HDL_RUN,
	MVT_HDL_STOP
}MVT_HDL_STATE;

typedef enum
{
	MVT_HDL_OK = 0,
	MVT_HDL_WAINTING,
	MVT_HDL_BUSY
}MVT_HDL_DRIVER_ERROR;

typedef struct
{
	MVT_HDL_STATE state_name;
	MVT_HDL_DRIVER_ERROR(*state_process)(ROBOT6900_HANDLER*);
}MVT_HDL_FSM_PROCESS;

void generate_mvt_hdl_flag(ROBOT6900_HANDLER* h_robot6900);
MVT_HDL_DRIVER_ERROR movement_process(ROBOT6900_HANDLER* h_robot6900);

MVT_HDL_DRIVER_ERROR mvh_hdl_idle(ROBOT6900_HANDLER* h_robot6900);
MVT_HDL_DRIVER_ERROR mvh_hdl_init(ROBOT6900_HANDLER* h_robot6900);
MVT_HDL_DRIVER_ERROR mvh_hdl_armed(ROBOT6900_HANDLER* h_robot6900);
MVT_HDL_DRIVER_ERROR mvh_hdl_running(ROBOT6900_HANDLER* h_robot6900);
MVT_HDL_DRIVER_ERROR mvh_hdl_stop(ROBOT6900_HANDLER* h_robot6900);

void motor_ramp(ROBOT6900_HANDLER* h_robot6900);
void motor_set_duty(MOTOR_ID _motor_id, uint8_t _duty);
void motor_set_dir(MOTOR_ID _motor_id, uint8_t _dir);
void motor_start_stop(MOTOR_ID _motor_id, uint8_t _start);

void decelerrate(ROBOT6900_HANDLER* h_robot6900, MOTOR_ID _motor_id, uint8_t* _duty);
void motor_delay(ROBOT6900_HANDLER* h_robot6900, uint32_t _delay);

#endif /* INC_MOVEMENT_HANDLER_H_ */
