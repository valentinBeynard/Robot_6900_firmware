/*
 * movement_handler.c
 *
 *  Created on: Jan 14, 2021
 *      Author: valbe
 */
#include "movement_handler.h"
#include "../spi_handler/spi_handler.h"


/*
    FULL STATE MACHINE

    Liste des Etats de la machines d'état du Parser associés à leur fonction
*/
const MVT_HDL_FSM_PROCESS mvt_handler_fsm[5] = {
    {MVT_HDL_WAIT, &mvh_hdl_idle},
    {MVT_HDL_INIT, &mvh_hdl_init},
	{MVT_HDL_ARMED, &mvh_hdl_armed},
	{MVT_HDL_RUN, &mvh_hdl_running},
	{MVT_HDL_STOP, &mvh_hdl_stop}
};

// Flag error related to Movement Handler behavior
MVT_HDL_DRIVER_ERROR MVT_HDL_Parser_Log = MVT_HDL_OK;

/* Current state of the command parser state machine */
MVT_HDL_STATE MVT_HDL_current_state = MVT_HDL_WAIT;

uint8_t motor_0_duty = DEFAULT_MOTORS_DUTY;
uint8_t motor_1_duty = DEFAULT_MOTORS_DUTY;


MVT_HDL_DRIVER_ERROR movement_process(ROBOT6900_HANDLER* h_robot6900)
{
	// Full State Machine Call
	MVT_HDL_Parser_Log = (mvt_handler_fsm[MVT_HDL_current_state]).state_process(h_robot6900);


	// Update Debug LED state regarding Parser state flags
	generate_parser_flag(h_robot6900);

	return MVT_HDL_Parser_Log;
}

MVT_HDL_DRIVER_ERROR mvh_hdl_idle(ROBOT6900_HANDLER* h_robot6900)
{
	MVT_HDL_DRIVER_ERROR mvt_hdl_log = MVT_HDL_OK;

	if(h_robot6900->robot_state->Etat_Movement == Mouvement_Post_Launch)
	{
		h_robot6900->robot_state->motor_0_duty = DEFAULT_MOTORS_DUTY;
		h_robot6900->robot_state->motor_1_duty = DEFAULT_MOTORS_DUTY;

		MVT_HDL_current_state = MVT_HDL_INIT;
		mvt_hdl_log = MVT_HDL_WAINTING;
	}
	else
	{
		MVT_HDL_current_state = MVT_HDL_WAIT;
	}

	return mvt_hdl_log;
}

MVT_HDL_DRIVER_ERROR mvh_hdl_init(ROBOT6900_HANDLER* h_robot6900)
{
	MVT_HDL_DRIVER_ERROR mvt_hdl_log = MVT_HDL_OK;
	SPI_DRIVER_ERROR spi_log = SPI_DRIVER_OK;

	uint8_t sub_state = 0;
	uint8_t param_data[2 * 2 * 3];

	// Create Data packets

	// Stop M0
	param_data[1] = FPGA_SFD | FPGA_CMD_MOTOR_ID_0 | FPGA_CMD_START_STOP;	// SFD + MOTOT_ID + CMD
	param_data[0] = (FPGA_CMD_V_STOP << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_START_STOP) << 7 );	// Payload and Frame Checker
	// Stop M1
	param_data[3] = FPGA_SFD | FPGA_CMD_MOTOR_ID_1 | FPGA_CMD_START_STOP;	// SFD + MOTOT_ID + CMD
	param_data[2] = (FPGA_CMD_V_STOP << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_START_STOP) << 7 );	// Payload and Frame Checker

	// DIR M0 = 0
	param_data[5] = FPGA_SFD | FPGA_CMD_MOTOR_ID_0 | FPGA_CMD_ROT_DIRECTION;	// SFD + MOTOT_ID + CMD
	param_data[4] = (FPGA_CMD_V_DIR0 << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_START_STOP) << 7 );	// Payload and Frame Checker
	// DIR M1 = 0
	param_data[7] = FPGA_SFD | FPGA_CMD_MOTOR_ID_1 | FPGA_CMD_ROT_DIRECTION;	// SFD + MOTOT_ID + CMD
	param_data[6] = (FPGA_CMD_V_DIR1 << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_START_STOP) << 7 );	// Payload and Frame Checker

	// DUTY M0 = 0
	param_data[9] = FPGA_SFD | FPGA_CMD_MOTOR_ID_0 | FPGA_CMD_DUTY_CYCLE;	// SFD + MOTOT_ID + CMD
	param_data[8] = (DEFAULT_MOTORS_DUTY << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_DUTY_CYCLE) << 7 );	// Payload and Frame Checker
	// DUTY M1 = 0
	param_data[11] = FPGA_SFD | FPGA_CMD_MOTOR_ID_1 | FPGA_CMD_DUTY_CYCLE;	// SFD + MOTOT_ID + CMD
	param_data[10] = (DEFAULT_MOTORS_DUTY << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_DUTY_CYCLE) << 7 );	// Payload and Frame Checker

//	// Start M0
//	param_data[13] = FPGA_SFD | FPGA_CMD_MOTOR_ID_0 | FPGA_CMD_START_STOP;	// SFD + MOTOT_ID + CMD
//	param_data[12] = (FPGA_CMD_V_START << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_START_STOP) << 7 );	// Payload and Frame Checker
//
//	// Start M1
//	param_data[15] = FPGA_SFD | FPGA_CMD_MOTOR_ID_1 | FPGA_CMD_START_STOP;	// SFD + MOTOT_ID + CMD
//	param_data[14] = (FPGA_CMD_V_START << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_START_STOP) << 7 );	// Payload and Frame Checker

	// Send SPI Packets
	do
	{
		spi_log = spi_request(SPI_RQT_FPGA, param_data, 2 * 3);
	}while(spi_log != SPI_DONE);

	MVT_HDL_current_state = MVT_HDL_ARMED;
	//MVT_HDL_current_state = MVT_HDL_RUN;


	return mvt_hdl_log;

}

MVT_HDL_DRIVER_ERROR mvh_hdl_armed(ROBOT6900_HANDLER* h_robot6900)
{
	MVT_HDL_DRIVER_ERROR mvt_hdl_log = MVT_HDL_OK;
	SPI_DRIVER_ERROR spi_log = SPI_DRIVER_OK;

	uint8_t sub_state = 0;
	uint8_t param_data[2 * 2];

	if(h_robot6900->robot_state->Etat_Movement == Start)
	{

		// Start M0
		param_data[1] = FPGA_SFD | FPGA_CMD_MOTOR_ID_0 | FPGA_CMD_START_STOP;	// SFD + MOTOT_ID + CMD
		param_data[0] = (FPGA_CMD_V_START << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_START_STOP) << 7 );	// Payload and Frame Checker
		// Start M1
		param_data[3] = FPGA_SFD | FPGA_CMD_MOTOR_ID_1 | FPGA_CMD_START_STOP;	// SFD + MOTOT_ID + CMD
		param_data[2] = (FPGA_CMD_V_START << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_START_STOP) << 7 );	// Payload and Frame Checker


		// Send SPI Packets
		do
		{
			spi_log = spi_request(SPI_RQT_FPGA, param_data, 2 * 2);
		}while(spi_log != SPI_DRIVER_DONE);

		MVT_HDL_current_state = MVT_HDL_RUN;

	}
	else
	{
		MVT_HDL_current_state = MVT_HDL_ARMED;
		mvt_hdl_log = MVT_HDL_WAINTING;
	}

	return mvt_hdl_log;
}

MVT_HDL_DRIVER_ERROR mvh_hdl_running(ROBOT6900_HANDLER* h_robot6900)
{
	MVT_HDL_DRIVER_ERROR mvt_hdl_log = MVT_HDL_OK;

	if(h_robot6900->robot_state->Etat_Movement != Stopper)
	{
		motor_ramp(h_robot6900);
		MVT_HDL_current_state = MVT_HDL_RUN;
	}
	else
	{
		MVT_HDL_current_state = MVT_HDL_STOP;
	}


	return mvt_hdl_log;
}

MVT_HDL_DRIVER_ERROR mvh_hdl_stop(ROBOT6900_HANDLER* h_robot6900)
{
	decelerrate(h_robot6900, MOTOR_BOTH, &h_robot6900->robot_state->motor_0_duty);
	motor_start_stop(MOTOR_BOTH, FPGA_CMD_V_STOP);

	MVT_HDL_current_state = MVT_HDL_WAIT;
}

void motor_ramp(ROBOT6900_HANDLER* h_robot6900)
{
	// Last time since previous ramp
	static uint32_t last_ramp_time_ms = 0;
	// Motor spinning direction during previous ramp
	static uint8_t last_dir = FPGA_CMD_V_DIR0;

	// Checking time. Duty cycle changing each RAMP_INT_MS milliseconds (5000)
	if(ABS(h_robot6900->robot_state->time_ms, last_ramp_time_ms) > RAMP_INT_MS)
	{

		// Max duty cycle of 60%
		if(h_robot6900->robot_state->motor_0_duty >= 60)
		{
			// Prepare next ramp sequence

			// Decelerating motor by decreasing duty
			decelerrate(h_robot6900, MOTOR_BOTH, &(h_robot6900->robot_state->motor_0_duty));
			// Send SPI Stop motors command
			motor_start_stop(MOTOR_BOTH, FPGA_CMD_V_STOP);
			// Hard coded delay to avoid inertia issue with spinning motors
			motor_delay(h_robot6900, MOTOR_DELAY_MS);
			// Change motors spinning direction
			last_dir = (last_dir == FPGA_CMD_V_DIR0 ? FPGA_CMD_V_DIR1 : FPGA_CMD_V_DIR0);
			motor_set_dir(MOTOR_BOTH, last_dir);
			// Hard coded delay to avoid inertia issue with spinning motors
			motor_delay(h_robot6900, MOTOR_DELAY_MS);
			// Send SPI Start motors command
			motor_start_stop(MOTOR_BOTH, FPGA_CMD_V_START);
		}
		else
		{
			// Accelerate motors increasing duty
			h_robot6900->robot_state->motor_0_duty += RAMP_DUTY_OFFSET;
			h_robot6900->robot_state->motor_1_duty += RAMP_DUTY_OFFSET;
		}

		// Update last ramp timer
		last_ramp_time_ms = h_robot6900->robot_state->time_ms;

		// Send SPI command for duty cycle on both motors
		motor_set_duty(MOTOR_BOTH, h_robot6900->robot_state->motor_0_duty);

	}


}

void decelerrate(ROBOT6900_HANDLER* h_robot6900, MOTOR_ID _motor_id, uint8_t* _duty)
{
	do
	{
		*_duty -= DECELERATE_DUTY_OFFSET;
		motor_set_duty(MOTOR_BOTH, *_duty);
		motor_delay(h_robot6900, 500);
	}while(*_duty > DECELERATE_DUTY);

	motor_start_stop(MOTOR_BOTH, FPGA_CMD_V_STOP);

	//h_robot6900->robot_state->motor_0_duty = *_duty;
	h_robot6900->robot_state->motor_1_duty = *_duty;
}

void motor_delay(ROBOT6900_HANDLER* h_robot6900, uint32_t _delay)
{
	uint32_t _time = 0;
	_time = h_robot6900->robot_state->time_ms;

	// Delay
	do{}while(ABS(h_robot6900->robot_state->time_ms, _time) < _delay);
}

void motor_set_duty(MOTOR_ID _motor_id, uint8_t _duty)
{
	SPI_DRIVER_ERROR spi_log = SPI_DRIVER_OK;

	uint8_t param_data[4];
	uint8_t _size = 0;


	if(_motor_id == MOTOR0)
	{
		param_data[0] = (_duty << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_DUTY_CYCLE) << 7 );	// Payload and Frame Checker
		param_data[1] = FPGA_SFD | FPGA_CMD_MOTOR_ID_0 | FPGA_CMD_DUTY_CYCLE;	// SFD + MOTOT_ID + CMD
		_size = 1;
	}
	else if(_motor_id == MOTOR1)
	{
		param_data[0] = (_duty << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_DUTY_CYCLE) << 7 );	// Payload and Frame Checker
		param_data[1] = FPGA_SFD | FPGA_CMD_MOTOR_ID_1 | FPGA_CMD_DUTY_CYCLE;	// SFD + MOTOT_ID + CMD
		_size = 1;
	}
	else
	{
		param_data[0] = (_duty << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_DUTY_CYCLE) << 7 );	// Payload and Frame Checker
		param_data[1] = FPGA_SFD | FPGA_CMD_MOTOR_ID_0 | FPGA_CMD_DUTY_CYCLE;	// SFD + MOTOT_ID + CMD

		param_data[2] = (_duty << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_DUTY_CYCLE) << 7 );	// Payload and Frame Checker
		param_data[3] = FPGA_SFD | FPGA_CMD_MOTOR_ID_1 | FPGA_CMD_DUTY_CYCLE;	// SFD + MOTOT_ID + CMD


		_size = 2;
	}


	// Send SPI Packets
	do
	{
		spi_log = spi_request(SPI_RQT_FPGA, param_data, _size);
	}while(spi_log != SPI_DRIVER_DONE);

}

void motor_set_dir(MOTOR_ID _motor_id, uint8_t _dir)
{
	SPI_DRIVER_ERROR spi_log = SPI_DRIVER_OK;

	uint8_t param_data[4];
	uint8_t _size = 0;


	if(_motor_id == MOTOR0)
	{
		// DIR M0 = 0
		param_data[1] = FPGA_SFD | FPGA_CMD_MOTOR_ID_0 | FPGA_CMD_ROT_DIRECTION;	// SFD + MOTOT_ID + CMD
		param_data[0] = (_dir << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_ROT_DIRECTION) << 7 );	// Payload and Frame Checker
		_size = 1;
	}
	else if(_motor_id == MOTOR1)
	{
		param_data[1] = FPGA_SFD | FPGA_CMD_MOTOR_ID_1 | FPGA_CMD_ROT_DIRECTION;	// SFD + MOTOT_ID + CMD
		param_data[0] = (_dir << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_ROT_DIRECTION) << 7 );	// Payload and Frame Checker
		_size = 1;
	}
	else
	{
		param_data[1] = FPGA_SFD | FPGA_CMD_MOTOR_ID_0 | FPGA_CMD_ROT_DIRECTION;	// SFD + MOTOT_ID + CMD
		param_data[0] = (_dir << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_ROT_DIRECTION) << 7 );	// Payload and Frame Checker

		_dir = (_dir == 1 ? 0 : 1);
		param_data[3] = FPGA_SFD | FPGA_CMD_MOTOR_ID_1 | FPGA_CMD_ROT_DIRECTION;	// SFD + MOTOT_ID + CMD
		param_data[2] = ( _dir << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_ROT_DIRECTION) << 7 );	// Payload and Frame Checker
		_size = 2;
	}


	// Send SPI Packets
	do
	{
		spi_log = spi_request(SPI_RQT_FPGA, param_data, _size);
	}while(spi_log != SPI_DRIVER_DONE);

}

void motor_start_stop(MOTOR_ID _motor_id, uint8_t _start)
{
	SPI_DRIVER_ERROR spi_log = SPI_DRIVER_OK;

	uint8_t param_data[4];
	uint8_t _size = 0;


	if(_motor_id == MOTOR0)
	{
		// DIR M0 = 0
		param_data[1] = FPGA_SFD | FPGA_CMD_MOTOR_ID_0 | FPGA_CMD_START_STOP;	// SFD + MOTOT_ID + CMD
		param_data[0] = (_start << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_START_STOP) << 7 );	// Payload and Frame Checker
		_size = 1;
	}
	else if(_motor_id == MOTOR1)
	{
		param_data[1] = FPGA_SFD | FPGA_CMD_MOTOR_ID_1 | FPGA_CMD_START_STOP;	// SFD + MOTOT_ID + CMD
		param_data[0] = (_start << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_START_STOP) << 7 );	// Payload and Frame Checker
		_size = 1;
	}
	else
	{
		param_data[1] = FPGA_SFD | FPGA_CMD_MOTOR_ID_0 | FPGA_CMD_START_STOP;	// SFD + MOTOT_ID + CMD
		param_data[0] = (_start << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_START_STOP) << 7 );	// Payload and Frame Checker

		param_data[3] = FPGA_SFD | FPGA_CMD_MOTOR_ID_1 | FPGA_CMD_ROT_DIRECTION;	// SFD + MOTOT_ID + CMD
		param_data[2] = (_start << 1 ) | FPGA_frame_checker(  ((0xFFFF) & FPGA_CMD_START_STOP) << 7 );	// Payload and Frame Checker
		_size = 2;
	}


	// Send SPI Packets
	do
	{
		spi_log = spi_request(SPI_RQT_FPGA, param_data, _size);
	}while(spi_log != SPI_DRIVER_DONE);

}
/*
 * Generate the Debug LEDs logics regarding global FSM state (COMMANDS_PARSER_ERROR)
 */
void generate_mvt_hdl_flag(ROBOT6900_HANDLER* h_robot6900)
{
//	static COMMANDS_PARSER_ERROR previous_log = PARSER_OK;
//
//	// Do not change LEDs statues is parser's flags didn't change
//	if(CMD_Parser_Log != previous_log)
//	{
//		h_robot6900->robot_state->debug_leds = ((CMD_Parser_Log & PARSER_OK) == 1 ? 0x00 : DB_LED3);
//		h_robot6900->robot_state->debug_leds |= ((CMD_Parser_Log & PARSER_NO_CMD) == PARSER_NO_CMD ? DB_LED7 : 0x00);
//		h_robot6900->robot_state->debug_leds |= ((CMD_Parser_Log & PARSER_WRONG_ID) == PARSER_WRONG_ID || (CMD_Parser_Log & PARSER_WRONG_CRC) == PARSER_WRONG_CRC ? DB_LED8 : 0x00);
//		h_robot6900->robot_state->debug_leds |= ((CMD_Parser_Log & PARSER_PIPELINE_FULL) == PARSER_PIPELINE_FULL || (CMD_Parser_Log & PARSER_WRONG_CRC) == PARSER_WRONG_CRC ? DB_LED9 : 0x00);
//
//		// Update last parser's flag values
//		previous_log = CMD_Parser_Log;
//	}

}
