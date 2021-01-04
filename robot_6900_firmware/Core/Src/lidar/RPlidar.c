/*
 * RPlidar.c
 *
 *  Created on: Oct 16, 2020
 *      Author: valbe
 */

#include "RPlidar.h"

/*
############################################################################
#
#	RP_Lidar Driver variables
#
############################################################################
*/

/* UART5 Handler */
UART_HandleTypeDef* _huart;

/* Timer 7 handler */
TIM_HandleTypeDef* _htim7;

/* LiDAR Rx_Buffer */
uint8_t RP_Rx_buffer[RX_NBR_BUFFER][RX_BUFFER_SIZE];

/* LiDAR data after calculation for a 360 degrees scan */
RPLIDAR_DATA RPlidar_data[SAMPLES_PER_360];

/* Index of the current buffer used by the DMA channel */
uint8_t current_DMA_buffer = 0;

/* Buffer state Flag : 1 if data ready to be process, 0 if already processed/not processed */
volatile uint8_t buffer_ready = 0;

/* DMA Enable Flag - Set to 1 for UART in DMA mode - Set to 0 for INTERRUPT Mode */
uint8_t DMA_Rx = 0;

/* Size of the next UART Rx packet when receiving in INT. mode */
uint8_t NVIC_Rx_next_size = 0;

/* Next packet size for DMA */
uint16_t DMA_next_packet_size = 0;

/* Current state of the RP_lidar FSM */
RPLIDAR_STATE RPlidar_current_state = RP_INIT;

/* RP_lidar Flag error */
RPLIDAR_ERROR RPlidar_Log = RPLIDAR_INIT;

/* Timout Flag. Set by timer interrupt. 1 if RPlidar Timerout, otherwise 0 */
volatile uint8_t RPlidar_timeout = 0;

/* Global counter of hardware issue */
uint8_t RPlidar_hard_failure_cnt = 0;

/* Start Motor RPM calculation Flag */
uint8_t RPM_init = 0;

/* Timer interrupt counter for RPM calculation */
volatile uint16_t RPM_cnt_1ms = 0;

/* RP_lidar motor RPM */
float RPlidar_RPM = 0.0f;


//uint8_t RPlidar_OUTPUT_cnt = 0;
uint16_t RPlidar_OUTPUT_cnt = 0;

/* RP_lidar Descriptor packet */
RPLIDAR_DESCRIPTOR_PCK RPlidar_descriptor_pck =
		{
				0,
				0,
				0,
				0,
				0,
				0,
		};

#ifdef RPLIDAR_DEBUG

// DMA Receive counter
static uint8_t DMA_cnt = 0;

// 360 degrees Scan information : Nbr of point + scan number + Motor RPM
RP_SCAN_DEBUG scan_DEBUG[SCAN_DEBUG_SIZE];

#endif


/*
    FULL STATE MACHINE

    Liste des Etats de la machines d'état du Parser associés à leur fonction
*/
const RPLIDAR_FSM_PROCESS RPlidar_FSM[9] = {
    {RP_INIT, &init_state},
	{RP_GET_HEALTH, &health_state},
	{RP_COM_ERR, &COM_error_state},
	{RP_HARD_ERR, &hardware_error_state},
	{RP_RESET, &reset_state},
	{RP_START_SCAN, &start_scan_state},
	{RP_SAMPLING, &sampling_state},
	{RP_PROCESS, &processing_state},
    {RP_STOP, &stop_state}
};

/*
############################################################################
#
#	DEFINE Calculation Macro
#
############################################################################
*/

/* Rifle buffer Handling */
#define RX_BUFFER_LEVEL_N(n, i)	RP_Rx_buffer[(current_DMA_buffer + RX_NBR_BUFFER + n) % RX_NBR_BUFFER][i]
#define AVAILABLE_RX_BUFFER(index)	RX_BUFFER_LEVEL_N(-1, index)
#define OLD_RX_BUFFER(index)	RX_BUFFER_LEVEL_N(-2, index)

/* Cabin Access  */
#define BF_CABIN(cabin_index, data_i)	OLD_RX_BUFFER(EXPRESS_SCAN_CABIN_OFFSET + cabin_index * EXPRESS_SCAN_CABIN_SIZE + data_i)

/* EXPRESS Scan Distance calculation */
#define F_CABIN_DISTANCE1(cabin_index)	( (BF_CABIN(cabin_index, 0) & 0xFC) + ((uint16_t)BF_CABIN(cabin_index, 1) << 6) )
#define F_CABIN_DISTANCE2(cabin_index)	( (BF_CABIN(cabin_index, 2) & 0xFC) + ((uint16_t)BF_CABIN(cabin_index, 3) << 6) )

/* EXPRESS Scan Angle calculation */
#define F_CABIN_DANGLE1(cabin_index)	( (BF_CABIN(cabin_index, 4) & 0x0F) + ((uint8_t)( (BF_CABIN(cabin_index, 0) & 0x03) << 4 ) ) )
#define F_CABIN_DANGLE2(cabin_index)	( (BF_CABIN(cabin_index, 4) & 0xF0) + ((uint8_t)( (BF_CABIN(cabin_index, 2) & 0x03) << 4 ) ) )

/* EXPRESS Scan Angle calculation */
#define DIFF_ANGLE(a, b)	(a <= b ? (b-a) : (360+b-a))

/*
############################################################################
#
#	NVIC Interrupt routines
#
############################################################################
*/

/*
 * UART Interruption routine.
 *
 * Update the identifier of the next buffer to be filled,
 * and set buffer ready to be process flag to 1.
 *
 * NVIC routine is either reloaded in Simple INT mode or DMA mode, regarding
 * the DMA_Rx flag
 */
void NVIC_RPlidar_INT(UART_HandleTypeDef *huart)
{
	/* Switch DMA buffer */
	current_DMA_buffer = (current_DMA_buffer + 1) % RX_NBR_BUFFER;

	/* Set buffer ready to be process */
	buffer_ready = 1;


	if(DMA_Rx)
	{
		/* Reload DMA process with new buffer */
		HAL_UART_Receive_DMA(_huart, RP_Rx_buffer[current_DMA_buffer], DMA_next_packet_size);
#ifdef RPLIDAR_DEBUG
		DMA_cnt++;
#endif
	}
	else
	{
		HAL_UART_Receive_IT(_huart, RP_Rx_buffer[current_DMA_buffer], NVIC_Rx_next_size);
	}

}

/*
 * RP_lidar Timer interrupt for Timeout communication
 * and motor RPM calculation if enabled.
 *
 */
void NVIC_Timout_1ms_INT(TIM_HandleTypeDef* htim)
{
	if(RPM_init == 1)
	{
		RPM_cnt_1ms += 1;
		RPlidar_OUTPUT_cnt += 1;
	}
	else
	{
		RPlidar_timeout += 1;
	}
}

/*
############################################################################
#
#	Functional behaviors
#
############################################################################
*/

/*
 * Initialize RP_lidar peripherals.
 *
 * Get UART and TIMER pointer to their handler.
 * Disable motor spinning by driving RP_Lidar CTL Pin to LOW
 *
 * Set RP_lidar FSM to initiate state.
 *
 */
void RPlidar_init(ROBOT6900_HANDLER* h_robot6900, UART_HandleTypeDef *huart, TIM_HandleTypeDef* htim7)
{
	// Initiate ptr
	_huart = huart;
	_htim7 = htim7;

	h_robot6900->RPlidar->data = (uint16_t*)(&RPlidar_data);

	// Disable Lidar motor
	HAL_GPIO_WritePin(GPIOA, RPLIDAR_EN_Pin, GPIO_PIN_RESET);

	RPlidar_current_state = RP_INIT;
}

/*
 * Main RP_lidar command. Only external entrance to the RP_Lidar.
 *
 * Handle the RP_Lidar Full State Machine.
 *
 * Initialise the RP_lidar by checking COMmunication state and HARDware state
 * by sending a HEALTH request.
 *
 * Then, start an EXPRESS SCAN request to begin the acquisition of data.
 *
 * The behavior of the FSM is then a constant exchange between a SAMPLING state,
 * waiting for a EXPRESS SCAN response packet from the DMA, and the PROCESS state,
 * which is calculating compressed data and filling the RP_lidar data buffer.
 *
 * Return the global FSM state (COMMANDS_PARSER_ERROR)
 *
 */
RPLIDAR_ERROR RPlidar_process(ROBOT6900_HANDLER* h_robot6900)
{
#ifndef DEBUG
	HAL_IWDG_Refresh(h_robot6900->_hiwdg);
#endif

	// Full State Machine Call
	RPlidar_Log = (RPlidar_FSM[RPlidar_current_state]).state_process(h_robot6900);

	if(RPlidar_OUTPUT_cnt >= RPLIDAR_OUTPUT_100ms)
	{
		h_robot6900->RPlidar->RPlidar_update = 1;
		RPlidar_OUTPUT_cnt = 0;
	}

	return RPlidar_Log;
}

/*
 * RP_INIT State.
 *
 * Free all rifle buffers and set to 0 all RP_Lidar Flags, Enable motor spinning
 * by driving RP_Lidar CTL Pin to HIGH (max speed) and switch to HEALTH state
 *
 * Return RPLIDAR_ERROR
 *
 */
RPLIDAR_ERROR init_state(ROBOT6900_HANDLER* h_robot6900)
{

	// Clear buffers
	for(uint8_t i = 0 ; i < RX_NBR_BUFFER; i++)
	{
		memset(RP_Rx_buffer[i], 0, RX_BUFFER_SIZE);
	}

	buffer_ready = 0;
	current_DMA_buffer = 0;
	RPlidar_timeout = 0;
	RPlidar_hard_failure_cnt = 0;

	// Enable Motor spinning
	HAL_GPIO_WritePin(GPIOA, RPLIDAR_EN_Pin, GPIO_PIN_SET);

	RPlidar_current_state = RP_GET_HEALTH;

	return RPlidar_Log;
}

/*
 * RP_GET_HEALTH State.
 *
 * Build and send Health packet in a blocking way. Wait for response in INT UART mode.
 * Check on Descriptor packet and health packet responses to detect COM or HARDware error
 *
 * If no error, jump to RP_START_SCAN, otherwise, jump to RP_COM_ERR or RP_HARD_ERR
 *
 * Return RPLIDAR_ERROR
 *
 */
RPLIDAR_ERROR health_state(ROBOT6900_HANDLER* h_robot6900)
{
	static uint8_t health_process_step = 0;
	static uint8_t descriptor_received = 0;

	RPLIDAR_ERROR log = RPLIDAR_OK;

	// health packet
	uint8_t health_pck[HEALTH_PCK_SIZE];
	HAL_StatusTypeDef uart_log = HAL_ERROR;

	// RPlidar status and error carried by the HEALTH descriptor
	uint8_t status = 0;
	uint16_t error_code = 0;

	/* First step of the state, just build the packet and send it */
	if(health_process_step == 0)
	{
		// Build Health packet
		health_pck[0] = REQUEST_SoF;
		health_pck[1] = RP_GET_HEALTH_CMD;



		// Send GET_HEALTH pck
		while(uart_log != HAL_OK)
		{
			uart_log = HAL_UART_Transmit(_huart, health_pck, HEALTH_PCK_SIZE, 0xFFFF);
		}

		// Initiate Rx INT to get Descriptor Pck in NVIC INT mode
		HAL_UART_Receive_IT(_huart, RP_Rx_buffer[current_DMA_buffer], RESPONSE_DESCRIPTOR_SIZE);

		// Prepare NEXT NVIC interurpt for response packet this time
		NVIC_Rx_next_size = HEALTH_RESPONSE_SIZE;

		// Start Timeout to test COM error
		RPlidar_start_timeout();

		health_process_step = 1;
	}

	/* Second step of the state, wait for lidar response and check for Timeout or Protection flag */
	if(health_process_step == 1)
	{
		// Read Descriptor response and Packet Response
		if(buffer_ready == 1)
		{
			// Read Packet Descriptor
			if(descriptor_received == 0)
			{
				// Read, check and save descriptor pck
				descriptor_received = RPlidar_get_descriptor_pck();
				// Prepare NEXT NVIC interurpt for response packet this time
				NVIC_Rx_next_size = RESPONSE_DESCRIPTOR_SIZE;
			}
			else
			{
				// Read 4th byte of the Health response descriptor, containing status description
				status = AVAILABLE_RX_BUFFER(0);
				// Get error_code in case
				error_code = ((uint16_t)(AVAILABLE_RX_BUFFER(1)) << 8) + AVAILABLE_RX_BUFFER(2);

				buffer_ready = 0;

				// Check Protection error
				if(status == HEALTH_PROTECTION_ERR)
				{
					RPlidar_hard_failure_cnt++;
					if(RPlidar_hard_failure_cnt < HEALTH_HARD_FAILURE_LIMIT)
					{
						RPlidar_current_state = RP_RESET;	/* ------------> */
						health_process_step = 0;
						descriptor_received = 0;
					}
					else
					{
						RPlidar_current_state = RP_HARD_ERR;	/* ------------> */
						health_process_step = 0;
						descriptor_received = 0;
					}

					// Update RPlidar handler
					h_robot6900->RPlidar->state = status;
					h_robot6900->RPlidar->error_code = error_code;
				}
				// Evrything's OK, then jump to next state
				else
				{
					// Update RPlidar handler
					h_robot6900->RPlidar->state = status;
					h_robot6900->RPlidar->error_code = error_code;

					// Jump to next state
					RPlidar_current_state = RP_START_SCAN;	/* ------------> */
					health_process_step = 0;
					descriptor_received = 0;
				}
			}
		}
		// Wait for Response Descriptor paquet
		else
		{
			// Check if communication with the lidar timed out
			if(RPlidar_timeout >= 2)
			{
				RPlidar_timeout = 0;
				RPlidar_abord_timeout();
				RPlidar_current_state = RP_COM_ERR;	/* ------------> */
			}
		}
	}

	// If we jump to another step, reinitialize the inside step value to 0
	if(RPlidar_current_state != RP_GET_HEALTH)
	{
		health_process_step = 0;
	}

	return log;
}

/*
 * RP_RESET State.
 *
 * Send a REQUEST packet to the RP_lidar and wait 2 ms before jump to HEALTH State
 *
 * Return RPLIDAR_ERROR
 *
 */
RPLIDAR_ERROR reset_state(ROBOT6900_HANDLER* h_robot6900)
{
	static uint8_t reset_process_step = 0;
	static uint8_t wait_cnt = 0;

	RPLIDAR_ERROR log = RPLIDAR_OK;

	uint8_t reset_pck[HEALTH_PCK_SIZE];
	HAL_StatusTypeDef uart_log = HAL_ERROR;

	if(reset_process_step == 0)
	{
		// Build Reset packet
		reset_pck[0] = REQUEST_SoF;
		reset_pck[1] = RP_RESET_CMD;

		// Send RESET_PCK pck
		while(uart_log != HAL_OK)
		{
			uart_log = HAL_UART_Transmit(_huart, reset_pck, RESET_PCK_SIZE, 0xFFFF);
		}

		// Start Timeout to test COM error
		RPlidar_start_timeout();

		reset_process_step = 1;
	}
	else
	{
		// Wait for the 2ms WAIT before the lidar wakes up
		if(RPlidar_timeout)
		{
			wait_cnt++;
			RPlidar_timeout = 0;
			if(wait_cnt >= 2)	// TIMER_INT each 1ms, so wait_cnt > 2 to get 2ms
			{
				RPlidar_current_state = RP_GET_HEALTH;	/* ------------> */
				reset_process_step = 0;
				wait_cnt = 0;
			}
		}
	}

	return log;

}

/*
 * RP_START_SCAN State.
 *
 * Send an EXPRESS SCAN packet to the RP_lidar, update Descriptor packet and jump
 * to RP_SAMPLING State
 *
 * Return RPLIDAR_ERROR
 *
 */
RPLIDAR_ERROR start_scan_state(ROBOT6900_HANDLER* h_robot6900)
{
	static uint8_t reset_process_step = 0;

	RPLIDAR_ERROR log = RPLIDAR_OK;

	uint8_t scan_pck[START_SCAN_PCK_SIZE];
	HAL_StatusTypeDef uart_log = HAL_ERROR;

	/* PCK INITIALISATION AND TX */
	if(reset_process_step == 0)
	{
		// Build EXPRESS Scan packet
		scan_pck[0] = REQUEST_SoF;
		scan_pck[1] = RP_EXPRESS_SCAN_CMD;

		scan_pck[2] = 5; // Payload Size

		scan_pck[3] = 0; // Working Mode. Set to 0. See datasheet
		scan_pck[4] = 0;	// Reserved
		scan_pck[5] = 0;	// Reserved
		scan_pck[6] = 0;	// Reserved
		scan_pck[7] = 0;	// Reserved

		scan_pck[8] = RPlidar_checksum(scan_pck, START_SCAN_PCK_SIZE - 1); // Checksum

		// Prepare Rx to switch to DMA mode for constant DATA acquisition
		DMA_Rx = 1;
		DMA_next_packet_size = EXPRESS_SCAN_RESPONSE_SIZE;

		// Initiate Rx INT to get Descriptor Pck in NVIC INT mode
		HAL_UART_Receive_IT(_huart, RP_Rx_buffer[current_DMA_buffer], RESPONSE_DESCRIPTOR_SIZE);

		// Send RESET_PCK pck
		while(uart_log != HAL_OK)
		{
			uart_log = HAL_UART_Transmit(_huart, scan_pck, START_SCAN_PCK_SIZE, 0xFFFF);
		}


		// Start Timeout to test COM error
		RPlidar_start_timeout();

		reset_process_step = 1;
	}
	/* RECEIVE DESCRIPTOR, CHECK VALIDITY AND TIMEOUT */
	else
	{
		// Check if communication with the lidar timed out
		if(buffer_ready == 1)
		{
			// Read, check and save descriptor pck. Return 1 if OK
			if(RPlidar_get_descriptor_pck() == 1)
			{
				RPlidar_current_state = RP_SAMPLING;	/* ------------> */
				reset_process_step = 0;
			}
		}
		// Wait for Response Descriptor paquet
		else
		{
			// Check if communication with the lidar timed out
			if(RPlidar_timeout >= 20)
			{
				RPlidar_timeout = 0;
				RPlidar_abord_timeout();
				RPlidar_current_state = RP_COM_ERR;	/* ------------> */
				reset_process_step = 0;
			}
		}

	}

	return log;
}

/*
 * <!> Contain an bug if the RP_lidar FSM is reset during uC still working. When RP_SAMPLING will be
 * 'first' entered, only one buffer will be filled instead of 2
 * <!>
 *
 * RP_SAMPLING State.
 *
 * When the FSM first enter in this state, the RPLidar will stay in the RP_SAMPLING until
 * 2 Rifle Buffer are filled, in order to have 2 data packet for the process calculation.
 *
 * Then, when RP_SAMPLING is re-entered, the state will only wait for 1 Rifle buffer to be
 * filled.
 *
 * Return RPLIDAR_ERROR
 *
 */
RPLIDAR_ERROR sampling_state(ROBOT6900_HANDLER* h_robot6900)
{
	static uint8_t init_process_buffer = 0;

	RPLIDAR_ERROR log = RPLIDAR_OK;

	uint8_t timeout_cnt = 0;

	/* WAIT FOR EXPRESS SCAN RESPONSE PCK */
	if(buffer_ready == 1)
	{
		// Wait for the 1st buffer to be filled
		if(init_process_buffer == 0)
		{
			init_process_buffer = 1;
			buffer_ready = 0;
		}
		// Wait for the 2nd buffer to be filled
		else
		{
			buffer_ready = 0;
			RPlidar_current_state = RP_PROCESS;	/* ------------> */
		}

	}
	else
	{
		// Timeout Check (2ms)
		if(RPlidar_timeout)
		{
			RPlidar_timeout = 0;
			if(timeout_cnt >= 20)
			{
				RPlidar_current_state = RP_COM_ERR;	/* ------------> */
				init_process_buffer = 0;
			}
			else
			{
				timeout_cnt++;
			}
		}
	}

	return log;
}

/*
 * RP_PROCESS State.
 *
 * Process 2 packets data in order to get N de-compressed points (distance and angle).
 * Each data packet contains 16 cabins, containing informations about 2 points.
 * Then, each RP_PROCESS state will produce 32 de-compressed points.
 *
 * Check for data validation before calculation (Checksum) and generate JUMP in COM error
 * if false.
 *
 * Return RPLIDAR_ERROR
 *
 */
RPLIDAR_ERROR processing_state(ROBOT6900_HANDLER* h_robot6900)
{
	static uint16_t current_data_index = 0;

#ifdef RPLIDAR_DEBUG
	static uint16_t nbr_scan = 0;
	static uint8_t process_cnt = 0;
	static uint16_t max_ptnumber = 0;
#endif

	uint8_t pck_checksum = 0;
	uint8_t real_checksum = 0;

	uint8_t S_parameter = 0;

	float start_angle = 0;
	float next_start_angle = 0;

	uint16_t pt_index = 0;

	RPLIDAR_ERROR log = RPLIDAR_OK;

	/* TODO CHECKSUM TEST */
	real_checksum = RPlidar_checksum(&AVAILABLE_RX_BUFFER(2), EXPRESS_SCAN_RESPONSE_SIZE-2);
	pck_checksum = ( (AVAILABLE_RX_BUFFER(0) & 0x0F) + ((AVAILABLE_RX_BUFFER(1) & 0x0F) << 4) );

	if(real_checksum != pck_checksum)
	{
		RPlidar_current_state = RP_COM_ERR;
	}

	/* Get the and REBUILD the Angle of the current Pi packet */
	start_angle = (OLD_RX_BUFFER(2) +
			( (uint16_t)( (OLD_RX_BUFFER(3) & EXPRESS_SCAN_ANGLE_LSB_MASK) ) << 8) );
	start_angle = start_angle / 64.0;

	/* Get the and REBUILD the Angle of the next Pi+1 packet */
	next_start_angle = (AVAILABLE_RX_BUFFER(2) +
			( (uint16_t)( (AVAILABLE_RX_BUFFER(3) & EXPRESS_SCAN_ANGLE_LSB_MASK) ) << 8) );
	next_start_angle = next_start_angle / 64.0;


	/* Calculate the S parameter, in order to detect a new 360 Scan */
	S_parameter = (start_angle > next_start_angle ? 1 : 0);

#ifdef RPLIDAR_DEBUG
	process_cnt++;
#endif
	// If new scan detected, refresh data by starting at RPlidar_data[0]
	if(S_parameter == 1)
	{
		RPlidar_RPM = RPlidar_measure_RPM();
		// Times 2 since h_robot6900->RPlidar->data is a 16bits ptr instead of 32bits ptr
		h_robot6900->RPlidar->data_size = 2 * current_data_index;

#ifdef RPLIDAR_DEBUG
		scan_DEBUG[nbr_scan].scan_ID = nbr_scan;
		scan_DEBUG[nbr_scan].nbr_pt = current_data_index;
		scan_DEBUG[nbr_scan].RPM_motor = RPlidar_RPM;

		if(nbr_scan == 500)
		{
			nbr_scan = 0;
		}
		else
		{
			nbr_scan++;
		}
#endif

		current_data_index = 0;
	}



	/* CALCULATE DATA POINTS using Pi (in LiDAR_process_buffer) and Pi+1 */
	/* Processing de-compressing calculation */
	for(uint8_t cabin_i = 0 ; cabin_i < EXPRESS_SCAN_CABIN_NBR; cabin_i++)
	{
		// Point index in the result RPLIDAR_DATA array
		pt_index = (cabin_i * 2);

		// Security check on index
		if(current_data_index + pt_index + 1 >= SAMPLES_PER_360)
		{
			RPlidar_current_state = RP_COM_ERR;
			return log;
		}

		// Process point 1 in Cabin Ci
		RPlidar_data[current_data_index + pt_index].distance = F_CABIN_DISTANCE1(cabin_i);
		RPlidar_data[current_data_index + pt_index].angle = start_angle + (DIFF_ANGLE(start_angle, next_start_angle) / 32.0) * pt_index - (float)(F_CABIN_DANGLE1(cabin_i));

		// Process point 2 in Cabin Ci
		RPlidar_data[current_data_index + pt_index + 1].distance = F_CABIN_DISTANCE2(cabin_i);
		RPlidar_data[current_data_index + pt_index + 1].angle = start_angle + (DIFF_ANGLE(start_angle, next_start_angle) / 32.0) * (pt_index+1) - (float)(F_CABIN_DANGLE2(cabin_i));


	}

	// Save last point index to be added in the array
	current_data_index += pt_index + 1;

#ifdef RPLIDAR_DEBUG
	if(current_data_index > max_ptnumber)
	{
		max_ptnumber = current_data_index;
	}
#endif

	RPlidar_current_state = RP_SAMPLING;	/* ------------> */

	return log;
}

/*
 * <!> TO BE DONE
 *
 * RP_STOP State.
 *
 * Stop the Scan process of the RP_lidar, and stay in this state until the FSM is reloaded
 *
 *
 * Return RPLIDAR_ERROR
 *
 */
RPLIDAR_ERROR stop_state(ROBOT6900_HANDLER* h_robot6900)
{

}

/*
 * RP_COM_ERR State.
 *
 * Notify a COMmunication error. Set the h_robot6900->robot_state->debug_leds to the
 * RP_lidar_COM_Error configuration.
 *
 * Stay in this state until the FSM is reloaded
 *
 * Return RPLIDAR_ERROR
 *
 */
RPLIDAR_ERROR COM_error_state(ROBOT6900_HANDLER* h_robot6900)
{
	RPLIDAR_ERROR log = RPLIDAR_OK;

	h_robot6900->robot_state->debug_leds |= 0b00100001;

	return log;
}

/*
 * RP_HARD_ERR State.
 *
 * Notify an HARDware error. Set the h_robot6900->robot_state->debug_leds to the
 * RP_lidar_HARD_Error configuration.
 *
 * Stay in this state until the FSM is reloaded
 *
 * Return RPLIDAR_ERROR
 *
 */
RPLIDAR_ERROR hardware_error_state(ROBOT6900_HANDLER* h_robot6900)
{
	RPLIDAR_ERROR log = RPLIDAR_OK;

	h_robot6900->robot_state->debug_leds |= 0b00100010;

	return log;
}

/*
 * Start the Timeout Timer counter
 *
 * Set the counter to 0 to ensure the first Int. fired is still 1ms
 */
void RPlidar_start_timeout()
{
	_htim7->Instance->CNT = 0;
	HAL_TIM_Base_Start_IT(_htim7);
}

/*
 * Stop the Timeout Timer counter INT.
 */
void RPlidar_abord_timeout()
{
	HAL_TIM_Base_Stop_IT(_htim7);
}

/*
 * Calculate RP_lidar motors speed in Rotation Per Minute,
 * using time taken to acquire a 360 degree scan
 */
float RPlidar_measure_RPM()
{
	float rpm = 0;

	// If RPM measurement was not enabled before
	if(RPM_init == 0)
	{
		RPM_init = 1;
		RPM_cnt_1ms = 0;

		// Init and start 1ms Timer Interrupt
		RPlidar_abord_timeout();
		_htim7->Instance->CNT = 0;
		HAL_TIM_Base_Start_IT(_htim7);

	}
	else
	{
		rpm = ( (60.0 * 1000.0) / (1.0 * RPM_cnt_1ms));
		RPM_cnt_1ms = 0;
//		_htim7->Instance->CNT = 0;
	}

	return rpm;
}

/*
 * Calculate Checksum of a packet of data of length 'packet_size'
 *
 * Bit XOR between all packet bytes
 */
uint8_t RPlidar_checksum(uint8_t* packet, uint8_t packet_size)
{
	uint8_t checksum = 0;

	for(uint8_t i = 0 ; i < packet_size; i++)
	{
		checksum ^= *(packet + i);
	}

	return checksum;
}

/*
 *	Check Descriptor packet by looking for SoF1 and SoF2
 *
 *	Update the global RP_lidar descriptor packet
 */
uint8_t RPlidar_get_descriptor_pck()
{
	uint8_t* _descriptor_pck;
	uint8_t validity = 0;

	// Read Rx buffer
	_descriptor_pck = &(AVAILABLE_RX_BUFFER(0));
	buffer_ready = 0;

	// Check is data are the Descriptor Register by matching the SoF1 and SoF2
	if( *(_descriptor_pck) == RESPONSE_DESCRIPTOR_SoF1 && *(_descriptor_pck + 1) == RESPONSE_DESCRIPTOR_SoF2)
	{
		validity = 1;
	}
	else
	{
		validity = 0;
	}

	// Save descriptor in memory
	RPlidar_descriptor_pck = *(RPLIDAR_DESCRIPTOR_PCK*)(_descriptor_pck);

//	memset(&AVAILABLE_RX_BUFFER(0), 0, RESPONSE_DESCRIPTOR_SIZE);


	return validity;
}


