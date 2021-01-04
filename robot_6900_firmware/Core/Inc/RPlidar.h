/*
 * RPlidar.h
 *
 *  Created on: Oct 16, 2020
 *      Author: valbe
 */

#ifndef INC_RPLIDAR_H_
#define INC_RPLIDAR_H_

/* HAL Library */
#include "main.h"

#include "commands.h"

//#############################################################################
//        DEBUG
//#############################################################################

//#define RPLIDAR_DEBUG

//#############################################################################
//        DEBUG
//#############################################################################

/* Start of Frame byte of a request command */
#define REQUEST_SoF	0xA5

/* List of RP_lidar commands IDentifiers */
#define RP_STOP_CMD	0x25
#define RP_RESET_CMD	0x40
#define RP_SCAN_CMD	0x20	// <!> Not Implemented
#define RP_EXPRESS_SCAN_CMD	0x82
#define RP_FORCE_SCAN_CMD	0x21	// <!> Not Implemented
#define RP_GET_INFO_CMD	0x50	// <!> Not Implemented
#define RP_GET_HEALTH_CMD	0x52
#define RP_GET_SAMPLERATE	0x59	// <!> Not Implemented

/* RP_lidar request packet size for UART Tx */
#define HEALTH_PCK_SIZE	2
#define RESET_PCK_SIZE	2
#define START_SCAN_PCK_SIZE	9

/* RP_lidar Descriptor packet response size for UART Int. mode Rx */
#define RESPONSE_DESCRIPTOR_SIZE	7
/* RP_lidar Descriptor packet Start of Frame byte 1 */
#define RESPONSE_DESCRIPTOR_SoF1	0xA5
/* RP_lidar Descriptor packet Start of Frame byte 2 */
#define RESPONSE_DESCRIPTOR_SoF2	0x5A

/* RP_lidar Health packet response size for UART Int. mode Rx */
#define HEALTH_RESPONSE_SIZE	3
/* Health response protection value when error */
#define HEALTH_PROTECTION_ERR	0x02
/* Number of failure accepted during the Get Health process */
#define HEALTH_HARD_FAILURE_LIMIT	10

/* Size of an EXPRESS SCAN Response packet */
#define EXPRESS_SCAN_RESPONSE_SIZE	84
/* Number of Cabin in an EXPRESS SCAN Response packet */
#define EXPRESS_SCAN_CABIN_NBR	16
/* Byte position of the first cabin in the EXPRESS SCAN Response packet */
#define EXPRESS_SCAN_CABIN_OFFSET	4
/* Size in byte of a Cabin */
#define EXPRESS_SCAN_CABIN_SIZE	5

//#define EXPRESS_SCAN_S_MASK	0x80
/* MASK for Low byte of the angle of a point */
#define EXPRESS_SCAN_ANGLE_LSB_MASK	0x7F


/* Number of 'Rifle' buffers for the DMA Rx */
#define RX_NBR_BUFFER	3
/* Size of 1 'Rifle' buffer for acquiring a EXPRESS Scan packet response */
#define RX_BUFFER_SIZE	EXPRESS_SCAN_RESPONSE_SIZE

/* Number of sample data in a 360 Scan at 4KHz */
#define SAMPLES_PER_360	608

#define RPLIDAR_OUTPUT_100ms	2000

#ifdef RPLIDAR_DEBUG

#define SCAN_DEBUG_SIZE	29
typedef struct
{
	uint16_t scan_ID;
	uint16_t nbr_pt;
	float RPM_motor;
}RP_SCAN_DEBUG;

#endif

/* List of all RP_lidar Error Flags */
typedef enum
{
	RPLIDAR_INIT = 0x00,
	RPLIDAR_OK = 0x01,
	RPLIDAR_UNKNOWN = 0x02,
	RPLIDAR_PIPELINE_FULL = 0x04,
	RPLIDAR_WRONG_ID = 0x08,
	RPLIDAR_WRONG_CRC = 0x10,
	RPLIDAR_NO_CMD = 0x20
}RPLIDAR_ERROR;


/* RP_lidar Descriptor packet */
typedef struct __attribute__((__packed__))
{
	uint8_t SoF1;
	uint8_t SoF2;
	uint32_t response_length;
	uint8_t data_type;
}RPLIDAR_DESCRIPTOR_PCK;


/* RP_lidar Point Data */
typedef struct __attribute__((__packed__))
{
	uint16_t distance;
	float angle;
}RPLIDAR_DATA;


/* RP_lidar FSM states */
typedef enum
{
  RP_INIT = 0,
  RP_GET_HEALTH,
  RP_COM_ERR,
  RP_HARD_ERR,
  RP_RESET,
  RP_START_SCAN,
  RP_SAMPLING,
  RP_PROCESS,
  RP_STOP,
}RPLIDAR_STATE;


/* RP_lidar FSM state description */
typedef struct
{
	RPLIDAR_STATE state_name;
	RPLIDAR_ERROR(*state_process)(ROBOT6900_HANDLER*);
}RPLIDAR_FSM_PROCESS;

//#############################################################################
//        NVIC Interrupt routines
//#############################################################################

void NVIC_RPlidar_INT(UART_HandleTypeDef *huart);
void NVIC_Timout_1ms_INT(TIM_HandleTypeDef* htim);

//#############################################################################
//        Core Code of RPlidar.c
//#############################################################################

void RPlidar_init(ROBOT6900_HANDLER* h_robot6900, UART_HandleTypeDef *huart, TIM_HandleTypeDef* htim7);
RPLIDAR_ERROR RPlidar_process(ROBOT6900_HANDLER* h_robot6900);

//#############################################################################
//        FSM functions
//#############################################################################

RPLIDAR_ERROR init_state(ROBOT6900_HANDLER* h_robot6900);
RPLIDAR_ERROR health_state(ROBOT6900_HANDLER* h_robot6900);
RPLIDAR_ERROR reset_state(ROBOT6900_HANDLER* h_robot6900);
RPLIDAR_ERROR start_scan_state(ROBOT6900_HANDLER* h_robot6900);
RPLIDAR_ERROR sampling_state(ROBOT6900_HANDLER* h_robot6900);
RPLIDAR_ERROR processing_state(ROBOT6900_HANDLER* h_robot6900);
RPLIDAR_ERROR stop_state(ROBOT6900_HANDLER* h_robot6900);
RPLIDAR_ERROR COM_error_state(ROBOT6900_HANDLER* h_robot6900);
RPLIDAR_ERROR hardware_error_state(ROBOT6900_HANDLER* h_robot6900);

//#############################################################################
//        Utils functions
//#############################################################################

void RPlidar_start_timeout();
void RPlidar_abord_timeout();
uint8_t RPlidar_checksum(uint8_t* packet, uint8_t packet_size);
uint8_t RPlidar_get_descriptor_pck();
float RPlidar_measure_RPM();

#endif /* INC_RPLIDAR_H_ */
