#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

/* HAL Library */
#include "main.h"

#include "commands.h"

/* Number of implemented commands */
#define NUMBER_OF_COMMAND 8

// NVIC Interrupt buffer size in byte
#define NVIC_INT_BYTE_SIZE	CMD_PACKET_SIZE

// Maximum number of stored command in the waiting pipe
#define MAX_COMMAND_STACK_SIZE	5

// Tx output buffer size in byte
#define TX_BUFFER_SIZE	32

typedef enum
{
	PARSER_INIT = 0x00,
	PARSER_OK = 0x01,
	PARSER_UNKNOWN = 0x02,
	PARSER_PIPELINE_FULL = 0x04,
	PARSER_WRONG_ID = 0x08,
	PARSER_WRONG_CRC = 0x10,
	PARSER_NO_CMD = 0x20
}COMMANDS_PARSER_ERROR;

typedef enum
{
  WAIT,
  GET_COMMAND,
}COMMANDS_PARSER_STATE;

typedef struct
{
  COMMANDS_PARSER_STATE state_name;
  COMMANDS_PARSER_ERROR(*state_process)(ROBOT6900_HANDLER*);
}PARSER_FSM_PROCESS;

//#############################################################################
//        µP Related Code
//#############################################################################

void NVIC_command_parser_INT(UART_HandleTypeDef *huart);
void uart_init(UART_HandleTypeDef *huart, CRC_HandleTypeDef* _hcrc);

//#############################################################################
//        Core Code of commands_parser.c
//#############################################################################

COMMANDS_PARSER_ERROR cmd_parser_process(ROBOT6900_HANDLER* h_robot6900);
COMMANDS_PARSER_ERROR get_command(ROBOT6900_HANDLER* h_robot6900);
COMMANDS_PARSER_ERROR wait(ROBOT6900_HANDLER* h_robot6900);
COMMANDS_PARSER_ERROR command_integrity(uint8_t* _raw_packet, CMD_PACKET* _cmd);
COMMANDS_PARSER_ERROR update_pipeline();

void parser_return(ROBOT6900_HANDLER* h_robot6900);
uint8_t parser_OUTPUT_status(ROBOT6900_HANDLER* h_robot6900);
uint8_t parser_OUTPUT_RPlidar(ROBOT6900_HANDLER* h_robot6900);

void generate_parser_flag(ROBOT6900_HANDLER* h_robot6900);


#endif
