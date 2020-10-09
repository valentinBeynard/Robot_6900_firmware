#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

/* HAL Library */
#include "main.h"

#include "commands.h"

/* Number of implemented commands */
#define NUMBER_OF_COMMAND 2

// NVIC Interrupt buffer size in byte
#define NVIC_INT_BYTE_SIZE	CMD_PACKET_SIZE

// Maximum number of stored command in the waiting pipe
#define MAX_COMMAND_STACK_SIZE	5

typedef enum
{
  PARSER_OK = 0,
  PARSER_UNKNOWN,
  PARSER_PIPELINE_FULL,
  PARSER_WRONG_ID,
  PARSER_WRONG_CRC
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

void cmd_parser_process(ROBOT6900_HANDLER* h_robot6900);
COMMANDS_PARSER_STATE get_command(ROBOT6900_HANDLER* h_robot6900);
COMMANDS_PARSER_STATE wait(ROBOT6900_HANDLER* h_robot6900);
COMMANDS_PARSER_STATE command_integrity(uint8_t* _raw_packet, CMD_PACKET* _cmd);
COMMANDS_PARSER_STATE update_pipeline();

//// Taille en octect que l'on alloue au buffer qui récupère la commande envoyé
//#define COMMAND_BUFFER_SIZE 32
//
//// Taille en octect que l'on alloue au buffer qui envoie les informations au PC
//#define MSG_INFO_BUFFER_SIZE 32
//

//
//#define COMMAND_MAX_BYTES 12
//#define MAX_PARAM_NUMBER  5
//
//// Byte de stop à la fin de chaque commande, imposé par le CdC
//#define STOP_BYTE 0x0D
//
//// Byte séparateur entre les différents éléments d'une commande
//#define COMMAND_SEPARATOR 0x20
//
//// Byte séparateur entre un arguments complexe et sa valeur
//#define ARG_VALUE_SEPARATOR 0x3A
//
//// Byte retourné par le parser en cas de commande valide
//#define COMMAND_CONFIRM_BYTE	0x3E
//
//// Byte retourné par le parser en cas de commande invalide
//#define COMMAND_ERROR_BYTE	0x23
//
//typedef struct
//{
//  byte has_command;
//  OUT_M1 * commands;
//	IN_M1 * informations;
//}PARSER_RESULT;
//

//

//
//
//
///*

#endif
