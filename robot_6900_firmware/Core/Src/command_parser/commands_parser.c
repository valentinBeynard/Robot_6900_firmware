#include "commands_parser.h"
#include <string.h>
#include <stdio.h>

/*
    DISPATCH TABLE

    Tableau regroupant commande et fonction associée
    On utilise un tablleau de structure CMD_ afin de référencer toutes les
    commandes implémentées et les lier à une fonctionnalitée.
*/
const CMD_ dispatch_table [NUMBER_OF_COMMAND] = {

	{0xA1, default_process},
    {0xA2, default_process}

};

/*
    FULL STATE MACHINE

    Liste des Etats de la machines d'état du Parser associés à leur fonction
*/
const PARSER_FSM_PROCESS full_state_machine[2] = {
    {WAIT, &wait},
    {GET_COMMAND, &get_command}
};


/*
############################################################################
#
#	High Purpose Variables
#
############################################################################
*/

UART_HandleTypeDef* _huart;


// UART Rx_buffer, filled by NVIC interrupt
uint8_t Rx_buffer[NVIC_INT_BYTE_SIZE] = {0, 0, 0, 0};

/* Buffer pipe to stock commands before handling them.
 * Add + 1 to keep an empty 32bits command space to easily
 * shift data when updating the pipeline
*/
uint8_t* CMDs_buffer[MAX_COMMAND_STACK_SIZE + 1];

// Current size of the waiting command pipeline
uint8_t CMDs_buffer_size = 0;

/* Full pipeline flag */
uint8_t CMDs_buffer_full = 0;

// Flag error related to command parser behavior
COMMANDS_PARSER_ERROR CMD_Parser_Log = PARSER_OK;

/* Current state of the command parser state machine */
COMMANDS_PARSER_STATE current_state = WAIT;

/* Last packet IDentifier for communication reliability */
uint8_t last_ID = 0;

/* Flag counter of all bad packet received */
uint8_t faulty_pck = 0;

/* Pointer to the global crc engine defined in main.c */
CRC_HandleTypeDef* _hcrc;


///* Buffer principal dans lesquel est stocké tout caractère reçu sur la liaison UART du µP */
//byte raw_data[COMMAND_BUFFER_SIZE];
//
///* Pointer permettant de parcourir le Buffer Principal */
//byte buffer_index = 0;
//
//byte MSG_buffer[MSG_INFO_BUFFER_SIZE] = "Start Epreuve !\n";

/*
############################################################################
#
#	NVIC Interrupt routines
#
############################################################################
*/

void NVIC_command_parser_INT(UART_HandleTypeDef *huart)
{

	if(CMDs_buffer_size < MAX_COMMAND_STACK_SIZE)
	{
		CMDs_buffer[CMDs_buffer_size] = Rx_buffer;
		CMDs_buffer_size++;
		CMDs_buffer_full = 0;
		// Reactivate receive on Interrupt on 1 byte
		HAL_UART_Receive_IT(huart, &Rx_buffer, NVIC_INT_BYTE_SIZE);
	}
	else
	{
		CMDs_buffer_full = 1;
	}
}

/*
############################################################################
#
#	Functionnal behaviours
#
############################################################################
*/

void uart_init(UART_HandleTypeDef *huart, CRC_HandleTypeDef* hcrc)
{
	// Initiate ptr
	_hcrc = hcrc;
	_huart = huart;

	// Activate UART Receive Interrupt each 4 bytes received
	HAL_UART_Receive_IT(_huart, &Rx_buffer, NVIC_INT_BYTE_SIZE);


}

void cmd_parser_process(ROBOT6900_HANDLER* h_robot6900)
{
	HAL_IWDG_Refresh(h_robot6900->_hiwdg);

	if(CMDs_buffer_size > 0)
	{
		current_state = GET_COMMAND;
	}
	else{
		current_state = WAIT;
	}

	// Full State Machine Call
	CMD_Parser_Log = (full_state_machine[current_state]).state_process(h_robot6900);

	// Process Parser Logs
	if(CMDs_buffer_full == 1)
	{
		// Warm user about the Full pipeline
		HAL_GPIO_WritePin(GPIOA, LOG_HARDFAULT_Pin, GPIO_PIN_SET);
		// As we process one command in the pipeline, it's no more full. Reactivate NVIC
		HAL_UART_Receive_IT(_huart, &Rx_buffer, NVIC_INT_BYTE_SIZE);
	}

	if(CMD_Parser_Log != PARSER_OK)
	{
		// Warn user with warning LED
		HAL_GPIO_WritePin(GPIOA, LOG_WARNING_Pin, GPIO_PIN_SET);

	}
}

COMMANDS_PARSER_STATE get_command(ROBOT6900_HANDLER* h_robot6900)
{
	COMMANDS_PARSER_STATE parser_log = PARSER_OK;
	uint8_t* raw_packet = 0;
	CMD_PACKET cmd;

	// Get oldest command in the queue
	raw_packet = CMDs_buffer[0];

	// Check raw packet validity and create a cmd defined struct
	parser_log = command_integrity(raw_packet, &cmd);
	if(parser_log == PARSER_OK)
	{
		// Looking if the command is defined
		for(uint8_t i = 0 ; i < NUMBER_OF_COMMAND; i++)
		{
			if(strcmp(&cmd.name, dispatch_table[i].name) == 0)
			{
				// Process command function
				dispatch_table[i].process(&cmd, h_robot6900);
			}
		}
	}

	// Remove command from the buffer and shift next one
	parser_log = update_pipeline();


	return parser_log;
}

COMMANDS_PARSER_STATE command_integrity(uint8_t* _raw_packet, CMD_PACKET* _cmd)
{
	COMMANDS_PARSER_STATE parser_log = PARSER_OK;
	uint8_t crc_buffer[2];

	// Parse 32bits raw_data to the packet_structure. Avoid 4 first bits SoF
	_cmd = (COMMANDS_PARSER_STATE*)(_raw_packet + CMD_PACKEt_SoF_SIZE);

	// Check Packet ID
	if(_cmd->ID != last_ID + 1)
	{
		parser_log = PARSER_WRONG_ID;
	}
	// Check CRC-8
	else
	{
		crc_buffer[0] = _cmd->name;
		crc_buffer[1] = _cmd->data;
		if( (uint8_t)(HAL_CRC_Calculate(_hcrc, crc_buffer, 2)) != _cmd->crc)
		{
			parser_log = PARSER_WRONG_CRC;
		}
	}

	return parser_log;
}

COMMANDS_PARSER_STATE update_pipeline()
{
	COMMANDS_PARSER_STATE parser_log = PARSER_OK;

	// Disable NVIC Interrupt before process on pipeline
	HAL_NVIC_DisableIRQ(UART5_IRQn);

	for(uint8_t i = 0 ; i < CMDs_buffer_size ; i++)
	{

		*(CMDs_buffer + i) = *(CMDs_buffer + i + 1);
	}

	if(CMDs_buffer_size - 1 >= 0)
	{
		CMDs_buffer_size -= 1;
	}
	else
	{
		parser_log = PARSER_UNKNOWN;
		CMDs_buffer_size = 0;
	}

	// Enable NVIC Interrupt after critical process on pipeline
	HAL_NVIC_EnableIRQ(UART5_IRQn);

	return parser_log;
}

COMMANDS_PARSER_STATE wait(ROBOT6900_HANDLER* h_robot6900)
{

}

