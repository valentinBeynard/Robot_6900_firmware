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
    {0xA2, check_serial}

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
#	Parser High Purpose Variables
#
############################################################################
*/

/* UART5 Handler */
UART_HandleTypeDef* _huart;

// UART Rx_buffer, filled by NVIC interrupt
uint8_t Rx_buffer[NVIC_INT_BYTE_SIZE] = {0, 0, 0, 0, 0};

/* Buffer pipe to stock commands before handling them.
 * Add + 1 to keep an empty 32bits command space to easily
 * shift data when updating the pipeline
*/
uint8_t* CMDs_buffer[MAX_COMMAND_STACK_SIZE + 1];

// Current size of the waiting command pipeline
volatile uint8_t CMDs_buffer_size = 0;

/* Full pipeline flag */
uint8_t CMDs_buffer_full = 0;

// Flag error related to command parser behavior
COMMANDS_PARSER_ERROR CMD_Parser_Log = PARSER_OK;

/* Current state of the command parser state machine */
COMMANDS_PARSER_STATE current_state = WAIT;

/* Last packet IDentifier for communication reliability on Rx Side */
uint8_t rx_last_ID = 0;

/* Flag counter of all bad packet received */
uint8_t faulty_pck = 0;

/* Pointer to the global crc engine defined in main.c */
CRC_HandleTypeDef* _hcrc;

/*
############################################################################
#
#	General Shell Variables
#
############################################################################
*/

/* Last packet IDentifier for communication reliability on Tx Side */
uint8_t tx_last_ID = 0;

/* Tx buffer for general shell returning data to the HOST */
uint8_t Tx_buffer[TX_BUFFER_SIZE];

/*
############################################################################
#
#	NVIC Interrupt routines
#
############################################################################
*/

/*
 * INTERRUPT function for command parser
 *
 * Fired each time the UART receives a byte command pck, and fill the command pipeline
 *
 */
void NVIC_command_parser_INT(UART_HandleTypeDef *huart)
{

	if(CMDs_buffer_size < MAX_COMMAND_STACK_SIZE)
	{
		/* Fill pipeline and update ptr */
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

/* Initial the command parser */
void uart_init(UART_HandleTypeDef *huart, CRC_HandleTypeDef* hcrc)
{
	// Initiate ptr
	_hcrc = hcrc;
	_huart = huart;

	// Activate UART Receive Interrupt each 4 bytes received
	HAL_UART_Receive_IT(_huart, &Rx_buffer, NVIC_INT_BYTE_SIZE);
}

/*
 * Main Parser command. Only external entrance to the command parser.
 *
 * Handle the Command Parser Full State Machine.
 *
 * Each time the pipeline contains a command pck, the FSM switches to the
 * GET_COMMAND state. WAIT otherwise.
 *
 * Return the global FSM state (COMMANDS_PARSER_ERROR)
 *
 */
COMMANDS_PARSER_ERROR cmd_parser_process(ROBOT6900_HANDLER* h_robot6900)
{
#ifndef DEBUG
	HAL_IWDG_Refresh(h_robot6900->_hiwdg);
#endif

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
		// As we process one command in the pipeline, it's no more full. Reactivate NVIC
		HAL_UART_Receive_IT(_huart, &Rx_buffer, NVIC_INT_BYTE_SIZE);

		// Notify Pipeline is full
		CMD_Parser_Log |= PARSER_PIPELINE_FULL;
	}

	// Update Debug LED state regarding Parser state flags
	generate_parser_flag(h_robot6900);

	return CMD_Parser_Log;
}

/*
 * GET_COMMAND State.
 *
 * Get oldest packet in the pipeline, check command integrity and match
 * packet command to the dispatch table
 *
 * Update the pipeline at the end (shifting data in the queue)
 *
 * Return the global FSM state (COMMANDS_PARSER_ERROR)
 *
 */
COMMANDS_PARSER_ERROR get_command(ROBOT6900_HANDLER* h_robot6900)
{
	COMMANDS_PARSER_ERROR parser_log = PARSER_OK;
	uint8_t* raw_packet = 0;
	CMD_PACKET cmd;
	uint8_t cmd_defined = 0;

	// Get oldest command in the queue
	raw_packet = CMDs_buffer[0];

	// Check raw packet validity and create a cmd defined struct
	parser_log = command_integrity(raw_packet, &cmd);

	// Search for command in the dispatch table
	if(parser_log == PARSER_OK)
	{
		// Looking if the command is defined
		for(uint8_t i = 0 ; i < NUMBER_OF_COMMAND; i++)
		{
			if(cmd.name == dispatch_table[i].name)
			{
				cmd_defined = 1;
				// Process command function
				dispatch_table[i].process(&cmd, h_robot6900);
			}
		}

		// Check if command was found
		if(!cmd_defined)
		{
			parser_log = PARSER_NO_CMD;
		}
	}

	// Remove command from the buffer and shift next one
	parser_log |= update_pipeline();


	return parser_log;
}

/*
 * Check command packet integrity (ID and CRC-8)
 *
 * Return the global FSM state (COMMANDS_PARSER_ERROR)
 */
COMMANDS_PARSER_ERROR command_integrity(uint8_t* _raw_packet, CMD_PACKET* _cmd)
{
	COMMANDS_PARSER_ERROR parser_log = PARSER_OK;
	uint8_t crc_buffer[2];

	// Avoid SoF byte
	_raw_packet+=1;

	// Parse 32bits raw_data to the packet_structure.
	*(_cmd) = *(CMD_PACKET*)(_raw_packet);

	// Check Packet ID
	if(_cmd->ID != rx_last_ID + 1)
	{
		parser_log = PARSER_WRONG_ID;
	}
	// Check CRC-8
	else
	{
		/* If ID right, update ID flag for next packet */
		if(_cmd->ID == 0xFF)
		{
			rx_last_ID = 0;
		}
		else
		{
			rx_last_ID = _cmd->ID;
		}

		// Calculate 8bits CRC and check for validity
		crc_buffer[0] = _cmd->name;
		crc_buffer[1] = _cmd->data;
		if( (uint8_t)(HAL_CRC_Calculate(_hcrc, crc_buffer, 2)) != _cmd->crc)
		{
			parser_log = PARSER_WRONG_CRC;
		}

		/*
		 * Nothing if CRC wrong
		 */
	}

	return parser_log;
}

/*
 * Update the command pipeline.
 *
 * Shit all data to the left by 5 bytes, in order to get next raw packet at
 * index 0
 *
 * Return the global FSM state (COMMANDS_PARSER_ERROR)
 */
COMMANDS_PARSER_ERROR update_pipeline()
{
	COMMANDS_PARSER_ERROR parser_log = PARSER_INIT;

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

/*
 * WAIT State.
 *
 * Nothing to do.
 *
 */
COMMANDS_PARSER_ERROR wait(ROBOT6900_HANDLER* h_robot6900)
{
	return CMD_Parser_Log;
}

/*
############################################################################
#
#	General Return Shell Functions
#
############################################################################
*/

/*
 * Check if firmware has to return data to the HOST, and transmit data on UART5
 *
 * Return global robot state, LIDar packet, ect...
 */
void parser_return(ROBOT6900_HANDLER* h_robot6900)
{
	uint8_t tx_pck_size = 0;

	/* Clear TX_BUFFER */
	memset(Tx_buffer, 0, TX_BUFFER_SIZE);

	/* Build the Robot State Output Packet */
	if(h_robot6900->robot_state.status_update)
	{
		//tx_pck_size = parser_OUTPUT_status(h_robot6900);

		/* DEBUG */
		Tx_buffer[0] = 'G';
		tx_pck_size = 1;
		/* DEBUG */
	}

	/* Send data if there is data to transmit in the buffer */
	if(tx_pck_size != 0)
	{
		HAL_UART_Transmit_IT(_huart, &Tx_buffer, tx_pck_size);
	}

	/* Clear Robot Update Sate Flag */
	h_robot6900->robot_state.status_update = 0;

}


uint8_t parser_OUTPUT_status(ROBOT6900_HANDLER* h_robot6900)
{
	uint8_t l_pck_state = sizeof(ROBOT6900_STATE);
	uint8_t packet_size = 1 + 1 + l_pck_state + 1;

	/* Init Start of Frame */
	Tx_buffer[0] = (CMD_START_OF_FRAME & tx_last_ID);
	/* Init Type of Packet */
	Tx_buffer[1] = TX_TYPE_STATE_PCK;

	/* Cast Robot State data to packet byte */
	Tx_buffer[2] = (uint8_t*)(&h_robot6900->robot_state);

	/* Add CRC for Data */
	Tx_buffer[2 + l_pck_state] = HAL_CRC_Calculate(_hcrc, Tx_buffer[2], l_pck_state);

	return packet_size;
}

/*
 * Generate the Debug LEDs logics regarding global FSM state (COMMANDS_PARSER_ERROR)
 */
void generate_parser_flag(ROBOT6900_HANDLER* h_robot6900)
{
	static COMMANDS_PARSER_ERROR previous_log = PARSER_OK;

	// Do not change LEDs statues is parser's flags didn't change
	if(CMD_Parser_Log != previous_log)
	{
		h_robot6900->robot_state.debug_leds = ((CMD_Parser_Log & PARSER_OK) == 1 ? 0x00 : DB_LED3);
		h_robot6900->robot_state.debug_leds |= ((CMD_Parser_Log & PARSER_NO_CMD) == PARSER_NO_CMD ? DB_LED7 : 0x00);
		h_robot6900->robot_state.debug_leds |= ((CMD_Parser_Log & PARSER_WRONG_ID) == PARSER_WRONG_ID || (CMD_Parser_Log & PARSER_WRONG_CRC) == PARSER_WRONG_CRC ? DB_LED8 : 0x00);
		h_robot6900->robot_state.debug_leds |= ((CMD_Parser_Log & PARSER_PIPELINE_FULL) == PARSER_PIPELINE_FULL || (CMD_Parser_Log & PARSER_WRONG_CRC) == PARSER_WRONG_CRC ? DB_LED9 : 0x00);

		// Update last parser's flag values
		previous_log = CMD_Parser_Log;
	}

}

