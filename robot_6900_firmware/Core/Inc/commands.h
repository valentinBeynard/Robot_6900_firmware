#ifndef COMMAND_H

#define COMMAND_H

#include "../Src/robot_handler.h"
#include "stm32f3xx_hal.h"


/* Length in byte of a command packet */
#define CMD_PACKET_SIZE	5

#define CMD_START_OF_FRAME	0xA0

/* Size description of packet component in bits */
#define CMD_PACKEt_SoF_SIZE	8
#define CMD_PACKET_ID_SIZE	8
#define CMD_PACKET_CMD_SIZE	8
#define CMD_PACKET_DATA_SIZE	8
#define CMD_PACKET_CRC_SIZE	8

/* Size description of packet for Return packets */
#define TX_PACKET_SoF_SIZE	4
#define TX_PACKET_ID_SIZE	4
#define TX_PACKET_TYPE_SIZE	8
#define TX_PACKET_CRC_SIZE	8

#define TX_TYPE_STATE_PCK	0x01
#define TX_TYPE_DESCRIPTOR_PCK	0x02


/* Packet description */
typedef struct __attribute__((__packed__))
{
	uint8_t ID;
	uint8_t name;
	uint8_t data;
	uint8_t crc;
}CMD_PACKET;

/* Command description : a name + a function */
typedef struct
{
  uint8_t name;
  uint8_t(*process)(CMD_PACKET*, ROBOT6900_HANDLER*);
}CMD_;

uint8_t default_process(CMD_PACKET* _cmd_pck, ROBOT6900_HANDLER* h_robot6900);
uint8_t check_serial(CMD_PACKET* _cmd_pck, ROBOT6900_HANDLER* h_robot6900);

//*/
//
//#define max(a,b)            (((a) > (b)) ? (a) : (b))
//#define min(a,b)            (((a) < (b)) ? (a) : (b))
//
//// Taille max du buffer pour un seul mot de commande/argument
//#define ARGS_BUFFER_SIZE  20
//
//// Nombre max de mots composants une commande que l'on peut récupérer avec le buffer lors du parse
//#define MAX_COMMAND_WORD  10
//
//
//

#endif
