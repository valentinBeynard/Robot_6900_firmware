#ifndef COMMAND_H

#define COMMAND_H

#include "../Src/robot_handler.h"
#include "stm32f3xx_hal.h"


/* Length in byte of a command packet */
#define CMD_PACKET_SIZE	4

/* Size description of packet component in bits */
#define CMD_PACKEt_SoF_SIZE	4
#define CMD_PACKET_ID_SIZE	4
#define CMD_PACKET_CMD_SIZE	8
#define CMD_PACKET_DATA_SIZE	8
#define CMD_PACKET_CRC_SIZE	8

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

//

//
//

//
//
///*
//  Commande de démarage de l'épreuve : D [Numéro Epreuve]
//  Si aucun paramètre rentré, alors on démarre à l'épreuve 1
//*/
//byte epreuve_cmd(CMD_PACKET* cmd_packet);
//byte epreuve_stop(CMD_PACKET* cmd_packet);
//
//byte safety_break_cmd(CMD_PACKET* cmd_packet);
//byte set_default_speed_cmd(CMD_PACKET* cmd_packet);
//byte move_forward_cmd(CMD_PACKET* cmd_packet);
//byte move_backward_cmd(CMD_PACKET* cmd_packet);
//byte move_stop_cmd(CMD_PACKET* cmd_packet);
//byte rigth_rotation_cmd(CMD_PACKET* cmd_packet);
//byte left_rotation_cmd(CMD_PACKET* cmd_packet);
//byte complete_rotation_cmd(CMD_PACKET* cmd_packet);
//byte angle_rotation_cmd(CMD_PACKET* cmd_packet);
//byte move_to_cmd(CMD_PACKET* cmd_packet);
//
//
//byte detecte_obstacle(CMD_PACKET* cmd_packet);
//byte servo_move_cmd(CMD_PACKET* cmd_packet);
//byte light_beam_ON_cmd(CMD_PACKET* cmd_packet);
//byte light_beam_OFF_cmd(CMD_PACKET* cmd_packet);
//byte generate_sound_cmd(CMD_PACKET* cmd_packet);
//byte photo_cmd(CMD_PACKET* cmd_packet);
//byte photo_OFF_cmd(CMD_PACKET* cmd_packet);
//byte aux_cmd(CMD_PACKET* cmd_packet);
//
//#else
//
////#error Multiple define !
//
#endif
