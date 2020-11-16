#include "commands.h"

uint8_t default_process(CMD_PACKET* _cmd_pck, ROBOT6900_HANDLER* h_robot6900)
{
  //printf("Non-implemented command !!");
	

	return 1;
}

uint8_t check_serial(CMD_PACKET* _cmd_pck, ROBOT6900_HANDLER* h_robot6900)
{
  //printf("Non-implemented command !!");
	h_robot6900->robot_state->Robot_State = Running;

	h_robot6900->robot_state->status_update = 1;
	return 1;
}


