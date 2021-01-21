#include "commands.h"

uint8_t default_process(CMD_PACKET* _cmd_pck, ROBOT6900_HANDLER* h_robot6900)
{
  //printf("Non-implemented command !!");
	

	return 1;
}

uint8_t switch_ON(CMD_PACKET* _cmd_pck, ROBOT6900_HANDLER* h_robot6900)
{
  //printf("Non-implemented command !!");
	h_robot6900->robot_state->Etat_Movement = Mouvement_Post_Launch;

	h_robot6900->robot_state->status_update = 1;
	return 1;
}

uint8_t start_motor(CMD_PACKET* _cmd_pck, ROBOT6900_HANDLER* h_robot6900)
{
  //printf("Non-implemented command !!");
	h_robot6900->robot_state->Etat_Movement = Start;

	h_robot6900->robot_state->status_update = 1;
	return 1;
}

uint8_t stop_motor(CMD_PACKET* _cmd_pck, ROBOT6900_HANDLER* h_robot6900)
{
  //printf("Non-implemented command !!");
	h_robot6900->robot_state->Etat_Movement = Stopper;

	//h_robot6900->robot_state->status_update = 1;
	return 1;
}

uint8_t start_lidar(CMD_PACKET* _cmd_pck, ROBOT6900_HANDLER* h_robot6900)
{
  //printf("Non-implemented command !!");
	h_robot6900->robot_state->LiDAR_State = LiDAR_ACQ_Const;

	h_robot6900->robot_state->status_update = 1;
	return 1;
}

uint8_t washer_brosse(CMD_PACKET* _cmd_pck, ROBOT6900_HANDLER* h_robot6900)
{
  //printf("Non-implemented command !!");
	if(_cmd_pck->data == 0x01)
	{
		h_robot6900->robot_state->Etat_Washer = Washer_Brosse_ON;
	}
	else
	{
		h_robot6900->robot_state->Etat_Washer = Washer_Brosse_OFF;
	}

	h_robot6900->robot_state->status_update = 1;
	return 1;
}

uint8_t washer_pump(CMD_PACKET* _cmd_pck, ROBOT6900_HANDLER* h_robot6900)
{
  //printf("Non-implemented command !!");
	if(_cmd_pck->data == 0x01)
	{
		h_robot6900->robot_state->Etat_Washer = Washer_Pump_ON;
	}
	else
	{
		h_robot6900->robot_state->Etat_Washer = Washer_Pump_OFF;
	}


	h_robot6900->robot_state->status_update = 1;
	return 1;
}

uint8_t washer_UVC(CMD_PACKET* _cmd_pck, ROBOT6900_HANDLER* h_robot6900)
{
  //printf("Non-implemented command !!");
	if(_cmd_pck->data == 0x01)
	{
		h_robot6900->robot_state->Etat_Washer = Washer_UVC_ON;
	}
	else
	{
		h_robot6900->robot_state->Etat_Washer = Washer_UVC_OFF;
	}

	h_robot6900->robot_state->status_update = 1;
	return 1;
}
