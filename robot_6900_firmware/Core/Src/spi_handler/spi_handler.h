/*
 * spi_handler.h
 *
 *  Created on: Oct 6, 2020
 *      Author: valbe
 */

#ifndef SRC_SPI_HANDLER_SPI_HANDLER_H_
#define SRC_SPI_HANDLER_SPI_HANDLER_H_

#include "../Src/robot_handler.h"
#include "main.h"

/* Maximum number of stored SPI command in the waiting pipeline */
#define SPI_MAX_COMMAND_STACK_SIZE	10

#define FPGA_DATA_SIZE	16
#define ACC_DATA_SIZE	16




typedef enum
{
  SPI_RQT_ACC = 0,
  SPI_RQT_FPGA
}SPI_RQT_TYPE;


typedef enum
{
	SPI_DRIVER_BUSY = 0,
	SPI_DRIVER_DONE = 1,
	SPI_DRIVER_OK
}SPI_DRIVER_ERROR;

typedef enum
{
  SPI_READY,
  SPI_PROCESSING,
  SPI_DONE
}SPI_DRIVER_STATE;

void spi_driver_init(SPI_HandleTypeDef *hspi1);
SPI_DRIVER_ERROR spi_request(SPI_RQT_TYPE* _request_type, uint8_t* _data, uint8_t size);
uint8_t* getBuffer(SPI_RQT_TYPE* _request_type);
void select_Slave(SPI_RQT_TYPE* _request_type, uint8_t low);

uint8_t FPGA_frame_checker(uint16_t x);


#endif /* SRC_SPI_HANDLER_SPI_HANDLER_H_ */
