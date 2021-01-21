/*
 * spi_handler.c
 *
 *  Created on: Oct 6, 2020
 *      Author: valbe
 */

#include "spi_handler.h"

/*
############################################################################
#
#	High Purpose Variables
#
############################################################################
*/

SPI_HandleTypeDef* _hspi1;

SPI_DRIVER_STATE spi_driver_current_state = SPI_READY;

volatile uint8_t spi_data_ready = 0;

uint8_t fpga_data[FPGA_DATA_SIZE];

uint8_t acc_data[ACC_DATA_SIZE];

uint8_t d[2];


/*
############################################################################
#
#	NVIC Interrupt routines
#
############################################################################
*/
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
	spi_data_ready = 1;
}
/*
############################################################################
#
#	Functionnal behaviours
#
############################################################################
*/

void spi_driver_init(SPI_HandleTypeDef *hspi1)
{
	// Initiate ptr
	_hspi1 = hspi1;

	// Test FPGA Data
//	fpga_data[0] = FPGA_SFD | FPGA_CMD_START_STOP;	// SFD and CMD
//	fpga_data[1] = 0x02 | FPGA_frame_checker( ( ((0xFFFF) & FPGA_CMD_START_STOP) << 7 ) | 0x02);	// Payload and Frame Checker


}

SPI_DRIVER_ERROR spi_request(SPI_RQT_TYPE* _request_type, uint8_t* _data, uint8_t size)
{
	SPI_DRIVER_ERROR spi_log = SPI_DRIVER_BUSY;

	// Request new SPI transfer
	if(spi_driver_current_state == SPI_READY)
	{
		// Select Slave
		//select_Slave(_request_type, 0);
		HAL_GPIO_WritePin(GPIOD, CS_FPGA_Pin, GPIO_PIN_RESET);

		// Start SPI exchange in NVIC mode
//		HAL_SPI_TransmitReceive_IT(_hspi1, _data, getBuffer(_request_type), size);
		HAL_SPI_Transmit_IT(_hspi1, _data, size);

		spi_log = SPI_DRIVER_BUSY;
		spi_driver_current_state = SPI_PROCESSING;
	}
	else if(spi_driver_current_state == SPI_PROCESSING)
	{
		// Data finally received
		if(spi_data_ready == 1)
		{
			// Deselect the Slave
			//select_Slave(_request_type, 1);
			HAL_GPIO_WritePin(GPIOD, CS_FPGA_Pin, GPIO_PIN_SET);

			// Reset flag
			spi_data_ready = 0;
			// Set SPI Handler work task to DONE
			spi_driver_current_state = SPI_DONE;

			spi_log = SPI_DRIVER_DONE;
		}

	}
	else
	{
		spi_log = SPI_DRIVER_OK;
		// Set SPI Handler available for new transfer
		spi_driver_current_state = SPI_READY;
	}

	return spi_log;
}

uint8_t* getBuffer(SPI_RQT_TYPE* _request_type)
{
	uint8_t* ptr_buffer = 0;

	switch(*_request_type)
	{
	case SPI_RQT_FPGA:
		ptr_buffer = fpga_data;
		break;
	case SPI_RQT_ACC:
		ptr_buffer = acc_data;
		break;
	}

	return ptr_buffer;
}

void select_Slave(SPI_RQT_TYPE* _request_type, uint8_t low)
{
	switch(*_request_type)
	{
		case SPI_RQT_FPGA:
			HAL_GPIO_WritePin(GPIOD, CS_FPGA_Pin, (low == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET));
			break;
		case SPI_RQT_ACC:
			HAL_GPIO_WritePin(GPIOD, CS_Accelerometer_Pin, (low == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET));
			break;
	}
}

/*
 * Process bit parity for Frame Checker bit on FPGA packet
 * Bit parity on 10 bits value x
 */
uint8_t FPGA_frame_checker(uint16_t x){
    return ( (x>>9) ^
			(x>>8) ^
			(x>>7) ^
			(x>>6) ^
			(x>>5) ^
			(x>>4) ^
			(x>>3) ^
			(x>>2) ^
			(x>>1) ^
			(x) ) & 1;
}

