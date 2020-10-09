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

uint8_t spi_data_ready = 0;

uint8_t fpga_data[FPGA_DATA_SIZE];

uint8_t acc_data[ACC_DATA_SIZE];

/*
############################################################################
#
#	NVIC Interrupt routines
#
############################################################################
*/
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
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

}

SPI_DRIVER_ERROR spi_request(SPI_RQT_TYPE* _request_type, uint8_t* _data, uint8_t size)
{
	SPI_DRIVER_ERROR spi_log = SPI_DRIVER_BUSY;

	// Request new SPI transfer
	if(spi_driver_current_state == SPI_READY)
	{
		// Select Slave
		select_Slave(_request_type, 0);

		// Start SPI exchange in NVIC mode
		HAL_SPI_TransmitReceive_IT(_hspi1, _data, getBuffer(_request_type), size);

		spi_driver_current_state = SPI_PROCESSING;
	}
	else
	{
		// Data finally received
		if(spi_data_ready == 1)
		{
			// Deselect the Slave
			select_Slave(_request_type, 1);
			// Reset flag
			spi_data_ready = 0;
			// Set SPI Handler available for new transfer
			spi_driver_current_state = SPI_READY;

			spi_log = SPI_DRIVER_OK;
		}

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

