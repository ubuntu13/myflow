/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "gyro.h"
#include <math.h>

SPI_HandleTypeDef SPI2_Handle;

float gyro_scale;
float x_rate_offset = 0.0f, y_rate_offset = 0.0f, z_rate_offset = 0.0f;
const float gyro_offset_lp_gain = 0.0001;
static int sensor_range;

enum
{
    GYRO_DPS250 = 0, GYRO_DPS500, GYRO_DPS2000
} GYRO_DPS_Values;

static float scaling_factors[] =
{8.75f, 17.5f, 70.0f};//mdps/digit

/**
 * @brief Configures Gyro
 */
void gyro_config()
{
	/* spi first */
	spi_config();

	/* gyro */
	l3gd20_config();
}

/**
 * @brief Read out newest gyro value
 *
 * @param x_rate Return value x rate
 * @param y_rate Return value y rate
 * @param z_rate Return value z rate
 */
void gyro_read(float* x_rate, float* y_rate, float* z_rate, int16_t* gyro_temp)
{
	int16_t x_rate_raw, y_rate_raw, z_rate_raw;
	x_rate_raw = 0;
	y_rate_raw = 0;
	z_rate_raw = 0;

	/* read out */
	x_rate_raw = (uint8_t)l3gd20_SendHalfWord(0x8000 | 0x2800);
	x_rate_raw |= ((uint16_t)l3gd20_SendHalfWord(0x8000 | 0x2900)) << 8;
	y_rate_raw = (uint8_t)l3gd20_SendHalfWord(0x8000 | 0x2A00);
	y_rate_raw |= ((uint16_t)l3gd20_SendHalfWord(0x8000 | 0x2B00)) << 8;
	z_rate_raw = (uint8_t)l3gd20_SendHalfWord(0x8000 | 0x2C00);
	z_rate_raw |= ((uint16_t)l3gd20_SendHalfWord(0x8000 | 0x2D00)) << 8;

	/* offset elimination */
	x_rate_offset = (1.0f - gyro_offset_lp_gain) * x_rate_offset + gyro_offset_lp_gain * ((float) x_rate_raw) * gyro_scale;
	y_rate_offset = (1.0f - gyro_offset_lp_gain) * y_rate_offset + gyro_offset_lp_gain * ((float) y_rate_raw) * gyro_scale;
	z_rate_offset = (1.0f - gyro_offset_lp_gain) * z_rate_offset + gyro_offset_lp_gain * ((float) z_rate_raw) * gyro_scale;
	*x_rate = x_rate_raw * gyro_scale - x_rate_offset;
	*y_rate = y_rate_raw * gyro_scale - y_rate_offset;
	*z_rate = z_rate_raw * gyro_scale - z_rate_offset;

	int8_t temp_raw = 0;
	temp_raw = (int8_t)l3gd20_SendHalfWord(0x8000 | 0x2600);
	*gyro_temp = (L3GD20_TEMP_OFFSET_CELSIUS-(int16_t)temp_raw)*100;//Temperature * 100 in centi-degrees Celsius [degcelsius*100]
}

/**
 * @brief Configures l3gd20 gyroscope
 */
void l3gd20_config()
{
	/* write configurations (see data-sheet)
	 * 0x read/write | 0x address | 0x value
	 */

	/* enable sensor, 760Hz, bandwidth 30Hz */
	l3gd20_SendHalfWord(0x0000 | 0x2000 | 0x00CF);

	if (global_data.param[PARAM_GYRO_SENSITIVITY_DPS] == 250)
	{
		/* enable +-250dps range */
		l3gd20_SendHalfWord(0x0000 | 0x2300 | 0x0000);
	    sensor_range = GYRO_DPS250;
	}
	else if (global_data.param[PARAM_GYRO_SENSITIVITY_DPS] == 500)
	{
		/* enable +-500dps range */
		l3gd20_SendHalfWord(0x0000 | 0x2300 | 0x0010);
	    sensor_range = GYRO_DPS500;
	}
	else if (global_data.param[PARAM_GYRO_SENSITIVITY_DPS] == 2000)
	{
		/* enable +-2000dps range */
		l3gd20_SendHalfWord(0x0000 | 0x2300 | 0x0020);
	    sensor_range = GYRO_DPS2000;
	}
	else
	{
		/* wrong configuration -> reset to default*/
		global_data.param[PARAM_GYRO_SENSITIVITY_DPS] = 500;
		l3gd20_SendHalfWord(0x0000 | 0x2300 | 0x0010);
	    sensor_range = GYRO_DPS500;
	}

	gyro_scale = scaling_factors[sensor_range] / (1000.0f) * (float)M_PI / 180.0f; // scaling_factors in mdps/digit to dps/digit, degree to radian
}

/**
  * @brief  Configures the SPI Peripheral.
  */
void spi_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable the SPI clock */
	__HAL_RCC_SPI2_CLK_ENABLE();

	/* Enable GPIO clocks */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Connect SPI pins to AF5 */
	GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
	GPIO_InitStructure.Alternate = GPIO_AF5_SPI2;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.Pin = SPIx_SCK_PIN;
	HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.Pin =  SPIx_MOSI_PIN;
	HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);

	/* SPI MISO pin configuration */
	GPIO_InitStructure.Pin =  SPIx_MISO_PIN;
	HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);

	/* Configure l3gd20 CS pin in output pushpull mode */
	GPIO_InitStructure.Pin = l3gd20_CS_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(l3gd20_CS_GPIO_PORT, &GPIO_InitStructure);

	/* Deselect the l3gd20: Chip Select high */
	l3gd20_CS_HIGH();

	/* SPI configuration */
	SPI2_Handle.Init.Direction = SPI_DIRECTION_2LINES;
	SPI2_Handle.Init.Mode = SPI_MODE_MASTER;
	SPI2_Handle.Init.DataSize = SPI_DATASIZE_16BIT;
	SPI2_Handle.Init.NSS = SPI_NSS_SOFT;
	SPI2_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    SPI2_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPI2_Handle.Init.CLKPhase = SPI_PHASE_2EDGE;

	SPI2_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPI2_Handle.Init.CRCPolynomial = DISABLE;
	HAL_SPI_Init(&SPI2_Handle);

	__HAL_SPI_ENABLE(&SPI2_Handle);
}

/**
  * @brief  Reads a byte from the SPI Flash.
  * @note   This function must be used only if the Start_Read_Sequence function
  *         has been previously called.
  * @param  None
  * @retval Byte Read from the SPI Flash.
  */
uint8_t l3gd20_ReadByte(void)
{
	return (l3gd20_SendByte(l3gd20_DUMMY_BYTE));
	l3gd20_CS_HIGH();
}

/**
  * @brief  Sends a byte through the SPI interface and return the byte received
  *         from the SPI bus.
  * @param  byte: byte to send.
  * @retval The value of the received byte.
  */
uint8_t l3gd20_SendByte(uint8_t byte)
{
	l3gd20_CS_LOW();
	/* Loop while DR register in not emplty */
	while (__HAL_SPI_GET_FLAG(&SPI2_Handle, SPI_FLAG_TXE) == RESET);

	/* Send byte through the SPI2 peripheral */
	SPI2->DR = byte;

	/* Wait to receive a byte */
	while (__HAL_SPI_GET_FLAG(&SPI2_Handle, SPI_FLAG_RXNE) == RESET);

	/* Return the byte read from the SPI bus */
	return SPI2->DR;
}

/**
  * @brief  Sends a Half Word through the SPI interface and return the Half Word
  *         received from the SPI bus.
  * @param  HalfWord: Half Word to send.
  * @retval The value of the received Half Word.
  */
uint16_t l3gd20_SendHalfWord(uint16_t HalfWord)
{
	/* Loop while DR register in not empty */
	while (__HAL_SPI_GET_FLAG(&SPI2_Handle, SPI_FLAG_TXE) == RESET);
	l3gd20_CS_LOW();

	/* Send Half Word through the sFLASH peripheral */
	SPI2->DR = HalfWord;

	/*!< Wait to receive a Half Word */
	while (__HAL_SPI_GET_FLAG(&SPI2_Handle, SPI_FLAG_RXNE) == RESET);
	l3gd20_CS_HIGH();

	/* Return the Half Word read from the SPI bus */
	return SPI2->DR;
}

/**
  * @brief  Enables the write access to the FLASH.
  * @param  None
  * @retval None
  */
void l3gd20_WriteEnable(void)
{
	/* Select the FLASH: Chip Select low */
	l3gd20_CS_LOW();

	/* Send "Write Enable" instruction */
	l3gd20_SendByte(l3gd20_CMD_WREN);

	/* Deselect the FLASH: Chip Select high */
	l3gd20_CS_HIGH();
}

/**
  * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
  *         status register and loop until write operation has completed.
  * @param  None
  * @retval None
  */
void l3gd20_WaitForWriteEnd(void)
{
	uint8_t flashstatus = 0;

	/* Select the FLASH: Chip Select low */
	l3gd20_CS_LOW();

	/* Send "Read Status Register" instruction */
	l3gd20_SendByte(l3gd20_CMD_RDSR);

	/* Loop as long as the memory is busy with a write cycle */
	do
	{
		/* Send a dummy byte to generate the clock needed by the FLASH
		 * and put the value of the status register in FLASH_Status variable
		 */
		flashstatus = l3gd20_SendByte(l3gd20_DUMMY_BYTE);
	}
	while ((flashstatus & l3gd20_WIP_FLAG) == SET); /* Write in progress */

	/* Deselect the FLASH: Chip Select high */
	l3gd20_CS_HIGH();
}

uint8_t getGyroRange()
{
    return sensor_range;
}

int getGyroScalingFactor()
{
    return scaling_factors[sensor_range];
}
