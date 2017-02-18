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

#include <stdint.h>
#include "utils.h"
#include "usart.h"
#include "settings.h"
#include "sonar.h"
#include "sonar_mode_filter.h"

GPIO_InitTypeDef GPIOE_InitStructure;
UART_HandleTypeDef UART4_Handle;

#define SONAR_SCALE	1000.0f
#define SONAR_MIN	0.12f		/** 0.12m sonar minimum distance */
#define SONAR_MAX	3.5f		/** 3.50m sonar maximum distance */

extern int atoi (__const char *__nptr);
extern uint32_t get_boot_time_us(void);

static char data_buffer[5]; // array for collecting decoded data

static volatile uint32_t last_measure_time = 0;
static volatile uint32_t measure_time = 0;
static volatile float dt = 0.0f;
static volatile int valid_data;
static volatile int data_counter = 0;
static volatile int data_valid = 0;
static volatile int new_value = 0;

static volatile uint32_t sonar_measure_time_interrupt = 0;
static volatile uint32_t sonar_measure_time = 0;

/* kalman filter states */
float x_pred = 0.0f; // m
float v_pred = 0.0f;
float x_post = 0.0f; // m
float v_post = 0.0f; // m/s

float sonar_raw = 0.0f;  // m

float sonar_mode = 0.0f;
float sonar_valid = false;				/**< the mode of all sonar measurements */

/**
  * @brief  Triggers the sonar to measure the next value
  *
  * see datasheet for more info
  */
void sonar_trigger()
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
}

/**
  * @brief  Sonar interrupt handler
*/
void UART4_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&UART4_Handle, UART_IT_RXNE) != RESET)
	{
		/* Read one byte from the receive data register */
		uint8_t data = (uint8_t)(UART4->DR & (uint16_t)0x01FF);

		if (data == 'R')
		{
			/* this is the first char (start of transmission) */
			data_counter = 0;
			data_valid = 1;

			/* set sonar pin 4 to low -> we want triggered mode */
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
		}
		else if (0x30 <= data && data <= 0x39)
		{
			if (data_valid)
			{
				data_buffer[data_counter] = data;
				data_counter++;
			}
		}
		else if (data == 0x0D)
		{
			if (data_valid && data_counter == 4)
			{
				data_buffer[4] = 0;
				int temp = atoi(data_buffer);

				/* use real-world maximum ranges to cut off pure noise */
				if ((temp > SONAR_MIN*SONAR_SCALE) && (temp < SONAR_MAX*SONAR_SCALE))
				{
					/* it is in normal sensor range, take it */
					last_measure_time = measure_time;
					measure_time = get_boot_time_us();
					sonar_measure_time_interrupt = measure_time;
					dt = ((float)(measure_time - last_measure_time)) / 1000000.0f;

					valid_data = temp;
					sonar_mode = insert_sonar_value_and_get_mode_value(valid_data / SONAR_SCALE);
					new_value = 1;
					sonar_valid = true;
				}
				else
				{
					sonar_valid = false;
				}
			}

			data_valid = 0;
		}
		else
		{
			data_valid = 0;
		}
	}
}

/**
  * @brief  Basic Kalman filter
  */
void sonar_filter()
{
	/* no data for long time */
	if (dt > 0.25f) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred = x_post + dt * v_pred;
	v_pred = v_post;

	float x_new = sonar_mode;
	sonar_raw = x_new;
	x_post = x_pred + global_data.param[PARAM_SONAR_KALMAN_L1] * (x_new - x_pred);
	v_post = v_pred + global_data.param[PARAM_SONAR_KALMAN_L2] * (x_new - x_pred);

}


/**
  * @brief  Read out newest sonar data
  *
  * @param  sonar_value_filtered Filtered return value
  * @param  sonar_value_raw Raw return value
  */
bool sonar_read(float* sonar_value_filtered, float* sonar_value_raw)
{
	/* getting new data with only around 10Hz */
	if (new_value) {
		sonar_filter();
		new_value = 0;
		sonar_measure_time = get_boot_time_us();
	}

	/* catch post-filter out of band values */
	if (x_post < SONAR_MIN || x_post > SONAR_MAX) {
		sonar_valid = false;
	}

	*sonar_value_filtered = x_post;
	*sonar_value_raw = sonar_raw;

	return sonar_valid;
}

/**
 * @brief  Configures the sonar sensor Peripheral.
 */
void sonar_config(void)
{
	valid_data = 0;

	/* Enable GPIO clocks*/
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/* Configure l3gd20 CS pin in output pushpull mode */
	GPIOE_InitStructure.Pin = GPIO_PIN_8;
	GPIOE_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
	GPIOE_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIOE_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIOE_InitStructure);

	/* Configures the nested vectored interrupt controller.*/
	HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
	/* Enable the USARTx Interrupt*/
	HAL_NVIC_EnableIRQ(UART4_IRQn);

	/* Enable the UART clock*/
	__HAL_RCC_UART4_CLK_ENABLE();
	/* Enable GPIO clocks*/
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStructure_Serial2;

	/* Connect UART pins to AF7 (me: or AF8?)*/
	GPIO_InitStructure_Serial2.Alternate = GPIO_AF8_UART4;

	GPIO_InitStructure_Serial2.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure_Serial2.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure_Serial2.Pull = GPIO_PULLUP;

	/* USART RX pin configuration*/
	GPIO_InitStructure_Serial2.Pin = GPIO_PIN_11;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure_Serial2);

	UART4_Handle.Instance = UART4;
	UART4_Handle.Init.BaudRate = 9600;
	UART4_Handle.Init.WordLength = UART_WORDLENGTH_8B;
	UART4_Handle.Init.StopBits = UART_STOPBITS_1;
	UART4_Handle.Init.Parity = UART_PARITY_NONE;
	UART4_Handle.Init.Mode = UART_MODE_RX;

	/* Configure the UART4*/
	HAL_UART_Init(&UART4_Handle); //Already enabled

	/* Enable UART4 interrupt*/
	__HAL_UART_ENABLE_IT(&UART4_Handle, UART_IT_RXNE);
}

uint32_t get_sonar_measure_time()
{
    return sonar_measure_time;
}

uint32_t get_sonar_measure_time_interrupt()
{
    return sonar_measure_time_interrupt;
}

