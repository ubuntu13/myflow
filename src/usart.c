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

#include "usart.h"

USART_HandleTypeDef USART2_Handle;
USART_HandleTypeDef USART3_Handle;

#define TXBUFFERSIZE   	(64*64) // 4 KByte
#define RXBUFFERSIZE   	(64*64)

/* prototypes */
uint8_t usart2_tx_ringbuffer_push(const uint8_t* ch, uint8_t len);
uint8_t usart3_tx_ringbuffer_push(const uint8_t* ch, uint8_t len);
int usart2_char_available(void);
int usart3_char_available(void);
uint8_t usart2_rx_ringbuffer_pop(void);
uint8_t usart3_rx_ringbuffer_pop(void);
uint8_t usart2_rx_ringbuffer_push_from_usart(void);
uint8_t usart3_rx_ringbuffer_push_from_usart(void);
uint8_t usart2_tx_ringbuffer_pop_to_usart(void);
uint8_t usart3_tx_ringbuffer_pop_to_usart(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void usart_init(void);

/* fill output buffers with some asciis to start with */
uint8_t usart2_tx_buffer[TXBUFFERSIZE] = "\r\n    ____ _  ____ __  ________    ____ _       __\r\n   / __ \\ |/ / // / / ____/ /   / __ \\ |     / /\r\n  / /_/ /   / // /_/ /_  / /   / / / / | /| / / \r\n / ____/   /__  __/ __/ / /___/ /_/ /| |/ |/ /  \r\n/_/   /_/|_| /_/ /_/   /_____/\\____/ |__/|__/   \r\n                                                \r\n";
uint8_t usart2_rx_buffer[RXBUFFERSIZE] = "";
uint8_t usart3_tx_buffer[TXBUFFERSIZE] = "\r\n    ____ _  ____ __  ________    ____ _       __\r\n   / __ \\ |/ / // / / ____/ /   / __ \\ |     / /\r\n  / /_/ /   / // /_/ /_  / /   / / / / | /| / / \r\n / ____/   /__  __/ __/ / /___/ /_/ /| |/ |/ /  \r\n/_/   /_/|_| /_/ /_/   /_____/\\____/ |__/|__/   \r\n                                                \r\n";
uint8_t usart3_rx_buffer[RXBUFFERSIZE] = "";

int usart2_tx_counter_read = 0;
int usart2_tx_counter_write = 300;
int usart2_rx_counter_read = 0;
int usart2_rx_counter_write = 0;
int usart3_tx_counter_read = 0;
int usart3_tx_counter_write = 300;
int usart3_rx_counter_read = 0;
int usart3_rx_counter_write = 0;

/**
  * @brief  Push one byte to ringbuffer of USART2
  */
uint8_t usart2_tx_ringbuffer_push(const uint8_t* ch, uint8_t len)
{
	__HAL_USART_DISABLE_IT(&USART2_Handle, USART_IT_TXE);

	/* if there is free space in buffer */
	if ((((usart2_tx_counter_read - usart2_tx_counter_write) - 1) + TXBUFFERSIZE) % TXBUFFERSIZE > len)
	{
		uint8_t i;
		for (i = 0; i < len; i++)
		{
			usart2_tx_buffer[usart2_tx_counter_write] = ch[i];
			usart2_tx_counter_write = (usart2_tx_counter_write + 1) % TXBUFFERSIZE;
		}

		__HAL_USART_ENABLE_IT(&USART2_Handle, USART_IT_TXE);
		return 1;
	}

	__HAL_USART_ENABLE_IT(&USART2_Handle, USART_IT_TXE);
	return 0;
}

/**
  * @brief  Push one byte to ringbuffer of USART3
  */
uint8_t usart3_tx_ringbuffer_push(const uint8_t* ch, uint8_t len)
{
	__HAL_USART_DISABLE_IT(&USART3_Handle, USART_IT_TXE);

	/* if there is free space in buffer */
	if ((((usart3_tx_counter_read - usart3_tx_counter_write) - 1) + TXBUFFERSIZE) % TXBUFFERSIZE > len)
	{
		uint8_t i;
		for (i = 0; i < len; i++)
		{
			usart3_tx_buffer[usart3_tx_counter_write] = ch[i];
			usart3_tx_counter_write = (usart3_tx_counter_write + 1) % TXBUFFERSIZE;
		}

		__HAL_USART_ENABLE_IT(&USART3_Handle, USART_IT_TXE);
		return 1;
	}

	__HAL_USART_ENABLE_IT(&USART3_Handle, USART_IT_TXE);
	return 0;
}

/**
  * @brief  Check character availability USART2
  */
int usart2_char_available(void)
{
	return (usart2_rx_counter_read != usart2_rx_counter_write);
}

/**
  * @brief  Check character availability USART3
  */
int usart3_char_available(void)
{
	return (usart3_rx_counter_read != usart3_rx_counter_write);
}

/**
  * @brief  Pop one byte from ringbuffer of USART2
  */
uint8_t usart2_rx_ringbuffer_pop(void)
{
	__HAL_USART_DISABLE_IT(&USART2_Handle, USART_IT_TXE);

	uint8_t value = usart2_rx_buffer[usart2_rx_counter_read];
	usart2_rx_counter_read = (usart2_rx_counter_read + 1) % TXBUFFERSIZE;

	__HAL_USART_ENABLE_IT(&USART2_Handle, USART_IT_TXE);
	return value;
}

/**
  * @brief  Pop one byte from ringbuffer of USART3
  */
uint8_t usart3_rx_ringbuffer_pop(void)
{
	__HAL_USART_DISABLE_IT(&USART3_Handle, USART_IT_TXE);

	uint8_t value = usart3_rx_buffer[usart3_rx_counter_read];
	usart3_rx_counter_read = (usart3_rx_counter_read + 1) % TXBUFFERSIZE;

	__HAL_USART_ENABLE_IT(&USART3_Handle, USART_IT_TXE);
	return value;
}

/**
  * @brief  Copy from USART2 to ringbuffer
  */
uint8_t usart2_rx_ringbuffer_push_from_usart(void)
{
	usart2_rx_buffer[usart2_rx_counter_write] = (uint8_t)(USART2->DR & (uint16_t)0x01FF);
	int temp = (usart2_rx_counter_write + 1) % TXBUFFERSIZE;

	if(temp == usart2_rx_counter_read)
	{
		return 0;
	}

	usart2_rx_counter_write = temp;
	return 1;
}

/**
  * @brief  Copy from USART3 to ringbuffer
  */
uint8_t usart3_rx_ringbuffer_push_from_usart(void)
{
	//USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
	usart3_rx_buffer[usart3_rx_counter_write] = (uint8_t)(USART3->DR & (uint16_t)0x01FF);
	int temp = (usart3_rx_counter_write + 1) % TXBUFFERSIZE;

	if(temp == usart3_rx_counter_read)
	{
		return 0;
	}

	usart3_rx_counter_write = temp;
	return 1;
}

/**
  * @brief  Copy from ringbuffer to USART2
  */
uint8_t usart2_tx_ringbuffer_pop_to_usart(void)
{
	if (usart2_tx_counter_read != usart2_tx_counter_write)
	{
		USART2->DR = (usart2_tx_buffer[usart2_tx_counter_read] & (uint16_t)0x01FF);
		usart2_tx_counter_read= (usart2_tx_counter_read+1) % TXBUFFERSIZE;
		return 1;
	}
	return 0;
}

/**
  * @brief  Copy from ringbuffer to USART3
  */
uint8_t usart3_tx_ringbuffer_pop_to_usart(void)
{
	if (usart3_tx_counter_read != usart3_tx_counter_write)
	{
		USART3->DR = (usart3_tx_buffer[usart3_tx_counter_read] & (uint16_t)0x01FF);
		usart3_tx_counter_read= (usart3_tx_counter_read+1) % TXBUFFERSIZE;
		return 1;
	}
	return 0;
}

/**
  * @brief  USART2 interrupt handler
  */
void USART2_IRQHandler(void)
{
	FlagStatus sta;

	sta = __HAL_USART_GET_FLAG(&USART2_Handle, USART_FLAG_RXNE);
	if(sta != RESET)
	{
		if(usart2_rx_ringbuffer_push_from_usart() == 0)
		{
			/* Disable the Receive interrupt if buffer is full */
			__HAL_USART_DISABLE_IT(&USART2_Handle, USART_IT_RXNE);
		}
		return;
	}

	sta = __HAL_USART_GET_FLAG(&USART2_Handle, USART_FLAG_TXE);
	if(sta != RESET)
	{
		if(usart2_tx_ringbuffer_pop_to_usart() == 0)
		{
			/* Disable the Transmit interrupt if buffer is empty */
			__HAL_USART_DISABLE_IT(&USART2_Handle, USART_IT_TXE);
		}

		return;
	}
}

/**
  * @brief  USART3 interrupt handler
  */
void USART3_IRQHandler(void)
{
	if(__HAL_USART_GET_FLAG(&USART3_Handle, USART_FLAG_RXNE) != RESET)
	{
		if(usart3_rx_ringbuffer_push_from_usart() == 0)
		{
			/* Disable the Receive interrupt if buffer is full */
			__HAL_USART_DISABLE_IT(&USART3_Handle, USART_IT_RXNE);
		}
		return;
	}

	if(__HAL_USART_GET_FLAG(&USART3_Handle, USART_FLAG_TXE) != RESET)
	{
		if(usart3_tx_ringbuffer_pop_to_usart() == 0)
		{
			/* Disable the Transmit interrupt if buffer is empty */
			__HAL_USART_DISABLE_IT(&USART3_Handle, USART_IT_TXE);
		}

		return;
	}
}

/**
  * @brief  Configures USART2 and USART3 for communication
  */
void usart_init(void)
{
	/* Configures the nested vectored interrupt controller */
	HAL_NVIC_SetPriority(USART2_IRQn, 0 , 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_SetPriority(USART3_IRQn, 0 , 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable the USART clocks */
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_USART3_CLK_ENABLE();

	/* Enable GPIO clock */
	__HAL_RCC_GPIOD_CLK_ENABLE();

	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;

	/* USART2 TX pin configuration */
	GPIO_InitStructure.Pin = GPIO_PIN_5;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* USART2 RX pin configuration */
	GPIO_InitStructure.Pin = GPIO_PIN_6;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* USART3 TX pin configuration */
	GPIO_InitStructure.Pin = GPIO_PIN_8;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART3;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* USART3 RX pin configuration */
	GPIO_InitStructure.Pin = GPIO_PIN_9;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART3;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* USARTx configured as follow:
		- BaudRate = 115200 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
	*/

	USART2_Handle.Init.WordLength = USART_WORDLENGTH_8B;
	USART2_Handle.Init.StopBits = USART_STOPBITS_1;
	USART2_Handle.Init.Parity = USART_PARITY_NONE;
	//USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART2_Handle.Init.Mode = USART_MODE_TX_RX;
	USART2_Handle.Instance = USART2;
	USART2_Handle.Init.BaudRate = global_data.param[PARAM_USART2_BAUD];
	HAL_USART_Init(&USART2_Handle);
	//__HAL_USART_ENABLE(&USART2_Handle);

	USART3_Handle.Init.WordLength = USART_WORDLENGTH_8B;
	USART3_Handle.Init.StopBits = USART_STOPBITS_1;
	USART3_Handle.Init.Parity = USART_PARITY_NONE;
	//USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART3_Handle.Init.Mode = USART_MODE_TX_RX;
	USART3_Handle.Instance = USART3;
	USART3_Handle.Init.BaudRate = global_data.param[PARAM_USART3_BAUD];
	HAL_USART_Init(&USART3_Handle);
	//__HAL_USART_ENABLE(&USART3_Handle);

	/* Enable the Transmit interrupt: this interrupt is generated when
	* the transmit data register is empty
	*/
	__HAL_USART_ENABLE_IT(&USART2_Handle, USART_IT_TXE);
	__HAL_USART_ENABLE_IT(&USART3_Handle, USART_IT_TXE);

	/* Enable the Receive interrupt: this interrupt is generated when
	* the receive data register is not empty
	*/
	__HAL_USART_ENABLE_IT(&USART2_Handle, USART_IT_RXNE);
	__HAL_USART_ENABLE_IT(&USART3_Handle, USART_IT_RXNE);
}

