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

#include "mavlink_bridge_header.h"
#include <mavlink.h>
#include "utils.h"
#include "settings.h"
#include "dcmi.h"
#include "led.h"

DCMI_HandleTypeDef DCMI_Handle;
DMA_HandleTypeDef DMA2_Handle;
TIM_HandleTypeDef TIM_Handle;
TIM_OC_InitTypeDef TIM3_OC3_Init;
I2C_HandleTypeDef I2C2_Handle;

/* counters */
volatile uint8_t image_counter = 0;
volatile uint32_t frame_counter;
volatile uint32_t time_last_frame = 0;
volatile uint32_t cycle_time = 0;
volatile uint32_t time_between_next_images;
volatile uint8_t dcmi_calibration_counter = 0;

/* state variables */
volatile uint8_t dcmi_image_buffer_memory0 = 1;
volatile uint8_t dcmi_image_buffer_memory1 = 2;
volatile uint8_t dcmi_image_buffer_unused = 3;
volatile uint8_t calibration_used;
volatile uint8_t calibration_unused;
volatile uint8_t calibration_mem0;
volatile uint8_t calibration_mem1;

/* image buffers */
uint8_t dcmi_image_buffer_8bit_1[FULL_IMAGE_SIZE];
uint8_t dcmi_image_buffer_8bit_2[FULL_IMAGE_SIZE];
uint8_t dcmi_image_buffer_8bit_3[FULL_IMAGE_SIZE];

uint32_t time_between_images;

/* extern functions */
//extern uint32_t get_boot_time_us(void);
//extern void delay(unsigned msec);

/**
 * @brief Initialize DCMI DMA and enable image capturing
 */
void enable_image_capture(void)
{
	//dcmi_clock_init();

	dcmi_hw_init();
	dcmi_dma_init(global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT]);
	//OV2640_JPEGConfig(JPEG_160x120);//OV2640_QQVGAConfig();
	//dma_it_init();
	//dcmi_it_init();
	/* Enable DMA2 stream 1 and DCMI interface then start image capture */
	//__HAL_DMA_ENABLE(DCMI_Handle.DMA_Handle);
	//__HAL_DCMI_ENABLE(&DCMI_Handle);
	//DCMI->CR |= DCMI_CR_CM;
	DCMI->CR |= DCMI_CR_CAPTURE;
}

/**
 * @brief DMA reconfiguration after changing image window
 */
void dma_reconfigure(void)
{
	dcmi_dma_disable();

	if(global_data.param[PARAM_VIDEO_ONLY])
		dcmi_dma_init(FULL_IMAGE_SIZE);
	else
		dcmi_dma_init(global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT]);

	dcmi_dma_enable();
}

/**
 * @brief Calibration image collection routine restart
 */
void dcmi_restart_calibration_routine(void)
{
	/* wait until we have all 4 parts of image */
	while(frame_counter < 4){}
	frame_counter = 0;
	dcmi_dma_enable();
}

/**
 * @brief Interrupt handler of DCMI */
void DCMI_IRQHandler(void)
{
	static uint16_t i = 0;

	if (__HAL_DCMI_GET_IT_SOURCE(&DCMI_Handle, DCMI_IT_LINE) != RESET)
	{
		__HAL_DCMI_CLEAR_FLAG(&DCMI_Handle, DCMI_IT_LINE);
	}

	if (__HAL_DCMI_GET_IT_SOURCE(&DCMI_Handle, DCMI_IT_FRAME) != RESET)
	{
		__HAL_DCMI_CLEAR_FLAG(&DCMI_Handle, DCMI_IT_FRAME);
		i++;
		if(i == 1)
		{
			i = 0;
			LEDToggle(3);
		}
	}

	if (__HAL_DCMI_GET_IT_SOURCE(&DCMI_Handle, DCMI_IT_VSYNC) != RESET)
	{
		__HAL_DCMI_CLEAR_FLAG(&DCMI_Handle, DCMI_IT_VSYNC);
	}

	return;
}

/**
 * @brief Interrupt handler of DCMI DMA stream
 */
void DMA2_Stream1_IRQHandler(void)
{
	/* transfer completed */
	if (__HAL_DMA_GET_FLAG(DCMI_Handle.DMA_Handle, DMA_FLAG_TCIF1_5) != RESET)
	{
		__HAL_DMA_CLEAR_FLAG(DCMI_Handle.DMA_Handle, DMA_FLAG_TCIF1_5);
		frame_counter++;

		if(global_data.param[PARAM_VIDEO_ONLY])
		{
			if (frame_counter >= 4)
			{
				dcmi_dma_disable();
				calibration_used = ((DCMI_Handle.DMA_Handle->Instance->CR) & DMA_SxCR_CT);
				calibration_unused = dcmi_image_buffer_unused;
				calibration_mem0 = dcmi_image_buffer_memory0;
				calibration_mem1 = dcmi_image_buffer_memory1;
			}
		}

		return;
	}

	/* transfer half completed
	 *
	 * We use three buffers and switch the buffers if dma transfer
	 * is in half state.
	 */
	if (__HAL_DMA_GET_FLAG(DCMI_Handle.DMA_Handle, DMA_FLAG_HTIF1_5) != RESET)
	{
		__HAL_DMA_CLEAR_FLAG(DCMI_Handle.DMA_Handle, DMA_FLAG_HTIF1_5);
	}

	dma_swap_buffers();
}

/**
 * @brief Swap DMA image buffer addresses
 */
void dma_swap_buffers(void)
{
	/* check which buffer is in use */
	if ((DCMI_Handle.DMA_Handle->Instance->CR) & DMA_SxCR_CT)
	{
		/* swap dcmi image buffer */
		if (dcmi_image_buffer_unused == 1)
			HAL_DMAEx_ChangeMemory(DCMI_Handle.DMA_Handle, (uint32_t) dcmi_image_buffer_8bit_1, MEMORY0);
		else if (dcmi_image_buffer_unused == 2)
			HAL_DMAEx_ChangeMemory(DCMI_Handle.DMA_Handle, (uint32_t) dcmi_image_buffer_8bit_2, MEMORY0);
		else
			HAL_DMAEx_ChangeMemory(DCMI_Handle.DMA_Handle, (uint32_t) dcmi_image_buffer_8bit_3, MEMORY0);

		int tmp_buffer = dcmi_image_buffer_memory0;
		dcmi_image_buffer_memory0 = dcmi_image_buffer_unused;
		dcmi_image_buffer_unused = tmp_buffer;
	}
	else
	{
		/* swap dcmi image buffer */
		if (dcmi_image_buffer_unused == 1)
			HAL_DMAEx_ChangeMemory(DCMI_Handle.DMA_Handle, (uint32_t) dcmi_image_buffer_8bit_1, MEMORY1);
		else if (dcmi_image_buffer_unused == 2)
			HAL_DMAEx_ChangeMemory(DCMI_Handle.DMA_Handle, (uint32_t) dcmi_image_buffer_8bit_2, MEMORY1);
		else
			HAL_DMAEx_ChangeMemory(DCMI_Handle.DMA_Handle, (uint32_t) dcmi_image_buffer_8bit_3, MEMORY1);

		int tmp_buffer = dcmi_image_buffer_memory1;
		dcmi_image_buffer_memory1 = dcmi_image_buffer_unused;
		dcmi_image_buffer_unused = tmp_buffer;
	}

	/* set next time_between_images */
	//cycle_time = get_boot_time_us() - time_last_frame;
	//time_last_frame = get_boot_time_us();

	if(image_counter) // image was not fetched jet
	{
		time_between_next_images = time_between_next_images + cycle_time;
	}
	else
	{
		time_between_next_images = cycle_time;
	}

	/* set new image true and increment frame counter*/
	image_counter += 1;

	return;
}

uint32_t get_time_between_images(void)
{
	return time_between_images;
}

uint32_t get_frame_counter(void)
{
	return frame_counter;
}

/**
 * @brief Copy image to fast RAM address
 *
 * @param current_image Current image buffer
 * @param previous_image Previous image buffer
 * @param image_size Image size of the image to copy
 * @param image_step Image to wait for (if 1 no waiting)
 */
void dma_copy_image_buffers(uint8_t ** current_image, uint8_t ** previous_image, uint16_t image_size, uint8_t image_step)
{
	/* swap image buffers */
	uint8_t * tmp_image = *current_image;
	*current_image = *previous_image;
	*previous_image = tmp_image;

	/* wait for new image if needed */
	while(image_counter < image_step) {}
	image_counter = 0;

	/* time between images */
	time_between_images = time_between_next_images;

	/* copy image */
	if (dcmi_image_buffer_unused == 1)
	{
		for (uint16_t pixel = 0; pixel < image_size; pixel++)
			(*current_image)[pixel] = (uint8_t)(dcmi_image_buffer_8bit_1[pixel]);
	}
	else if (dcmi_image_buffer_unused == 2)
	{
		for (uint16_t pixel = 0; pixel < image_size; pixel++)
			(*current_image)[pixel] = (uint8_t)(dcmi_image_buffer_8bit_2[pixel]);
	}
	else
	{
		for (uint16_t pixel = 0; pixel < image_size; pixel++)
			(*current_image)[pixel] = (uint8_t)(dcmi_image_buffer_8bit_3[pixel]);
	}
}

/**
 * @brief Send calibration image with MAVLINK over USB
 *
 * @param image_buffer_fast_1 Image buffer in fast RAM
 * @param image_buffer_fast_2 Image buffer in fast RAM
 */
void send_calibration_image(uint8_t ** image_buffer_fast_1, uint8_t ** image_buffer_fast_2) {

	/*  transmit raw 8-bit image */
	/* TODO image is too large for this transmission protocol (too much packets), but it works */
	mavlink_msg_data_transmission_handshake_send(
			MAVLINK_COMM_2,
			MAVLINK_DATA_STREAM_IMG_RAW8U,
			FULL_IMAGE_SIZE * 4,
			FULL_IMAGE_ROW_SIZE * 2,
			FULL_IMAGE_COLUMN_SIZE * 2,
			FULL_IMAGE_SIZE * 4 / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1,
			MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN,
			100);

	uint16_t frame = 0;
	uint8_t image = 0;
	uint8_t frame_buffer[MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN];

	for (int i = 0; i < FULL_IMAGE_SIZE * 4; i++)
	{

		if (i % FULL_IMAGE_SIZE == 0 && i != 0)
		{
			image++;
		}

		if (i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN == 0 && i != 0)
		{
			mavlink_msg_encapsulated_data_send(MAVLINK_COMM_2, frame, frame_buffer);
			frame++;
			HAL_Delay(2);//delay(2);
		}

		if (image == 0 )
		{
			frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t)(*image_buffer_fast_1)[i % FULL_IMAGE_SIZE];
		}
		else if (image == 1 )
		{
			frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t)(*image_buffer_fast_2)[i % FULL_IMAGE_SIZE];
		}
		else if (image == 2)
		{
			if (calibration_unused == 1)
				frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_1[i % FULL_IMAGE_SIZE];
			else if (calibration_unused == 2)
				frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_2[i % FULL_IMAGE_SIZE];
			else
				frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_3[i % FULL_IMAGE_SIZE];
		}
		else
		{
			if (calibration_used)
			{
				if (calibration_mem0 == 1)
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_1[i % FULL_IMAGE_SIZE];
				else if (calibration_mem0 == 2)
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_2[i % FULL_IMAGE_SIZE];
				else
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_3[i % FULL_IMAGE_SIZE];
			}
			else
			{
				if (calibration_mem1 == 1)
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_1[i % FULL_IMAGE_SIZE];
				else if (calibration_mem1 == 2)
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_2[i % FULL_IMAGE_SIZE];
				else
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_3[i % FULL_IMAGE_SIZE];
			}
		}
	}

	mavlink_msg_encapsulated_data_send(MAVLINK_COMM_2, frame, frame_buffer);

}

/**
 * @brief Initialize/Enable DCMI Interrupt
 */
void dcmi_it_init()
{
	HAL_NVIC_SetPriority(DCMI_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DCMI_IRQn);

	__HAL_DCMI_ENABLE_IT(&DCMI_Handle, DCMI_IT_FRAME);
	__HAL_DCMI_ENABLE_IT(&DCMI_Handle, DCMI_IT_VSYNC);
	__HAL_DCMI_ENABLE_IT(&DCMI_Handle, DCMI_IT_LINE);
	__HAL_DCMI_ENABLE_IT(&DCMI_Handle, DCMI_IT_OVF);
	__HAL_DCMI_ENABLE_IT(&DCMI_Handle, DCMI_IT_ERR);

}

/**
 * @brief Initialize/Enable DMA Interrupt
 */
void dma_it_init()
{
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

	//__HAL_DMA_ENABLE_IT(DCMI_Handle.DMA_Handle, DMA_IT_HT); // half transfer interrupt
	//__HAL_DMA_ENABLE_IT(DCMI_Handle.DMA_Handle, DMA_IT_TC); // transfer complete interrupt
}

/**
 * @brief Enable DCMI DMA stream
 */
void dcmi_dma_enable()
{
	/* Enable DMA2 stream 1 and DCMI interface then start image capture */
	//__HAL_DMA_ENABLE(DCMI_Handle.DMA_Handle);
	__HAL_DCMI_ENABLE(&DCMI_Handle);
	DCMI->CR |= (uint32_t)DCMI_CR_CAPTURE;
	//dma_it_init();
	dcmi_it_init();
}

/**
 * @brief Disable DCMI DMA stream
 */
void dcmi_dma_disable()
{
	/* Disable DMA2 stream 1 and DCMI interface then stop image capture */
	HAL_DCMI_Stop(&DCMI_Handle);
}

void reset_frame_counter()
{
	frame_counter = 0;
}

/**
 * @brief HW initialization of DCMI clock
 */
void dcmi_clock_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM3 clock enable */
	__HAL_RCC_TIM3_CLK_ENABLE();

	/* GPIOC clock enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* GPIOC Configuration:  TIM3 CH3 (PC8)  */
	GPIO_InitStructure.Pin = GPIO_PIN_8;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Time base configuration */
	TIM_Handle.Instance = TIM3;
	TIM_Handle.Init.Period = 3;
	TIM_Handle.Init.Prescaler = 0;
	TIM_Handle.Init.ClockDivision = 0;
	TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_Base_Init(&TIM_Handle);

	/* PWM1 Mode configuration: Channel3 */
	HAL_TIM_PWM_Init(&TIM_Handle);
	TIM3_OC3_Init.OCMode = TIM_OCMODE_PWM1;
	TIM3_OC3_Init.OCPolarity = TIM_OCPOLARITY_HIGH;
	TIM3_OC3_Init.Pulse = 2;

	HAL_TIM_PWM_ConfigChannel(&TIM_Handle, &TIM3_OC3_Init, TIM_CHANNEL_3);

	/* TIM3 enable counter */
	__HAL_TIM_ENABLE(&TIM_Handle);
}

/**
 * @brief HW initialization DCMI
 */
void dcmi_hw_init(void)
{
	uint16_t image_size = global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT];

	/* Reset image buffers */
	for (int i = 0; i < image_size; i++)
	{
		dcmi_image_buffer_8bit_1 [i] = 0;
		dcmi_image_buffer_8bit_2 [i] = 0;
		dcmi_image_buffer_8bit_3 [i] = 0;
	}
}

void HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable DCMI clock */
	__HAL_RCC_DCMI_CLK_ENABLE();

	/*** Configures the DCMI GPIOs to interface with the OV2640 camera module ***/
	/* Enable DCMI GPIOs clocks */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/* DCMI GPIO configuration */
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Alternate = GPIO_AF13_DCMI;

	GPIO_InitStructure.Pin = GPIO_PIN_4 | GPIO_PIN_6; // HSYNC PIXCLK
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_6| GPIO_PIN_7; //D5 VSYNC
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_6| GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_12; //D0 D1 D8 D9
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6; //D2 D3 D4 D6 D7
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

	HAL_NVIC_SetPriority(DCMI_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DCMI_IRQn);
}

/**
  * @brief  Configures DCMI/DMA to capture image from the mt9v034 camera.
  *
  * @param  buffer_size Buffer size in bytes
  */
void dcmi_dma_init(uint16_t buffer_size)
{
	reset_frame_counter();

	/* DCMI configuration */
	DCMI_Handle.Instance = DCMI;
	DCMI_Handle.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;

	DCMI_Handle.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
	//DCMI_Handle.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;

	DCMI_Handle.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
	//DCMI_Handle.Init.VSPolarity = DCMI_VSPOLARITY_LOW;

	DCMI_Handle.Init.HSPolarity = DCMI_HSPOLARITY_HIGH;
	//DCMI_Handle.Init.HSPolarity = DCMI_HSPOLARITY_LOW;

	DCMI_Handle.Init.CaptureRate = DCMI_CR_ALL_FRAME;
	DCMI_Handle.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;

	/* DCMI configuration */
	HAL_DCMI_Init(&DCMI_Handle);

	__HAL_DCMI_ENABLE_IT(&DCMI_Handle, DCMI_IT_FRAME);
	__HAL_DCMI_ENABLE_IT(&DCMI_Handle, DCMI_IT_VSYNC);
	__HAL_DCMI_ENABLE_IT(&DCMI_Handle, DCMI_IT_LINE);
	__HAL_DCMI_ENABLE_IT(&DCMI_Handle, DCMI_IT_OVF);
	__HAL_DCMI_ENABLE_IT(&DCMI_Handle, DCMI_IT_ERR);

	/* Configures the DMA2 to transfer Data from DCMI */
	/* Enable DMA2 clock */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA2 Stream1 Configuration */
	__HAL_LINKDMA(&DCMI_Handle, DMA_Handle, DMA2_Handle);

	DCMI_Handle.DMA_Handle->Instance = DMA2_Stream1;
	HAL_DMA_DeInit(DCMI_Handle.DMA_Handle);
	DCMI_Handle.DMA_Handle->Init.Channel = DMA_CHANNEL_1;
	DCMI_Handle.DMA_Handle->Init.Direction = DMA_PERIPH_TO_MEMORY;
	DCMI_Handle.DMA_Handle->Init.PeriphInc = DMA_PINC_DISABLE;
	DCMI_Handle.DMA_Handle->Init.MemInc = DMA_MINC_ENABLE;
	DCMI_Handle.DMA_Handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	DCMI_Handle.DMA_Handle->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	DCMI_Handle.DMA_Handle->Init.Mode = DMA_CIRCULAR;
	DCMI_Handle.DMA_Handle->Init.Priority = DMA_PRIORITY_HIGH;
	DCMI_Handle.DMA_Handle->Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	DCMI_Handle.DMA_Handle->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	DCMI_Handle.DMA_Handle->Init.MemBurst = DMA_MBURST_SINGLE;
	DCMI_Handle.DMA_Handle->Init.PeriphBurst = DMA_PBURST_SINGLE;

	/* DMA2 IRQ channel Configuration */
	HAL_DMA_Init(DCMI_Handle.DMA_Handle);

	__HAL_UNLOCK(DCMI_Handle.DMA_Handle);

	/* DMA start */
	HAL_DMAEx_MultiBufferStart_IT(\
			DCMI_Handle.DMA_Handle,\
			(uint32_t)&DCMI->DR,\
			(uint32_t) dcmi_image_buffer_8bit_1,\
			(uint32_t) dcmi_image_buffer_8bit_2,\
			buffer_size / 4);

	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

	//__HAL_DMA_ENABLE_IT(DCMI_Handle.DMA_Handle, DMA_IT_TC);
	//HAL_DMA_Start_IT(DCMI_Handle.DMA_Handle, DCMI->DR, (uint32_t) dcmi_image_buffer_8bit_1, buffer_size / 4);
}
