/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Samuel Zihlmann <samuezih@ee.ethz.ch>
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

#include "led.h"

/* LED GPIOs */
const uint16_t LED_GPIO_PINS[LEDn] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
GPIO_TypeDef* LED_GPIO_PORTS[LEDn] = {GPIOD, GPIOD, GPIOD, GPIOD};

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED_ACT
  *     @arg LED_COM
  *     @arg LED_ERR
  * @retval None
  */
void LEDInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the GPIO_LED Clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;

  for(uint8_t i=0; i<LEDn; i++)
  {
	  GPIO_InitStructure.Pin = LED_GPIO_PINS[i];
	  HAL_GPIO_Init(LED_GPIO_PORTS[i], &GPIO_InitStructure);
  }

  LEDOff(0);
  LEDOff(1);
  LEDOff(2);
  LEDOff(3);

  /* Enable the GPIO_LED Clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_InitStructure.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg LED_ACT
  *     @arg LED_COM
  *     @arg LED_ERR
  * @retval None
  */
void LEDOn(Led_TypeDef Led)
{
	HAL_GPIO_WritePin(LED_GPIO_PORTS[Led], LED_GPIO_PINS[Led], GPIO_PIN_RESET);
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg LED_ACT
  *     @arg LED_COM
  *     @arg LED_ERR
  * @retval None
  */
void LEDOff(Led_TypeDef Led)
{
	HAL_GPIO_WritePin(LED_GPIO_PORTS[Led],LED_GPIO_PINS[Led], GPIO_PIN_SET);
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg LED_ACT
  *     @arg LED_COM
  *     @arg LED_ERR
  * @retval None
  */
void LEDToggle(Led_TypeDef Led)
{
	HAL_GPIO_TogglePin(LED_GPIO_PORTS[Led],LED_GPIO_PINS[Led]);
}
