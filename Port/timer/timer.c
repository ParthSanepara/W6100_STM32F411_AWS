/**
 * Copyright (c) 2021 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>

#include "main.h"

#include "timer.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Timer */
extern TIM_HandleTypeDef htim1;
void (*callback_ptr)(void) = NULL;

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Timer */
void wizchip_1ms_timer_initialize(void (*callback)(void))
{
	callback_ptr = callback;
	HAL_TIM_Base_Start_IT(&htim1);
}

// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim1)
  {
		if (callback_ptr != NULL)
			callback_ptr();
  }
}

/* Delay */
void wizchip_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}