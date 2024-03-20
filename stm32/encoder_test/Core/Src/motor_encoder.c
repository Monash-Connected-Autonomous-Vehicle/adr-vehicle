/*
 * motor_encoder.c
 *
 *  Created on: Mar 20, 2024
 *      Author: linal
 */
#include "motor_encoder.h"
void reset_encoder(encoder_instance *encoder_value)
{
	encoder_value->last_counter_value = 0;
	encoder_value->position = 0;
	encoder_value->velocity = 0;
}
void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim)
{
	// new counter reading, we compare this to encoder_value->last_counter_value
	uint32_t new_counter = __HAL_TIM_GET_COUNTER(htim);

	// assume velocity to be zero for first time
	static uint8_t first_time = 0;
	if (!first_time)
	{
		encoder_value->velocity = 0;
		first_time = 1;
	}
	else
	{
		// no movement
		if (new_counter == encoder_value->last_counter_value)
		{
			encoder_value->velocity = 0;
		}
		// forward movement unless overflow has occured
		else if (new_counter > encoder_value->last_counter_value)
		{
			// overflow has occured -> use __HAL_TIM_GET_AUTORELOAD(htim)
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder_value->velocity = -encoder_value->last_counter_value - (__HAL_TIM_GET_AUTORELOAD(htim) - new_counter);
			}
			else
			{
				encoder_value->velocity = new_counter - encoder_value->last_counter_value;
			}
		}
		// backward movement unless overflow has occured
		else
		{
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder_value->velocity = new_counter - encoder_value->last_counter_value;
			}
			// overflow has occured
			else
			{
				encoder_value->velocity = encoder_value->last_counter_value + (__HAL_TIM_GET_AUTORELOAD(htim) - new_counter);
			}
		}
	}
	encoder_value->last_counter_value = new_counter;
	// poaition is integration of velocity
	encoder_value->position = encoder_value->position + encoder_value->velocity;
}
