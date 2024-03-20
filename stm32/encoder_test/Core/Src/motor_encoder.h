/*
 * motor_encoder.h
 *
 *  Created on: Mar 20, 2024
 *      Author: linal
 */

#ifndef SRC_MOTOR_ENCODER_H_
#define SRC_MOTOR_ENCODER_H_

#include "stdint.h"
#include "main.h"

typedef struct{
	int16_t velocity; // anglular velocity
	int64_t position;
	uint32_t last_counter_value;
}encoder_instance;

void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim);
void reset_encoder(encoder_instance *encoder_value);

#endif /* SRC_MOTOR_ENCODER_H_ */
