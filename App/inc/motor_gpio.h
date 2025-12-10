/*
 * motor_gpio.h
 *
 *  Created on: Dec 10, 2025
 *      Author: andrey
 */

#ifndef MOTOR_GPIO_H_
#define MOTOR_GPIO_H_

#include "main.h" // Для HAL_GPIO_WritePin, GPIO_TypeDef, GPIO_PIN_x

void Motor_SetDirection(uint8_t motor_id, uint8_t direction);
void Motor_Enable(uint8_t motor_id);
void Motor_Disable(uint8_t motor_id);
void Motor_ToggleStepPin(uint8_t motor_id); // Используется в прерывании

#endif /* MOTOR_GPIO_H_ */
