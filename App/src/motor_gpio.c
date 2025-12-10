/*
 * motor_gpio.c
 *
 *  Created on: Dec 10, 2025
 *      Author: andrey
 */


#include "motor_gpio.h"
#include "app_config.h" // Для MOTOR_COUNT

// Определение пинов для каждого мотора (STEP, DIR, EN)
// Здесь будет использоваться фактическое назначение пинов из main.h (MX_GPIO_Init)

// STEP-пины (GPIOA)
const uint16_t STEP_PINS[MOTOR_COUNT] = {
		GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_4, GPIO_PIN_5,
        GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_15
		};

// DIR-пины (GPIOB)
const uint16_t DIR_PINS[MOTOR_COUNT] = {
		GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3,
        GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7
		};

// EN-пины (GPIOB)
const uint16_t EN_PINS[MOTOR_COUNT] = {
		GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11,
        GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15
		};


/**
  * @brief Устанавливает направление движения для заданного мотора.
  * @param motor_id ID мотора (0-7).
  * @param direction Направление (0 для одного, 1 для другого).
  * @retval None
  */
void Motor_SetDirection(uint8_t motor_id, uint8_t direction)
{
	if (motor_id >= MOTOR_COUNT) return;
	if (direction == 1) { // Условно, вперед
		HAL_GPIO_WritePin(GPIOB, DIR_PINS[motor_id], GPIO_PIN_SET);
		}
	else { // Условно, назад
		HAL_GPIO_WritePin(GPIOB, DIR_PINS[motor_id], GPIO_PIN_RESET);
		}
}

/**
  * @brief Включает драйвер заданного мотора.
  * @param motor_id ID мотора (0-7).
  * @retval None
  */
void Motor_Enable(uint8_t motor_id)
{
	if (motor_id >= MOTOR_COUNT) return;

	// EN-пин обычно активен в низком уровне, поэтому RESET для включения
    HAL_GPIO_WritePin(GPIOB, EN_PINS[motor_id], GPIO_PIN_RESET);
    }

/**
  * @brief Отключает драйвер заданного мотора.
  * @param motor_id ID мотора (0-7).
  * @retval None
  */
void Motor_Disable(uint8_t motor_id)
{
	if (motor_id >= MOTOR_COUNT) return;
	HAL_GPIO_WritePin(GPIOB, EN_PINS[motor_id], GPIO_PIN_SET);
}

/**
  * @brief Переключает пин STEP для заданного мотора (для использования в прерывании).
  * @param motor_id ID мотора (0-7).
  * @retval None
  */
void Motor_ToggleStepPin(uint8_t motor_id)
{
	if (motor_id >= MOTOR_COUNT) return;
	// В прерывании таймера этот пин будет переключаться.
	// Пока только заглушка.
    HAL_GPIO_TogglePin(GPIOA, STEP_PINS[motor_id]);
}



