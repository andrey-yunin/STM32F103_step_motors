 /*
 * task_tmc2209_manager.c
 *
 *  Created on: Dec 8, 2025
 *      Author: andrey
 */

#include "task_tmc2209_manager.h"
#include "main.h"
#include "cmsis_os.h"
#include "tmc2209_driver.h" // Для TMC2209_Handle_t
#include "app_config.h" // Для MOTOR_COUNT
#include "app_queues.h"

// В будущем здесь будут объявления глобальных переменных
// extern osMessageQueueId_t tmc_manager_queueHandle;
// extern TMC2209_Handle_t tmc_drivers[8]; // Массив хэндлов для 8 драйверов


// Внешние переменные, объявленные в main.c
extern TMC2209_Handle_t tmc_drivers[MOTOR_COUNT];
extern UART_HandleTypeDef huart1; // Хэндл UART1
extern UART_HandleTypeDef huart2; // Хэндл UART2


void app_start_task_tmc2209_manager(void *argument)
{
	 // Ждем небольшую паузу, чтобы все остальные части системы успели запуститься
	 osDelay(100);

/*
	 // --- Инициализация всех 8-ми драйверов ---
	 for (uint8_t i = 0; i < MOTOR_COUNT; i++)
		 {
		 UART_HandleTypeDef* huart_ptr = (i < 4) ? &huart1 : &huart2; // Моторы 0-3 на UART1, 4-7 на UART2
	     uint8_t slave_addr = i % 4; // Адрес на шине (0, 1, 2, 3)

	     TMC2209_Init(&tmc_drivers[i], huart_ptr, slave_addr);

	     // Здесь мы будем вызывать функции настройки.
	     // Пока они не реализованы в tmc2209_driver.c,
	     // они будут возвращать ошибку, но это нормально.
	     // TMC2209_SetMotorCurrent(&tmc_drivers[i], 80, 50);
	     // TMC2209_SetMicrosteps(&tmc_drivers[i], 16);

		 }

*/

	 // --- Инициализация первых 4-х драйверов ---
	 for (uint8_t i = 0; i < 4; i++) // Пока только первые 4 для теста
		 {
		 UART_HandleTypeDef* huart_ptr = &huart1;
		 uint8_t slave_addr = i;
		 TMC2209_Init(&tmc_drivers[i], huart_ptr, slave_addr);

		 // Раскомментируйте эти строки
		 TMC2209_SetMotorCurrent(&tmc_drivers[i], 70, 40); // 70% рабочий ток, 40% ток удержания
		 TMC2209_SetMicrosteps(&tmc_drivers[i], 16);
		 }

	// Бесконечный цикл задачи

	for(;;)
		{
		osDelay(1000);
		}
}
