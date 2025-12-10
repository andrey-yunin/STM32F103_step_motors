/*
 * task_motion_controller.c
 *
 *  Created on: Dec 8, 2025
 *      Author: andrey
 */

#include "task_motion_controller.h"
#include "main.h"
#include "cmsis_os.h"
#include "motion_planner.h" // Для MotorMotionState_t
#include "app_config.h"
#include "app_globals.h"
#include "app_queues.h"

// --- Внешние хэндлы HAL ---
extern TIM_HandleTypeDef htim2; // Хэндл таймера TIM2 из main.c

// --- Прототипы функций для работы с GPIO моторов ---
// Эти функции находятся в motor_gpio.c
void Motor_SetDirection(uint8_t motor_id, uint8_t direction);
void Motor_Enable(uint8_t motor_id);

void app_start_task_motion_controller(void *argument)
{
	MotionCommand_t received_motion_cmd; // Буфер для принятой команды движения

	// --- Инициализация состояний всех 8 моторов ---
	for(uint8_t i = 0; i < MOTOR_COUNT; i++) {
		MotionPlanner_InitMotorState(&motor_states[i], 0); // Все моторы начинаются с позиции 0
		}
	// Бесконечный цикл задачи
	for(;;)
		{
		// 1. Ждем задание на движение в очереди motion_queue
		if (osMessageQueueGet(motion_queueHandle, &received_motion_cmd, NULL, osWaitForever) == osOK)
			{
			// --- Проверка, не занят ли уже контроллер движения ---
			if (g_active_motor_id != -1) {
				// Контроллер занят, мы не можем запустить новый мотор.
				// В будущем здесь можно будет ставить команды в очередь.
				// Пока просто игнорируем новую команду.
				continue;
				}
			uint8_t motor_id = received_motion_cmd.motor_id;
			if (motor_id >= MOTOR_COUNT) {
				// Неверный ID мотора, игнорируем команду
				continue;
				}
			// 2. Рассчитываем новое целевое положение и шаги
			// В нашей текущей реализации, `received_motion_cmd.steps` - это количество шагов
			// Мы передаем его в MotionPlanner_CalculateNewTarget, который вычислит направление и т.д.
			int32_t target_pos = motor_states[motor_id].current_position + received_motion_cmd.steps; // Пример для относительного движения
			int32_t steps_to_go = MotionPlanner_CalculateNewTarget(&motor_states[motor_id], target_pos);
			if (steps_to_go == 0) {
				// Движение не требуется
				continue;
				}
			// 3. Устанавливаем направление движения
			Motor_SetDirection(motor_id, motor_states[motor_id].direction);
			// 4. Включаем драйвер мотора
			Motor_Enable(motor_id);
			// 5. Устанавливаем флаг активного мотора ПЕРЕД запуском таймера
			g_active_motor_id = motor_id;
			// 6. Запускаем таймер TIM2 для генерации STEP-импульсов
			// Период будет динамически меняться в прерывании для ускорения/замедления.
			// Пока используем начальный период для максимальной скорости.
			uint32_t initial_pulse_period = 1000000 / motor_states[motor_id].max_speed_steps_per_sec;
			htim2.Instance->PSC = (SystemCoreClock / 1000000) - 1; // Предделитель для микросекунд
			htim2.Instance->ARR = initial_pulse_period - 1;       // Период для первого импульса
			htim2.Instance->CNT = 0;                              // Сбрасываем счетчик
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1); // Запускаем таймер в режиме прерываний
			// 7. Ждем в цикле, пока прерывание не сообщит о завершении движения,
			// сбросив g_active_motor_id в -1. Это простой способ синхронизации для первого теста.
			while (g_active_motor_id != -1) {
				osDelay(5); // Проверяем каждые 5 мс, чтобы не блокировать планировщик
				}
			// Как только мы вышли из этого цикла, движение завершено,
			// и задача готова принять новую команду.
			}
		}
}
