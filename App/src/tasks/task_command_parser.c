/*
 * task_command_parser.c
 *
 *  Created on: Dec 8, 2025
 *      Author: andrey
 */

#include "task_command_parser.h"
#include "main.h"             // Для HAL-функций
#include "cmsis_os.h"         // Для osDelay, osMessageQueueXxx
#include "command_protocol.h" // Для CAN_Command_t, CommandID_t
#include "app_queues.h"       // Для хэндлов очередей
#include "app_config.h"       // Для MotionCommand_t
#include "app_globals.h"



void app_start_task_command_parser(void *argument)
{
	CAN_Command_t received_command; // Буфер для принятой команды от CAN_Handler
	MotionCommand_t motion_cmd;     // Буфер для команды движения, для Motion_Controller
	CanTxFrame_t tx_response_frame; // Буфер для отправки ответов

    // --- Логика определения ID исполнителя (пока заглушка) ---
	// В будущем здесь будет чтение ID из Flash.
	// Пока для тестирования, примем, что наш ID = 0, если он еще не установлен
	if (g_performer_id == 0xFF) {
		g_performer_id = 0;
		}


	// Бесконечный цикл задачи
	for(;;)
		{
		// --- 3. Диспетчеризация команды ---
	    switch (received_command.command_id)
	        {
	        // --- Команды движения ---
	        case CMD_MOVE_ABSOLUTE:
	        case CMD_MOVE_RELATIVE:
	        case CMD_SET_SPEED:
	        case CMD_SET_ACCELERATION:
	        case CMD_STOP:
	        	// Заполняем структуру MotionCommand_t
	        	//received_command.motor_id содержит чистый ID мотора (0-15),
	        	//парсинг StdId и извлечение motor_id полностью происходит в task_can_handler.
	        	motion_cmd.motor_id = received_command.motor_id;
	        	motion_cmd.steps = received_command.payload; // Payload может быть шагами или скоростью

	        	// Отправляем команду движения в очередь Task_Motion_Controller
	            osMessageQueuePut(motion_queueHandle, &motion_cmd, 0, 0);
	         break;

	         // --- Команды для TMC-драйверов ---
	         case CMD_GET_STATUS:
	         case CMD_SET_CURRENT:
	         case CMD_ENABLE_MOTOR:
	        	 // Отправляем CAN_Command_t напрямую в очередь Task_TMC_Manager
	             osMessageQueuePut(tmc_manager_queueHandle, &received_command, 0, 0);
	             // >>> НАЧАЛО ДОБАВЛЕННОГО КОДА ДЛЯ ТЕСТИРОВАНИЯ <<<
	             // Для первого физического теста:
	             // Если это команда CMD_ENABLE_MOTOR, мы будем мигать светодиодом PC13
	             // чтобы визуально подтвердить, что команда была получена и обработана.
	             if (received_command.command_id == CMD_ENABLE_MOTOR) {
	            	 // Предполагается, что LED находится на порту C, пин 13.
	            	 // Убедитесь, что этот пин настроен как GPIO_Output в CubeMX и не используется драйвером TMC.
	            	 // Если ваш LED на другом пине, измените GPIOC и GPIO_PIN_13 соответственно.
	            	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	            	 }
	             // <<< КОНЕЦ ДОБАВЛЕННОГО КОДА ДЛЯ ТЕСТИРОВАНИЯ >>>

	         break;
	         // --- Специальные команды ---
	         case CMD_PERFORMER_ID_SET:
	         // Команда установки ID исполнителя (для провизионинга)
	         // payload содержит новый ID
	         g_performer_id = (uint8_t)received_command.payload;
	         // TODO: Реализовать запись в Flash здесь
	         // TODO: Отправить подтверждение через can_tx_queue

	         // Пример ответа:
	         tx_response_frame.header.StdId = 0x200 | (g_performer_id << 3) | 0xFF; // Ответ от исполнителя
	         tx_response_frame.header.IDE = CAN_ID_STD;
	         tx_response_frame.header.RTR = CAN_RTR_DATA;
	         tx_response_frame.header.DLC = 2;
	         tx_response_frame.data[0] = CMD_PERFORMER_ID_SET;
	         tx_response_frame.data[1] = g_performer_id; // Подтверждаем установленный ID
	         osMessageQueuePut(can_tx_queueHandle, &tx_response_frame, 0, 0);
	         break;

	         default:
	        	 // Неизвестная команда
	             // TODO: Отправить ошибку через can_tx_queue
	         break;
	            }
	    }
	osDelay(1); // Небольшая задержка, чтобы избежать "голодания" других задач
}

