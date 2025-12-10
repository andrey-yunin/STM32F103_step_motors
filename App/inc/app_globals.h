/*
 * app_globals.h
 *
 *  Created on: Dec 10, 2025
 *      Author: andrey
 */

#ifndef APP_GLOBALS_H_
#define APP_GLOBALS_H_

#include <stdint.h>
#include "motion_planner.h" // Для MotorMotionState_t
#include "tmc2209_driver.h" // Для TMC2209_Handle_t
#include "app_config.h"     // Для MOTOR_COUNT


// Определение и инициализация глобальной переменной
extern uint8_t g_performer_id; // 0xFF означает, что ID еще не установлен


// Переменная для хранения ID мотора, который сейчас движется.
// volatile - чтобы компилятор не оптимизировал ее.
// -1 означает, что ни один мотор не активен.
extern volatile int8_t g_active_motor_id;

// Объявление глобальных массивов для моторов/драйверов
extern MotorMotionState_t motor_states[MOTOR_COUNT];
extern TMC2209_Handle_t tmc_drivers[MOTOR_COUNT];


#endif /* APP_GLOBALS_H_ */
