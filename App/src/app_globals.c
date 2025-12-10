/*
 * app_globals.c
 *
 *  Created on: Dec 10, 2025
 *      Author: andrey
 */


#include "app_globals.h"

// Определения глобальных переменных

uint8_t g_performer_id = 0xFF;
volatile int8_t g_active_motor_id = -1;
MotorMotionState_t motor_states[MOTOR_COUNT];
TMC2209_Handle_t tmc_drivers[MOTOR_COUNT];


