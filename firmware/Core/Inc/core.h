#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>

#include "stm32f1xx.h"

void Core_Init();

/**
 * @brief Calls the TIM_IRQHandler in "motion.h" and "probe.h"
 *
 * @param TIMx
 */
void Core_TIM_IRQHandler(TIM_TypeDef* TIMx);

/**
 * Each G-code must be seperated by newline character.
 * Supported G-code are listed below.
 *
 * G0 [F<rate>] [X<pos>] [Y<pos] [Z <pos>] | linear move, rate(mm/min),
 * pos(mm) G1 [F<rate>] [X<pos>] [Y<pos] [Z <pos>] | linear move,
 * rate(mm/min), pos(mm) G28 [X] [Y] [Z]                         | auto home
 * G90                                     | absolute positioning
 * G91                                     | relative positioning
 * M17                                     | enable steppers
 * M18                                     | disable steppers
 */
void Core_EnqueueCommand(const uint8_t* cmd, uint32_t len);

void Core_Loop();

#endif