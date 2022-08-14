#ifndef MOTION_H
#define MOTION_H

#include "stm32f1xx.h"

void Motion_Init();

void Motion_TIM_IRQHandler(TIM_TypeDef* TIMx);

/**
 * @brief Move a specified displacement at a specified speed
 *
 * @param dx delta x in mm
 * @param dy delta y in mm
 * @param dz delta z in mm
 * @param speed speed in mm/min
 */
void Motion_Move(float dx, float dy, float dz, float speed, float* actual_dx,
                 float* actual_dy, float* actual_dz);

void Motion_HomeX();
void Motion_HomeY();
void Motion_HomeZ();

#endif