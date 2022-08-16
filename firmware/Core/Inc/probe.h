#ifndef PROBE_H
#define PROBE_H

#include <stdint.h>

void Probe_Init();

void Probe_TIM_UP_IRQHandler();

void Probe_Pulse(uint8_t ms);

void Probe_Reset();

#endif