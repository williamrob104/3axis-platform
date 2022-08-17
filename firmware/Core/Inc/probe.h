#ifndef PROBE_H
#define PROBE_H

#include <stdint.h>

void Probe_Init();

void Probe_TIM_UP_IRQHandler();

void Probe_DMA_IRQHandler();

uint16_t* Probe_Sense(float* sample_period_ms, uint16_t* num_samples);

void Probe_Reset();

#endif