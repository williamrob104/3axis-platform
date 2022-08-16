#include "probe.h"

#include <stdbool.h>

#include "main.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_tim.h"

#define RESET_PULSE_WIDTH 1  // ms

static bool pulse_finish;

static void Probe_InitTimer() {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  TIM_InitStruct.Prescaler = 7199;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 10;  // change later, pulse width = ARR/10 ms
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);

  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 1;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, PROBE_PULSE_CHANNEL, &TIM_OC_InitStruct);
  LL_TIM_OC_Init(TIM1, PROBE_RESET_CHANNEL, &TIM_OC_InitStruct);

  LL_TIM_SetOnePulseMode(TIM1, LL_TIM_ONEPULSEMODE_SINGLE);

  LL_TIM_EnableAllOutputs(TIM1);

  NVIC_SetPriority(TIM1_UP_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(TIM1_UP_IRQn);
}

static void Probe_InitGPIO() {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  // probe control pins
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

  GPIO_InitStruct.Pin = PROBE_PULSE_Pin;
  LL_GPIO_Init(PROBE_PULSE_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PROBE_RESET_Pin;
  LL_GPIO_Init(PROBE_RESET_GPIO_Port, &GPIO_InitStruct);

  /* LED pin */
  LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
  GPIO_InitStruct.Pin = LED_Pin;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}

void Probe_Init() {
  Probe_InitTimer();
  Probe_InitGPIO();
}

void Probe_TIM_UP_IRQHandler() {
  LL_TIM_ClearFlag_UPDATE(TIM1);
  pulse_finish = true;
}

void Probe_Pulse(uint8_t ms) {
  pulse_finish = false;
  LL_TIM_SetAutoReload(TIM1, (uint32_t)ms * 10);
  LL_TIM_CC_EnableChannel(TIM1, PROBE_PULSE_CHANNEL);
  LL_TIM_EnableCounter(TIM1);

  while (!pulse_finish) {
    // wait for pulse finish
  }
  LL_TIM_CC_DisableChannel(TIM1, PROBE_PULSE_CHANNEL);
}

void Probe_Reset() {
  pulse_finish = false;
  LL_TIM_SetAutoReload(TIM1, (RESET_PULSE_WIDTH)*10);
  LL_TIM_CC_EnableChannel(TIM1, PROBE_RESET_CHANNEL);
  LL_TIM_EnableCounter(TIM1);

  while (!pulse_finish) {
    // wait for pulse finish
  }
  LL_TIM_CC_DisableChannel(TIM1, PROBE_RESET_CHANNEL);
}