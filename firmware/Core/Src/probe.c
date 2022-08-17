#include "probe.h"

#include <stdbool.h>

#include "main.h"
#include "stm32f1xx.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_tim.h"

#define RESET_PULSE_WIDTH 1  // ms
#define SENSE_PULSE_WIDTH 5  // ms
#define SAMPLE_PERIOD_PER_CHANNEL \
  14.f / 12000.f  // (14 ticks/sample) / (12000 ticks/ms)
#define NUM_SAMPLES_PER_CHANNEL \
  3500  // (3500 samples) * (14/12000 ms/sample) = (4.083 ms)

static bool pulse_finish;

uint32_t data[NUM_SAMPLES_PER_CHANNEL];

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

  LL_TIM_EnableIT_UPDATE(TIM1);
  NVIC_SetPriority(TIM1_UP_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(TIM1_UP_IRQn);
}

static void Probe_InitADC() {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_DUAL_REG_INTERL_FAST;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  LL_ADC_Init(ADC2, &ADC_InitStruct);

  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM1_CH1;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode =
      LL_ADC_REG_CONV_SINGLE;  // change later  to LL_ADC_REG_CONV_CONTINUOUS
  ADC_REG_InitStruct.DMATransfer =
      LL_ADC_REG_DMA_TRANSFER_NONE;  // change later to
                                     // LL_ADC_REG_DMA_TRANSFER_UNLIMITED
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);

  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
  LL_ADC_REG_StartConversionExtTrig(ADC2, LL_ADC_REG_TRIG_EXT_RISING);

  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0,
                                LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_0,
                                LL_ADC_SAMPLINGTIME_1CYCLE_5);

  LL_DMA_InitTypeDef DMA_InitStruct = {0};
  DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&ADC1->DR);
  DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)data;
  DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  DMA_InitStruct.NbData = 0;  // change later to select number of data
  DMA_InitStruct.Priority = LL_DMA_PRIORITY_MEDIUM;
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &DMA_InitStruct);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  NVIC_SetPriority(DMA1_Channel1_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  LL_ADC_Enable(ADC1);
  LL_ADC_Enable(ADC2);
  LL_ADC_StartCalibration(ADC1);
  LL_ADC_StartCalibration(ADC2);
  while (LL_ADC_IsCalibrationOnGoing(ADC1)) {
    // wait for ALD1 calibration complete
  }
  while (LL_ADC_IsCalibrationOnGoing(ADC2)) {
    // wait for ALD1 calibration complete
  }
}

static void Probe_InitGPIO() {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  // analog input pins
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pin = A0_Pin;
  LL_GPIO_Init(A0_GPIO_Port, &GPIO_InitStruct);

  // probe control pins (timer output)
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

  GPIO_InitStruct.Pin = PROBE_PULSE_Pin;
  LL_GPIO_Init(PROBE_PULSE_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PROBE_RESET_Pin;
  LL_GPIO_Init(PROBE_RESET_GPIO_Port, &GPIO_InitStruct);

  /* LED pin */
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

  GPIO_InitStruct.Pin = LED_Pin;
  LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}

void Probe_Init() {
  Probe_InitTimer();
  Probe_InitADC();
  Probe_InitGPIO();
}

void Probe_TIM_UP_IRQHandler() {
  LL_TIM_ClearFlag_UPDATE(TIM1);
  pulse_finish = true;
}

void Probe_DMA_IRQHandler() {
  LL_DMA_ClearFlag_TC1(DMA1);

  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);

  // stop ADC conversion
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
  LL_ADC_REG_SetContinuousMode(ADC2, LL_ADC_REG_CONV_SINGLE);
}

uint16_t* Probe_Sense(float* sample_period_ms, uint16_t* num_samples) {
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, NUM_SAMPLES_PER_CHANNEL);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_CONTINUOUS);
  LL_ADC_REG_SetContinuousMode(ADC2, LL_ADC_REG_CONV_CONTINUOUS);

  pulse_finish = false;
  LL_TIM_SetAutoReload(TIM1, (SENSE_PULSE_WIDTH)*10);
  LL_TIM_CC_EnableChannel(TIM1, PROBE_PULSE_CHANNEL);
  LL_TIM_EnableCounter(TIM1);

  while (!pulse_finish) {
    // wait for pulse finish
  }
  LL_TIM_CC_DisableChannel(TIM1, PROBE_PULSE_CHANNEL);

  if (sample_period_ms) *sample_period_ms = SAMPLE_PERIOD_PER_CHANNEL / 2;
  if (num_samples) *num_samples = NUM_SAMPLES_PER_CHANNEL * 2;
  return (uint16_t*)data;
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