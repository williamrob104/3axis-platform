#include "motion.h"

#include <math.h>

#include "main.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_utils.h"

#define TIM_X TIM4
#define TIM_Y TIM3
#define TIM_Z TIM2

#define TIM_CLOCK SystemCoreClock

#define HOME_SPEED_FAST 800  // mm/min
#define HOME_SPEED_SLOW 150  // mm/min
#define HOME_REBOUNCE 5      // mm

static uint8_t moving;
static uint32_t remain_Nx, remain_Ny, remain_Nz;

static void Motion_InitTimer() {
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  TIM_InitStruct.Prescaler = 3599;  // change later to adjust pulse frequency
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 199;  // change later to adjust pulse frequency
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;

  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_InitStruct.CompareValue = 99;  // change later to adjust pulse width
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableIT_CC1(TIM2);
  NVIC_SetPriority(TIM2_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableIT_CC1(TIM3);
  NVIC_SetPriority(TIM3_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(TIM3_IRQn);

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableIT_CC1(TIM4);
  NVIC_SetPriority(TIM4_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(TIM4_IRQn);
}

static void Motion_InitGPIO() {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);

  /* step control pins (timer output) */
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

  GPIO_InitStruct.Pin = STEP_X_Pin;
  LL_GPIO_Init(STEP_X_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = STEP_Y_Pin;
  LL_GPIO_Init(STEP_Y_GPIO_Port, &GPIO_InitStruct);
  LL_GPIO_AF_RemapPartial_TIM3();

  GPIO_InitStruct.Pin = STEP_Z_Pin;
  LL_GPIO_Init(STEP_Z_GPIO_Port, &GPIO_InitStruct);
  LL_GPIO_AF_RemapPartial1_TIM2();

  /* zero limit pins */
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;

  GPIO_InitStruct.Pin = ZERO_X_Pin;
  LL_GPIO_Init(ZERO_X_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ZERO_Y_Pin;
  LL_GPIO_Init(ZERO_Y_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ZERO_Z_Pin;
  LL_GPIO_Init(ZERO_Z_GPIO_Port, &GPIO_InitStruct);

  /* direction control pins */
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

  GPIO_InitStruct.Pin = DIR_X_Pin;
  LL_GPIO_Init(DIR_X_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DIR_Y_Pin;
  LL_GPIO_Init(DIR_Y_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DIR_Z_Pin;
  LL_GPIO_Init(DIR_Z_GPIO_Port, &GPIO_InitStruct);

  /* motor enable control pin */
  LL_GPIO_SetOutputPin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin);  // !Enable=1
  GPIO_InitStruct.Pin = MOTOR_EN_Pin;
  LL_GPIO_Init(MOTOR_EN_GPIO_Port, &GPIO_InitStruct);

  /* LED pin */
  LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
  GPIO_InitStruct.Pin = LED_Pin;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}

void Motion_Init() {
  Motion_InitTimer();
  Motion_InitGPIO();
}

void Motion_TIM_IRQHandler(TIM_TypeDef* TIMx) {
  LL_TIM_ClearFlag_CC1(TIMx);
  if (TIMx == TIM_X) {
    if (--remain_Nx == 0) {
      LL_TIM_DisableCounter(TIMx);
      moving &= ~0b001;
    }
  } else if (TIMx == TIM_Y) {
    if (--remain_Ny == 0) {
      LL_TIM_DisableCounter(TIMx);
      moving &= ~0b010;
    }
  } else if (TIMx == TIM_Z) {
    if (--remain_Nz == 0) {
      LL_TIM_DisableCounter(TIMx);
      moving &= ~0b100;
    }
  }
}

static void ForwardKinematics(float Nx, float Ny, float Nz, float* dx,
                              float* dy, float* dz) {
  *dx = 0.16f * Nx;
  *dy = 0.16f * Ny;
  *dz = 0.0314159265358979f * Ny + 0.0314159265358979f * Nz;
}

static void InverseKinematics(float dx, float dy, float dz, float* Nx,
                              float* Ny, float* Nz) {
  *Nx = 6.25f * dx;
  *Ny = 6.25f * dy;
  *Nz = -6.25f * dy + 31.8309886183791f * dz;
}

static void SetTimerPeriod(TIM_TypeDef* TIMx, float tick_period) {
  float a = ceilf(tick_period / 65536);
  int32_t prescalar = (int32_t)a - 1;
  int32_t autoreload = (int32_t)(tick_period / a) - 1;
  if (prescalar < 0) prescalar = 0;
  if (prescalar > 65535) prescalar = 65535;
  if (autoreload < 1) autoreload = 1;
  if (autoreload > 65535) autoreload = 65535;

  LL_TIM_SetPrescaler(TIMx, prescalar);
  LL_TIM_SetAutoReload(TIMx, autoreload);
  LL_TIM_OC_SetCompareCH1(TIMx, (autoreload + 1) >> 1);
  LL_TIM_GenerateEvent_UPDATE(TIMx);
}

void Motion_Move(float dx, float dy, float dz, float speed, float* actual_dx,
                 float* actual_dy, float* actual_dz) {
  float Nx, Ny, Nz;
  InverseKinematics(dx, dy, dz, &Nx, &Ny, &Nz);
  Nx = roundf(Nx);
  Ny = roundf(Ny);
  Nz = roundf(Nz);

  int32_t Nx_i = Nx;
  int32_t Ny_i = Ny;
  int32_t Nz_i = Nz;

  speed = fabsf(speed);
  float ticks =
      sqrtf(dx * dx + dy * dy + dz * dz) * 60 / speed * (float)TIM_CLOCK;

  if (Nx_i != 0) {
    moving |= 0b001;
    if (Nx_i > 0) {
      LL_GPIO_SetOutputPin(DIR_X_GPIO_Port, DIR_X_Pin);
      remain_Nx = Nx_i;
    } else {
      LL_GPIO_ResetOutputPin(DIR_X_GPIO_Port, DIR_X_Pin);
      remain_Nx = -Nx_i;
    }
    SetTimerPeriod(TIM_X, ticks / fabsf(Nx));
  }
  if (Ny_i != 0) {
    moving |= 0b010;
    if (Ny_i > 0) {
      LL_GPIO_SetOutputPin(DIR_Y_GPIO_Port, DIR_Y_Pin);
      remain_Ny = Ny_i;
    } else {
      LL_GPIO_ResetOutputPin(DIR_Y_GPIO_Port, DIR_Y_Pin);
      remain_Ny = -Ny_i;
    }
    SetTimerPeriod(TIM_Y, ticks / fabsf(Ny));
  }
  if (Nz_i != 0) {
    moving |= 0b100;
    if (Nz_i > 0) {
      LL_GPIO_SetOutputPin(DIR_Z_GPIO_Port, DIR_Z_Pin);
      remain_Nz = Nz_i;
    } else {
      LL_GPIO_ResetOutputPin(DIR_Z_GPIO_Port, DIR_Z_Pin);
      remain_Nz = -Nz_i;
    }
    SetTimerPeriod(TIM_Z, ticks / fabsf(Nz));
  }

  // enable steppers
  LL_GPIO_ResetOutputPin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin);
  // start stepping
  if (moving & 0b001) LL_TIM_EnableCounter(TIM_X);
  if (moving & 0b010) LL_TIM_EnableCounter(TIM_Y);
  if (moving & 0b100) LL_TIM_EnableCounter(TIM_Z);

  ForwardKinematics(Nx, Ny, Nz, actual_dx, actual_dy, actual_dz);

  while (moving) {  // wait for motor steps to finish
  }
}

static void Motion_MoveVelocity(float dxdt, float dydt, float dzdt) {
  dxdt = dxdt / 60.0f;  // mm/s
  dydt = dydt / 60.0f;  // mm/s
  dzdt = dzdt / 60.0f;  // mm/s

  float dNxdt, dNydt, dNzdt;
  InverseKinematics(dxdt, dydt, dzdt, &dNxdt, &dNydt, &dNzdt);

  uint8_t move = 0;
  if (dNxdt != 0) {
    move |= 0b001;
    if (dNxdt > 0)
      LL_GPIO_SetOutputPin(DIR_X_GPIO_Port, DIR_X_Pin);
    else
      LL_GPIO_ResetOutputPin(DIR_X_GPIO_Port, DIR_X_Pin);
    SetTimerPeriod(TIM_X, (float)TIM_CLOCK / fabsf(dNxdt));
  }
  if (dNydt != 0) {
    move |= 0b010;
    if (dNydt > 0)
      LL_GPIO_SetOutputPin(DIR_Y_GPIO_Port, DIR_Y_Pin);
    else
      LL_GPIO_ResetOutputPin(DIR_Y_GPIO_Port, DIR_Y_Pin);
    SetTimerPeriod(TIM_Y, (float)TIM_CLOCK / fabsf(dNydt));
  }
  if (dNzdt != 0) {
    move |= 0b100;
    if (dNzdt > 0)
      LL_GPIO_SetOutputPin(DIR_Z_GPIO_Port, DIR_Z_Pin);
    else
      LL_GPIO_ResetOutputPin(DIR_Z_GPIO_Port, DIR_Z_Pin);
    SetTimerPeriod(TIM_Z, (float)TIM_CLOCK / fabsf(dNzdt));
  }

  // enable steppers
  LL_GPIO_ResetOutputPin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin);
  // start stepping
  if (move & 0b001) LL_TIM_EnableCounter(TIM_X);
  if (move & 0b010) LL_TIM_EnableCounter(TIM_Y);
  if (move & 0b100) LL_TIM_EnableCounter(TIM_Z);
}

static void Motion_Stop() {
  LL_TIM_DisableCounter(TIM_X);
  LL_TIM_DisableCounter(TIM_Y);
  LL_TIM_DisableCounter(TIM_Z);
}

void Motion_HomeX() {
  Motion_MoveVelocity(-HOME_SPEED_FAST, 0, 0);
  while (LL_GPIO_IsInputPinSet(ZERO_X_GPIO_Port, ZERO_X_Pin)) {
    // wait for end stop engage
  }
  Motion_Stop();
}

void Motion_HomeY() {
  Motion_MoveVelocity(0, -HOME_SPEED_FAST, 0);
  while (LL_GPIO_IsInputPinSet(ZERO_Y_GPIO_Port, ZERO_Y_Pin)) {
    // wait for end stop engage
  }
  Motion_Stop();
}

void Motion_HomeZ() {
  Motion_MoveVelocity(0, 0, -HOME_SPEED_FAST);
  while (LL_GPIO_IsInputPinSet(ZERO_Z_GPIO_Port, ZERO_Z_Pin)) {
    // wait for end stop engage
  }
  Motion_Stop();

  float a;
  Motion_Move(0, 0, HOME_REBOUNCE, HOME_SPEED_FAST, &a, &a,
              &a);  // end stop disengage

  Motion_MoveVelocity(0, 0, -HOME_SPEED_SLOW);
  while (LL_GPIO_IsInputPinSet(ZERO_Z_GPIO_Port, ZERO_Z_Pin)) {
    // wait for end stop engage
  }
  Motion_Stop();
}
