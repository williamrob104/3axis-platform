#include "core.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdlib.h>

#include "main.h"
#include "motion.h"
#include "probe.h"
#include "ring_buffer.h"
#include "usbd_cdc_if.h"

#define CMD_BUFFER_SIZE 8192
#define LINE_BUFFER_SIZE 80

RingBuffer cmd_buffer;
uint8_t line_buffer[LINE_BUFFER_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS;

static void transmitMsg(const char msg[]) {
  USBD_CDC_HandleTypeDef* hcdc =
      (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  while (hcdc->TxState != 0) {
    // wait for transfer complete
  }
  CDC_Transmit_FS((uint8_t*)msg, (uint16_t)strlen(msg));
}

void Core_Init() {
  static uint8_t _buf[CMD_BUFFER_SIZE];
  RingBuffer_Init(&cmd_buffer, _buf, CMD_BUFFER_SIZE);

  Motion_Init();
  Probe_Init();
}

void Core_TIM_IRQHandler(TIM_TypeDef* TIMx) {
  if (TIMx == TIM1)
    Probe_TIM_UP_IRQHandler();
  else
    Motion_TIM_IRQHandler(TIMx);
}

void Core_EnqueueCommand(const uint8_t* cmd, uint32_t len) {
  for (uint32_t i = 0; i < len; ++i) {
    RingBuffer_Put(&cmd_buffer, cmd[i]);
  }
}

static bool isGraph(uint8_t c) { return isgraph(c); }

static bool isNewline(uint8_t c) { return c == '\n' || c == '\r'; }

static bool Core_ExecuteGcode(const uint8_t* str);

void Core_Loop() {
  uint8_t c = 0;
  bool c_valid = false;

  // skip pass control and white-space charaters
  while (!isGraph(c)) {
    c = RingBuffer_Get(&cmd_buffer, &c_valid);
  }

  // read until newline character
  uint16_t end = 0;
  while (!isNewline(c)) {
    if (c_valid) {
      if (c == 0) c = ' ';
      line_buffer[end++] = c;
    }
    c = RingBuffer_Get(&cmd_buffer, &c_valid);
  }
  line_buffer[end] = 0;

  if (!Core_ExecuteGcode(line_buffer)) {
    char msg[LINE_BUFFER_SIZE];
    sprintf(msg, "Unknown command \"%s\"\n", line_buffer);
    transmitMsg(msg);
  }
  transmitMsg("OK\n");
}

static bool absolute_positioning = true;
static float pos_x = 0, pos_y = 0, pos_z = 0;
static float speed = 800;

static bool Core_ExecuteGcode_G0(const uint8_t* str) {
  /* linear move */
  bool has_f = false;
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  float cmd_f, cmd_x, cmd_y, cmd_z;

  const uint8_t* p = str;
  while (true) {
    while (*p != 0 && !isGraph(*p)) {
      p++;
    }
    if (*p == 0) break;

    if (*p == 'F') {
      p++;
      has_f = true;
      cmd_f = strtof((char*)p, (char**)&p);
    } else if (*p == 'X') {
      p++;
      has_x = true;
      cmd_x = strtof((char*)p, (char**)&p);
    } else if (*p == 'Y') {
      p++;
      has_y = true;
      cmd_y = strtof((char*)p, (char**)&p);
    } else if (*p == 'Z') {
      p++;
      has_z = true;
      cmd_z = strtof((char*)p, (char**)&p);
    } else {
      return false;
    }
  }

  if (has_f) speed = cmd_f;

  if (!(has_x || has_y || has_z)) return true;

  float dx = 0;
  float dy = 0;
  float dz = 0;
  if (absolute_positioning) {
    if (has_x) dx = cmd_x - pos_x;
    if (has_y) dy = cmd_y - pos_y;
    if (has_z) dz = cmd_z - pos_z;

  } else {
    if (has_x) dx = cmd_x;
    if (has_y) dy = cmd_y;
    if (has_z) dz = cmd_z;
  }

  float actual_dx, actual_dy, actual_dz;
  Motion_Move(dx, dy, dz, speed, &actual_dx, &actual_dy, &actual_dz);
  pos_x += actual_dx;
  pos_y += actual_dy;
  pos_z += actual_dz;

  return true;
}

static bool Core_ExecuteGcode_G1(const uint8_t* str) {
  return Core_ExecuteGcode_G0(str);
}

static bool Core_ExecuteGcode_G28(const uint8_t* str) {
  /* auto home */
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;

  const uint8_t* p = str;
  while (true) {
    while (*p != 0 && !isGraph(*p)) {
      p++;
    }
    if (*p == 0) break;

    if (*p == 'X') {
      p++;
      has_x = true;
    } else if (*p == 'Y') {
      p++;
      has_y = true;
    } else if (*p == 'Z') {
      p++;
      has_z = true;
    } else {
      return false;
    }
  }

  if (!(has_x || has_y || has_z)) {
    has_x = true;
    has_y = true;
    has_z = true;
  }

  if (has_x) {
    Motion_HomeX();
    pos_x = 0;
  }
  if (has_y) {
    Motion_HomeY();
    pos_y = 0;
  }
  if (has_z) {
    Motion_HomeZ();
    pos_z = 0;
  }
  return true;
}

static bool Core_ExecuteGcode_G90(const uint8_t* str) {
  /* absolute positioning */
  absolute_positioning = true;
  return true;
}

static bool Core_ExecuteGcode_G91(const uint8_t* str) {
  /* relative positioning */
  absolute_positioning = false;
  return true;
}

static bool Core_ExecuteGcode_M17(const uint8_t* str) {
  /* enable steppers */
  LL_GPIO_ResetOutputPin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin);
  return true;
}

static bool Core_ExecuteGcode_M18(const uint8_t* str) {
  /* disable steppers */
  LL_GPIO_SetOutputPin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin);
  return true;
}

static bool strEqual(const char str1[], const uint8_t* str2, uint16_t len2) {
  uint16_t i = 0;
  while (str1[i] != 0 && i < len2) {
    if (str1[i] != str2[i]) return false;
    i++;
  }
  if (str1[i] == 0 && i == len2)
    return true;
  else
    return false;
}

static bool Core_ExecuteGcode(const uint8_t* str) {
  // read till first white-space
  uint16_t i = 0;
  while (str[i] != 0 && isGraph(str[i])) {
    i++;
  }

  if (strEqual("G0", str, i))
    return Core_ExecuteGcode_G0(str + i);
  else if (strEqual("G1", str, i))
    return Core_ExecuteGcode_G1(str + i);
  else if (strEqual("G28", str, i))
    return Core_ExecuteGcode_G28(str + i);
  else if (strEqual("G90", str, i))
    return Core_ExecuteGcode_G90(str + i);
  else if (strEqual("G91", str, i))
    return Core_ExecuteGcode_G91(str + i);
  else if (strEqual("M17", str, i))
    return Core_ExecuteGcode_M17(str + i);
  else if (strEqual("M18", str, i))
    return Core_ExecuteGcode_M18(str + i);
  else
    return false;
}
