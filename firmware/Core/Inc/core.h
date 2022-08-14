#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>

void Core_Init();

void Core_EnqueueCommand(const uint8_t* cmd, uint32_t len);

void Core_Loop();

#endif