#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint8_t* buf;
  uint8_t* head;  // position to read at
  uint8_t* tail;  // position to insert at
  uint32_t capacity;
  bool full;
} RingBuffer;

void RingBuffer_Init(RingBuffer* ring_buffer, uint8_t* buf, uint32_t capacity);

bool RingBuffer_Put(RingBuffer* ring_buffer, uint8_t val);

uint8_t RingBuffer_Get(RingBuffer* ring_buffer, bool* success);

#endif