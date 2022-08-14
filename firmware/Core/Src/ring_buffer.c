#include "ring_buffer.h"

void RingBuffer_Init(RingBuffer* me, uint8_t* buf, uint32_t capacity) {
  me->buf = buf;
  me->head = buf;
  me->tail = buf;
  me->capacity = capacity;
  me->full = false;
}

bool RingBuffer_Put(RingBuffer* me, uint8_t val) {
  if (me->full) return false;

  // insert value
  *(me->tail) = val;

  // advance tail
  me->tail++;
  if (me->tail == me->buf + me->capacity) me->tail = me->buf;

  // check if full
  if (me->tail == me->head) me->full = true;

  return true;
}

uint8_t RingBuffer_Get(RingBuffer* me, bool* success) {
  if (me->head == me->tail && !me->full) {
    *success = false;
    return 0;
  }

  // read value
  uint8_t val = *(me->head);

  // advance head
  me->head++;
  if (me->head == me->buf + me->capacity) me->head = me->buf;

  // no full
  me->full = false;

  *success = true;
  return val;
}