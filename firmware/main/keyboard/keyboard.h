#ifndef KEYBOARD_H
#define KEYBOARD_H

void setup_keyboard_poller();

void poll_keyboard(ChatBufferState* state);

// static int ilog2(uint64_t x);

#endif