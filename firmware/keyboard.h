// ==================================================================
// keyboard.h
// Declarations for keyboard polling and hardware key handling
// ==================================================================
#ifndef KEYBOARD_H
#define KEYBOARD_H

#include "chat_logic.h"

void setup_keyboard_poller();

void poll_keyboard(ChatBufferState* state);

#endif