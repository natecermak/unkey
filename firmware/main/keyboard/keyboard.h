#ifndef KEYBOARD_H
#define KEYBOARD_H

// ==================================================================
// keyboard.h
// Declarations for keyboard polling and hardware key handling
// ==================================================================

void setup_keyboard_poller();

void poll_keyboard(ChatBufferState* state);

#endif