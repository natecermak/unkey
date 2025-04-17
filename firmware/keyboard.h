// ==================================================================
// keyboard.h
// Declarations for keyboard polling and hardware key handling
// ==================================================================
#ifndef KEYBOARD_H
#define KEYBOARD_H

#include "chat_logic.h"

// Runs poller at 100 Hz:
static const int keyboard_poller_period_usec = 1000;

void setup_keyboard_poller();

#endif
