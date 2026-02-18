// ==================================================================
// mock_display.cpp
// Provides a mock implementations from display.cpp
// for unit testing deliver_message, check_for_complete_packet, etc.
// ==================================================================
#include "chat_logic.h"

// Counter for how many times display_chat_history() was called in a test
int display_called = 0;

// Mock version of display_chat_history: increments counter instead of touching hardware
void display_chat_history(ChatBufferState* state) {
  // (void)state;
  display_called++;
}
