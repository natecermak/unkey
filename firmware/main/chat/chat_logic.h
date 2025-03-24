#ifndef CHAT_LOGIC_H
#define CHAT_LOGIC_H

#include "../config.h"

void add_message_to_chat_history(ChatBufferState* state, const char* message_text, const char* sender, const char* recipient);

// void incoming_message_callback();

#endif