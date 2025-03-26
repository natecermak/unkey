#ifndef CHAT_LOGIC_H
#define CHAT_LOGIC_H

#include "../config.h"

ChatBufferState* get_chat_buffer_state();

void _debug_print_message(message_t msg);

void _debug_print_chat_history(ChatBufferState* state);

void add_message_to_chat_history(ChatBufferState* state, const char* message_text, const char* sender, const char* recipient);

void packetize_message(const char* message, char* transmit_buffer);

void send_message(const char* message_text);

void incoming_message_callback();

#endif