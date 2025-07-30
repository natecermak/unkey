// ==================================================================
// chat_logic.h
// Declarations for chat buffer state, message logging, and packetization
// ==================================================================
#ifndef CHAT_LOGIC_H
#define CHAT_LOGIC_H

#include <IntervalTimer.h>

#include "comm.h"
#include "config.h"

extern IntervalTimer test_incoming_message;

ChatBufferState* get_chat_buffer_state();

void add_message_to_chat_history(ChatBufferState* state, const char* message_text, const char* sender, const char* recipient);

// TODO: packetize_message is not actually needed as a global func beyond visibility for testing package.
void packetize_message(const char* message, char* transmit_buffer);

void send_message(const char* message_text);

void incoming_message_callback();

#endif
