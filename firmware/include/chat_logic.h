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

// TODO: packetize_message is not actually needed as a global func beyond visibility for testing package.
// May want to remove this decl from header at some point
void packetize_message(const char* message, char* transmit_buffer);

void send_message(const char* message_text);

void incoming_message_callback();

#endif
