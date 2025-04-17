// ==================================================================
// chat_logic.h
// Declarations for chat buffer state, message logging, and packetization
// ==================================================================
#ifndef CHAT_LOGIC_H
#define CHAT_LOGIC_H

#include <IntervalTimer.h>

#include "comm.h"
#include "config.h"

ChatBufferState* get_chat_buffer_state();

extern IntervalTimer test_incoming_message;

void send_message(const char* message_text);

void incoming_message_callback();

#endif
