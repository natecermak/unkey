// ==================================================================
// comm.h
// Declarations for analog signal transmission, reception, and DSP setup
// ==================================================================
#ifndef COMM_H
#define COMM_H

#include <stdint.h>

#include "config.h"

extern uint16_t tx_display_buffer_length;

void transmit_message(const char* message_to_transmit, const tx_parameters_t* tx_parameters);

void setup_receiver();

void setup_transmitter();

#endif
