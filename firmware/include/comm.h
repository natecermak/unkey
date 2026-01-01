// ==================================================================
// comm.h
// Declarations for analog signal transmission, reception, and DSP setup
// ==================================================================
#ifndef COMM_H
#define COMM_H

#include <stdint.h>

#include "config.h"

extern uint16_t tx_display_buffer_length;

// Since these are updated within an interrupt, made these volatile-qualified to ensure the compiler is reading/writing these directly to/from memory instead of to/from a CPU register with potentially outdated data:
extern volatile float curr_mag_2kHz;
extern volatile float curr_mag_2_2kHz;
extern volatile bool mag_ready;

void transmit_message(const char* message_to_transmit, const tx_parameters_t* tx_parameters);

void setup_receiver();

void setup_transmitter();

void process_rx_windows();

// ------------------------------------------------------------------
// Testing Accessors
// ------------------------------------------------------------------

#ifdef UNIT_TEST

#include "goertzel.h"

// Declares a function that returns a pointer to the internal bitstream[] array, which is static in comm.cpp:
uint8_t* _test_get_window_stream();

// Declares a function that returns a pointer to the internal bit_index variable
int* _test_get_window_index();

// Declares a function that returns a pointer to the internal gs[] array, which holds the Goertzel filter states (used to determine bit values):
goertzel_state* _test_get_goertzel_state();

char* _test_get_delivered_message();

// Used by tests to verify what message was "delivered" without calling any hardware-dependent code:
void _test_set_deliver_fn(void (*fn)(const char*));

// Exposes a pointer to adc_window_counter so tests can modify/reset it:
uint8_t* _test_get_adc_window_counter();

extern const uint32_t buffer_size;

volatile uint16_t* _test_get_adc_buffer_curr_half();

// Otherwise local functions that only need to be exposed globally for testing:
void adc_buffer_full_interrupt();
void decode_single_bit_from_adc_window(const uint16_t* samples, size_t size);
void deliver_message(const char* message);
void get_bit_from_top_frequency();
void check_for_complete_packet();

#endif // UNIT_TEST

#endif // COMM_H