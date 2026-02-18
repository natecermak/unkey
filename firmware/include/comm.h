// ==================================================================
// comm.h
// / Declarations for analog signal transmission/reception and DSP (FSK/Goertzel
// ==================================================================
#ifndef COMM_H
#define COMM_H

#include <stdint.h>

#include "config.h"

#define buffer_size 410
#define WINDOWS_PER_BIT 2

extern uint16_t tx_display_buffer_length;

// Updated in an ISR; volatile prevents the compiler from caching these values in registers
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

// Returns a pointer to the internal window_stream[] array (static in comm.cpp)
uint8_t* _test_get_window_stream();

// Returns a pointer to the internal window_index variable
int* _test_get_window_index();

// Returns a pointer to the internal gs[] array holding Goertzel filter states
goertzel_state* _test_get_goertzel_state();

char* _test_get_delivered_message();

// Used by tests to capture/verify delivered messages without hardware-dependent code
void _test_set_deliver_fn(void (*fn)(const char*));

// Exposes adc_window_counter so tests can modify/reset it
uint8_t* _test_get_adc_window_counter();

extern const uint8_t PACKET_START1;
extern const uint8_t PACKET_START2;
extern const uint8_t PACKET_END1;
extern const uint8_t PACKET_END2;

volatile uint16_t* _test_get_adc_dma_window();

// Normally internal functions; exposed only for unit tests
void adc_buffer_full_interrupt();
void decode_single_bit_from_adc_window(const uint16_t* samples, size_t size);
void deliver_message(const char* message);
void get_bit_from_top_frequency();
// Returns payload length on success (and fills message, null-terminated); 0 if no complete packet.
size_t check_for_complete_packet(char* message, size_t message_cap);

#endif // UNIT_TEST

#endif // COMM_H