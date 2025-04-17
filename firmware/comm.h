// ==================================================================
// comm.h
// Handles analog signal transmission, reception, and DSP setup
// ==================================================================
#ifndef COMM_H
#define COMM_H

#include <stdint.h>
#include "config.h"

// ADC will sample at freq of 81.92 kHz:
constexpr uint32_t adc_frequency = 81920;

// Size of buffer where ADC data will be stored:
constexpr uint32_t buffer_size = 10240;

extern uint16_t tx_display_buffer_length;

void transmit_message(const char* message_to_transmit, const tx_parameters_t* tx_parameters);

void write_to_dac(uint8_t address, uint16_t value);

void setup_receiver();

void setup_transmitter();

inline void set_tx_power_enable(bool enable);

void adc_buffer_full_interrupt();

void set_charge_amplifier_gain(uint8_t gain_index);

#endif