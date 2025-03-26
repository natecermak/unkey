#ifndef COMM_H
#define COMM_H

void transmit_message(const char* message_to_transmit, const tx_parameters_t* tx_parameters);

void write_to_dac(uint8_t address, uint16_t value);

void setup_receiver();

void setup_transmitter();

inline void set_tx_power_enable(bool enable);

void adc_buffer_full_interrupt();

void set_charge_amplifier_gain(uint8_t gain_index);

#endif