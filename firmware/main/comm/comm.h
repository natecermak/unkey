#ifndef COMM_H
#define COMM_H

void transmit_message(const char* message_to_transmit, const tx_parameters_t* tx_parameters);

void write_to_dac(uint8_t address, uint16_t value);

#endif