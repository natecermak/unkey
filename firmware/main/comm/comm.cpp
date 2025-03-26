#include "comm.h"

/**
 * Transmits a message by modulating each character's bits into analog tones.
 * Note: address of 0 --> writing to channel 0 of the DAC.
 * write_to_dac(address=0, val=0): DAC receives a 24-bit message that sets the output to the minimum voltage (0V)
 * write_to_dac(address=0, val=4095): DAC receives a 24-bit message that sets the output to the maximum voltage
 */
void transmit_message(const char* message_to_transmit, const tx_parameters_t* tx_parameters) {
  for (int i = 0; message_to_transmit[i] != '\0'; i++) {
    Serial.print("Processing letter: ");
    Serial.println(message_to_transmit[i]);
    char letter = message_to_transmit[i];

    // Translates each of char's 8 bits into a corresponding frequency starting with msb:
    for (int j = 7; j >= 0; j--) {
      int bit = (letter >> j) & 1;
      // w is the angular frequency, wherein w = 2 * pi * f
      float w = (bit) ? (2 * PI * tx_parameters->freq_high / 1e6)
                      : (2 * PI * tx_parameters->freq_low / 1e6);
      // Start time for the current bit period:
      unsigned long bit_start = micros();
      unsigned long time_usec;
      uint16_t dac_value;
      // Generates a sine wave for current bit for 10 ms:
      while ((time_usec = micros() - bit_start) < tx_parameters->usec_per_bit) {
        // Gets the phase angle at curr time in microsec and scales for 12 bit DAC:
        dac_value = (uint16_t)(((sin(w * time_usec) + 1.0) / 2.0) * 409);
        noInterrupts();
        write_to_dac(0, dac_value);
        interrupts();
      }
    }
  }
}

/**
 * Sends data to a DAC via SPI: prepares a 3-byte buffer with an address and a 12-bit value, then sends the data using SPI communication.
 * MCP48CXDX1 -- 24-bit messages.
 * top byte: 5-bit address, 2 "command bits", 1 dont-care
 * bottom 2 bytes: 4 dont-care, 12 data bits
 */
void write_to_dac(uint8_t address, uint16_t value) {
  uint8_t buf[3];
  // Bits 1 and 2 must be 0 to write:
  buf[0] = (address << 3);
  buf[1] = (uint8_t)(value >> 8);
  buf[2] = (uint8_t)value;

  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dac_cs, LOW);
  SPI.transfer(buf, 3);
  digitalWrite(dac_cs, HIGH);
  SPI.endTransaction();
}