#include <ADC.h>
#include <DMAChannel.h>
#include <ILI9341_t3n.h>
#include <SPI.h>
#include <Wire.h>

#include "config.h"
#include "hardware_config.h"
#include "comm/goertzel.h"
#include "comm/goertzel.cpp"
#include "comm/comm.h"
#include "comm/comm.cpp"
#include "chat/chat_logic.h"
#include "chat/chat_logic.cpp"
#include "display/display.h"
#include "display/display.cpp"
#include "keyboard/keyboard.h"
#include "keyboard/keyboard.cpp"

/**
 * The main/top-level setup function that initializes the serial communication, SPI, and I2C;
 * calls other setup functions to configure the screen, receiver, transmitter, and keyboard poller.
 * Usage: The first function called in the program to initialize all components.
 */
void setup() {
  // Initializes serial communication with Teensy at baud rate of 9600 bps:
  Serial.begin(9600);
  // Checks if connection is working and waits up to 5 sec for it to happen:
  while (!Serial && millis() < 5000) ;
  delay(100);
  Serial.println("============================\nStarting setup()");

  // SPI commuincation bus for keyboard/display etc:
  SPI.begin();
  // I2C communication bus for charge amplifier:
  Wire.begin();
  // Specifies 12-bit resolution:
  analogReadResolution(12);

  test_incoming_message.begin(incoming_message_callback, 1000000);
  setup_screen();
  setup_receiver();
  setup_transmitter();
  setup_keyboard_poller();

  Serial.println("setup() complete\n============================");
}

/**
 * Continuously polls the battery voltage and updates the display with batt charge level.
 */
void loop() {
  // uint16_t val = 2048 + 2047 * sin(2*3.14159*micros()/1e6 * 1.5e3); // tryna make a number between 0 and 2^12 i.e. 12 bits

  poll_battery();
}
