// ==================================================================
// main.ino
// Entry point: initializes all modules and runs main event loop
// ==================================================================
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

  // Testing purposes only:
  test_incoming_message.begin(incoming_message_callback, 1000000);

  // Module setup:
  setup_screen();
  setup_receiver();
  setup_transmitter();
  setup_keyboard_poller();

  Serial.println("setup() complete\n============================");
}

void loop() {
  // uint16_t val = 2048 + 2047 * sin(2*3.14159*micros()/1e6 * 1.5e3);

  poll_battery();
}
