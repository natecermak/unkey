// ==================================================================
// main.cpp
// Arduino entry point: initializes modules in setup() and runs loop()
// ==================================================================
#include <SPI.h>
#include <Wire.h>

#include "battery.h"
#include "chat_logic.h"
#include "comm.h"
#include "config.h"
#include "display.h"
#include "goertzel.h"
#include "hardware_config.h"
#include "keyboard.h"

#ifndef UNIT_TEST

void setup() {
  Serial.begin(9600);

  // Waits up to 5s for the Serial connection (USB) to become available
  while (!Serial && millis() < 5000) ;
  delay(100);

  // SPI bus init (keyboard/display/etc.)
  SPI.begin();
  // I2C bus init (charge amplifier)
  Wire.begin();
  // Sets ADC read resolution to 12-bit
  analogReadResolution(12);

  // Testing only: timer-driven simulated incoming messages (currently disabled)
  // test_incoming_message.begin(incoming_message_callback, 1000000);

  // Module init
  setup_screen();
  setup_receiver();
  setup_transmitter();
  setup_keyboard_poller();
}

void loop() {
  process_rx_windows();

  // Disabled: seems to interfere with RX/decoding timing; revisit/verify interaction with receiving logic
  // poll_battery();
}

#endif