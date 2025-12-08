// ==================================================================
// main.cpp
// Entry point: initializes all modules and runs main event loop
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
  // Runs once per bit analysis window (so every 5 ms, when get_bit_from_top_frequency() updates the magnitudes):
  if (mag_ready) {
    mag_ready = false;  // Clears the flag so we only print once
    Serial.println(curr_mag_2kHz + curr_mag_2_2kHz);
  }

  poll_battery();
}

#endif