// ==================================================================
// decode_single_bit_from_adc_window.cpp
// Input: Filled ADC samples and current Goertzel states
// Output: Appends a decoded bit to bitstream[] and parses message
// Run just this test with $ pio test -e teensy40_test -f "receiving/test_decode_single_bit_from_adc_window"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"

void setUp(void) {}

void tearDown(void) {}

void test_decode_single_bit_from_adc_window(void) {
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_decode_single_bit_from_adc_window);
  UNITY_END();
}

void loop() {}