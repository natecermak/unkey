// ==================================================================
// decode_single_bit_from_adc_window.cpp
// Input: Filled ADC samples and current Goertzel states
// Output: Appends a decoded bit to bitstream[] and parses message
// Run just this test with $ pio test -e teensy40_test -f "receiving/test_decode_single_bit_from_adc_window"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"

void setUp(void) {
  // Ensures decode_single_bit_from_adc_window processes data on first call:
  *_test_get_print_ctr() = 0;

  // Resets bitstream index:
  *_test_get_bit_index() = 0;

  // Provides fake Goertzel output - strong freq 1, no freq 0:
  goertzel_state* gs = _test_get_goertzel_state();
  gs[1].y_re = 10; gs[1].y_im = 0;
  gs[0].y_re = 0;  gs[0].y_im = 0;
}

void tearDown(void) {}

void test_decode_single_bit_from_adc_window(void) {
  decode_single_bit_from_adc_window();

  TEST_ASSERT_EQUAL(1, *_test_get_bit_index());
  // Could also inspect bitstream[0] if you want:
  // TEST_ASSERT_TRUE(_test_get_bitstream()[0] == 0 || _test_get_bitstream()[0] == 1);
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_decode_single_bit_from_adc_window);
  UNITY_END();
}

void loop() {}