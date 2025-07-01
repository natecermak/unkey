// ==================================================================
// get_bit_from_top_frequency.cpp
// Unit tests for get_bit_from_top_frequency() – ensures frequency magnitudes
// are converted to correct binary values and buffered properly
// Run just this test with $ pio test -e teensy40_test -f "receiving/test_get_bit_from_top_frequency"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"

void setUp(void) {
}

void tearDown(void) {
}

void test_get_bit_from_top_frequency_appends_correct_bit(void) {
  // Returns a pointer to the internal array of goertzel_state structs:
  goertzel_state* gs = _test_get_goertzel_state();

  gs[0].y_re = 0.0f;
  gs[0].y_im = 0.0f;
  gs[1].y_re = 0.0f;
  gs[1].y_im = 0.0f;

  gs[0].y_re = 3.0f;
  gs[0].y_im = 0.0f;
  gs[1].y_re = 10.0f;
  gs[1].y_im = 0.0f;

  *_test_get_bit_index() = 0;

  get_bit_from_top_frequency();

  TEST_ASSERT_EQUAL_UINT8(0x1, _test_get_bitstream()[0]);
}


void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_get_bit_from_top_frequency_appends_correct_bit);
  UNITY_END();
}

void loop() {}
