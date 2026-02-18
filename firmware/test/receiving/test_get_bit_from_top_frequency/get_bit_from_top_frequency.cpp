// ==================================================================
// get_bit_from_top_frequency.cpp
// Input: Goertzel frequency magnitudes in gs[]
// Output: Appends a 0 or 1 based on dominant frequency
// Run just this test with:
//  pio test -e teensy40_test -f "receiving/test_get_bit_from_top_frequency"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"

static void prime_phase_with_alternating_bins() {
  goertzel_state* gs = _test_get_goertzel_state();
  // Alternate 2.0 kHz strong (bin 0) and 2.2 kHz strong (bin 1)
  for (int i = 0; i < 7; ++i) {
    if ((i % 2) == 0) { // even -> bin0 stronger
      gs[0].y_re = 120.0f; gs[0].y_im = 0.0f;
      gs[1].y_re =   2.0f; gs[1].y_im = 0.0f;
    } else {            // odd -> bin1 stronger
      gs[0].y_re =   2.0f; gs[0].y_im = 0.0f;
      gs[1].y_re = 120.0f; gs[1].y_im = 0.0f;
    }
    get_bit_from_top_frequency(); // records window history; won’t append yet
  }
  // Reset bit index in case any previous state existed
  *_test_get_window_index() = 0;
}

void setUp(void) {
  *_test_get_window_index() = 0;
}

void tearDown(void) {
}

void test_get_bit_from_top_frequency_sets_bit_to_0_when_mag0_is_stronger(void) {
  goertzel_state* gs = _test_get_goertzel_state();

  gs[0].y_re = 100.0f; gs[0].y_im = 0.0f;
  gs[1].y_re =   2.0f; gs[1].y_im = 0.0f;

  get_bit_from_top_frequency();

  TEST_ASSERT_EQUAL_UINT8(0, _test_get_window_stream()[0]);
}

void test_get_bit_from_top_frequency_does_not_overflow_buffer(void) {
  // Pretends buffer is full:
  static const int MAX_BITS = 256;
  *_test_get_window_index() = MAX_BITS;

  // This shouldn't append anything:
  get_bit_from_top_frequency();

  // Checks that index didn't change:
  TEST_ASSERT_EQUAL(MAX_BITS, *_test_get_window_index());
}

void test_get_bit_from_top_frequency_appends_correct_bit(void) {
  prime_phase_with_alternating_bins();

  // Returns a pointer to the internal array of goertzel_state structs:
  goertzel_state* gs = _test_get_goertzel_state();

  gs[0].y_re = 0.0f; gs[0].y_im = 0.0f;
  gs[1].y_re = 0.0f; gs[1].y_im = 0.0f;

  gs[0].y_re =   3.0f; gs[0].y_im = 0.0f;
  gs[1].y_re = 100.0f; gs[1].y_im = 0.0f;

  get_bit_from_top_frequency();

  TEST_ASSERT_EQUAL_UINT8(0x1, _test_get_window_stream()[0]);
}


void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_get_bit_from_top_frequency_sets_bit_to_0_when_mag0_is_stronger);
  RUN_TEST(test_get_bit_from_top_frequency_does_not_overflow_buffer);
  RUN_TEST(test_get_bit_from_top_frequency_appends_correct_bit);
  UNITY_END();
}

void loop() {}
