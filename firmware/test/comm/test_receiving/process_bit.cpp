// ==================================================================
// process_bit.cpp
// Unit tests for process_bit() – ensures frequency magnitudes
// are converted to correct binary values and buffered properly
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"
#include "comm_internal.h"

void setUp(void) {
}

void tearDown(void) {
}

void test_process_bit_appends_correct_bit_and_increments_bit_index(void) {
  // Returns a pointer to the internal array of goertzel_state structs:
  goertzel_state* gs = _test_get_goertzel_state();

  // Prevents state carryover between tests:
  gs[0].y_re = 0.0f;
  gs[0].y_im = 0.0f;
  gs[1].y_re = 0.0f;
  gs[1].y_im = 0.0f;

  // Sets gs[0] to have a higher magnitude than gs[1]:
  gs[0].y_re = 3.0f;  // bin 0 has a magnitude of sqrt(3^2 + 0^2) = 3.0
  gs[0].y_im = 0.0f;
  gs[1].y_re = 1.0f;  // bin 1 has a magnitude of sqrt(1^2 + 0^2) = 1.0
  gs[1].y_im = 0.0f;

  // Resets index:
  *_test_get_bit_index() = 0;

  // Expect this to record a 0 since bin 0 has greater magnitude than bin 1:
  process_bit();

  // Assert first bit written to bitstream is a 0 (_test_get_bitstream()[0] ~ bitstream[0]):
  TEST_ASSERT_EQUAL_UINT8(0x0, _test_get_bitstream()[0]);

  // Assert that after calling process_bit(), the bit_index has been incremented from 0 to 1:
  TEST_ASSERT_EQUAL_INT(1, *_test_get_bit_index());
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_process_bit_appends_correct_bit_and_increments_bit_index);
  UNITY_END();
}

void loop() {}
