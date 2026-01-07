// ==================================================================
// decode_single_bit_from_adc_window.cpp
// Input: Filled ADC samples and current Goertzel states
// Output: Appends a decoded bit to window_stream[] and checks for packet completion
// Run just this test with:
//  pio test -e teensy40_test -f "receiving/test_decode_single_bit_from_adc_window"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"
#include "goertzel.h"

static uint16_t mock_adc_buffer[410];

// Helper func/fixture that feeds alternating windows so find_bit_boundaries() can lock
// because decode_single_bit_from_adc_window() depends on state built up across previous windows.
static void prime_phase_for_decode() {
  const float sr = 81920.0f, amp = 1500.0f, off = 2048.0f;

  for (int w = 0; w < 7; ++w) {
    const float f = (w % 2 == 0) ? 2000.0f : 2200.0f;
    for (uint32_t i = 0; i < buffer_size; ++i) {
      float t = (float)i / sr;
      mock_adc_buffer[i] = (uint16_t)(off + amp * sinf(2.0f * PI * f * t));
    }
    decode_single_bit_from_adc_window(mock_adc_buffer, buffer_size);
  }

  *_test_get_window_index() = 0;
}

void setUp(void) {
  // Resets indices/state the tests directly rely on:
  *_test_get_window_index() = 0;
  *_test_get_adc_window_counter() = 0;

  // Initializes the two Goertzel bins (normally done in setup_receiver()).
  initialize_goertzel(&_test_get_goertzel_state()[0], 2000, 81920);
  initialize_goertzel(&_test_get_goertzel_state()[1], 2200, 81920);
}

void tearDown(void) {}

void test_skips_decoding_when_signal_is_too_weak(void) {
  // All-zero samples => Goertzel outputs magnitude ~0 for both bins,
  // which triggers the MIN_TONE_MAGNITUDE early return path.
  for (uint32_t i = 0; i < buffer_size; ++i) {
    mock_adc_buffer[i] = 0;
  }

  decode_single_bit_from_adc_window(mock_adc_buffer, buffer_size);

  // Should not append any bits.
  TEST_ASSERT_EQUAL(0, *_test_get_window_index());
}

void test_correct_bit_extracted_after_alignment_locks(void) {
  prime_phase_for_decode();

  // Now feeds a strong 2.2kHz window. Once alignment is locked, the code appends
  // one bit per window. We expect a '1' because 2.2kHz bin should win.
  const float sr = 81920.0f, amp = 1500.0f, off = 2048.0f;
  const float f = 2200.0f;

  for (uint32_t i = 0; i < buffer_size; ++i) {
    float t = (float)i / sr;
    mock_adc_buffer[i] = (uint16_t)(off + amp * sinf(2.0f * PI * f * t));
  }

  decode_single_bit_from_adc_window(mock_adc_buffer, buffer_size);

  uint8_t* bitstream = _test_get_window_stream();
  TEST_ASSERT_EQUAL(1, *_test_get_window_index());
  TEST_ASSERT_EQUAL(1, bitstream[0]);
}

void test_goertzel_states_reset_after_decode(void) {
  goertzel_state* gs = _test_get_goertzel_state();

  for (int i = 0; i < 2; i++) {
    gs[i].s = 1.0f;
    gs[i].s_z1 = 1.0f;
    gs[i].n = 5;
  }

  // Feeds a valid non-weak window so we reach the reset_goertzel() call at end.
  // (Use a tone rather than constant, but any non-zero input will do here.)
  const float sr = 81920.0f, amp = 1500.0f, off = 2048.0f;
  const float f = 2000.0f;

  for (uint32_t i = 0; i < buffer_size; ++i) {
    float t = (float)i / sr;
    mock_adc_buffer[i] = (uint16_t)(off + amp * sinf(2.0f * PI * f * t));
  }

  decode_single_bit_from_adc_window(mock_adc_buffer, buffer_size);

  for (int i = 0; i < 2; i++) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, gs[i].s);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, gs[i].s_z1);
    TEST_ASSERT_EQUAL(0, gs[i].n);
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_skips_decoding_when_signal_is_too_weak);
  RUN_TEST(test_correct_bit_extracted_after_alignment_locks);
  RUN_TEST(test_goertzel_states_reset_after_decode);
  UNITY_END();
}

void loop() {}
