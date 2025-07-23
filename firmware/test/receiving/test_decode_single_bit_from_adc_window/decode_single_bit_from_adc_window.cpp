// ==================================================================
// decode_single_bit_from_adc_window.cpp
// Input: Filled ADC samples and current Goertzel states
// Output: Appends a decoded bit to bitstream[] and parses message
// Run just this test with $ pio test -e teensy40_test -f "receiving/test_decode_single_bit_from_adc_window"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"
#include "goertzel.h"

void setUp(void) {
  // Ensures decode_single_bit_from_adc_window processes data on first call:
  *_test_get_adc_window_counter() = 0;

  // Resets bitstream index:
  *_test_get_bit_index() = 0;

  // Normally this happens in setup_receiver(), but we don’t call that in tests because it configures ADC, DMA, and other hardware:
  initialize_goertzel(&_test_get_goertzel_state()[0], 2000, 81920);  // freq 0
  initialize_goertzel(&_test_get_goertzel_state()[1], 2200, 81920);  // freq 1

}

void tearDown(void) {}

void test_decoding_single_bit_increments_bitstream_index(void) {
  *_test_get_adc_window_counter() = 0;

  // Fills ADC buffer with a strong 2200 Hz sine wave:
  float frequency = 2200.0f;
  float sampling_rate = 81920.0f;
  float amplitude = 1500.0f;
  float offset = 2048.0f;

  uint16_t* adc_buffer = _test_get_adc_buffer_full_bit();
  for (uint32_t i = 0; i < buffer_size * 2; i++) {
    float t = (float)i / sampling_rate;
    float sine = sinf(2.0f * PI * frequency * t);
    adc_buffer[i] = (uint16_t)(offset + amplitude * sine);
  }

  decode_single_bit_from_adc_window();

  TEST_ASSERT_EQUAL(1, *_test_get_bit_index());
}


void test_skips_decoding_when_gating_fails(void) {
  // Sets adc_window_counter to 1, which is not a multiple of SCAN_CHAIN_LENGTH and therefore the decoding logic shouldn't run:
  *_test_get_adc_window_counter() = 1;

  decode_single_bit_from_adc_window();

  // If the bit index hasn't been incremented, we know the decoding logic was skipped:
  TEST_ASSERT_EQUAL(0, *_test_get_bit_index());
}

void test_correct_bit_extracted_from_strongest_freq(void) {
  // Simulates Goertzel output: stronger signal at index 1 (freq 1)
  // freq 0 → magnitude = 3; freq 1 → magnitude = 10
  goertzel_state* gs = _test_get_goertzel_state();
  gs[0].y_re =   3; gs[0].y_im = 0;
  gs[1].y_re = 100; gs[1].y_im = 0;

  uint16_t* adc_buffer = _test_get_adc_buffer_full_bit	();

  float frequency = 2200.0f;
  float sampling_rate = 81920.0f;
  float amplitude = 1500.0f;
  float offset = 2048.0f;

  for (uint32_t i = 0; i < buffer_size * 2; i++) {
    float t = (float)i / sampling_rate;
    float sine = sinf(2.0f * PI * frequency * t);
    adc_buffer[i] = (uint16_t)(offset + amplitude * sine);
  }

  decode_single_bit_from_adc_window();

  uint8_t* bitstream = _test_get_bitstream();
  // Expect freq with magnitude 10 to get picked, so the first bit should be a 1:
  TEST_ASSERT_EQUAL(1, bitstream[0]);
}

void test_goertzel_states_reset_after_decode() {
  goertzel_state* gs = _test_get_goertzel_state();

  // Gives the filters some non-zero internal state:
  for (int i = 0; i < 10; i++) {
    gs[i].s = 1.0f;
    gs[i].s_z1 = 1.0f;
    gs[i].n = 5;
  }

  // Tests that after each bit is decoded, reset_goertzel is doing what it's supposed to be doing:
  decode_single_bit_from_adc_window();

  // After decoding, these internal states should be cleared:
  for (int i = 0; i < 10; i++) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, gs[i].s);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, gs[i].s_z1);
    TEST_ASSERT_EQUAL(0, gs[i].n);
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_decoding_single_bit_increments_bitstream_index);
  RUN_TEST(test_skips_decoding_when_gating_fails);
  RUN_TEST(test_correct_bit_extracted_from_strongest_freq);
  RUN_TEST(test_goertzel_states_reset_after_decode);
  UNITY_END();
}

void loop() {}