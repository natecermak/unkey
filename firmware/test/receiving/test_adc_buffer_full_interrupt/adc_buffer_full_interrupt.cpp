// ==================================================================
// adc_buffer_full_interrupt.cpp
// Input: Filled ADC DMA buffer with sampled data
// Output: Copies data for processing, triggers frequency analysis and bit extraction
// Run just this test with $ pio test -e teensy40_test -f "receiving/test_adc_buffer_full_interrupt"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"

// Helper to fill DMA buffer:
void fill_dma_buffer(uint16_t value, bool alternate_values = false) {
  volatile uint16_t* dma_buf = _test_get_adc_buffer_curr_half();
  for (size_t i = 0; i < buffer_size * 2; i++) {
    dma_buf[i] = alternate_values ? ((i % 2 == 0) ? 0 : value) : value;
  }
  // Flushes CPU cache for adc_buffer_curr_half to ensure all written values are committed to RAM2.
  // Without this, cached writes may not be visible to DMA or other code that reads from RAM:
  arm_dcache_flush((void*)dma_buf, sizeof(uint16_t) * buffer_size);
}

// Helper to clear adc_buffer_full_bit:
void clear_adc_buffer_full_bit	() {
  uint16_t* copy = _test_get_adc_buffer_full_bit	();
  for (size_t i = 0; i < buffer_size * 2; i++) {
    copy[i] = 0;
  }
}

void setUp(void) {
  // Fills adc_buffer_curr_half with known test data:
  fill_dma_buffer(1234);

  // Gets pointer to adc_buffer_full_bit and zeroes it out:
  clear_adc_buffer_full_bit	();

  // Resets bit index:
  *_test_get_bit_index() = 0;

  // Resets adc_window_counter so decode will actually run:
  *_test_get_adc_window_counter() = 0;

  // Provides expected Goertzel outputs for bit extraction:
  goertzel_state* gs = _test_get_goertzel_state();
  gs[1].y_re = 10; gs[1].y_im = 0;  // Strong freq 1
  gs[0].y_re = 0;  gs[0].y_im = 0;  // No freq 0

}

void tearDown(void) {}

void test_buffer_copy(void) {
  adc_buffer_full_interrupt();
  adc_buffer_full_interrupt();

  // Confirsm adc_buffer_full_bit matches adc_buffer_curr_half (1234 pattern):
  uint16_t* copy = _test_get_adc_buffer_full_bit	();
  for (size_t i = 0; i < buffer_size * 2; i++) {
    TEST_ASSERT_EQUAL(1234, copy[i]);
  }
}

void test_bit_extraction_triggered(void) {
  // Fills both halves with a strong sine wave:
  float frequency = 2200.0f;
  float sampling_rate = 81920.0f;
  float amplitude = 2047.0f;
  float offset = 2048.0f;

  uint16_t* half1 = _test_get_adc_buffer_prev_half();
  volatile uint16_t* half2 = _test_get_adc_buffer_curr_half();

  for (uint32_t i = 0; i < buffer_size; i++) {
    float t = (float)i / sampling_rate;
    float sine = sinf(2.0f * PI * frequency * t);
    half1[i] = (uint16_t)(offset + amplitude * sine);
    half2[i] = (uint16_t)(offset + amplitude * sine);
  }

  adc_buffer_full_interrupt();

  // Fills half2 again to simulate new DMA buffer:
  for (uint32_t i = 0; i < buffer_size; i++) {
    float t = (float)i / sampling_rate;
    float sine = sinf(2.0f * PI * frequency * t);
    half2[i] = (uint16_t)(offset + amplitude * sine);
  }
  // Flushes CPU cache after manually updating adc_buffer_curr_half again.
  // Required because RAM2 is cacheable — without this, adc_buffer_full_interrupt()
  // might read stale data from RAM instead of the updated values:
  arm_dcache_flush((void*)half2, sizeof(uint16_t) * buffer_size);

  adc_buffer_full_interrupt();

  TEST_ASSERT_EQUAL(1, *_test_get_bit_index());
}

void test_adc_buffer_with_silence_does_not_append_bit(void) {
  fill_dma_buffer(0);
  clear_adc_buffer_full_bit	();

  adc_buffer_full_interrupt();
  adc_buffer_full_interrupt();

  uint16_t* copy = _test_get_adc_buffer_full_bit	();
  for (size_t i = 0; i < buffer_size * 2; i++) {
    TEST_ASSERT_EQUAL(0, copy[i]);
  }

  TEST_ASSERT_EQUAL(0, *_test_get_bit_index());
}

void test_buffer_copy_max_values(void) {
  fill_dma_buffer(UINT16_MAX);
  clear_adc_buffer_full_bit	();

  adc_buffer_full_interrupt();
  adc_buffer_full_interrupt();

  // Confirms adc_buffer_full_bit	matches max pattern:
  uint16_t* copy = _test_get_adc_buffer_full_bit	();
  for (size_t i = 0; i < buffer_size * 2; i++) {
    TEST_ASSERT_EQUAL(UINT16_MAX, copy[i]);
  }

  // Confirms that one bit was appended:
  TEST_ASSERT_EQUAL(1, *_test_get_bit_index());
}

void test_buffer_copy_alternating(void) {
  fill_dma_buffer(UINT16_MAX, true);
  clear_adc_buffer_full_bit	();

  adc_buffer_full_interrupt();
  adc_buffer_full_interrupt();

  // Confirms adc_buffer_full_bit matches alternating pattern:
  uint16_t* copy = _test_get_adc_buffer_full_bit();
  for (size_t i = 0; i < buffer_size * 2; i++) {
    uint16_t expected = (i % 2 == 0) ? 0 : UINT16_MAX;
    TEST_ASSERT_EQUAL(expected, copy[i]);
  }

  // Confirms bit extraction still ran:
  TEST_ASSERT_EQUAL(1, *_test_get_bit_index());
}

void test_adc_window_counter_gating(void) {
  // Sets adc_window_counter so that % SCAN_CHAIN_LENGTH != 0
  *_test_get_adc_window_counter() = 1;  // Any nonzero value that fails the mod check

  // Resets bit index:
  *_test_get_bit_index() = 0;

  adc_buffer_full_interrupt();
  adc_buffer_full_interrupt();

  // Since adc_window_counter gating skipped processing, bit_index should stay 0:
  TEST_ASSERT_EQUAL(0, *_test_get_bit_index());
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_buffer_copy);
  RUN_TEST(test_bit_extraction_triggered);
  RUN_TEST(test_adc_buffer_with_silence_does_not_append_bit);
  RUN_TEST(test_buffer_copy_max_values);
  RUN_TEST(test_buffer_copy_alternating);
  RUN_TEST(test_adc_window_counter_gating);
  UNITY_END();
}

void loop() {}
