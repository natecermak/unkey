// ==================================================================
// adc_buffer_full_interrupt.cpp
// Input: Filled ADC DMA buffer with sampled data
// Output: Copies data for processing, triggers frequency analysis and bit extraction
// Run just this test with $ pio test -e teensy40_test -f "receiving/test_adc_buffer_full_interrupt"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"

// Helper to fill DMA buffer
void fill_dma_buffer(uint16_t value, bool alternate_values = false) {
  volatile uint16_t* dma_buf = _test_get_dma_adc_buff1();
  for (size_t i = 0; i < buffer_size; i++) {
    dma_buf[i] = alternate_values ? ((i % 2 == 0) ? 0 : value) : value;
  }
}

// Helper to clear adc_buffer_copy
void clear_adc_buffer_copy() {
  uint16_t* copy = _test_get_adc_buffer_copy();
  for (size_t i = 0; i < buffer_size; i++) {
    copy[i] = 0;
  }
}

void setUp(void) {
  // Fills dma_adc_buff1 with known test data:
  fill_dma_buffer(1234);

  // Gets pointer to adc_buffer_copy and zeroes it out:
  clear_adc_buffer_copy();

  // Reset bit index
  *_test_get_bit_index() = 0;

  // Reset print_ctr so decode will actually run
  *_test_get_print_ctr() = 0;

  // Provide expected Goertzel outputs for bit extraction
  goertzel_state* gs = _test_get_goertzel_state();
  gs[1].y_re = 10; gs[1].y_im = 0;  // Strong freq 1
  gs[0].y_re = 0;  gs[0].y_im = 0;  // No freq 0

}

void tearDown(void) {}

void test_buffer_copy(void) {
  adc_buffer_full_interrupt();

  // Confirm adc_buffer_copy matches dma_adc_buff1 (1234 pattern)
  uint16_t* copy = _test_get_adc_buffer_copy();
  for (size_t i = 0; i < buffer_size; i++) {
    TEST_ASSERT_EQUAL(1234, copy[i]);
  }
}

void test_bit_extraction_triggered(void) {
  adc_buffer_full_interrupt();

  // Confirm that one bit was appended
  TEST_ASSERT_EQUAL(1, *_test_get_bit_index());
}

void test_buffer_copy_zeros(void) {
  fill_dma_buffer(0);
  clear_adc_buffer_copy();

  adc_buffer_full_interrupt();

  uint16_t* copy = _test_get_adc_buffer_copy();
  for (size_t i = 0; i < buffer_size; i++) {
    TEST_ASSERT_EQUAL(0, copy[i]);
  }

  TEST_ASSERT_EQUAL(1, *_test_get_bit_index());
}

void test_buffer_copy_max_values(void) {
  fill_dma_buffer(UINT16_MAX);
  clear_adc_buffer_copy();

  adc_buffer_full_interrupt();

  // Confirm adc_buffer_copy matches max pattern
  uint16_t* copy = _test_get_adc_buffer_copy();
  for (size_t i = 0; i < buffer_size; i++) {
    TEST_ASSERT_EQUAL(UINT16_MAX, copy[i]);
  }

  // Confirm that one bit was appended
  TEST_ASSERT_EQUAL(1, *_test_get_bit_index());
}

void test_buffer_copy_alternating(void) {
  fill_dma_buffer(UINT16_MAX, true);
  clear_adc_buffer_copy();

  adc_buffer_full_interrupt();

  // Confirm adc_buffer_copy matches the alternating pattern
  uint16_t* copy = _test_get_adc_buffer_copy();
  for (size_t i = 0; i < buffer_size; i++) {
    uint16_t expected = (i % 2 == 0) ? 0 : UINT16_MAX;
    TEST_ASSERT_EQUAL(expected, copy[i]);
  }

  // Confirm bit extraction still ran
  TEST_ASSERT_EQUAL(1, *_test_get_bit_index());
}

void test_print_ctr_gating(void) {
  // Set print_ctr so that % SCAN_CHAIN_LENGTH != 0
  *_test_get_print_ctr() = 1;  // Any nonzero value that fails the mod check

  // Reset bit index
  *_test_get_bit_index() = 0;

  adc_buffer_full_interrupt();

  // Since print_ctr gating skipped processing, bit_index should stay 0
  TEST_ASSERT_EQUAL(0, *_test_get_bit_index());
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_buffer_copy);
  RUN_TEST(test_bit_extraction_triggered);
  RUN_TEST(test_buffer_copy_zeros);
  RUN_TEST(test_buffer_copy_max_values);
  RUN_TEST(test_buffer_copy_alternating);
  RUN_TEST(test_print_ctr_gating);
  UNITY_END();
}

void loop() {}
