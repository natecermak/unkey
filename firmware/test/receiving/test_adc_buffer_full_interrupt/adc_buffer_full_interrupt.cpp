// ==================================================================
// adc_buffer_full_interrupt.cpp
// Input:
// Output:
// Run just this test with $ pio test -e teensy40_test -f "receiving/test_adc_buffer_full_interrupt"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"

void setUp(void) {
  // Fills dma_adc_buff1 with known test data:
  volatile uint16_t* dma_buf = _test_get_dma_adc_buff1();
  for (size_t i = 0; i < buffer_size; i++) {
    dma_buf[i] = 1234;
  }

  // Gets pointer to adc_buffer_copy and zero it out:
  uint16_t* copy = _test_get_adc_buffer_copy();
  for (size_t i = 0; i < buffer_size; i++) {
    // Clears buffer so we can verify memcpy works:
    copy[i] = 0;
  }
}

void tearDown(void) {}

void test_adc_buffer_full_interrupt(void) {
  adc_buffer_full_interrupt();

  // Checks that adc_buffer_copy now contains the copied data:
  uint16_t* copy = _test_get_adc_buffer_copy();
  // Checks first value:
  TEST_ASSERT_EQUAL(1234, copy[0]);
  // Checks last value:
  TEST_ASSERT_EQUAL(1234, copy[buffer_size - 1]);

  // Checks that 1 bit was appended:
  TEST_ASSERT_EQUAL(1, *_test_get_bit_index());

}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_adc_buffer_full_interrupt);
  UNITY_END();
}

void loop() {}
