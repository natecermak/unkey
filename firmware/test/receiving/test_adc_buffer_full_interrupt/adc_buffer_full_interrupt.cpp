// ==================================================================
// adc_buffer_full_interrupt.cpp
// Input: Filled ADC DMA buffer with sampled data
// Output: Copies data for processing, triggers frequency analysis and bit extraction
// Run just this test with $ pio test -e teensy40_test -f "receiving/test_adc_buffer_full_interrupt"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"
#include "goertzel.h"

static void prime_alignment_with_alternating_windows() {
  const float sr = 81920.0f, amp = 2047.0f, off = 2048.0f;
  volatile uint16_t* buf = _test_get_adc_buffer_curr_half();
  for (int w = 0; w < 7; ++w) {                 // need 7 recent windows
    float f = (w % 2 == 0) ? 2000.0f : 2200.0f; // 0,1,0,1,... for lock
    for (uint32_t i = 0; i < buffer_size; ++i) {
      float t = (float)i / sr;
      buf[i] = (uint16_t)(off + amp * sinf(2.0f * PI * f * t));
    }
    arm_dcache_flush((void*)buf, sizeof(uint16_t) * buffer_size);
    adc_buffer_full_interrupt();
  }
}

// Helper to fill DMA buffer:
void fill_dma_buffer(uint16_t value, bool alternate_values = false) {
  volatile uint16_t* dma_buf = _test_get_adc_buffer_curr_half();
  for (size_t i = 0; i < buffer_size; i++) {
    dma_buf[i] = alternate_values ? ((i % 2 == 0) ? 0 : value) : value;
  }
  // Flushes CPU cache for adc_buffer_curr_half to ensure all written values are committed to RAM2.
  // Without this, cached writes may not be visible to DMA or other code that reads from RAM:
  arm_dcache_flush((void*)dma_buf, sizeof(uint16_t) * buffer_size);
}

void setUp(void) {
  fill_dma_buffer(0);

  // Resets bit index:
  *_test_get_bit_index() = 0;

  // Resets adc_window_counter so decode will actually run:
  *_test_get_adc_window_counter() = 0;

  // Initialize Goertzel states so magnitude math is valid in tests
  goertzel_state* gs = _test_get_goertzel_state();
  initialize_goertzel(&gs[0], 2000, 81920);  // 2.0 kHz
  initialize_goertzel(&gs[1], 2200, 81920);  // 2.2 kHz
}

void tearDown(void) {}

void test_bit_extraction_triggered(void) {
  prime_alignment_with_alternating_windows();

  // Fills both halves with a strong sine wave:
  float frequency = 2200.0f;
  float sampling_rate = 81920.0f;
  float amplitude = 2047.0f;
  float offset = 2048.0f;

  volatile uint16_t* curr_half = _test_get_adc_buffer_curr_half();

  for (uint32_t i = 0; i < buffer_size; i++) {
    float t = (float)i / sampling_rate;
    float sine = sinf(2.0f * PI * frequency * t);
    curr_half[i] = (uint16_t)(offset + amplitude * sine);
  }
  arm_dcache_flush((void*)curr_half, sizeof(uint16_t) * buffer_size);
  adc_buffer_full_interrupt();

  // Fills curr_half again to simulate new DMA buffer:
  for (uint32_t i = 0; i < buffer_size; i++) {
    float t = (float)i / sampling_rate;
    float sine = sinf(2.0f * PI * frequency * t);
    curr_half[i] = (uint16_t)(offset + amplitude * sine);
  }
  // Flushes CPU cache after manually updating adc_buffer_curr_half again.
  // Required because RAM2 is cacheable — without this, adc_buffer_full_interrupt()
  // might read stale data from RAM instead of the updated values:
  arm_dcache_flush((void*)curr_half, sizeof(uint16_t) * buffer_size);

  adc_buffer_full_interrupt();

  TEST_ASSERT_EQUAL_MESSAGE(1, *_test_get_bit_index(),
    "Bit extraction failed: after feeding two 2.2 kHz buffers, bit_index should be 1 (got 0).");
}

void test_adc_buffer_with_silence_does_not_append_bit(void) {
  fill_dma_buffer(0);

  // Disarm any prior state (feed several weak windows to clear history)
  for (int i = 0; i < 6; ++i) {  // 6 is enough regardless of scan length
    adc_buffer_full_interrupt();
  }
  // Now start fresh for this assertion
  *_test_get_bit_index() = 0;
  *_test_get_adc_window_counter() = 0;

  adc_buffer_full_interrupt();
  adc_buffer_full_interrupt();

  uint16_t* copy = (uint16_t*)_test_get_adc_buffer_curr_half();
  for (size_t i = 0; i < buffer_size; i++) {
    TEST_ASSERT_EQUAL_MESSAGE(0, copy[i],
      "Silence buffer check failed: DMA copy should remain zero-filled, but a nonzero sample was found.");
  }

  int idx = *_test_get_bit_index();
  uint8_t* bits = _test_get_bitstream();

  // Silence may append zero-bits; it must not produce any '1's.
  for (int i = 0; i < idx; ++i) {
    TEST_ASSERT_EQUAL_MESSAGE(0, bits[i],
      "Silence decoding failed: bitstream should contain only 0s for silence, but a 1 was appended.");
  }
}

void test_buffer_copy_max_values(void) {
  prime_alignment_with_alternating_windows();
  uint16_t* copy = (uint16_t*)_test_get_adc_buffer_curr_half();
  fill_dma_buffer(UINT16_MAX);
  adc_buffer_full_interrupt();
  adc_buffer_full_interrupt();

  // Confirms adc_buffer_full_bit	matches max pattern:
  for (size_t i = 0; i < buffer_size; i++) {
    TEST_ASSERT_EQUAL_MESSAGE(UINT16_MAX, copy[i],
      "Max-value buffer copy failed: every DMA sample should be UINT16_MAX, but a different value was found.");
  }

  // Confirms that one bit was appended:
  TEST_ASSERT_EQUAL_MESSAGE(1, *_test_get_bit_index(),
    "Bit extraction failed: after two max-value buffers, bit_index should be 1 (got 0).");
}

void test_buffer_copy_alternating(void) {
  prime_alignment_with_alternating_windows();
  uint16_t* copy = (uint16_t*)_test_get_adc_buffer_curr_half();
  fill_dma_buffer(UINT16_MAX, true);
  adc_buffer_full_interrupt();
  adc_buffer_full_interrupt();

  // Confirms adc_buffer_full_bit matches alternating pattern:
  for (size_t i = 0; i < buffer_size; i++) {
    uint16_t expected = (i % 2 == 0) ? 0 : UINT16_MAX;
    TEST_ASSERT_EQUAL_MESSAGE(expected, copy[i],
      "Alternating buffer copy failed: DMA samples should alternate 0/UINT16_MAX, but pattern mismatch found.");
  }

  // Confirms bit extraction still ran:
  TEST_ASSERT_EQUAL_MESSAGE(1, *_test_get_bit_index(),
    "Bit extraction failed: alternating buffer should still produce a bit, but bit_index stayed 0.");
}

void test_adc_window_counter_gating(void) {
  // Sets adc_window_counter so that % SCAN_CHAIN_LENGTH != 0
  *_test_get_adc_window_counter() = 1;  // Any nonzero value that fails the mod check

  // Resets bit index:
  *_test_get_bit_index() = 0;

  adc_buffer_full_interrupt();
  adc_buffer_full_interrupt();

  // Since adc_window_counter gating skipped processing, bit_index should stay 0:
  TEST_ASSERT_EQUAL_MESSAGE(0, *_test_get_bit_index(),
    "Window gating failed: with adc_window_counter misaligned, bit_index should remain 0 (got 1).");
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_bit_extraction_triggered);
  RUN_TEST(test_adc_buffer_with_silence_does_not_append_bit);
  RUN_TEST(test_buffer_copy_max_values);
  RUN_TEST(test_buffer_copy_alternating);
  RUN_TEST(test_adc_window_counter_gating);
  UNITY_END();
}

void loop() {}
