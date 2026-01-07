// ==================================================================
// adc_buffer_full_interrupt.cpp
// Input: Filled ADC DMA buffer with sampled data
// Output: ISR queues windows; process_rx_windows() drains queue and appends bits to window_stream
// Run just this test with:
//  pio test -e teensy40_test -f "receiving/test_adc_buffer_full_interrupt"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"
#include "goertzel.h"

static void fill_dma_with_sine(float frequency_hz) {
  const float sr = 81920.0f, amp = 2047.0f, off = 2048.0f;
  volatile uint16_t* buf = _test_get_adc_buffer_curr_half();

  for (uint32_t i = 0; i < buffer_size; ++i) {
    float t = (float)i / sr;
    buf[i] = (uint16_t)(off + amp * sinf(2.0f * PI * frequency_hz * t));
  }

  // RAM2 is cacheable; ensure CPU writes hit OCRAM before ISR reads it
  arm_dcache_flush((void*)buf, sizeof(uint16_t) * buffer_size);
}

static void fill_dma_constant(uint16_t value) {
  volatile uint16_t* buf = _test_get_adc_buffer_curr_half();
  for (uint32_t i = 0; i < buffer_size; ++i) buf[i] = value;
  arm_dcache_flush((void*)buf, sizeof(uint16_t) * buffer_size);
}

// Prime the receiver alignment by feeding alternating windows.
// IMPORTANT: after each ISR call, drain via process_rx_windows() so the 2-window queue doesn't overrun.
static void prime_alignment_with_alternating_windows() {
  for (int w = 0; w < 7; ++w) { // needs 7 recent windows for boundary detection
    float f = (w % 2 == 0) ? 2000.0f : 2200.0f;
    fill_dma_with_sine(f);
    adc_buffer_full_interrupt();
    process_rx_windows();
  }
}

void setUp(void) {
  // Reset counters/streams used by tests
  *_test_get_adc_window_counter() = 0;
  *_test_get_window_index() = 0;

  // Initialize Goertzel states so magnitude math is valid in tests
  goertzel_state* gs = _test_get_goertzel_state();
  initialize_goertzel(&gs[0], 2000, 81920);  // 2.0 kHz
  initialize_goertzel(&gs[1], 2200, 81920);  // 2.2 kHz
}

void tearDown(void) {}

void test_isr_queues_and_process_drains_one_window_into_window_stream(void) {
  prime_alignment_with_alternating_windows();

  // Keep alignment state, but start the stream fresh for this assertion:
  *_test_get_window_index() = 0;

  // Queue one strong "1" window (2200 Hz), then drain:
  fill_dma_with_sine(2200.0f);
  adc_buffer_full_interrupt();
  process_rx_windows();

  int idx = *_test_get_window_index();
  TEST_ASSERT_GREATER_OR_EQUAL_INT(1, idx);

  uint8_t* stream = _test_get_window_stream();
  TEST_ASSERT_EQUAL_UINT8(1, stream[idx - 1]);
}

void test_isr_can_queue_two_windows_before_drain_and_produces_two_stream_entries(void) {
  prime_alignment_with_alternating_windows();
  *_test_get_window_index() = 0;

  // Queue two windows (queue depth is 2), drain once
  fill_dma_with_sine(2200.0f);
  adc_buffer_full_interrupt();

  fill_dma_with_sine(2200.0f);
  adc_buffer_full_interrupt();

  process_rx_windows();

  int idx = *_test_get_window_index();
  TEST_ASSERT_EQUAL_INT(2, idx);

  uint8_t* stream = _test_get_window_stream();
  TEST_ASSERT_EQUAL_UINT8(1, stream[0]);
  TEST_ASSERT_EQUAL_UINT8(1, stream[1]);
}

void test_silence_window_does_not_append_to_window_stream(void) {
  prime_alignment_with_alternating_windows();
  *_test_get_window_index() = 0;

  // Pure silence → both mags should be under the gate → no append
  fill_dma_constant(0);
  adc_buffer_full_interrupt();
  process_rx_windows();

  TEST_ASSERT_EQUAL_INT(0, *_test_get_window_index());
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_isr_queues_and_process_drains_one_window_into_window_stream);
  RUN_TEST(test_isr_can_queue_two_windows_before_drain_and_produces_two_stream_entries);
  RUN_TEST(test_silence_window_does_not_append_to_window_stream);
  UNITY_END();
}

void loop() {}
