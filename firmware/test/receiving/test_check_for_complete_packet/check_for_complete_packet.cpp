// ==================================================================
// check_for_complete_packet.cpp
// Input: window_stream[] (window-level bit guesses) and window_index (count of windows)
// Output: message (passed to deliver_message())
// Run just this test with:
//  pio test -e teensy40_test -f "receiving/test_check_for_complete_packet"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"
#include "mock_display.h"

// Write a byte-aligned packet into window_stream.
// IMPORTANT: each byte spans WINDOWS_PER_BIT * 8 windows.
static void write_packet_bytes_into_window_stream(const uint8_t* bytes, size_t byte_len, int off = 0) {
  uint8_t* ws = _test_get_window_stream();

  const int BYTE_STRIDE = WINDOWS_PER_BIT * 8;

  // Clear enough space so old data doesn't interfere
  size_t total_windows = (size_t)off + (size_t)BYTE_STRIDE * byte_len;
  for (size_t i = 0; i < total_windows; i++) {
    ws[i] = 0;
  }

  for (size_t bi = 0; bi < byte_len; bi++) {
    size_t base = (size_t)off + (size_t)BYTE_STRIDE * bi;
    uint8_t byte = bytes[bi];

    // MSB-first bits
    for (int b = 0; b < 8; b++) {
      uint8_t bit = (byte >> (7 - b)) & 1;

      size_t bit_start = base + (size_t)WINDOWS_PER_BIT * b;
      for (int w = 0; w < WINDOWS_PER_BIT; w++) {
        ws[bit_start + (size_t)w] = bit;
      }
    }
  }

  *_test_get_window_index() = (int)total_windows; // window count
}

void setUp(void) {
  display_called = 0;

  _test_set_deliver_fn([](const char* msg) {
    strncpy(_test_get_delivered_message(), msg, MAX_TEXT_LENGTH);
    _test_get_delivered_message()[MAX_TEXT_LENGTH - 1] = '\0';
  });

  strcpy(_test_get_delivered_message(), "");
  *_test_get_window_index() = 0;
}

void tearDown(void) {}

void test_parses_message_when_valid_packet_present() {
  const uint8_t bytes[] = {
    PACKET_START1, PACKET_START2, 'H', 'i', PACKET_END1, PACKET_END2
  };

  write_packet_bytes_into_window_stream(bytes, sizeof(bytes));

  check_for_complete_packet();

  TEST_ASSERT_EQUAL_STRING("Hi", _test_get_delivered_message());
  TEST_ASSERT_EQUAL(0, *_test_get_window_index());
}

void test_ignores_packet_with_no_framing() {
  const uint8_t bytes[] = { 0x6A, 0x78, 0x4A };

  write_packet_bytes_into_window_stream(bytes, sizeof(bytes));

  check_for_complete_packet();

  TEST_ASSERT_EQUAL_STRING("", _test_get_delivered_message());
}

void test_rejects_message_that_exceeds_max_length() {
  static uint8_t bytes[2 + MAX_TEXT_LENGTH + 2];
  size_t idx = 0;

  bytes[idx++] = PACKET_START1;
  bytes[idx++] = PACKET_START2;
  for (int i = 0; i < MAX_TEXT_LENGTH; i++) {
    bytes[idx++] = 'A';
  }
  bytes[idx++] = PACKET_END1;
  bytes[idx++] = PACKET_END2;

  write_packet_bytes_into_window_stream(bytes, idx);

  check_for_complete_packet();

  TEST_ASSERT_EQUAL_STRING("", _test_get_delivered_message());
}

void test_does_not_deliver_with_only_start_header() {
  const uint8_t bytes[] = { PACKET_START1, PACKET_START2 };

  write_packet_bytes_into_window_stream(bytes, sizeof(bytes));

  check_for_complete_packet();

  TEST_ASSERT_EQUAL_STRING("", _test_get_delivered_message());
}

void test_does_not_deliver_with_only_stop_footer() {
  const uint8_t bytes[] = { PACKET_END1, PACKET_END2 };

  write_packet_bytes_into_window_stream(bytes, sizeof(bytes));

  check_for_complete_packet();

  TEST_ASSERT_EQUAL_STRING("", _test_get_delivered_message());
}

void test_ignores_noise_before_header() {
  const uint8_t bytes[] = {
    0xFF, PACKET_START1, PACKET_START2, 'H', 'i', PACKET_END1, PACKET_END2
  };

  write_packet_bytes_into_window_stream(bytes, sizeof(bytes));

  check_for_complete_packet();

  TEST_ASSERT_EQUAL_STRING("Hi", _test_get_delivered_message());
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_parses_message_when_valid_packet_present);
  RUN_TEST(test_ignores_packet_with_no_framing);
  RUN_TEST(test_does_not_deliver_with_only_start_header);
  RUN_TEST(test_does_not_deliver_with_only_stop_footer);
  RUN_TEST(test_ignores_noise_before_header);
  UNITY_END();
}

void loop() {}
