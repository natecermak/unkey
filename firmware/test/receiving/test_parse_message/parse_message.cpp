// ==================================================================
// parse_message.cpp
// Input: bitstream[] (with bit_index bits)
// Output: message (passed to deliver_message())
// Run just this test with $ pio test -e teensy40_test -f "receiving/test_parse_message"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"
#include "mock_display.h"

void setUp(void) {
  // Resets counter before each test:
  display_called = 0;

  // Redirects delivered messages to test buffer for verification during unit tests:
  _test_set_deliver_fn([](const char* msg) {
    strncpy(_test_get_delivered_message(), msg, MAX_TEXT_LENGTH);
  });
}

void tearDown(void) {}

void test_parses_message_when_valid_packet_present() {
  // Set up valid packet: [START][H][i][END]
  uint8_t test_bits[] = {
    0,0,0,0,0,0,0,1,  // 0x01
    0,0,0,0,0,0,1,0,  // 0x02
    0,1,0,0,1,0,0,0,  // 'H'
    0,1,1,0,1,0,0,1,  // 'i'
    0,0,0,0,0,0,1,1,  // 0x03
    0,0,0,0,0,1,0,0   // 0x04
  };

  int* bit_index = _test_get_bit_index();
  uint8_t* bitstream = _test_get_bitstream();

  // Copy bits into test bitstream
  for (size_t i = 0; i < sizeof(test_bits); i++) {
    bitstream[i] = test_bits[i];
  }
  *bit_index = sizeof(test_bits);

  // Reset captured message
  strcpy(_test_get_delivered_message(), "");

  parse_message();

  // Confirm that the message "Hi" was parsed and delivered
  TEST_ASSERT_EQUAL_STRING("Hi", _test_get_delivered_message());
}

void test_ignores_packet_with_no_framing() {
  // Construct bitstream that lacks proper framing bytes
  uint8_t bits[] = {
    0,1,1,0,1,0,1,0,  // 0x6A
    0,1,1,1,1,0,0,0,  // 0x78
    0,1,0,0,1,0,1,0   // 0x4A
  };

  int* bit_index = _test_get_bit_index();
  uint8_t* bitstream = _test_get_bitstream();

  for (size_t i = 0; i < sizeof(bits); i++) {
    bitstream[i] = bits[i];
  }
  *bit_index = sizeof(bits);

  // Clear any previous message
  strcpy(_test_get_delivered_message(), "");

  parse_message();

  // Confirm no message was parsed/delivered:
  TEST_ASSERT_EQUAL_STRING("", _test_get_delivered_message());
}

void test_truncates_message_that_exceeds_max_length() {
  // Construct message: [START][A x 410][END]
  const int A_COUNT = MAX_PACKET_SIZE - 4;

  static uint8_t test_bits[8 * (2 + A_COUNT + 2)];
  size_t i = 0;

  // Header bytes:
  uint8_t start[] = {0x01, 0x02};
  for (uint8_t b : start)
    for (int j = 7; j >= 0; j--) test_bits[i++] = (b >> j) & 1;

  // Message bytes:
  for (int k = 0; k < A_COUNT; k++) {
    uint8_t a = 'A';
    for (int j = 7; j >= 0; j--) test_bits[i++] = (a >> j) & 1;
  }

  // Footer bytes:
  uint8_t end[] = {0x03, 0x04};
  for (uint8_t b : end)
    for (int j = 7; j >= 0; j--) test_bits[i++] = (b >> j) & 1;

  memcpy(_test_get_bitstream(), test_bits, i);
  *_test_get_bit_index() = i;

  parse_message();

  const char* delivered = _test_get_delivered_message();
  TEST_ASSERT_EQUAL_INT(MAX_TEXT_LENGTH - 1, strlen(delivered));
  TEST_ASSERT_EQUAL_CHAR('A', delivered[0]);
}

void test_does_not_parse_message_with_only_start_header() {
  uint8_t test_bits[] = {
    0,0,0,0,0,0,0,1,
    0,0,0,0,0,0,1,0
  };

  memcpy(_test_get_bitstream(), test_bits, sizeof(test_bits));
  *_test_get_bit_index() = sizeof(test_bits);

  strcpy(_test_get_delivered_message(), "");

  parse_message();

  TEST_ASSERT_EQUAL_STRING("", _test_get_delivered_message());
}

void test_does_not_parse_message_with_only_stop_footer() {
  uint8_t test_bits[] = {
    0,0,0,0,0,0,1,1,
    0,0,0,0,0,1,0,0
  };

  memcpy(_test_get_bitstream(), test_bits, sizeof(test_bits));
  *_test_get_bit_index() = sizeof(test_bits);

  strcpy(_test_get_delivered_message(), "");

  parse_message();

  TEST_ASSERT_EQUAL_STRING("", _test_get_delivered_message());
}

void test_ignores_noise_before_header() {
  // Arbitrary bit sequence + [START][H][i][END]
  uint8_t test_bits[] = {
    1,1,1,1,1,1,1,1,
    0,0,0,0,0,0,0,1,  // 0x01
    0,0,0,0,0,0,1,0,  // 0x02
    0,1,0,0,1,0,0,0,  // 'H'
    0,1,1,0,1,0,0,1,  // 'i'
    0,0,0,0,0,0,1,1,  // 0x03
    0,0,0,0,0,1,0,0   // 0x04
  };

  memcpy(_test_get_bitstream(), test_bits, sizeof(test_bits));
  *_test_get_bit_index() = sizeof(test_bits);

  strcpy(_test_get_delivered_message(), "");

  parse_message();

  TEST_ASSERT_EQUAL_STRING("Hi", _test_get_delivered_message());
}



void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_parses_message_when_valid_packet_present);
  RUN_TEST(test_ignores_packet_with_no_framing);
  RUN_TEST(test_truncates_message_that_exceeds_max_length);
  RUN_TEST(test_does_not_parse_message_with_only_start_header);
  RUN_TEST(test_does_not_parse_message_with_only_stop_footer);
  RUN_TEST(test_ignores_noise_before_header);
  UNITY_END();
}

void loop() {}