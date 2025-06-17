// ==================================================================
// parse_message.cpp
// Input: bitstream[] (with bit_index bits)
// Output: message (passed to deliver_message())
// Run just this test with $ pio test -e teensy40_test -f "receiving/test_parse_message"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"

void setUp(void) {}

void tearDown(void) {}

void test_parse_message(void) {
  // Bit pattern representing: packet start (0x01 0x02), 'A' (0x41), packet end (0x03 0x04):
  uint8_t sample_bit_sequence[] = {
    // 0x01 = 00000001
    0,0,0,0,0,0,0,1,
    // 0x02 = 00000010
    0,0,0,0,0,0,1,0,
    // 'A'  = 01000001
    0,1,0,0,0,0,0,1,
    // 0x03 = 00000011
    0,0,0,0,0,0,1,1,
    // 0x04 = 00000100
    0,0,0,0,0,1,0,0
  };

  // Loads bit sequence into the internal bitstream buffer for parse_message to process:
  for (size_t i = 0; i < sizeof(sample_bit_sequence); i++) {
    _test_get_bitstream()[i] = sample_bit_sequence[i];
  }

  // Sets bit_index to indicate how many bits are in the bitstream:
  *_test_get_bit_index() = sizeof(sample_bit_sequence);

  // Provides a test override for deliver_message that copies the delivered message into the test-accessible buffer:
  _test_set_deliver_fn([](const char* msg) {
    strncpy(_test_get_delivered_message(), msg, MAX_TEXT_LENGTH - 1);
    _test_get_delivered_message()[MAX_TEXT_LENGTH - 1] = '\0';
  });

  parse_message();

  TEST_ASSERT_EQUAL_STRING("A", _test_get_delivered_message());
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_parse_message);
  UNITY_END();
}

void loop() {}