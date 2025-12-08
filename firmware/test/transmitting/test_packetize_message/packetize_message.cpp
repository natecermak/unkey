// ==================================================================
// packetize_message.cpp
// Input: A const char* message and an output transmit buffer
// Output: Fills transmit buffer with framed message using packet delimiters (0x01 0x02 ... 0x03 0x04)
// Run just this test with $ pio test -e teensy40_test -f "transmitting/test_packetize_message"
// ==================================================================

#include <Arduino.h>
#include <unity.h>

#include "../include/chat_logic.h"

const char* MESSAGE_TO_TEST;
char TRANSMIT_BUFFER_TO_TEST[MAX_PACKET_SIZE];

void setUp(void) {
  MESSAGE_TO_TEST = "babka";
  memset(TRANSMIT_BUFFER_TO_TEST, 0, MAX_PACKET_SIZE);
}

void tearDown(void) {
}

void test_packetize_message(void) {
  // Calls packetize_message() with MESSAGE_TO_TEST and TRANSMIT_BUFFER_TO_TEST:
  packetize_message(MESSAGE_TO_TEST, TRANSMIT_BUFFER_TO_TEST);

  // Asserts that the contents of TRANSMIT_BUFFER_TO_TEST match expected output:
  uint8_t expected[] = {
    0x01, 0x02, 'b', 'a', 'b', 'k', 'a', 0x03, 0x04, '\0'
  };
  TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(
    expected, TRANSMIT_BUFFER_TO_TEST, sizeof(expected),
    "Packetization failed: transmit buffer should obvi contain header [01 02], message 'babka', and footer [03 04]."
  );
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_packetize_message);
  UNITY_END();
}

void loop() {}
