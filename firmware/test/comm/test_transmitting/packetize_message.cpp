// ==================================================================
// packetize_message.cpp
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
  // TODO: Can likely remove. Keeping for now for parity
}

void test_packetize_message(void) {
  // 1. Call packetize_message() with MESSAGE_TO_TEST and TRANSMIT_BUFFER_TO_TEST
  packetize_message(MESSAGE_TO_TEST, TRANSMIT_BUFFER_TO_TEST);

  // 2. Assert that the contents of TRANSMIT_BUFFER_TO_TEST match expected output
  uint8_t expected[] = {
    0x01, 0x02, 'b', 'a', 'b', 'k', 'a', 0x03, 0x04, '\0'
  };
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, TRANSMIT_BUFFER_TO_TEST, sizeof(expected));
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_packetize_message);
  UNITY_END();
}

void loop() {}
