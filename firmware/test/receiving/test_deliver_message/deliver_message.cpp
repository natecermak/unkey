// ==================================================================
// deliver_message.cpp
// Input: A const char* message (the string to deliver)
// Output: Updates chat history, updates display with that message
// Run just this test with $ pio test -e teensy40_test -f "receiving/test_deliver_message"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "chat_logic.h"
#include "comm.h"
#include "config.h"
#include "mock_display.h"

void setUp(void) {
  // Resets counter before each test:
  display_called = 0;
}

void tearDown(void) {}

void test_deliver_message(void) {

  deliver_message("Hello, world!");

  // Asserts that display_chat_history - which touches hardware - only gets called once from deliver_message, as expected.
  // (An example of using mock verification to test a side effect behavior)
  TEST_ASSERT_EQUAL(1, display_called);

  // Ensure one message was added
  ChatBufferState* state = get_chat_buffer_state();
  TEST_ASSERT_EQUAL(1, state->chat_history_message_count);

  // Gets the most recent message (or the one at write index - 1, adjusting for wraparound):
  int idx = (state->message_buffer_write_index + MAX_CHAT_MESSAGES - 1) % MAX_CHAT_MESSAGES;
  message_t* msg = &state->chat_history[idx];

  TEST_ASSERT_EQUAL_STRING("Hello, world!", msg->text);

  // Checks sender/recipient:
  TEST_ASSERT_EQUAL_STRING(RECIPIENT_VOID, msg->sender);
  TEST_ASSERT_EQUAL_STRING(RECIPIENT_UNKEY, msg->recipient);

}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_deliver_message);
  UNITY_END();
}

void loop() {}