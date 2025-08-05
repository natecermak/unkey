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
  display_called = 0;
  reset_chat_buffer_state();
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

void test_rejects_empty_message() {
  deliver_message("");

  // Should not trigger display:
  TEST_ASSERT_EQUAL(0, display_called);

  ChatBufferState* state = get_chat_buffer_state();
  TEST_ASSERT_EQUAL(0, state->chat_history_message_count);
}

void test_rejects_too_long_message() {
  char long_msg[MAX_TEXT_LENGTH + 10];
  memset(long_msg, 'A', sizeof(long_msg));
  long_msg[sizeof(long_msg) - 1] = '\0';

  deliver_message(long_msg);

  TEST_ASSERT_EQUAL(0, display_called);

  ChatBufferState* state = get_chat_buffer_state();
  TEST_ASSERT_EQUAL(0, state->chat_history_message_count);
}

void test_rejects_message_with_framing_bytes() {
  deliver_message("test\x01message");

  TEST_ASSERT_EQUAL(0, display_called);

  ChatBufferState* state = get_chat_buffer_state();
  TEST_ASSERT_EQUAL(0, state->chat_history_message_count);
}

void test_strips_escape_characters() {
  deliver_message("hi\rthe\tre");

  ChatBufferState* state = get_chat_buffer_state();
  TEST_ASSERT_EQUAL(1, state->chat_history_message_count);

  int idx = (state->message_buffer_write_index + MAX_CHAT_MESSAGES - 1) % MAX_CHAT_MESSAGES;
  TEST_ASSERT_EQUAL_STRING("hithere", state->chat_history[idx].text);
}

void test_multiple_valid_messages_displays_all() {
  deliver_message("one");
  deliver_message("two");

  ChatBufferState* state = get_chat_buffer_state();

  // Verifies that two messages were added to the chat buffer:
  TEST_ASSERT_EQUAL(2, state->chat_history_message_count);

  int latest_message_index = (state->message_buffer_write_index + MAX_CHAT_MESSAGES - 1) % MAX_CHAT_MESSAGES;

  // Confirm that the latest message ("two") was stored correctly:
  TEST_ASSERT_EQUAL_STRING("two", state->chat_history[latest_message_index].text);
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_deliver_message);
  RUN_TEST(test_rejects_empty_message);
  RUN_TEST(test_rejects_too_long_message);
  RUN_TEST(test_rejects_message_with_framing_bytes);
  RUN_TEST(test_strips_escape_characters);
  RUN_TEST(test_multiple_valid_messages_displays_all);
  UNITY_END();
}

void loop() {}