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
  TEST_ASSERT_EQUAL_MESSAGE(
    1, display_called,
    "Display side effect failed: display_chat_history should be called exactly once for a valid message."
  );

  // Ensure one message was added
  ChatBufferState* state = get_chat_buffer_state();
  TEST_ASSERT_EQUAL_MESSAGE(
    1, state->chat_history_message_count,
    "Chat buffer update failed: valid message should increment chat_history_message_count to 1."
  );

  // Gets the most recent message (or the one at write index - 1, adjusting for wraparound):
  int idx = (state->message_buffer_write_index + MAX_CHAT_MESSAGES - 1) % MAX_CHAT_MESSAGES;
  message_t* msg = &state->chat_history[idx];

  TEST_ASSERT_EQUAL_STRING_MESSAGE(
    "Hello, world!", msg->text,
    "Message text storage failed: expected \"Hello, world!\" in chat history."
  );
  TEST_ASSERT_EQUAL_STRING_MESSAGE(
    RECIPIENT_VOID, msg->sender,
    "Sender assignment failed: expected RECIPIENT_VOID."
  );
  TEST_ASSERT_EQUAL_STRING_MESSAGE(
    RECIPIENT_UNKEY, msg->recipient,
    "Recipient assignment failed: expected RECIPIENT_UNKEY."
  );
}

void test_rejects_empty_message() {
  deliver_message("");

  // Should not trigger display:
  TEST_ASSERT_EQUAL_MESSAGE(
    0, display_called,
    "Empty message guard failed: display_chat_history should not be called for empty input."
  );

  ChatBufferState* state = get_chat_buffer_state();
  TEST_ASSERT_EQUAL_MESSAGE(
    0, state->chat_history_message_count,
    "Empty message guard failed: chat_history_message_count should remain 0."
  );
}

void test_rejects_too_long_message() {
  char long_msg[MAX_TEXT_LENGTH + 10];
  memset(long_msg, 'A', sizeof(long_msg));
  long_msg[sizeof(long_msg) - 1] = '\0';

  deliver_message(long_msg);

  TEST_ASSERT_EQUAL_MESSAGE(
    0, display_called,
    "Length guard failed: display_chat_history should not be called for overly long message."
  );

  ChatBufferState* state = get_chat_buffer_state();
  TEST_ASSERT_EQUAL_MESSAGE(
    0, state->chat_history_message_count,
    "Length guard failed: chat_history_message_count should remain 0 for overly long message."
  );
}

void test_rejects_message_with_framing_bytes() {
  deliver_message("test\x01message");

  TEST_ASSERT_EQUAL_MESSAGE(
    0, display_called,
    "Framing guard failed: message containing protocol framing byte should not trigger display update."
  );

  ChatBufferState* state = get_chat_buffer_state();
  TEST_ASSERT_EQUAL_MESSAGE(
    0, state->chat_history_message_count,
    "Framing guard failed: message containing protocol framing byte should not be added to chat history."
  );
}

void test_strips_escape_characters() {
  deliver_message("hi\rthe\tre");

  ChatBufferState* state = get_chat_buffer_state();
  TEST_ASSERT_EQUAL_MESSAGE(
    1, state->chat_history_message_count,
    "Escape char handling failed: valid message with escapes should still increment message count."
  );

  int idx = (state->message_buffer_write_index + MAX_CHAT_MESSAGES - 1) % MAX_CHAT_MESSAGES;
  TEST_ASSERT_EQUAL_STRING_MESSAGE(
    "hithere", state->chat_history[idx].text,
    "Escape char handling failed: message should be stored without '\\r' and '\\t'."
  );
}

void test_multiple_valid_messages_displays_all() {
  deliver_message("one");
  deliver_message("two");

  ChatBufferState* state = get_chat_buffer_state();

  // Verifies that two messages were added to the chat buffer:
  TEST_ASSERT_EQUAL_MESSAGE(
    2, state->chat_history_message_count,
    "Multiple messages failed: chat_history_message_count should equal number of valid delivered messages."
  );

  int latest_message_index = (state->message_buffer_write_index + MAX_CHAT_MESSAGES - 1) % MAX_CHAT_MESSAGES;

  // Confirm that the latest message ("two") was stored correctly:
  TEST_ASSERT_EQUAL_STRING_MESSAGE(
    "two", state->chat_history[latest_message_index].text,
    "Multiple messages failed: latest message should be \"two\"."
  );
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