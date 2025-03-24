#include "chat_logic.h"
#include <string.h>  // for strncpy
#include <time.h>    // for time()

/**
 * Copies the provided message text, sender, and recipient into the chat history buffer,
 * updates the write index, and increments the message count (up to a maximum).
 */
void add_message_to_chat_history(ChatBufferState* state, const char* message_text, const char* sender, const char* recipient) {
  message_t curr_message;
  curr_message.timestamp = time(NULL);

  // Have to copy into curr_message like this bc message_text won't be available in mem:
  strncpy(curr_message.text, message_text, MAX_TEXT_LENGTH - 1);
  strncpy(curr_message.sender, sender, MAX_NAME_LENGTH - 1);
  strncpy(curr_message.recipient, recipient, MAX_NAME_LENGTH - 1);
  curr_message.text[MAX_TEXT_LENGTH - 1] = '\0';
  curr_message.sender[MAX_NAME_LENGTH - 1] = '\0';
  curr_message.recipient[MAX_NAME_LENGTH - 1] = '\0';

  // Ring buffer logic to overwrite oldest message when buffer is exceeded:
  state->chat_history[state->message_buffer_write_index] = curr_message;
  state->message_buffer_write_index = (state->message_buffer_write_index + 1) % MAX_CHAT_MESSAGES;
  if (state->chat_history_message_count < MAX_CHAT_MESSAGES) {
    state->chat_history_message_count++;
  }
}

/**
 * Currently used to simulated staggered incoming messages.
 */
// void incoming_message_callback() {
//   const char *test_message_text = TEST_MESSAGE_TEXT;
//   add_message_to_chat_history(&chat_buffer_state, test_message_text, RECIPIENT_VOID, RECIPIENT_UNKEY);
//   display_chat_history(&chat_buffer_state);
//   incoming_message_count++;
//   if (incoming_message_count >= TESTING_MESSAGE_COUNT_LIMIT) {
//     test_incoming_message.end();
//   }
// }
