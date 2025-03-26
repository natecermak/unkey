#include "chat_logic.h"
#include "../display/display.h"
#include <string.h>  // for strncpy
#include <time.h>    // for time()

IntervalTimer test_incoming_message;

static ChatBufferState chat_buffer_state = {0, 0, 0, {}};

int incoming_message_count = 0;

void _debug_print_message(message_t msg) {
  // Serial.printf("Timestamp: %lu \n", msg.timestamp); // %d would also work, %lu is long unsigned
  // Serial.print("Sender: ");
  // Serial.println(msg.sender);
  // Serial.print("Recipient: ");
  // Serial.println(msg.recipient);
  Serial.print("Text: ");
  Serial.println(msg.text);
  Serial.println();
}

void _debug_print_chat_history(ChatBufferState* state) {
  for (int i = 0; i < state->chat_history_message_count; i++) {
    _debug_print_message(state->chat_history[i]);
  }
}

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
 * Packetizes a message by adding header (SOH and STX) and footer (ETX and EOT) bytes to the original message,
 * storing the result in transmit_buffer.
 */
void packetize_message(const char* message, char* transmit_buffer) {
  // Adds header bytes: SOH (0x01) and STX (0x02)
  transmit_buffer[0] = 0x01;
  transmit_buffer[1] = 0x02;

  // Copies message into buffer, accounting for 2 header bytes:
  strncpy(&transmit_buffer[2], message, MAX_PACKET_SIZE - 4);
  int msg_len = strlen(&transmit_buffer[2]);

  // Adds footer bytes: ETX (0x03) and EOT (0x04)
  transmit_buffer[2 + msg_len]     = 0x03;
  transmit_buffer[2 + msg_len + 1] = 0x04;

  // Null-terminates the string:
  transmit_buffer[2 + msg_len + 2] = '\0';
}

/**
 * This function packetizes the given message by adding protocol-specific header and footer bytes,
 * transmits the resulting packet using predefined transmission parameters (e.g., 2000 Hz for a 0 bit,
 * 2200 Hz for a 1 bit, with a 10 ms bit period), and then logs the original message into the chat history,
 * then redraws the display.
 */
void send_message(const char* message_text) {
  char transmit_buffer[MAX_PACKET_SIZE];
  packetize_message(message_text, transmit_buffer);
  tx_parameters_t params = {2000, 2200, 10000};
  transmit_message(transmit_buffer, &params);
  add_message_to_chat_history(&chat_buffer_state, message_text, RECIPIENT_UNKEY, RECIPIENT_VOID);
  display_chat_history(&chat_buffer_state);
}

/**
 * Currently used to simulated staggered incoming messages.
 */
void incoming_message_callback() {
  const char *test_message_text = TEST_MESSAGE_TEXT;
  add_message_to_chat_history(&chat_buffer_state, test_message_text, RECIPIENT_VOID, RECIPIENT_UNKEY);
  display_chat_history(&chat_buffer_state);
  incoming_message_count++;
  if (incoming_message_count >= TESTING_MESSAGE_COUNT_LIMIT) {
    test_incoming_message.end();
  }
}

ChatBufferState* get_chat_buffer_state() {
  return &chat_buffer_state;
}
