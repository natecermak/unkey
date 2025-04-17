// ==================================================================
// display.cpp
// Handles screen setup and message rendering
// ==================================================================
#include "display.h"
#include "hardware_config.h"
#include "config.h"
#include "comm.h"

// ------------------------------------------------------------------
// External globals
// ------------------------------------------------------------------

extern char tx_display_buffer[];
// extern uint16_t tx_display_buffer_length;

// ------------------------------------------------------------------
// State
// ------------------------------------------------------------------

bool screen_on;
unsigned long time_of_last_battery_read_ms;
const unsigned long BATTERY_READ_PERIOD_MS = 1000;

// Initializes the display using pin numbers defined above, which get passed to the constructor:
ILI9341_t3n tft = ILI9341_t3n(tft_cs, tft_dc, tft_reset, tft_mosi, tft_sck, tft_miso);

// ------------------------------------------------------------------
// Functions
// ------------------------------------------------------------------

/**
 * Draws message character content (incorporating line breaks and text wrapping) in the chat history box for a given message.
 * A single message is defined as whatever text chars a user has entered into the text staging box when "send" is pressed.
 * text_start_x and text_start_y are passed in to indicate the position from which the function should begin drawing
 * the first character (i.e. the position of the top left corner of the first character).
 */
void draw_message_text(int length_limit, const char *text_to_draw, int text_start_x, int text_start_y, int wrap_limit) {
  int start_x = text_start_x;
  int start_y = text_start_y;
  int chars_in_current_line = 0;
  for (int curr_char_index = 0; curr_char_index < length_limit; curr_char_index++) {
    if (text_to_draw[curr_char_index] == '\n') {
      // Moves the cursor to the next line (adjust draw_start_y based on text size):
      start_x = text_start_x;
      start_y += LINE_HEIGHT;
      chars_in_current_line = 0;
    } else if (chars_in_current_line >= wrap_limit) {
      start_x = text_start_x;
      start_y += LINE_HEIGHT;
      chars_in_current_line = 1;
      tft.drawChar(start_x, start_y, text_to_draw[curr_char_index], ILI9341_BLACK, ILI9341_WHITE, TEXT_SIZE, TEXT_SIZE);
      start_x += CHAR_WIDTH;
    } else {
      // Draws the character at the current cursor position:
      tft.drawChar(start_x, start_y, text_to_draw[curr_char_index], ILI9341_BLACK, ILI9341_WHITE, TEXT_SIZE, TEXT_SIZE);
      start_x += CHAR_WIDTH;
      chars_in_current_line++;
    }
  }
}

/**
 * Clears the chat history display area and redraws messages from chat_history, starting with the message at
 * the currently set scroll position (with progressively older messages displayed above).
 * Redrawing is accomplished by first calculating the needed screen space for each message, factoring in
 * formatting rules, and also obscures any message content that appears outside the chat history box bounds.
 *
 * Global variables used:
 * MAX_CHAT_MESSAGES – The size of the circular buffer for storing chat messages.
 * CHAT_BOX_START_Y – The starting vertical position of the chat box on the display.
 * CHAT_BOX_HEIGHT – The height of the chat box on the display.
 * LINE_HEIGHT – The height of a single line of text.
 * CHAT_BOX_BOTTOM_PADDING – Padding below the last line in the chat box.
 * CHAT_BOX_START_X – The starting horizontal position of the chat box.
 * CHAT_BOX_WIDTH – The width of the chat box.
 * ILI9341_WHITE – Color constant for clearing the chat box background.
 * ILI9341_RED – Color constant for the chat box border.
 *
 * Drawing method params (for reference):
 * drawString(text_to_draw, draw_start_x, draw_start_y);
 * drawRect(rect_start_x, rect_start_y, rect_width, rect_height, rect_outline_color);
 * fillRect(rect_start_x, rect_start_y, rect_width, rect_height, rect_fill_color);
 */
void display_chat_history(ChatBufferState* state) {
  /*
    Note that curr_message_index points to the current message in the buffer being displayed or accessed, adjusted for the user's scroll position.
    So if message_scroll_offset is 0, that means the most recent message in the history is being shown at the bottom (UP has not been pressed).
    Pressing the UP key once increments message_scroll_offset to 1, which means the NEXT most recent message is displayed at the bottom.
    From there, the outer for loop is responsible for drawing all of the previous message, starting at chat_history[curr_message_index] and incrementing curr_message_index by 1.
  */
  int curr_message_index = (state->message_buffer_write_index - 1 - state->message_scroll_offset + MAX_CHAT_MESSAGES) % MAX_CHAT_MESSAGES;
  int curr_message_pos = CHAT_BOX_START_Y + CHAT_BOX_HEIGHT - LINE_HEIGHT - CHAT_BOX_BOTTOM_PADDING;
  int messages_to_display_count = state->chat_history_message_count - state->message_scroll_offset;

  // Clears entire chat box area
  tft.fillRect(CHAT_BOX_START_X, CHAT_BOX_START_Y, CHAT_BOX_WIDTH, CHAT_BOX_HEIGHT, ILI9341_WHITE);
  tft.drawRect(CHAT_BOX_START_X, CHAT_BOX_START_Y, CHAT_BOX_WIDTH, CHAT_BOX_HEIGHT, ILI9341_RED);

  // Iterates over and draws each message, starting with whatever message is at curr_message_index, then the next most recent message, and so on:
  for (int drawn_message_count = 0; drawn_message_count < messages_to_display_count; drawn_message_count++) {
    // Gets timestamp and formats as 00:00PM (or AM) for chat display:
    struct tm *timeinfo = localtime(&state->chat_history[curr_message_index].timestamp);
    char time_as_str[8];
    strftime(time_as_str, sizeof(time_as_str), "%I:%M%p", timeinfo);

    // Calculates number of lines this message will occupy:
    int line_count = 1;
    int text_length = strnlen(state->chat_history[curr_message_index].text, MAX_TEXT_LENGTH);
    int chars_in_current_line = 0;
    for (int curr_char_index = 0; curr_char_index < text_length; curr_char_index++) {
      if (state->chat_history[curr_message_index].text[curr_char_index] == '\n') {
        line_count++;
        chars_in_current_line = 0;
      } else if (chars_in_current_line >= CHAT_WRAP_LIMIT) {
        line_count++;
        chars_in_current_line = 1;
      } else {
        chars_in_current_line++;
      }
    }

    const int box_height = line_count * LINE_HEIGHT;
    const int border_start_y = curr_message_pos - box_height + BORDER_PADDING_Y;
    const int border_height = box_height + BORDER_PADDING_Y;
    int draw_start_y = curr_message_pos - box_height + LINE_HEIGHT;

    // Draws message box and timestamp for incoming messages:
    if (strcmp(state->chat_history[curr_message_index].recipient, RECIPIENT_UNKEY) == 0) {
      // Draws timestamp at current line:
      tft.drawString(time_as_str, INCOMING_TIMESTAMP_START_X, draw_start_y);
      tft.drawRect(INCOMING_BORDER_START_X, border_start_y, INCOMING_BORDER_WIDTH, border_height, ILI9341_BLUE);

    // Draws message box and timestamp for outgoing messages:
    } else {
      tft.drawString(time_as_str, OUTGOING_TIMESTAMP_START_X, draw_start_y);
      tft.drawRect(OUTGOING_BORDER_START_X, border_start_y, OUTGOING_BORDER_WIDTH, border_height, ILI9341_LIGHTGREY);
    }

    // Draws text chars for incoming messages:
    if (strcmp(state->chat_history[curr_message_index].recipient, RECIPIENT_UNKEY) == 0) {
      draw_message_text(text_length, state->chat_history[curr_message_index].text, INCOMING_TEXT_START_X, draw_start_y, CHAT_WRAP_LIMIT);
    // Draws text chars for outgoing messages:
    } else {
      draw_message_text(text_length, state->chat_history[curr_message_index].text, OUTGOING_TEXT_START_X, draw_start_y, CHAT_WRAP_LIMIT);
    }

    // Clips anything that scrolls past upper bound of chat history box (lib doesn't have a function for this)
    // ...Under battery display:
    tft.fillRect(CHAT_BOX_START_X, BATTERY_BOX_HEIGHT, CHAT_BOX_WIDTH, SPACE_UNDER_BATTERY_WIDTH, ILI9341_WHITE);
    // ...Next to battery display:
    tft.fillRect(BATTERY_BOX_WIDTH, 0, SPACE_BESIDE_BATTERY_WIDTH, CHAT_BOX_START_Y, ILI9341_WHITE);

    // Vertically separates messages:
    curr_message_pos -= (box_height + CHAT_BOX_LINE_PADDING);
    // Gets next most recent message from ring buffer:
    curr_message_index = (curr_message_index - 1 + MAX_CHAT_MESSAGES) % MAX_CHAT_MESSAGES;
  }
}

/**
 * Clears the display area where typed text is shown and redraws the boundary of the text box.
 * It also reprints the current contents of the tx_display_buffer.
 */
void redraw_typing_box() {
  tft.fillRect(TYPING_BOX_START_X, TYPING_BOX_START_Y, CHAT_BOX_WIDTH, TYPING_BOX_HEIGHT, ILI9341_WHITE);
  tft.drawRect(TYPING_BOX_START_X, TYPING_BOX_START_Y, CHAT_BOX_WIDTH, TYPING_BOX_HEIGHT, ILI9341_RED);
  draw_message_text(tx_display_buffer_length, tx_display_buffer, TYPING_CURSOR_X, TYPING_CURSOR_Y, SEND_WRAP_LIMIT);
}

/**
 * Resets the buffer that stores the text being typed on the keyboard. It clears the buffer and resets its length.
 * Keyboard will write to this, screen will display it.
 */
void reset_tx_display_buffer() {
  memset(tx_display_buffer, '\0', MAX_TEXT_LENGTH);
  // Fills buffer with null chars ('\0'):
  tx_display_buffer_length = 0;
}

/**
 * Initializes the TFT screen, sets the screen orientation, clears the screen, and draws some basic UI elements.
 */
void setup_screen() {
  pinMode(tft_led, OUTPUT);
  // Responsible for turning screen light on:
  digitalWrite(tft_led, HIGH);
  screen_on = true;

  pinMode(tft_sck, OUTPUT);

  tft.begin();
  tft.setRotation(2);
  tft.fillScreen(ILI9341_WHITE);
  // Draws chat history boundaries:
  tft.drawRect(CHAT_BOX_START_X, CHAT_BOX_START_Y, CHAT_BOX_WIDTH, CHAT_BOX_HEIGHT, ILI9341_RED);
  // Draws typing box boundaries:
  tft.drawRect(CHAT_BOX_START_X, TYPING_BOX_START_Y, CHAT_BOX_WIDTH, TYPING_BOX_HEIGHT, ILI9341_RED);
  // Sets cursor to starting position inside typing box:
  tft.setCursor(TYPING_CURSOR_X, TYPING_CURSOR_Y);

  reset_tx_display_buffer();
}

/**
 * Used in poll_battery to display the current battery level on the screen.
 */
inline float read_battery_voltage() {
  // 2.0 for 1:1 voltage divider, 3.3V is max ADC voltage, and ADC is 12-bit (4096 values)
  return 2.0 * analogRead(battery_monitor) * 3.3 / 4096;
}

/**
 * Periodically reads and displays the battery voltage on the screen.
 */
void poll_battery() {
  if (millis() - time_of_last_battery_read_ms > BATTERY_READ_PERIOD_MS) {
    time_of_last_battery_read_ms += BATTERY_READ_PERIOD_MS;
    float battery_volts = read_battery_voltage();

    tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);
    int16_t x, y;
    tft.getCursor(&x, &y);
    tft.setCursor(2, 2);
    tft.printf("battery %.2fV", battery_volts);
    tft.setCursor(x, y);
  }
}