#include "display.h"

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