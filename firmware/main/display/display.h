#ifndef DISPLAY_H
#define DISPLAY_H

#include <ILI9341_t3n.h>
#include "../config.h"

// This extern display object tells the compiler: “This object exists somewhere else (in main.ino),
// I just want to access it here.”
extern ILI9341_t3n tft;

void draw_message_text(int length_limit, const char *text_to_draw, int text_start_x, int text_start_y, int wrap_limit);

void display_chat_history(ChatBufferState* state);

#endif