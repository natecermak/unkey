#ifndef DISPLAY_H
#define DISPLAY_H

#include <ILI9341_t3n.h>
#include "../config.h"

// This extern display object tells the compiler: “This object exists somewhere else (in main.ino),
// I just want to access it here.”
extern ILI9341_t3n tft;

void draw_message_text(int length_limit, const char *text_to_draw, int text_start_x, int text_start_y, int wrap_limit);

void display_chat_history(ChatBufferState* state);

void poll_battery();

inline float read_battery_voltage();

static void reset_tx_display_buffer();

static void redraw_typing_box();

void setup_screen();

#endif