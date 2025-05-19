// ==================================================================
// display.h
// Declarations for screen setup and drawing messages
// ==================================================================
#ifndef DISPLAY_H
#define DISPLAY_H

#include <ILI9341_t3n.h>

#include "config.h"

extern ILI9341_t3n tft;

void display_chat_history(ChatBufferState* state);

void reset_tx_display_buffer();

void redraw_typing_box();

void setup_screen();

#endif
