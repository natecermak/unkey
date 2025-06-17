#ifndef MOCK_DISPLAY_H
#define MOCK_DISPLAY_H

#include "chat_logic.h"

extern int display_called;

void display_chat_history(ChatBufferState* state);

#endif
