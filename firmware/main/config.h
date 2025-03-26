#ifndef CONFIG_H
#define CONFIG_H

// ==================================================================
// config.h
// Central UI layout, messaging constants, and shared types
// ==================================================================

#include <time.h>

//----------------------------------------
// Chat Display Configuration
//----------------------------------------
#define CHAT_BOX_LINE_PADDING       11      // Extra vertical space between chat lines
#define CHAT_BOX_START_X            0       // Chat box horizontal offset (0 is flush to left screen bound)
#define CHAT_BOX_START_Y            25      // Chat box vertical offset (0 is flush to top screen bound)
#define CHAT_BOX_WIDTH              235
#define CHAT_BOX_HEIGHT             201
#define CHAT_BOX_BOTTOM_PADDING     3
#define CHAT_WRAP_LIMIT             16      // Maximum characters per line before wrapping
#define LINE_HEIGHT                 10      // Height of each text line in pixels

//----------------------------------------
// Text and Buffer Parameters
//----------------------------------------
#define CHAR_WIDTH                  7       // Width of each character in pixels
#define MAX_CHAT_MESSAGES           50      // Maximum messages stored in chat history
#define MAX_NAME_LENGTH             20      // Maximum length for sender/recipient names
#define MAX_PACKET_SIZE             405     // Maximum packet size (including header/footer)
#define MAX_TEXT_LENGTH             400     // Maximum length of message text

//----------------------------------------
// Incoming/Outgoing Message Parameters
//----------------------------------------
#define INCOMING_TEXT_START_X       70      // Received message horizontal offset
#define INCOMING_BORDER_START_X     62
#define INCOMING_BORDER_WIDTH       132
#define INCOMING_BORDER_MARGIN      6
#define OUTGOING_TEXT_START_X       120
#define OUTGOING_BORDER_START_X     112
#define OUTGOING_BORDER_WIDTH       122

//----------------------------------------
// Keyboard and Typing Configuration
//----------------------------------------
#define SCAN_CHAIN_LENGTH           56      // Number of keys to poll per cycle
#define SEND_WRAP_LIMIT             30      // Wrap limit for outgoing messages
#define TEXT_SIZE                   1       // Text size multiplier (depends on display)
#define INCOMING_TIMESTAMP_START_X  10      // Horizontal offset for incoming timestamps
#define OUTGOING_TIMESTAMP_START_X  60      // Horizontal offset for outgoing timestamps
#define TYPING_BOX_START_X          0       // Typing box horizontal offset
#define TYPING_BOX_START_Y          225     // Typing box vertical offset
#define TYPING_BOX_HEIGHT           90      // Typing box height
#define TYPING_CURSOR_X             2       // Typing cursor horizontal offset
#define TYPING_CURSOR_Y             227     // Typing cursor vertical offset

//----------------------------------------
// Keyboard Key Indexes
//----------------------------------------
#define CAP_KEY_INDEX               3
#define SYM_KEY_INDEX               40
#define BACK_KEY_INDEX              36
#define DEL_KEY_INDEX               37
#define RET_KEY_INDEX               44
#define SEND_KEY_INDEX              45
#define ESC_KEY_INDEX               48
#define MENU_KEY_INDEX              49
#define LEFT_KEY_INDEX              47
#define UP_KEY_INDEX                50
#define DOWN_KEY_INDEX              51
#define RIGHT_KEY_INDEX             46

const char* KEYBOARD_LAYOUT     = "1qa~zsw23edxcfr45tgvbhy67ujnmki89ol?~~p0~ ,.\n           ";
const char* KEYBOARD_LAYOUT_SYM = "!@#$%^&*()`~-_=+:;\'\"[]{}|\\/<>~~zxcvbnm?~~ ,.\n           ";

//----------------------------------------
// Battery and Display Spacing Configuration
//----------------------------------------
#define BATTERY_BOX_HEIGHT          10
#define SPACE_UNDER_BATTERY_WIDTH   15
#define BATTERY_BOX_WIDTH           80
#define SPACE_BESIDE_BATTERY_WIDTH  155     // Typically: CHAT_BOX_WIDTH - BATTERY_BOX_WIDTH
#define BORDER_PADDING_Y            6

//----------------------------------------
// Message and Test Constants
//----------------------------------------
#define RECIPIENT_UNKEY             "unkey"
#define RECIPIENT_VOID              "the void"
#define TEST_MESSAGE_TEXT           "Incoming from The Void"
#define TESTING_MESSAGE_COUNT_LIMIT 2

//----------------------------------------
// Type Definitions
//----------------------------------------
typedef struct {
  time_t timestamp;
  char sender[MAX_NAME_LENGTH];
  char recipient[MAX_NAME_LENGTH];
  char text[MAX_TEXT_LENGTH];
} message_t;

typedef struct _tx_parameters {
  float freq_low;
  float freq_high;
  uint32_t usec_per_bit;
} tx_parameters_t;

typedef struct {
  int message_buffer_write_index;
  int chat_history_message_count;
  int message_scroll_offset;  // Number of messages scrolled from most recent
  message_t chat_history[MAX_CHAT_MESSAGES];
} ChatBufferState;

#endif