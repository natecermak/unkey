// ==================================================================
// keyboard.cpp
// Handles keyboard scanning, polling, and input processing
// ==================================================================
#include "keyboard.h"
#include "hardware_config.h"
#include "config.h"
#include "display.h"
#include "chat_logic.h"
#include <Arduino.h>

// ------------------------------------------------------------------
// External globals
// ------------------------------------------------------------------

extern const int keyboard_poller_period_usec;
extern bool screen_on;
extern char tx_display_buffer[];

// ------------------------------------------------------------------
// State
// ------------------------------------------------------------------

IntervalTimer keyboard_poller_timer;

// Will likely store the state of keys (??) - using a 64 bit integer
// => 64 key states, i.e. 1 for pressed, 0 for not:
volatile uint64_t switch_state;

// TODO: this is too low, for testing only:
const unsigned long screen_timeout_ms = 10000;

// Useful for debouncing/long presses:
uint32_t time_of_last_press_ms;

// Runs poller at 100 Hz:
const int keyboard_poller_period_usec = 10000;

// ------------------------------------------------------------------
// Functions
// ------------------------------------------------------------------

/**
 * Configures the keyboard polling mechanism by setting up input/output pins
 * and starting a timer that calls poll_keyboard at regular intervals.
 */
void setup_keyboard_poller() {
  switch_state = 0;

  // Sets up SPI:
  pinMode(kb_load_n, OUTPUT);
  digitalWrite(kb_load_n, HIGH);
  pinMode(kb_clock, OUTPUT);
  pinMode(kb_data, INPUT);

  // Starts timer:
  if (!keyboard_poller_timer.begin([]() { poll_keyboard(get_chat_buffer_state()); }, keyboard_poller_period_usec)) {
    Serial.println("Failed setting up poller");
  }
}

/**
 * Calculates the integer base-2 logarithm of x.
 * Returns the position of the highest set bit.
 */
static int ilog2(uint64_t x) {
  int i = 0;
  while (x) {
    x = x >> 1;
    i++;
  }
  return i;
}

/**
 * Reads the state of the keyboard by polling shift registers.
 * Detects new key presses, updates the screen, and processes
 * actions for CAPS, SYM, SEND, arrow keys, and printable keys.
 */
void poll_keyboard(ChatBufferState* state) {
  // static int modifier = 0;

  // Latches keyboard state into shift registers
  // kb_load_n is an output pin which pulses a signal here from LOW to HIGH
  // low to hi transmission lets shift reg know to record keyboard state (8 bits per registr)
  digitalWrite(kb_load_n, LOW);
  delayNanoseconds(5);
  digitalWrite(kb_load_n, HIGH);
  delayNanoseconds(5);

  // Reads 64 bits of shift register, takes 2.63us, runs at 23 MHz
  // Note: bitbang to avoid taking up an SPI peripheral. Requires VERY little CPU time
  uint64_t read_buffer = 0;
  for (int bit = 0; bit < SCAN_CHAIN_LENGTH; bit++) {
    read_buffer |= (uint64_t)digitalReadFast(kb_data) << ((SCAN_CHAIN_LENGTH - 1) - bit); // kb_data is the state of each key press
    digitalWriteFast(kb_clock, LOW);
    digitalWriteFast(kb_clock, HIGH);
    delayNanoseconds(10);
  }
  digitalWriteFast(kb_clock, LOW);

  // Compares results to previous one to detect new key presses
  uint64_t new_press = ~read_buffer & switch_state;
  switch_state = read_buffer;

  if (new_press) {
    time_of_last_press_ms = millis();

    if (!screen_on) {
      screen_on = true;
      digitalWrite(tft_led, HIGH);
    } else {
      uint8_t key_index = ilog2(new_press) - 1;

      switch (key_index) {
        case CAP_KEY_INDEX:
          Serial.println("You pressed CAPS");
          break;
        case SYM_KEY_INDEX:
          Serial.println("You pressed SYM");
          break;
        case UP_KEY_INDEX:
          Serial.println("You pressed UP");
          /*
          Pressing "up" increments message_scroll_offset, which is used to determine which
          message should be displayed at the bottom of the history box. Any older messages are just
          redrawn above that message, and any content that exceeds the heigh of the box should be cut
          off anyway. Note: message_scroll_offset should never exceed (chat_history_message_count - 1)
          even if the user keeps pressing 'up'
          if (message_scroll_offset == chat_history_message_count - 1), that means the oldest message is
          currently displayed at the bottom of the history box
          */
          if (state->message_scroll_offset < state->chat_history_message_count - 1) {
            state->message_scroll_offset++;
            display_chat_history(state);
          }
          break;
        case DOWN_KEY_INDEX:
          Serial.println("You pressed DOWN");
          if (state->message_scroll_offset > 0) {
            state->message_scroll_offset--;
            display_chat_history(state);
          }
          break;
        case BACK_KEY_INDEX:
          Serial.println("You pressed BACKSPACE");
          tx_display_buffer[tx_display_buffer_length] = '\0';
          tx_display_buffer_length--;
          redraw_typing_box();
          break;
        case RET_KEY_INDEX:
          Serial.println("You pressed RETURN");
          if (tx_display_buffer_length < MAX_TEXT_LENGTH - 1) {
            tx_display_buffer[tx_display_buffer_length] = '\n';
            tx_display_buffer_length++;
          }
          redraw_typing_box();
          break;
        case SEND_KEY_INDEX:
          Serial.println("You pressed SEND");
          Serial.printf("Current message length: %d\n", tx_display_buffer_length);
          if (tx_display_buffer_length == 0) {
            Serial.println("No message to send");
            break;
          }
          send_message(tx_display_buffer);
          reset_tx_display_buffer();
          redraw_typing_box();
          break;
        default:
          char key = KEYBOARD_LAYOUT[key_index];
          tx_display_buffer[tx_display_buffer_length] = key;
          tx_display_buffer_length++;
          // Serial.printf("Read buffer %ul\n", ~read_buffer);
          Serial.printf("You pressed key_index=%d, key=\'%c\'\n", key_index, key);
          redraw_typing_box();
          // modifier = 0; // reset modifier keys
      }
    }
  } else if (screen_on && millis() - time_of_last_press_ms > screen_timeout_ms) {
    screen_on = false;
    //digitalWrite(tft_led, LOW);
  }
}
