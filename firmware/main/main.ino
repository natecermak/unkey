#include <ADC.h>
#include <DMAChannel.h>
#include <ILI9341_t3n.h>
#include <SPI.h>
#include <Wire.h>

#include "config.h"
#include "hardware_config.h"
#include "goertzel.h"

/*



*/


ChatBufferState chat_buffer_state = {0, 0, 0, {}};


// ------------------- ADC, DMA, and Goertzel filters -------------------- //
/**
 * - ADC is set up to sample an analog signal at 81.92 kHz
 * - DMA is configured to transfer ADC samples into the dma_adc_buff1 buffer
 * - Goertzel algo is initialized to analyze a signal for up to 10 diff frequencies
 */

// Used to interact with ADC hardware for configuring/reading analog signals:
ADC *adc = new ADC();
// Used to manage transfer of data between ADC and memory using direct memory access (doesn't use CPU => more efficient):
DMAChannel dma_ch1;

// ADC will sample at freq of 81.92 kHz:
const uint32_t adc_frequency = 81920;
// Size of buffer where ADC data will be stored:
const uint32_t buffer_size = 10240;
// Creates dma_adc_buff1 buffer:
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff1[buffer_size];
// Makes a copy of buffer:
uint16_t adc_buffer_copy[buffer_size];

uint8_t print_ctr = 0;
const uint8_t gs_len = 10;
// gs is an array that will store state for the Goertzel algo - each goertzel_state obj holds data to compute G algo for that frequency:
goertzel_state gs[gs_len];

// ------------------- Charge amplifier gain ------------------------------ //
const int adg728_i2c_address = 76;

// ------------------- Keyboard poller timer and state-------------------- //
IntervalTimer keyboard_poller_timer;
// Runs poller at 100 Hz:
const int keyboard_poller_period_usec = 10000;
// Will likely store the state of keys (??) - using a 64 bit integer => 64 key states, i.e. 1 for pressed, 0 for not:
volatile uint64_t switch_state;
// Useful for debouncing/long presses:
uint32_t time_of_last_press_ms;

// Keyboard modifiers:
const uint8_t CAP_KEY_INDEX = 3;
const uint8_t SYM_KEY_INDEX = 40;

const uint8_t BACK_KEY_INDEX = 36;
const uint8_t DEL_KEY_INDEX = 37;
const uint8_t RET_KEY_INDEX = 44;
const uint8_t SEND_KEY_INDEX = 45;
const uint8_t ESC_KEY_INDEX = 48;
const uint8_t MENU_KEY_INDEX = 49;

const uint8_t LEFT_KEY_INDEX = 47;
const uint8_t UP_KEY_INDEX = 50;
const uint8_t DOWN_KEY_INDEX = 51;
const uint8_t RIGHT_KEY_INDEX = 46;

const char *KEYBOARD_LAYOUT = "1qa~zsw23edxcfr45tgvbhy67ujnmki89ol?~~p0~ ,.\n           ";
const char *KEYBOARD_LAYOUT_SYM = "!@#$%^&*()`~-_=+:;\'\"[]{}|\\/<>~~zxcvbnm?~~ ,.\n           ";


// ------------------- TFT LCD ------------------------------------------- //
/**
 * Initializing the display using pins assigned above, which - as a reminder - interfaces with these things:
 * - Analog-to-Digital Converter (ADC)
 * - Keyboard or input device
 * - TFT display with ILI9341 controller via SPI
 * - DAC, power control, and a battery monitor
 */

// TODO: this is too low, for testing only:
bool screen_on;
const int screen_timeout_ms = 10000;
// Initializes the display using pin numbers defined above, which get passed to the constructor:
ILI9341_t3n tft = ILI9341_t3n(tft_cs, tft_dc, tft_reset, tft_mosi, tft_sck, tft_miso);

// ------------------- Transmit message field ---------------------------- //
char tx_display_buffer[MAX_TEXT_LENGTH];
uint16_t tx_display_buffer_length;

// ------------------- Battery monitoring -------------------------------- //
long time_of_last_battery_read_ms;
const long BATTERY_READ_PERIOD_MS = 1000;

// ------------------- DSP Utility functions ----------------------------- //
/**
 * Calculates the integer base 2 logarithm of x to give the position of the highest set bit.
 * Usage: Gets called by poll_keyboard to detect key presses
 */
int ilog2(uint64_t x) {
  int i = 0;
  while (x) {
    x = x >> 1;
    i++;
  }
  return i;
}

/**
 * Sets the gain on a charge amplifier by writing a specific value to an I2C device, a charge amplifier (controlled by gain_index).
 * It shifts 1U left by gain_index to generate a specific binary pattern and writes this value to the amplifier's address.
 * Usage: Called in setup_receiver to configure the amplifier gain
 */
void set_charge_amplifier_gain(uint8_t gain_index) {
  Wire.beginTransmission(adg728_i2c_address);
  Wire.write(1U << gain_index);
  Wire.endTransmission();
}

/**
 * Enables/disables transmission power by writing high or low to the tx_power_en pin.
 * Usage: Used in setup_transmitter to turn on the power before transmitting.
 */
inline void set_tx_power_enable(bool enable) {
  digitalWriteFast(tx_power_en, (enable) ? HIGH : LOW);
}

/**
 * Deals with ADC data when DMA buffer is full, and processes the data with Goertzel filters and prints frequency domain data.
 * Analog signals (voltages) --> digital values that can be processed by da Teensy
 * Usage: Attached to a DMA interrupt in setup_receiver and called automatically when the buffer fills up.
 */
void adc_buffer_full_interrupt() {
  dma_ch1.clearInterrupt();
  // mempcy copies a block of memory from one location to another:
  memcpy((void *)adc_buffer_copy, (void *)dma_adc_buff1, sizeof(dma_adc_buff1));
  if ((uint32_t)dma_adc_buff1 >= 0x20200000u)
    arm_dcache_delete((void *)dma_adc_buff1, sizeof(dma_adc_buff1));
  // Re-enables the DMA channel for next read:
  dma_ch1.enable();

  /**
   * Processes data: uses Goertzel algorithm to analyze the frequency content of a series of ADC samples
   */
  if (print_ctr++ % SCAN_CHAIN_LENGTH == 0) {
    for (size_t i = 0; i < buffer_size; i++) {
      //Serial.printf("%d\n", adc_buffer_copy[i]);
      for (int j = 0; j < gs_len; j++) {
        goertzel_state *g = &gs[j];
        update_goertzel(g, adc_buffer_copy[i]);
      }
    }
    for (int j = 0; j < gs_len; j++) {
      goertzel_state *g = &gs[j];
      finalize_goertzel(g);
      // Serial.printf("GS%d (%.0f Hz): %6.1f %6.1f \t %f %d\n",
      //   j, g->w0/6.28 * adc_frequency, g->y_re, g->y_im, sqrt(pow(g->y_re, 2) + pow(g->y_im, 2)), adc_buffer_copy[j]);
      reset_goertzel(g);
    }
  }
}

/**
 * Configures the system to receive data: initializes the ADC, configures Goertzel filters for frequency analysis,
 * sets the gain on the charge amplifier, sets up DMA channel for ADC to send data to a buffer super duper efficiently.
 */
void setup_receiver() {
  // Sets readPin_adc_0 as the input pin:
  pinMode(readPin_adc_0, INPUT);

  // Initializes Goertzel filters (TODO: Hardcoded frequencies here):
  for (int j = 0; j < gs_len; j++) {
    // The 2nd param sets the initial frequency for that filter: so 14000, 14200, 14400 etc
    initialize_goertzel(&gs[j], 15000 + (j - 5) * 200, adc_frequency);
  }

  // Sets gain on charge amplifier:
  set_charge_amplifier_gain(6);

  // Sets up ADC (for received audio signal):
  adc->adc0->setAveraging(1); // no averaging
  adc->adc0->setResolution(12); // bits
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  //adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

  // Sets up DMA:
  // Note: The following line raises a compiler warning because type-punning ADC1_R0 here violates strict aliasing rules, but in this case this warning can be safely ignored
  dma_ch1.source((volatile uint16_t &)(ADC1_R0));
  // Each time you read from adc you get 2 bytes, so that's why 2x:
  dma_ch1.destinationBuffer((uint16_t *)dma_adc_buff1, buffer_size * 2);
  dma_ch1.interruptAtCompletion();
  dma_ch1.disableOnCompletion();
  // When dma is done, calls adc_buffer_full_interrup which is a func:
  dma_ch1.attachInterrupt(&adc_buffer_full_interrupt);
  dma_ch1.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);

  // Enables the DMA channel:
  dma_ch1.enable();
  adc->adc0->enableDMA();
  adc->adc0->startSingleRead(readPin_adc_0);
  // This actually determines how fast to sample the signal, and starts timer to initiate dma transfer from adc to memory once 2x buffer size bytes reached
  adc->adc0->startTimer(adc_frequency);
}

/**
 * Sets up the transmitter by configuring output pins, enabling transmission power, and writing initial values to a DAC (Digital-to-Analog Converter).
 * Configures gain and voltage references for the DAC.
 */
void setup_transmitter() {
  pinMode(tx_power_en, OUTPUT);
  pinMode(xdcr_sw, OUTPUT);
  pinMode(dac_cs, OUTPUT);
  digitalWrite(dac_cs, HIGH);

  // TODO: FOR TESTING ONLY:
  set_tx_power_enable(true);  // tested: works
  delay(100);                 // wait for power to boot
  write_to_dac(0xA, 1U << 8);  // A is address for config, 8th bit is gain. set to gain=2
  write_to_dac(8, 1);          // 8 is address for VREF, 1 means use internal ref
}

/**
 * Sends data to a DAC via SPI: prepares a 3-byte buffer with an address and a 12-bit value, then sends the data using SPI communication.
 * MCP48CXDX1 -- 24-bit messages.
 * top byte: 5-bit address, 2 "command bits", 1 dont-care
 * bottom 2 bytes: 4 dont-care, 12 data bits
 */
void write_to_dac(uint8_t address, uint16_t value) {
  uint8_t buf[3];
  // Bits 1 and 2 must be 0 to write:
  buf[0] = (address << 3);
  buf[1] = (uint8_t)(value >> 8);
  buf[2] = (uint8_t)value;

  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dac_cs, LOW);
  SPI.transfer(buf, 3);
  digitalWrite(dac_cs, HIGH);
  SPI.endTransaction();
}

// ------------------- Screen Behavior Utility Functions ----------------- //

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
 * Transmits a message by modulating each character's bits into analog tones.
 * Note: address of 0 --> writing to channel 0 of the DAC.
 * write_to_dac(address=0, val=0): DAC receives a 24-bit message that sets the output to the minimum voltage (0V)
 * write_to_dac(address=0, val=4095): DAC receives a 24-bit message that sets the output to the maximum voltage
 */
void transmit_message(const char* message_to_transmit, const tx_parameters_t* tx_parameters) {
  for (int i = 0; message_to_transmit[i] != '\0'; i++) {
    Serial.print("Processing letter: ");
    Serial.println(message_to_transmit[i]);
    char letter = message_to_transmit[i];

    // Translates each of char's 8 bits into a corresponding frequency starting with msb:
    for (int j = 7; j >= 0; j--) {
      int bit = (letter >> j) & 1;
      // w is the angular frequency, wherein w = 2 * pi * f
      float w = (bit) ? (2 * PI * tx_parameters->freq_high / 1e6)
                      : (2 * PI * tx_parameters->freq_low / 1e6);
      // Start time for the current bit period:
      unsigned long bit_start = micros();
      unsigned long time_usec;
      uint16_t dac_value;
      // Generates a sine wave for current bit for 10 ms:
      while ((time_usec = micros() - bit_start) < tx_parameters->usec_per_bit) {
        // Gets the phase angle at curr time in microsec and scales for 12 bit DAC:
        dac_value = (uint16_t)(((sin(w * time_usec) + 1.0) / 2.0) * 409);
        noInterrupts();
        write_to_dac(0, dac_value);
        interrupts();
      }
    }
  }
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
 * Reads the state of a keyboard by polling a shift register connected to the keyboard’s data and clock lines.
 * It detects new key presses, updates the screen, and processes specific keys like CAPS, SYM, and SEND.
 * It also uses ilog2() to identify the index of the pressed key.
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
            display_chat_history(&chat_buffer_state);
          }
          break;
        case DOWN_KEY_INDEX:
          Serial.println("You pressed DOWN");
          if (state->message_scroll_offset > 0) {
            state->message_scroll_offset--;
            display_chat_history(&chat_buffer_state);
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

/**
 * Configures the keyboard polling mechanism by setting up input/output pins and starting a timer that calls
 * poll_keyboardat regular intervals.
 */
void setup_keyboard_poller() {
  switch_state = 0;

  // Sets up SPI:
  pinMode(kb_load_n, OUTPUT);
  digitalWrite(kb_load_n, HIGH);
  pinMode(kb_clock, OUTPUT);
  pinMode(kb_data, INPUT);

  // Starts timer:
  if (!keyboard_poller_timer.begin([]() { poll_keyboard(&chat_buffer_state); }, keyboard_poller_period_usec)) {
    Serial.println("Failed setting up poller");
  }

}

/**
 * Resets the buffer that stores the text being typed on the keyboard. It clears the buffer and resets its length.
 * Keyboard will write to this, screen will display it.
 */
static void reset_tx_display_buffer() {
  memset(tx_display_buffer, '\0', MAX_TEXT_LENGTH);
  // Fills buffer with null chars ('\0'):
  tx_display_buffer_length = 0;
}

/**
 * Clears the display area where typed text is shown and redraws the boundary of the text box.
 * It also reprints the current contents of the tx_display_buffer.
 */
static void redraw_typing_box() {
  tft.fillRect(TYPING_BOX_START_X, TYPING_BOX_START_Y, CHAT_BOX_WIDTH, TYPING_BOX_HEIGHT, ILI9341_WHITE);
  tft.drawRect(TYPING_BOX_START_X, TYPING_BOX_START_Y, CHAT_BOX_WIDTH, TYPING_BOX_HEIGHT, ILI9341_RED);
  draw_message_text(tx_display_buffer_length, tx_display_buffer, TYPING_CURSOR_X, TYPING_CURSOR_Y, SEND_WRAP_LIMIT);
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

/**
 * Currently used to simulated staggered incoming messages.
 */
int incoming_message_count = 0;
IntervalTimer test_incoming_message;
void incoming_message_callback() {
  const char *test_message_text = TEST_MESSAGE_TEXT;
  add_message_to_chat_history(&chat_buffer_state, test_message_text, RECIPIENT_VOID, RECIPIENT_UNKEY);
  display_chat_history(&chat_buffer_state);
  incoming_message_count++;
  if (incoming_message_count >= TESTING_MESSAGE_COUNT_LIMIT) {
    test_incoming_message.end();
  }
}

/**
 * The main/top-level setup function that initializes the serial communication, SPI, and I2C;
 * calls other setup functions to configure the screen, receiver, transmitter, and keyboard poller.
 * Usage: The first function called in the program to initialize all components.
 */
void setup() {
  // Initializes serial communication with Teensy at baud rate of 9600 bps:
  Serial.begin(9600);
  // Checks if connection is working and waits up to 5 sec for it to happen:
  while (!Serial && millis() < 5000) ;
  delay(100);
  Serial.println("============================\nStarting setup()");

  // SPI commuincation bus for keyboard/display etc:
  SPI.begin();
  // I2C communication bus for charge amplifier:
  Wire.begin();
  // Specifies 12-bit resolution:
  analogReadResolution(12);

  // test_incoming_message.begin(incoming_message_callback, 10000000);
  setup_screen();
  setup_receiver();
  setup_transmitter();
  setup_keyboard_poller();

  Serial.println("setup() complete\n============================");
}

/**
 * Continuously polls the battery voltage and updates the display with batt charge level.
 */
void loop() {
  // uint16_t val = 2048 + 2047 * sin(2*3.14159*micros()/1e6 * 1.5e3); // tryna make a number between 0 and 2^12 i.e. 12 bits

  poll_battery();
}
