#include <ADC.h>
#include <DMAChannel.h>
#include <ILI9341_t3n.h>
#include <SPI.h>
#include <Wire.h>
#include "goertzel.h"
// #include <ili9341_t3n_font_ComicSansMS.h> // how to import ili9341_t3n fonts, which should automatically include anti-aliasing

#define CHAT_BOX_LINE_PADDING 11 // Extra vertical space between lines in chat history box area
#define CHAT_BOX_START_X 0 // Chat history box horizontal offset (0 is flush to left screen bound)
#define CHAT_BOX_START_Y 25 // Chat history box vertical offset (0 is flush to top screen bound)
#define CHAT_BOX_WIDTH 235 // Chat history box width
#define CHAT_BOX_HEIGHT 201 // // Chat history box height
#define CHAT_BOX_BOTTOM_PADDING 3
#define CHAR_WIDTH 7 // Width of each character in pixels
#define CHAT_WRAP_LIMIT 16 // Sender message wrapping cut-off
#define LINE_HEIGHT 10 // Height of each line in pixels
#define MAX_CHAT_MESSAGES 50
#define MAX_NAME_LENGTH 20
#define MAX_TEXT_LENGTH 400 // Unit is characters
#define INCOMING_TEXT_START_X 70 // Received message horizontal offset
#define INCOMING_BORDER_START_X 62
#define INCOMING_BORDER_WIDTH 132
#define INCOMING_BORDER_MARGIN 6
#define OUTGOING_TEXT_START_X 120 // Sent message horizontal offset
#define OUTGOING_BORDER_START_X 112
#define OUTGOING_BORDER_WIDTH 122
#define SCAN_CHAIN_LENGTH 56  // 56 keys that need to be checked per polling cycle
#define SEND_WRAP_LIMIT 30 // Sender message wrapping cut-off`
#define TEXT_SIZE 1 // Text size multiplier (depends on your display)
#define INCOMING_TIMESTAMP_START_X 10 // Timestamp horizontal offset for incoming messages (0 is flush to left screen bound)
#define OUTGOING_TIMESTAMP_START_X 60 // Timestamp horizontal offset for outgoing messages (0 is flush to left screen bound)
#define TYPING_BOX_START_X 0 // Typing box horizontal offset (0 is flush to left screen bound)
#define TYPING_BOX_START_Y 225 // Typing box vertical offset (0 is flush to top screen bound
#define TYPING_BOX_HEIGHT 90 // Typing box height
#define TYPING_CURSOR_X 2 // Typing cursor horizontal offset (0 is flush to left screen bound)
#define TYPING_CURSOR_Y 227 // Typing cursor vertical offset (0 is flush to top screen bound
#define BATTERY_BOX_HEIGHT 10
#define SPACE_UNDER_BATTERY_WIDTH 15
#define BATTERY_BOX_WIDTH 80
#define SPACE_BESIDE_BATTERY_WIDTH 155 // 235 - BATTERY_BOX_WIDTH
#define BORDER_PADDING_Y 6
#define RECIPIENT_UNKEY "unkey"
#define RECIPIENT_VOID "the void"
#define TEST_MESSAGE_TEXT "Incoming from The Void"
#define TESTING_MESSAGE_COUNT_LIMIT 2

typedef struct {
  // message ID (might need once we need to handle incoming messages)
  // chat ID (only necessary if one device can have multiple chats)
  // sender device ID (" " "")
  // recipient device ID? (" " "")
  time_t timestamp;
  char sender[MAX_NAME_LENGTH];
  char recipient[MAX_NAME_LENGTH];
  char text[MAX_TEXT_LENGTH];
} message_t;

struct ChatBufferState {
  int message_buffer_write_index;
  int chat_history_message_count;
  int message_scroll_offset;  // Represents the # of messages scrolled up from most recent message
  message_t chat_history[MAX_CHAT_MESSAGES];
};

ChatBufferState chat_buffer_state = {0, 0, 0, {}};

// ------------------- PINS ----------------------------------------------- //
const int readPin_adc_0 = 15;
const int kb_load_n = 22;
const int kb_clock = 23;
const int kb_data = 21;

const int tft_miso = 12;
const int tft_led = 3;
const int tft_dc = 4;
const int tft_reset = 5;
const int tft_cs = 6;
const int tft_mosi = 11;
const int tft_sck = 13;

const int tx_power_en = 8;
const int xdcr_sw = 7;
const int dac_cs = 10;

const int battery_monitor = 20;

// ------------------- ADC, DMA, and Goertzel filters -------------------- //
/*
- ADC is set up to sample an analog signal at 81.92 kHz
- DMA is configured to transfer ADC samples into the dma_adc_buff1 buffer
- Goertzel algo is initialized to analyze a signal for up to 10 diff frequencies
*/
ADC *adc = new ADC();  // will be used to interact with ADC hardware for configuring/reading analog signals
DMAChannel dma_ch1;    // will be used to manage transfer of data between ADC and memory using direct memory access (doesn't use CPU => more efficient)

const uint32_t adc_frequency = 81920;                                                     // ADC will sample at freq of 81.92 kHz
const uint32_t buffer_size = 10240;                                                       // size of buffer where ADC data will be stored
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff1[buffer_size];  // creates dma_adc_buff1 buffer
uint16_t adc_buffer_copy[buffer_size];                                                    // makes a copy of buffer(?)

uint8_t print_ctr = 0;
const uint8_t gs_len = 10;
goertzel_state gs[gs_len];  // gs is an array that will store state for the Goertzel algo - each goertzel_state obj holds data to compute G algo for that frequency

// ------------------- Charge amplifier gain ------------------------------ //
const int adg728_i2c_address = 76;

// ------------------- Keyboard poller timer and state-------------------- //
IntervalTimer keyboard_poller_timer;
const int keyboard_poller_period_usec = 10000;  // run at 100 Hz
volatile uint64_t switch_state;                 // will likely store the state of keys (??) - using a 64 bit integer => 64 key states, i.e. 1 for pressed, 0 for not
uint32_t time_of_last_press_ms;                 // useful for debouncing/long presses

// keyboard modifiers
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
/* Initializing the display using pins assigned above, which - as a reminder - interfaces
  with these things:
  - Analog-to-Digital Converter (ADC)
  - Keyboard or input device
  - TFT display with ILI9341 controller via SPI
  - DAC, power control, and a battery monitor
*/
bool screen_on;
const int screen_timeout_ms = 10000;                                                    // todo: this is too low, for testing only
ILI9341_t3n tft = ILI9341_t3n(tft_cs, tft_dc, tft_reset, tft_mosi, tft_sck, tft_miso);  // initializes the display using pin numbers defined above, which get passed to the constructor

// ------------------- Transmit message field ---------------------------- //
char tx_display_buffer[MAX_TEXT_LENGTH];
uint16_t tx_display_buffer_length;

// ------------------- Battery monitoring -------------------------------- //
long time_of_last_battery_read_ms;
const long BATTERY_READ_PERIOD_MS = 1000;

// ------------------- DSP Utility functions ----------------------------- //
/*
 Calculates the integer base 2 logarithm of x to give the position of the highest set bit
 Usage: Gets called by poll_keyboard to detect key presses
*/
int ilog2(uint64_t x) {
  int i = 0;
  while (x) {
    x = x >> 1;
    i++;
  }
  return i;
}

/*
  Sets the gain on a charge amplifier by writing a specific value to an I2C device, a charge amplifier
  (controlled by gain_index).
  It shifts 1U left by gain_index to generate a specific binary pattern and writes this value to the amplifier's address.
  Usage: Called in setup_receiver to configure the amplifier gain
*/
void set_charge_amplifier_gain(uint8_t gain_index) {
  Wire.beginTransmission(adg728_i2c_address);
  Wire.write(1U << gain_index);
  Wire.endTransmission();
}

/*
  Enables/disables transmission power by writing high or low to the tx_power_en pin
  Usage: Used in setup_transmitter to turn on the power before transmitting.
*/
inline void set_tx_power_enable(bool enable) {
  digitalWriteFast(tx_power_en, (enable) ? HIGH : LOW);
}

/*
  Deals with ADC data when DMA buffer is full, and processes the data with Goertzel filters
  and prints frequency domain data
  Analog signals (voltages) --> digital values that can be processed by da Teensy
  Usage: Attached to a DMA interrupt in setup_receiver and called automatically when the buffer fills up.
*/
void adc_buffer_full_interrupt() {
  dma_ch1.clearInterrupt();
  memcpy((void *)adc_buffer_copy, (void *)dma_adc_buff1, sizeof(dma_adc_buff1)); // mempcy copies a block of memory from one location to another
  if ((uint32_t)dma_adc_buff1 >= 0x20200000u)
    arm_dcache_delete((void *)dma_adc_buff1, sizeof(dma_adc_buff1));
  dma_ch1.enable();  // Re-enables the DMA channel for next read

  /* 
    Processes data:
    Uses Goertzel algorithm to analyze the frequency content of a series of ADC samples
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

/*
  Configures the system to receive data: initializes the ADC,  
  configures Goertzel filters for frequency analysis, sets the gain on the charge amplifier,
  sets up DMA channel for ADC to send data to a buffer super duper efficiently.
  Direct Memory Access (DMA) --> data gets moved in/out of system memory w/o CPU involvement, so it's very efficient
  Charge amplifier --> converts super low charge signals to proportional voltage signals
*/
void setup_receiver() {
  // setup input pins
  pinMode(readPin_adc_0, INPUT);  // sets the readPin_adc_0 as the input pin

  // Initialize Goertzel filters (TODO: Hardcoded frequencies here)
  for (int j = 0; j < gs_len; j++) {  // iterating over Goertzel filter objects
    // The 2nd param sets the initial frequency for that filter: so 14000, 14200, 14400 etc
    initialize_goertzel(&gs[j], 15000 + (j - 5) * 200, adc_frequency); // adc_frequency is sampling freq 89.2 khz
  } // Measures amp of specific freq over some time period in a received audio signal --> bytes

  // Sets gain on charge amplifier
  set_charge_amplifier_gain(6);

  // Sets up ADC (for received audio signal)
  adc->adc0->setAveraging(1); // no averaging
  adc->adc0->setResolution(12); // bits
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // we want it to be
  //adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

  // Sets up DMA
  dma_ch1.source((volatile uint16_t &)(ADC1_R0));
  dma_ch1.destinationBuffer((uint16_t *)dma_adc_buff1, buffer_size * 2); // Each time you read from adc get 2 bytes, so that's why 2x
  dma_ch1.interruptAtCompletion();
  dma_ch1.disableOnCompletion();
  dma_ch1.attachInterrupt(&adc_buffer_full_interrupt); // When dma is done, call adc_buffer_full_interrup which is a func
  dma_ch1.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);

  dma_ch1.enable();  // Enables the DMA channel
  adc->adc0->enableDMA();
  adc->adc0->startSingleRead(readPin_adc_0);
  // This actually determines how fast to sample the signal, and starts timer to initiate dma transfer from adc to memory once 2x buffer size bytes reached
  adc->adc0->startTimer(adc_frequency);
}

/*
  Sets up the transmitter by configuring output pins, enabling transmission power, and 
  writing initial values to a DAC (Digital-to-Analog Converter). Configures gain and voltage references for the DAC.
*/
void setup_transmitter() {
  pinMode(tx_power_en, OUTPUT); // tx is transmit, rx is receive 
  pinMode(xdcr_sw, OUTPUT); // xdcr is transducer
  pinMode(dac_cs, OUTPUT);
  digitalWrite(dac_cs, HIGH);
  // TODO: FOR TESTING ONLY:
  set_tx_power_enable(true);  // tested: works
  delay(100);                 // wait for power to boot

  write_to_dac(0xA, 1U << 8);  // A is address for config, 8th bit is gain. set to gain=2
  write_to_dac(8, 1);          // 8 is address for VREF, 1 means use internal ref
}

/*
  Sends data to a DAC via SPI: prepares a 3-byte buffer with an address and a 12-bit value, 
  then sends the data using SPI communication.
*/
void write_to_dac(uint8_t address, uint16_t value) {
  /*
  MCP48CXDX1 -- 24-bit messages.
  top byte: 5-bit address, 2 "command bits", 1 dont-care
  bottom 2 bytes: 4 dont-care, 12 data bits
*/
  Serial.print("DAC Write - Address: ");
  Serial.print(address);
  Serial.print(", Value: ");
  Serial.println(value);

  uint8_t buf[3];
  buf[0] = (address << 3);  // bits 1 and 2 must be 0 to write.
  buf[1] = (uint8_t)(value >> 8);
  buf[2] = (uint8_t)value;

  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dac_cs, LOW);
  SPI.transfer(buf, 3);
  digitalWrite(dac_cs, HIGH);
  SPI.endTransaction();

  // float frequency = (bit) ? 2.2e3 : 2.0e3;
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

/*
  Draws message character content (incorporating line breaks and text wrapping) in the chat history box for a given message.
  A single message is defined as whatever text chars a user has entered into the text staging box when "send" is pressed.
  text_start_x and text_start_y are passed in to indicate the position from which the function should begin drawing
  the first character (i.e. the position of the top left corner of the first character).
*/
void draw_message_text(int length_limit, const char *text_to_draw, int text_start_x, int text_start_y, int wrap_limit) {
  int start_x = text_start_x;
  int start_y = text_start_y;
  int chars_in_current_line = 0;
  for (int curr_char_index = 0; curr_char_index < length_limit; curr_char_index++) {
    if (text_to_draw[curr_char_index] == '\n') {
      // Moves the cursor to the next line (adjust draw_start_y based on text size)
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
      // Draws the character at the current cursor position
      tft.drawChar(start_x, start_y, text_to_draw[curr_char_index], ILI9341_BLACK, ILI9341_WHITE, TEXT_SIZE, TEXT_SIZE);
      start_x += CHAR_WIDTH;
      chars_in_current_line++;
    }
  }
}

/*
  Clears the chat history display area and redraws messages from chat_history, starting with the message at
  the currently set scroll position (with progressively older messages displayed above).
  Redrawing is accomplished by first calculating the needed screen space for each message, factoring in
  formatting rules, and also obscures any message content that appears outside the chat history box bounds.

  Global variables used:
  - MAX_CHAT_MESSAGES – The size of the circular buffer for storing chat messages.
  - CHAT_BOX_START_Y – The starting vertical position of the chat box on the display.
  - CHAT_BOX_HEIGHT – The height of the chat box on the display.
  - LINE_HEIGHT – The height of a single line of text.
  - CHAT_BOX_BOTTOM_PADDING – Padding below the last line in the chat box.
  - CHAT_BOX_START_X – The starting horizontal position of the chat box.
  - CHAT_BOX_WIDTH – The width of the chat box.
  - ILI9341_WHITE – Color constant for clearing the chat box background.
  - ILI9341_RED – Color constant for the chat box border.

  Drawing method params (for reference):
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

  // Iterates over and draws each message, starting with whatever message is at curr_message_index, then the next most recent message, and so on
  for (int drawn_message_count = 0; drawn_message_count < messages_to_display_count; drawn_message_count++) {
    // Stuff to get timestamp:
    struct tm *timeinfo = localtime(&state->chat_history[curr_message_index].timestamp); // Converts Unix timestamp to local time format
    char time_as_str[8];  // Buffer for "00:00am" format (7 chars + '\0') to hold final string that will get displayed
    strftime(time_as_str, sizeof(time_as_str), "%I:%M%p", timeinfo); // Puts time in a 00:00PM (or AM) format

    // Stuff to calculate the number of lines this message will occupy:
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

    // Stuff to draw message box and timestamp for incoming messages:
    if (strcmp(state->chat_history[curr_message_index].recipient, RECIPIENT_UNKEY) == 0) {
      // Draws timestamp at current line:
      tft.drawString(time_as_str, INCOMING_TIMESTAMP_START_X, draw_start_y);
      tft.drawRect(INCOMING_BORDER_START_X, border_start_y, INCOMING_BORDER_WIDTH, border_height, ILI9341_BLUE);

    // Stuff to draw message box and timestamp for outgoing messages:
    } else {
      tft.drawString(time_as_str, OUTGOING_TIMESTAMP_START_X, draw_start_y);
      tft.drawRect(OUTGOING_BORDER_START_X, border_start_y, OUTGOING_BORDER_WIDTH, border_height, ILI9341_LIGHTGREY);
    }

    // Stuff to draw text chars for incoming messages:
    if (strcmp(state->chat_history[curr_message_index].recipient, RECIPIENT_UNKEY) == 0) {
      draw_message_text(text_length, state->chat_history[curr_message_index].text, INCOMING_TEXT_START_X, draw_start_y, CHAT_WRAP_LIMIT);
    // Stuff to draw text chars for outgoing messages:
    } else {
      draw_message_text(text_length, state->chat_history[curr_message_index].text, OUTGOING_TEXT_START_X, draw_start_y, CHAT_WRAP_LIMIT);
    }

    // Clips anything that scrolls past upper bound of chat history box (lib doesn't have a function for this)
    tft.fillRect(CHAT_BOX_START_X, BATTERY_BOX_HEIGHT, CHAT_BOX_WIDTH, SPACE_UNDER_BATTERY_WIDTH, ILI9341_WHITE); // under battery display
    tft.fillRect(BATTERY_BOX_WIDTH, 0, SPACE_BESIDE_BATTERY_WIDTH, CHAT_BOX_START_Y, ILI9341_WHITE); // next to battery display

    curr_message_pos -= (box_height + CHAT_BOX_LINE_PADDING); // Separates messages
    curr_message_index = (curr_message_index - 1 + MAX_CHAT_MESSAGES) % MAX_CHAT_MESSAGES; // Gets next most recent message from ring buffer
  }
}

void add_message_to_chat_history(ChatBufferState* state, const char* message_text, const char* sender, const char* recipient) {
  message_t curr_message {
    time(NULL), // Current time
    sender,
    recipient,
    ""
  };

  // Have to copy into curr_message like this bc message_text won't be available in mem
  strncpy(curr_message.text, message_text, MAX_TEXT_LENGTH - 1);
  strncpy(curr_message.sender, sender, MAX_NAME_LENGTH - 1);
  strncpy(curr_message.recipient, recipient, MAX_NAME_LENGTH - 1);
  curr_message.text[MAX_TEXT_LENGTH - 1] = '\0';
  curr_message.sender[MAX_NAME_LENGTH - 1] = '\0';
  curr_message.recipient[MAX_NAME_LENGTH - 1] = '\0';

  // Ring buffer logic to overwrite oldest message when buffer is exceeded
  state->chat_history[state->message_buffer_write_index] = curr_message;
  state->message_buffer_write_index = (state->message_buffer_write_index + 1) % MAX_CHAT_MESSAGES;
  if (state->chat_history_message_count < MAX_CHAT_MESSAGES) {
    state->chat_history_message_count++;
  }

}

void transmit_message() {
  // TODO: Encode/transmit to recipient device
}

/*
  str --> sequence of voltages
  str of chars --> 8 bits per char --> 8 voltages per char --> len(str) * 8 voltages in total

  address of 0 --> writing to channel 0 of the DAC.
  write_to_dac(address=0, val=0): DAC receives a 24-bit message that sets the output to the minimum voltage (0V)
  write_to_dac(address=0, val=4095): DAC receives a 24-bit message that sets the output to the maximum voltage
*/
void encode_message(const char* message_to_encode) {
  for (int i = 0; message_to_encode[i] != '\0'; i++) {
    char letter = message_to_encode[i]; // letter is an unsigned char

    Serial.print("Processing letter: ");
    Serial.println(letter);

    // Encode each of char's 8 bits into a corresponding frequency using msb
    // So each char will always generate 8 frequencies
    for (int j = 7; j >= 0; j--) {
      int bit = (letter >> j) & 1;

      // Generate voltage for bit (interrupts could disrupt the SPI transaction)
      noInterrupts();
      write_to_dac(0, (bit) ? 4095 : 0); // 4095 (12-bit): 111111111111
      interrupts();

      // Hold the voltage for 10 ms (bit duration) ~ baud rate of 100 bpsS
      delayMicroseconds(10); // Change back to 10
    }
  }
}

void send_message(const char* message_text) {
  encode_message(message_text);

  // transmit_message(message_text);

  add_message_to_chat_history(&chat_buffer_state, message_text, RECIPIENT_UNKEY, RECIPIENT_VOID);

  display_chat_history(&chat_buffer_state);
}

/*
  Reads the state of a keyboard by polling a shift register connected to the keyboard’s data and clock lines. 
  It detects new key presses, updates the screen, and processes specific keys like CAPS, SYM, and SEND. 
  It also uses ilog2() to identify the index of the pressed key.
*/
void poll_keyboard(ChatBufferState* state) {
  static int modifier = 0;

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
  // `~` is bitwise not, flips all 0s/1s
  // `&` is bitwise and
  uint64_t new_press = ~read_buffer & switch_state; // isolates the bits where a pressed key is now unpressed to find newly pressed keys
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

/*
  Configures the keyboard polling mechanism by setting up input/output pins and 
  starting a timer that calls poll_keyboard at regular intervals
*/
void setup_keyboard_poller() {
  switch_state = 0;

  // Sets up SPI
  pinMode(kb_load_n, OUTPUT);
  digitalWrite(kb_load_n, HIGH);
  pinMode(kb_clock, OUTPUT);
  pinMode(kb_data, INPUT);

  // Starts timer
  if (!keyboard_poller_timer.begin([]() { poll_keyboard(&chat_buffer_state); }, keyboard_poller_period_usec)) {
    Serial.println("Failed setting up poller");
  }

}

/*
  Resets the buffer that stores the text being typed on the keyboard. It clears the buffer and resets its length.
  Keyboard will write to this, screen will display it.
  Usage: Called in setup_screen to initialize the display buffer, and in poll_keyboard to reset it after the SEND key is pressed.
*/
static void reset_tx_display_buffer() {
  memset(tx_display_buffer, '\0', MAX_TEXT_LENGTH); // Fills buffer with null chars ('\0')
  tx_display_buffer_length = 0;
}

/*
  Clears the display area where typed text is shown and redraws the boundary of the text box. 
  It also reprints the current contents of the tx_display_buffer.
  Usage: Called in poll_keyboard when the buffer changes and needs to be updated on the screen.
*/
static void redraw_typing_box() {
  tft.fillRect(TYPING_BOX_START_X, TYPING_BOX_START_Y, CHAT_BOX_WIDTH, TYPING_BOX_HEIGHT, ILI9341_WHITE);
  tft.drawRect(TYPING_BOX_START_X, TYPING_BOX_START_Y, CHAT_BOX_WIDTH, TYPING_BOX_HEIGHT, ILI9341_RED);
  draw_message_text(tx_display_buffer_length, tx_display_buffer, TYPING_CURSOR_X, TYPING_CURSOR_Y, SEND_WRAP_LIMIT);
}

/*
  Initializes the TFT screen, sets the screen orientation, clears the screen, and draws some basic UI elements
*/
void setup_screen() {
  pinMode(tft_led, OUTPUT);
  digitalWrite(tft_led, HIGH); // Actual thing that turns screen light on
  screen_on = true;

  pinMode(tft_sck, OUTPUT); // sck is clock

  tft.begin();
  tft.setRotation(2);
  tft.fillScreen(ILI9341_WHITE);
  // Draws chat history boundaries
  tft.drawRect(CHAT_BOX_START_X, CHAT_BOX_START_Y, CHAT_BOX_WIDTH, CHAT_BOX_HEIGHT, ILI9341_RED);
  // Draws typing box boundaries
  tft.drawRect(CHAT_BOX_START_X, TYPING_BOX_START_Y, CHAT_BOX_WIDTH, TYPING_BOX_HEIGHT, ILI9341_RED);
  // Sets cursor to starting position inside typing box:
  tft.setCursor(TYPING_CURSOR_X, TYPING_CURSOR_Y);

  reset_tx_display_buffer();
}

/*
  Usage: Used in poll_battery to display the current battery level on the screen.
*/
inline float read_battery_voltage() {
  // 2.0 for 1:1 voltage divider, 3.3V is max ADC voltage, and ADC is 12-bit (4096 values)
  return 2.0 * analogRead(battery_monitor) * 3.3 / 4096;
}

/*
  Periodically reads and displays the battery voltage on the screen.
  Usage: Called in loop to update the battery status, and in setup_screen to initialize the display.
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

/*
  Currently used to simulated staggered incoming messages
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

/*
  The main/top-level setup function that initializes the serial communication, SPI, and I2C; 
  calls other setup functions to configure the screen, receiver, transmitter, and keyboard poller.
  Usage: The first function called in the program to initialize all components.
*/
void setup() {
  Serial.begin(9600);  // initializes serial communication with Teensy at baud rate of 9600 bps
  while (!Serial && millis() < 5000) ;  // Checks if connection is working and waits up to 5 sec for it to happen
  delay(100);
  Serial.println("============================\nStarting setup()");

  SPI.begin();               // SPI commuincation bus for keyboard/display etc
  Wire.begin();              // I2C communication bus for charge amplifier
  analogReadResolution(12);  // specifies 12-bit resolution

  // test_incoming_message.begin(incoming_message_callback, 10000000);
  // encode_message("Hello, World!");

  setup_screen();

  setup_receiver();

  setup_transmitter();

  setup_keyboard_poller();

  Serial.println("setup() complete\n============================");
}

/*
  Continuously polls the battery voltage and updates the display with batt charge level.
  Usage: Runs repeatedly after setup() completes, handling ongoing tasks.
*/
void loop() {
  uint16_t val = 2048 + 2047 * sin(2*3.14159*micros()/1e6 * 1.5e3); // tryna make a number between 0 and 2^12 i.e. 12 bits

  // encode_message("Hello");

  poll_battery();
}
