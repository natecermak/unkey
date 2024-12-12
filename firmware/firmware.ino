#include <ADC.h>
#include <DMAChannel.h>
#include <ILI9341_t3n.h>
#include <SPI.h>
#include <Wire.h>
#include "goertzel.h"
// #include "font_Arial.h"

#define CHAT_HISTORY_LINE_PADDING 11 // Extra vertical space between lines in chat history box area
#define CHAT_HISTORY_X 0 // Chat history box horizontal offset (0 is flush to left screen bound)
#define CHAT_HISTORY_Y 25 // Chat history box vertical offset (0 is flush to top screen bound)
#define CHAT_HISTORY_W 235 // Chat history box width
#define CHAT_HISTORY_H 201 // // Chat history box height
#define CHAT_HISTORY_BOTTOM_PADDING 3
#define CHAR_WIDTH 7 // Width of each character in pixels
#define CHAT_WRAP_LIMIT 16 // Sender message wrapping cut-off
#define LINE_HEIGHT 10 // Height of each line in pixels
#define MAX_CHAT_MESSAGES 50
#define MAX_NAME_LENGTH 20
#define MAX_TEXT_LENGTH 400 // Unit is characters
#define RECEIVED_MESSAGE_X 70 // Received message horizontal offset
#define SCAN_CHAIN_LENGTH 56  // 56 keys that need to be checked per polling cycle
#define SENT_MESSAGE_X 120 // Sent message horizontal offset
#define SEND_WRAP_LIMIT 30 // Sender message wrapping cut-off
#define TEXT_SIZE 1 // Text size multiplier (depends on your display)
#define TIMESTAMP_LEFT_INDENT 60 // Timestamp horizontal offset (0 is flush to left screen bound)
#define TYPING_BOX_LINE_PADDING 2 // Extra vertical space between lines in typing box area
#define TYPING_BOX_X 0 // Typing box horizontal offset (0 is flush to left screen bound)
#define TYPING_BOX_Y 225 // Typing box vertical offset (0 is flush to top screen bound
#define TYPING_BOX_H 90 // Typing box height
#define TYPING_CURSOR_X 2 // Typing cursor horizontal offset (0 is flush to left screen bound)
#define TYPING_CURSOR_Y 227 // Typing cursor vertical offset (0 is flush to top screen bound

// ------------------- TYPES ----------------------------------------------- //
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
/* Initializing the display using pins assigned above, which - as a remidner - interface
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
 Usage: Gets called by poll_keyboard to detect key presses (?)
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
  } // measures amp of specific freq over some time period in a received audio signal --> bytes

  // set gain on charge amplifier
  set_charge_amplifier_gain(6);

  // Set up ADC (for received audio signal)
  adc->adc0->setAveraging(1); // no averaging
  adc->adc0->setResolution(12); // bits
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // we want it to be
  //adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

  // Set up DMA
  dma_ch1.source((volatile uint16_t &)(ADC1_R0));
  dma_ch1.destinationBuffer((uint16_t *)dma_adc_buff1, buffer_size * 2); // each time you read from adc get 2 bytes, so that's why 2x
  dma_ch1.interruptAtCompletion();
  dma_ch1.disableOnCompletion();
  dma_ch1.attachInterrupt(&adc_buffer_full_interrupt); // when dma is done, call adc_buffer_full_interrup which is a func
  dma_ch1.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);

  dma_ch1.enable();  // Enable the DMA channel
  adc->adc0->enableDMA();
  adc->adc0->startSingleRead(readPin_adc_0);
  adc->adc0->startTimer(adc_frequency); // this actually determines how fast to sample signal, 
  // and starts timer to initiate dma transfer from adc to memory
  // once 2x buffer size bytes reached
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
  uint8_t buf[3];
  buf[0] = (address << 3);  // bits 1 and 2 must be 0 to write.
  buf[1] = (uint8_t)(value >> 8);
  buf[2] = (uint8_t)value;
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dac_cs, LOW);
  SPI.transfer(buf, 3);
  digitalWrite(dac_cs, HIGH);
  SPI.endTransaction();
}

// ------------------- Screen Behavior Vars/Constants -------------------- //

message_t chat_history[MAX_CHAT_MESSAGES];
int chat_history_message_count = 0;
int message_scroll_offset = 0; // Tracks the current scroll offset, representing the number of messages scrolled up from the most recent message
int message_buffer_write_index = 0; // Head pointer (next position to write)
// Note: No tail pointer needed unless we want to delete messages
IntervalTimer test_incoming_message;
int incoming_message_count = 0;
const int max_incoming = 4;

// ------------------- Screen Behavior Utility Functions ----------------- //

void _debug_print_message(message_t msg) {
  // Serial.printf("Timestamp: %lu \n", msg.timestamp); // %d would also work, %lu is long unsigned, which time_t is on teensy
  // Serial.print("Sender: ");
  // Serial.println(msg.sender);
  // Serial.print("Recipient: ");
  // Serial.println(msg.recipient);
  Serial.print("Text: ");
  Serial.println(msg.text);
  Serial.println();
}

void _debug_print_chat_history() {
  for (int i = 0; i < chat_history_message_count; i++) {
    _debug_print_message(chat_history[i]);
  }
}

/*
  Clears the chat history display area and redraws messages from chat_history, starting with the message at
  the currently set scroll position (with progressively older messages displayed above).
  Redrawing is accomplished by first calculating the needed screen space for each message, factoring in
  formatting rules, and also obscures any message content that appears outside the chat history box bounds.

  Global Variables Used:
  - `message_buffer_write_index`: Tracks the head index of the message buffer (ring buffer).
  - `message_scroll_offset`: Tracks the current scroll offset, representing the number of messages scrolled up from the most recent message
  - `MAX_CHAT_MESSAGES`: The total number of messages that can be stored in the chat buffer.
  - `chat_history`: The array storing all chat messages.
  - `chat_history_message_count`: The current number of messages in the chat history.
  - `CHAT_HISTORY_X`, `CHAT_HISTORY_Y`, `CHAT_HISTORY_W`, `CHAT_HISTORY_H`: Dimensions and position of the chat history box.
  - `LINE_HEIGHT`: The height of each line in pixels.
  - `CHAT_HISTORY_BOTTOM_PADDING`: Extra padding at the bottom of the chat history box.
  - `CHAT_WRAP_LIMIT`: Maximum number of characters per line before wrapping.
  - `MAX_TEXT_LENGTH`: Maximum allowable length of a single message.
  - `tft`: The display object used for drawing.
*/
void display_chat_history() {
  /*
    Note that curr_message_index points to the current message in the buffer being displayed or accessed, adjusted for the user's scroll position.
    So if message_scroll_offset is 0, that means the most recent message in the history is being shown at the bottom (UP has not been pressed).
    Pressing the UP key once increments message_scroll_offset to 1, which means the NEXT most recent message is displayed at the bottom.
    From there, the outer for loop is responsible for drawing all of the previous message, starting at chat_history[curr_message_index] and incrementing curr_message_index by 1.
  */
  int curr_message_index = (message_buffer_write_index - 1 - message_scroll_offset + MAX_CHAT_MESSAGES) % MAX_CHAT_MESSAGES;
  int curr_message_pos = CHAT_HISTORY_Y + CHAT_HISTORY_H - LINE_HEIGHT - CHAT_HISTORY_BOTTOM_PADDING;
  int messages_to_display_count = chat_history_message_count - message_scroll_offset;

  // Serial.println("✨✨✨✨✨✨✨✨✨");
  // Serial.printf("message_buffer_write_index: %d\n", message_buffer_write_index);
  // Serial.printf("message_scroll_offset: %d\n", message_scroll_offset);
  // Serial.printf("curr_message_index: %d\n", curr_message_index);
  // Serial.println("🧩🧩🧩🧩🧩🧩🧩🧩🧩");

  // Clear entire chat box area
  tft.fillRect(CHAT_HISTORY_X, 10, CHAT_HISTORY_W, CHAT_HISTORY_H + 13, ILI9341_WHITE);
  tft.drawRect(CHAT_HISTORY_X, CHAT_HISTORY_Y, CHAT_HISTORY_W, CHAT_HISTORY_H, ILI9341_RED);

  // Iterates over and draws each message, starting with whatever message is at curr_message_index, then the next most recent message, and so on
  for (int drawn_message_count = 0; drawn_message_count < messages_to_display_count; drawn_message_count++) {
    // Serial.printf("******** Loop #%d\n", drawn_message_count + 1); // This should never exceed the # of messages in chat history

    // Stuff to get timestamp:
    struct tm *timeinfo = localtime(&chat_history[curr_message_index].timestamp); // Converts Unix timestamp to local time format
    char time_as_str[8];  // Buffer for "00:00am" format (7 chars + '\0') to hold final string that will get displayed
    strftime(time_as_str, sizeof(time_as_str), "%I:%M%p", timeinfo); // Puts time in a 00:00PM (or AM) format

    // Stuff to calculate the number of lines this message will occupy:
    int line_count = 1;
    int text_length = strnlen(chat_history[curr_message_index].text, MAX_TEXT_LENGTH);
    for (int curr_char_index = 0; curr_char_index < text_length; curr_char_index++) {
      if (chat_history[curr_message_index].text[curr_char_index] == '\n' || (curr_char_index % CHAT_WRAP_LIMIT == 0 && curr_char_index > 0)) {
        line_count++;
      }
    }
    int box_height = line_count * LINE_HEIGHT;
    int top_line_y = curr_message_pos - box_height + LINE_HEIGHT;

    // Stuff to draw message box and timestamp for incoming messages:
    if (strcmp(chat_history[curr_message_index].recipient, "unkey") == 0) {
      tft.drawString(time_as_str, TIMESTAMP_LEFT_INDENT - 50, curr_message_pos - (box_height - LINE_HEIGHT)); // Draws timestamp at current line
      // tft.drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
      tft.drawRect(RECEIVED_MESSAGE_X - 8, curr_message_pos - box_height + 6, CHAT_WRAP_LIMIT * CHAR_WIDTH + 20, box_height + 5, ILI9341_BLUE);
    // ...and outgoing messages:
    } else {
      // Serial.println("💬 Drawing an outgoing message box and timestamp");
      tft.drawString(time_as_str, TIMESTAMP_LEFT_INDENT, curr_message_pos - (box_height - LINE_HEIGHT)); // Draws timestamp at current line
      tft.drawRect(SENT_MESSAGE_X - 8, curr_message_pos - box_height + 6, CHAT_WRAP_LIMIT * CHAR_WIDTH + 10, box_height + 5, ILI9341_LIGHTGREY);
    }
    // Serial.println("🔠 Text:");
    Serial.println(chat_history[curr_message_index].text);

    // Stuff to draw text chars for incoming messages:
    if (strcmp(chat_history[curr_message_index].recipient, "unkey") == 0) {
      int draw_start_x = RECEIVED_MESSAGE_X;
      int draw_start_y = top_line_y;
      for (int k = 0; k < text_length; k++) {
        if (chat_history[curr_message_index].text[k] == '\n') {
          // Creates a new line:
          draw_start_x = RECEIVED_MESSAGE_X;
          draw_start_y += LINE_HEIGHT;
        } else if (k % CHAT_WRAP_LIMIT == 0 && k > 0) {
          // Draws curr char on same line, then adds a new line:
          tft.drawChar(draw_start_x, draw_start_y, chat_history[curr_message_index].text[k], ILI9341_BLACK, ILI9341_WHITE, TEXT_SIZE, TEXT_SIZE);
          draw_start_x = RECEIVED_MESSAGE_X;
          draw_start_y += LINE_HEIGHT;
        } else {
          // Continues on same line:
          tft.drawChar(draw_start_x, draw_start_y, chat_history[curr_message_index].text[k], ILI9341_BLACK, ILI9341_WHITE, TEXT_SIZE, TEXT_SIZE);
          draw_start_x += CHAR_WIDTH;
        }
      }
    // ...and outgoing messages:
    } else {
      int draw_start_x = SENT_MESSAGE_X;
      int draw_start_y = top_line_y;
      for (int k = 0; k < text_length; k++) {
        if (chat_history[curr_message_index].text[k] == '\n') {
          draw_start_x = SENT_MESSAGE_X;
          draw_start_y += LINE_HEIGHT;
        } else if (k % CHAT_WRAP_LIMIT == 0 && k > 0) {
          tft.drawChar(draw_start_x, draw_start_y, chat_history[curr_message_index].text[k], ILI9341_BLACK, ILI9341_WHITE, TEXT_SIZE, TEXT_SIZE);
          draw_start_x = SENT_MESSAGE_X;
          draw_start_y += LINE_HEIGHT;
        } else {
          tft.drawChar(draw_start_x, draw_start_y, chat_history[curr_message_index].text[k], ILI9341_BLACK, ILI9341_WHITE, TEXT_SIZE, TEXT_SIZE);
          draw_start_x += CHAR_WIDTH;
        }
      }
    }

    // Clips anything that scrolls past upper bound of chat history box (lib doesn't have a function for this)
    tft.fillRect(CHAT_HISTORY_X, 10, CHAT_HISTORY_W, 15, ILI9341_WHITE); // under battery display
    tft.fillRect(80, 0, CHAT_HISTORY_W - 80, CHAT_HISTORY_Y, ILI9341_WHITE); // next to battery display

    curr_message_pos -= (box_height + CHAT_HISTORY_LINE_PADDING); // Separates messages
    curr_message_index = (curr_message_index - 1 + MAX_CHAT_MESSAGES) % MAX_CHAT_MESSAGES; // Gets next most recent message from ring buffer
  }
}

void add_message_to_chat_history(const char* message_text, const char* sender, const char* recipient) {
  message_t curr_message {
    time(NULL), // current time
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

  // Ring buffer logic to overwrite oldest message when buffer is exceeded:
  chat_history[message_buffer_write_index] = curr_message;
  message_buffer_write_index = (message_buffer_write_index + 1) % MAX_CHAT_MESSAGES;
  if (chat_history_message_count < MAX_CHAT_MESSAGES) {
    chat_history_message_count++;
  }

}

void transmit_message() {
  // TODO: Encode/transmit to recipient device
}

void send_message(const char* message_text) {
  // transmit_message(message_text);

  add_message_to_chat_history(message_text, "unkey", "the void");

  display_chat_history();
}

/*
  Reads the state of a keyboard by polling a shift register connected to the keyboard’s data and clock lines. 
  It detects new key presses, updates the screen, and processes specific keys like CAPS, SYM, and SEND. 
  It also uses ilog2() to identify the index of the pressed key.
*/
void poll_keyboard() {
  static int modifier = 0;

  // latch keyboard state into shift registers
  // kb_load_n is an output pin which pulses a signal here from LOW to HIGH 
  // low to hi transmission lets shift reg know to record keyboard state (8 bits per registr)
  digitalWrite(kb_load_n, LOW);
  delayNanoseconds(5);
  digitalWrite(kb_load_n, HIGH);
  delayNanoseconds(5);

  // read 64 bits of shift register, takes 2.63us, runs at 23 MHz
  // Note: bitbang to avoid taking up an SPI peripheral. Requires VERY little CPU time
  uint64_t read_buffer = 0;
  for (int bit = 0; bit < SCAN_CHAIN_LENGTH; bit++) {
    read_buffer |= (uint64_t)digitalReadFast(kb_data) << ((SCAN_CHAIN_LENGTH - 1) - bit); // kb_data is the state of each key press
    digitalWriteFast(kb_clock, LOW);
    digitalWriteFast(kb_clock, HIGH);
    delayNanoseconds(10);
  }
  digitalWriteFast(kb_clock, LOW);

  // compare results to previous one to detect new key presses
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
          off anyway.
          if (message_scroll_offset == chat_history_message_count - 1), that means the oldest message is
          currently displayed at the bottom of the history box
          */
          Serial.printf("message_scroll_offset: %d\n", message_scroll_offset);
          Serial.printf("chat_history_message_count: %d\n", chat_history_message_count);
          if (message_scroll_offset < chat_history_message_count - 1) {
            message_scroll_offset++;
            Serial.printf("✅ Scroll successful");
            display_chat_history();
          } else {
            // message_scroll_offset should never exceed (chat_history_message_count - 1)
            // even if the user keeps pressing up
            Serial.println("❌ Scrolling up not allowed");
          }
          break;
        case DOWN_KEY_INDEX:
          Serial.println("You pressed DOWN");
          if (message_scroll_offset > 0) {
            message_scroll_offset--;
            display_chat_history();
          } else {
            Serial.println("❌ Scrolling down not allowed");
          }
          break;
        case BACK_KEY_INDEX:
          Serial.println("You pressed BACKSPACE");
          tx_display_buffer[tx_display_buffer_length] = '\0';
          tx_display_buffer_length--;
          redraw_tx_display_window();
          break;
        case RET_KEY_INDEX:
          Serial.println("You pressed RETURN");
          if (tx_display_buffer_length < MAX_TEXT_LENGTH - 1) {
            tx_display_buffer[tx_display_buffer_length] = '\n';
            tx_display_buffer_length++;
          }
          redraw_tx_display_window();
          break;
        case SEND_KEY_INDEX:
          Serial.println("You pressed SEND");
          Serial.printf("Current message length: %d\n", tx_display_buffer_length);
          if (tx_display_buffer_length == 0) {
            Serial.println("No message to send");
            break;
          }
          // Just prints the current content of the buffer:
          // for (int i = 0; i < tx_display_buffer_length; i++) {
          //   Serial.print(tx_display_buffer[i]);
          //   Serial.print('|');
          // }
          // Serial.println();

          send_message(tx_display_buffer);

          // Clears message staging area after it's been sent
          reset_tx_display_buffer();
          redraw_tx_display_window();

          display_chat_history(); // 🔔 is this call necessary? test without
          break;
        default:
          char key = KEYBOARD_LAYOUT[key_index];
          tx_display_buffer[tx_display_buffer_length] = key;
          tx_display_buffer_length++;
          // Serial.printf("Read buffer %ul\n", ~read_buffer);
          // Serial.printf("You pressed key_index=%d, key=\'%c\'\n", key_index, key);
          redraw_tx_display_window(); // TODO: So adding this call makes the line break display correctly, but seems inefficient to call redraw_tx_display_window everytime - need to spend more time figuring out how drawString works and add logic here to handle line breaks w/o redraw?
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
  Serial.println("setup_keyboard_poller() called");
  switch_state = 0;

  // Set up SPI
  pinMode(kb_load_n, OUTPUT);
  digitalWrite(kb_load_n, HIGH);
  pinMode(kb_clock, OUTPUT);
  pinMode(kb_data, INPUT);

  // Start timer
  if (!keyboard_poller_timer.begin(poll_keyboard, keyboard_poller_period_usec)) {
    Serial.println("Failed setting up poller");
  }
}

/*
  Resets the buffer that stores the text being typed on the keyboard. It clears the buffer and resets its length.
  Usage: Called in setup_screen to initialize the display buffer, and in poll_keyboard to reset it after the SEND key is pressed.
*/
static void reset_tx_display_buffer() {
  // set up transmit display buffer. keyboard will write to this, screen will display it.
  memset(tx_display_buffer, '\0', MAX_TEXT_LENGTH); // fill buffer with null chars ('\0')
  tx_display_buffer_length = 0;
}

/*
  Clears the display area where typed text is shown and redraws the boundary of the text box. 
  It also reprints the current contents of the tx_display_buffer.
  Usage: Called in poll_keyboard when the buffer changes and needs to be updated on the screen.
*/
static void redraw_tx_display_window() {
  // Clears the typing box after message send:
  tft.fillRect(TYPING_BOX_X, TYPING_BOX_Y, CHAT_HISTORY_W, TYPING_BOX_H, ILI9341_WHITE);
  tft.drawRect(TYPING_BOX_X, TYPING_BOX_Y, CHAT_HISTORY_W, TYPING_BOX_H, ILI9341_RED);

  // TODO: this is stuff to display the line breaks - might want to move out into a helper func
  int draw_start_x = TYPING_CURSOR_X;
  int draw_start_y = TYPING_CURSOR_Y;
  for (int i = 0; i < tx_display_buffer_length; i++) {
    if (tx_display_buffer[i] == '\n') {
      // Move the cursor to the next line (adjust draw_start_y based on text size)
      draw_start_x = TYPING_CURSOR_X;
      draw_start_y += LINE_HEIGHT + TYPING_BOX_LINE_PADDING;
    } else if (i % SEND_WRAP_LIMIT == 0 && i > 0) {
      tft.drawChar(draw_start_x, draw_start_y, tx_display_buffer[i], ILI9341_BLACK, ILI9341_WHITE, TEXT_SIZE, TEXT_SIZE);
      draw_start_x = TYPING_CURSOR_X;
      draw_start_y += LINE_HEIGHT + TYPING_BOX_LINE_PADDING;
    } else {
      // Draw the character at the current cursor position
      tft.drawChar(draw_start_x, draw_start_y, tx_display_buffer[i], ILI9341_BLACK, ILI9341_WHITE, TEXT_SIZE, TEXT_SIZE);
      draw_start_x += CHAR_WIDTH;
    }
  }
}

/*
  Initializes the TFT screen, sets the screen orientation, clears the screen, and draws some basic UI elements
*/
void setup_screen() {
  pinMode(tft_led, OUTPUT);
  digitalWrite(tft_led, HIGH); // actual thing that turns screen light on
  screen_on = true; // global var for other stuff

  pinMode(tft_sck, OUTPUT); // sck is clock

  tft.begin();
  tft.setRotation(2);
  tft.fillScreen(ILI9341_WHITE);
  tft.drawRect(CHAT_HISTORY_X, CHAT_HISTORY_Y, CHAT_HISTORY_W, CHAT_HISTORY_H, ILI9341_RED); // chat history boundaries
  tft.drawRect(CHAT_HISTORY_X, TYPING_BOX_Y, CHAT_HISTORY_W, TYPING_BOX_H, ILI9341_RED); // typing box boundaries
  tft.setCursor(TYPING_CURSOR_X, TYPING_CURSOR_Y); // set cursor to starting position inside typing box

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

void incoming_message_callback() {
  const char *test_message_text = "Incoming from The Void";
  add_message_to_chat_history(test_message_text, "the void", "unkey");
  display_chat_history();
  incoming_message_count++;
  if (incoming_message_count >= max_incoming) {
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

  test_incoming_message.begin(incoming_message_callback, 10000000);

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
  // uint16_t val = 2048 + 2047 * sin(2*3.14159*micros()/1e6 * 1.5e3); // tryna make a number between 0 and 2^12 i.e. 12 bits

  // write_to_dac(0, val);

  poll_battery();
}
