#include <ADC.h>
#include <DMAChannel.h>
#include <ILI9341_t3n.h>
#include <SPI.h>
#include <Wire.h>
#include "goertzel.h"
// #include "font_Arial.h"

#define CHAR_WIDTH 7 // Width of each character in pixels
#define LINE_HEIGHT 10 // Height of each line in pixels
#define MAX_CHAT_MESSAGES 50
#define SCAN_CHAIN_LENGTH 56  // 56 keys that need to be checked per polling cycle
#define TEXT_SIZE 1 // Text size multiplier (depends on your display)
#define TX_DISPLAY_BUFFER_X 2 // horizontal offset when user starts typing (0 is flush to left screen bound)
#define TX_DISPLAY_BUFFER_Y 227 // vertical offset when user starts typing (0 is flush to top screen bound)
#define TX_DISPLAY_BUFFER_SIZE 400 // max message length
#define CHAT_WRAP_LIMIT 16 // Sender message wrapping cut-off
#define SEND_WRAP_LIMIT 30 // Sender message wrapping cut-off

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
char tx_display_buffer[TX_DISPLAY_BUFFER_SIZE];
uint16_t tx_display_buffer_length;

// ------------------- Battery monitoring --------------------------------- //
long time_of_last_battery_read_ms;
const long BATTERY_READ_PERIOD_MS = 1000;

// ------------------- Utility functions ---------------------------------- //
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
  Usage: Attached to a DMA interrupt in setup_receiver and called automatically when the buffer fills up.
*/
void adc_buffer_full_interrupt() {
  dma_ch1.clearInterrupt();
  // mempcy copies a block of memory from one location to another
  // memcpy(destination, source, number_of_bytes_to_copy) --> expects void * arguments (void pointer)
  memcpy((void *)adc_buffer_copy, (void *)dma_adc_buff1, sizeof(dma_adc_buff1));
  if ((uint32_t)dma_adc_buff1 >= 0x20200000u)
    arm_dcache_delete((void *)dma_adc_buff1, sizeof(dma_adc_buff1));
  dma_ch1.enable();  // Re-enable the DMA channel for next read

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
    // Serial.println();
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
  writing initial values to a DAC (Digital-to-Analog Converter). It configures gain and voltage references for the DAC.
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

struct message {
  // message ID (might need once we need to handle incoming messages)
  // chat ID (only necessary if one device can have multiple chats)
  // sender device ID (" " "")
  // recipient device ID? (" " "")
  time_t timestamp;
  char sender[20];
  char recipient[20];
  char text[TX_DISPLAY_BUFFER_SIZE];
};

struct message chat_history[MAX_CHAT_MESSAGES];
int chat_history_message_count = 0;

void print_message(struct message msg) {
    Serial.print("Timestamp: ");
    Serial.println(msg.timestamp);
    Serial.print("Sender: ");
    Serial.println(msg.sender);
    Serial.print("Recipient: ");
    Serial.println(msg.recipient);
    Serial.print("Text: ");
    Serial.println(msg.text);
}

void print_chat_history() {
  for (int i = 0; i < chat_history_message_count; i++) {
    print_message(chat_history[i]);
  }
}

void add_message_to_chat_history(const char* message_text) {
  if (chat_history_message_count < MAX_CHAT_MESSAGES) {
    struct message curr_message {
      time(NULL), // current time
      "Recipient",
      "Me",
      ""
    };
    strncpy(curr_message.text, message_text, TX_DISPLAY_BUFFER_SIZE - 1);
    curr_message.text[TX_DISPLAY_BUFFER_SIZE - 1] = '\0';

    print_message(curr_message);
    chat_history[chat_history_message_count] = curr_message;
    chat_history_message_count++;
  } else {
    Serial.println("Chat history is full");
  }
}

void render_chat_history() {
  print_chat_history();
  Serial.println("render_chat_history called");

  int chat_history_x = 120; // Note: this currently allows for 16 chars per line
  int chat_history_y = 30;

  for (int i = 0; i < chat_history_message_count; i++) {
    // Stuff to format timestamp into a sensical string:
    struct tm *timeinfo = localtime(&chat_history[i].timestamp);
    char time_str[8];  // Buffer for "12:34am" (7 chars + '\0')
    strftime(time_str, sizeof(time_str), "%I:%M%p", timeinfo);

    // Draws timestamp at current line:
    tft.drawString(time_str, 60, chat_history_y);

    // Draws message content, accounting for line breaks
    int cursor_x = chat_history_x;

    // TODO: Maybe a little hacky, using j in lieu of just getting the text message length?
    for (int j = 0; j < TX_DISPLAY_BUFFER_SIZE; j++) {
      // Since we don't know the length of the message:
      if (chat_history[i].text[j] == '\0') {
        break;
      }
      if (chat_history[i].text[j] == '\n' || (j % CHAT_WRAP_LIMIT == 0 && j > 0)) {
        // Moves the cursor to the next line:
        cursor_x = chat_history_x;
        chat_history_y += LINE_HEIGHT;
      } else {
        // Draws the character at the current cursor position:
        tft.drawChar(cursor_x, chat_history_y, chat_history[i].text[j], ILI9341_BLACK, ILI9341_WHITE, TEXT_SIZE, TEXT_SIZE);
        cursor_x += (CHAR_WIDTH);
      }
    }

    // Separates messages:
    chat_history_y += LINE_HEIGHT + 6;
  }

  // TODO: Make complete history scrollable
}

void transmit_message(char message) {
  // TODO: Encode/transmit to recipient device
}

void send_message(const char* message) {
  // transmit_message(message);

  add_message_to_chat_history(message);
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
        case BACK_KEY_INDEX:
          Serial.println("You pressed BACKSPACE");
          tx_display_buffer[tx_display_buffer_length] = '\0';
          tx_display_buffer_length--;
          redraw_tx_display_window();
          break;
        case RET_KEY_INDEX:
          Serial.println("You pressed RETURN!!");
          if (tx_display_buffer_length < TX_DISPLAY_BUFFER_SIZE - 1) {
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
          for (int i = 0; i < tx_display_buffer_length; i++) {
            Serial.print(tx_display_buffer[i]);
            Serial.print('|');
          }
          Serial.println();

          send_message(tx_display_buffer);

          // Clears message staging area after it's been sent
          reset_tx_display_buffer();
          redraw_tx_display_window();

          render_chat_history();
          break;
        default:
          char key = KEYBOARD_LAYOUT[key_index];
          tx_display_buffer[tx_display_buffer_length] = key;
          tx_display_buffer_length++;
          // tft.drawString(tx_display_buffer, tx_display_buffer_length, TX_DISPLAY_BUFFER_X, TX_DISPLAY_BUFFER_Y);
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
  memset(tx_display_buffer, '\0', TX_DISPLAY_BUFFER_SIZE); // fill buffer with null chars ('\0')
  tx_display_buffer_length = 0;
}

/*
  Clears the display area where typed text is shown and redraws the boundary of the text box. 
  It also reprints the current contents of the tx_display_buffer.
  Usage: Called in poll_keyboard when the buffer changes and needs to be updated on the screen.
*/
static void redraw_tx_display_window() {
  Serial.println("redraw_tx_display_window called");
  tft.fillRect(TX_DISPLAY_BUFFER_X - 1, TX_DISPLAY_BUFFER_Y - 1, 234, 90, ILI9341_WHITE); // ❓ For some reason these offsets were required to render the same boundaries in setup_screen, and I can't figure out why
  tft.drawRect(TX_DISPLAY_BUFFER_X - 2, TX_DISPLAY_BUFFER_Y - 2, 234, 90, ILI9341_RED); // same as above

  // TODO: this is stuff to display the line breaks - might want to move out into a helper func
  int cursor_x = TX_DISPLAY_BUFFER_X;
  int cursor_y = TX_DISPLAY_BUFFER_Y;
  for (int i = 0; i < tx_display_buffer_length; i++) {
    if (tx_display_buffer[i] == '\n' || (i % SEND_WRAP_LIMIT == 0 && i > 0)) {
      // Move the cursor to the next line (adjust cursor_y based on text size)
      cursor_x = TX_DISPLAY_BUFFER_X;
      cursor_y += LINE_HEIGHT + 2;
    } else {
      // Draw the character at the current cursor position
      tft.drawChar(cursor_x, cursor_y, tx_display_buffer[i], ILI9341_BLACK, ILI9341_WHITE, TEXT_SIZE, TEXT_SIZE);
      cursor_x += CHAR_WIDTH;
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
  // tft.setFont(Arial_16);
  tft.setRotation(2);

  tft.setCursor(0, 0);
  // tft.setTextWrap(true); // ❓ not working
  tft.fillScreen(ILI9341_WHITE);

  // tft.setTextSize(2);
  // tft.print("Hello, ocean!\n\n> ");
  // tft.setTextColor(0x07FF, 0xFD20); // currently this setting gets overwritten by poll_battery()

  tft.drawRect(0, 25, 235, 201, ILI9341_RED); // chat history
  tft.drawRect(0, 225, 235, 90, ILI9341_RED); // message staging area

  // set cursor to starting position inside typing box
  tft.setCursor(TX_DISPLAY_BUFFER_X, TX_DISPLAY_BUFFER_Y);
  // poll_battery();  // was working without (already happening in loop)

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
