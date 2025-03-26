#include <ADC.h>
#include <DMAChannel.h>
#include <ILI9341_t3n.h>
#include <SPI.h>
#include <Wire.h>

#include "config.h"
#include "hardware_config.h"
#include "comm/goertzel.h"
#include "comm/goertzel.cpp"
#include "comm/comm.h"
#include "comm/comm.cpp"
#include "chat/chat_logic.h"
#include "chat/chat_logic.cpp"
#include "display/display.h"
#include "display/display.cpp"
#include "keyboard/keyboard.h"



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

// ------------------- Screen Behavior Utility Functions ----------------- //

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

  test_incoming_message.begin(incoming_message_callback, 1000000);
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
