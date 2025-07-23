// ==================================================================
// comm.cpp
// Handles analog signal transmission, reception, and DSP setup
// ==================================================================
#include <Arduino.h> // for digitalWriteFast
#include <ADC.h>
#include <DMAChannel.h>
#include <SPI.h>
#include <Wire.h>

#include "comm.h"
#include "chat_logic.h"
#include "config.h"
#include "display.h"
#include "hardware_config.h"
#include "goertzel.h"

// ------------------------------------------------------------------
// Testing Accessors
// ------------------------------------------------------------------

// Holds a test-provided function pointer that overrides normal deliver_message behavior in tests:
#ifdef UNIT_TEST
static void (*deliver_fn_override)(const char*) = nullptr;
#endif

// ------------------------------------------------------------------
// State
// ------------------------------------------------------------------

/*
  adc_buffer_half_2 is filled by the DMA and declared with DMAMEM and __attribute__((aligned(32)))
  to place it in RAM2 (OCRAM), which is cacheable and requires manual cache management.

  RAM1 = DTCM (Tightly Coupled Memory) --> Not cacheable. Always in sync.

  RAM2 = OCRAM (what DMAMEM uses) --> Cacheable. Can get out of sync with the Teensy CPU’s data cache,
  which requires manual cache management: arm_dcache_flush() pushes CPU cache changes to RAM2;
  arm_dcache_delete() discards cache so the CPU reads fresh data from RAM2.

  adc_buffer_half_1 is filled by the CPU using memcpy() and lives in regular (uncached) RAM1.

  adc_buffer_full_bit is the concatenated buffer (half_1 + half_2) that gets analyzed.
*/

// Defines the number of ADC samples collected into a buffer (adc_buffer_half_2) every time the DMA completes a transfer
// Number samples collected per ADC window (buffer_size) = adc_sampling_rate * bit period = 81920 * 5 ms = 410 samples
#ifdef UNIT_TEST
const uint32_t buffer_size = 410;
#else
static const uint32_t buffer_size = 410;
#endif

// Holds 5 ms ADC window chunk that was filled before the currrent (adc_buffer_half_2):
uint16_t adc_buffer_half_1[buffer_size];

// adc_buffer_half_1 is considered valid when it holds a full previous DMA buffer,
// and therefore ready to pair with the current buffer (adc_buffer_half_2) for decoding:
bool buffer_half_1_is_valid = false;

// ADC will sample at freq of 81.92 kHz:
static const uint32_t adc_sampling_rate = 81920;

char tx_display_buffer[MAX_TEXT_LENGTH];
uint16_t tx_display_buffer_length = 0;

ADC *adc = new ADC();
DMAChannel dma_ch1;

// Creates second half of buffer that comprises one bit period when combined with adc_buffer_half_1, and
// DMAMEM places adc_buffer_half_2 in RAM2 (OCRAM):
DMAMEM static volatile uint16_t __attribute__((aligned(32))) adc_buffer_half_2[buffer_size];
uint16_t adc_buffer_full_bit[buffer_size * 2];

// Gets incremented every time decode_single_bit_from_adc_window() runs, and decoding
// only happens when adc_window_counter % SCAN_CHAIN_LENGTH == 0:
static uint8_t adc_window_counter = 0;

// An array that will store state for the Goertzel algo - each goertzel_state obj
// holds data to compute G algo for that frequency:
static const uint8_t gs_len = 10;
goertzel_state gs[gs_len];

// Charge amplifier gain:
static const int adg728_i2c_address = 76;

// For get_bit_from_top_frequency:
static const int MAX_BITS = 256;
static uint8_t bitstream[MAX_BITS];
static int bit_index = 0;
static const float MAGNITUDE_THRESHOLD = 5.0f; // TODO: adjust as needed based on testing

// Packet framing bytes:
// Note: Using two-byte delimiters is more reliable than using one
static const uint8_t PACKET_START1 = 0x01;
static const uint8_t PACKET_START2 = 0x02;
static const uint8_t PACKET_END1   = 0x03;
static const uint8_t PACKET_END2   = 0x04;

// ------------------------------------------------------------------
// Functions
// ------------------------------------------------------------------

/**
 * Sends data to a DAC via SPI: prepares a 3-byte buffer with an address and a 12-bit value, then
 * sends the data using SPI communication.
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
  digitalWrite(dac_cs_pin, LOW);
  SPI.transfer(buf, 3);
  digitalWrite(dac_cs_pin, HIGH);
  SPI.endTransaction();
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
 * Sets the gain on a charge amplifier by writing a specific value to an I2C device, a charge amplifier (controlled by gain_index).
 * It shifts 1U left by gain_index to generate a specific binary pattern and writes this value to the amplifier's address.
 */
void set_charge_amplifier_gain(uint8_t gain_index) {
  Wire.beginTransmission(adg728_i2c_address);
  Wire.write(1U << gain_index);
  Wire.endTransmission();
}

/**
 * Enables/disables transmission power by writing high or low to the tx_power_en_pin pin.
 */
inline void set_tx_power_enable(bool enable) {
  digitalWriteFast(tx_power_en_pin, (enable) ? HIGH : LOW);
}

/**
 * Sets up the transmitter by configuring output pins, enabling transmission power, and writing initial values to a DAC (Digital-to-Analog Converter).
 * Configures gain and voltage references for the DAC.
 */
void setup_transmitter() {
  pinMode(tx_power_en_pin, OUTPUT);
  pinMode(xdcr_sw_pin, OUTPUT);
  pinMode(dac_cs_pin, OUTPUT);
  digitalWrite(dac_cs_pin, HIGH);

  // TODO: FOR TESTING ONLY:
  set_tx_power_enable(true);  // tested: works
  delay(100);                 // wait for power to boot
  write_to_dac(0xA, 1U << 8);  // A is address for config, 8th bit is gain. set to gain=2
  write_to_dac(8, 1);          // 8 is address for VREF, 1 means use internal ref
}

/**
 * Adds message to chat history and refreshes display,
 * or uses test override if in UNIT_TEST mode.
 */
void deliver_message(const char* message) {
  // If a test override is set, this will call it instead of performing normal delivery logic.
  // Allows unit tests to capture or mock delivery without triggering hardware-dependent code:
#ifdef UNIT_TEST
  if (deliver_fn_override) {
    deliver_fn_override(message);
    return;
  }
#endif

  // TODO: Optionally handle escape sequences, validation, etc. For now we assume it's valid.

  ChatBufferState* state = get_chat_buffer_state();
  add_message_to_chat_history(state, message, RECIPIENT_VOID, RECIPIENT_UNKEY);
  display_chat_history(state);
}

/**
 * Appends 0 or 1 to bitstream based on which frequency has higher magnitude.
 */
void get_bit_from_top_frequency() {
  // Calculates the magnitude of the complex output for each frequency bin:
  float mag0 = sqrtf(powf(gs[0].y_re, 2) + powf(gs[0].y_im, 2));
  float mag1 = sqrtf(powf(gs[1].y_re, 2) + powf(gs[1].y_im, 2));
  float total_mag = mag0 + mag1;

  // Ignores noisy or weak signals:
  if (total_mag < MAGNITUDE_THRESHOLD) {
    return;
  }

  uint8_t bit = (mag1 > mag0) ? 1 : 0;

  if (bit_index < MAX_BITS) {
    bitstream[bit_index++] = bit;
  }

}

/**
 * Reconstructs bytes from bitstream, extracts a valid message
 * between header/footer, and delivers it to chat history.
 */
void parse_message() {
  // Buffer to hold reconstructed bytes from the bitstream:
  static char decoded_bytes[MAX_PACKET_SIZE];

  // Tracks how many full bytes have been reconstructed from the incoming bitstream[]:
  int byte_count = 0;

  // Converts bitstream[] into bytes and stores in decoded_bytes:
  for (int i = 0; i + 7 < bit_index; i += 8) {
    uint8_t byte = 0;
    for (int b = 0; b < 8; b++) {
      byte = (byte << 1) | bitstream[i + b];
    }
    // Stores the byte if there's space:
    if (byte_count < MAX_PACKET_SIZE) {
      decoded_bytes[byte_count++] = byte;
    }
  }

  // Scans for packet start (header):
  int start = -1;
  for (int i = 0; i < byte_count - 3; i++) {
    if ((uint8_t)decoded_bytes[i] == PACKET_START1 &&
        (uint8_t)decoded_bytes[i + 1] == PACKET_START2) {
      start = i + 2;
      break;
    }
  }
  // No valid header found, so return early:
  if (start == -1) return;

  // Scans for packet end (footer):
  int end = -1;
  for (int i = start; i < byte_count - 1; i++) {
    if ((uint8_t)decoded_bytes[i] == PACKET_END1 &&
        (uint8_t)decoded_bytes[i + 1] == PACKET_END2) {
      end = i;
      break;
    }
  }
  // No valid footer found, so return early:
  if (end == -1) return;

  // Copies message content into null-terminated string:
  char message[MAX_TEXT_LENGTH];
  int msg_len = end - start;
  if (msg_len >= MAX_TEXT_LENGTH) msg_len = MAX_TEXT_LENGTH - 1;
  strncpy(message, &decoded_bytes[start], msg_len);
  message[msg_len] = '\0';

  // Adds message to chat history and displays it:
  deliver_message(message);

  // Resets bit buffer:
  bit_index = 0;
}

/**
 * Applies a function to each goertzel_state in gs[].
 * Pass either one_param_fn (if sample == -1) or two_param_fn with a sample.
 * Only one function pointer should be non-NULL.
*/
void for_each_goertzel_state(void (*one_param_fn)(goertzel_state*), void (*two_param_fn)(goertzel_state*, int), int sample) {
  for (int i = 0; i < gs_len; i++) {
    // Gets a pointer to the j-th element of the gs array, which holds Goertzel filter state:
    goertzel_state* g = &gs[i];
    // Passes that pointer into whatever function so it can update the Goertzel state in-place:
    if (sample == -1 && one_param_fn != NULL) {
      one_param_fn(g);
    } else if (two_param_fn != NULL) {
      two_param_fn(g, sample);
    }
  }
}

/**
 * Processes one full ADC window to decode a bit, updates the bitstream, and attempts message parsing.
 */
void decode_single_bit_from_adc_window() {
  // Skips processing unless a full bit period's worth of data is ready:
  if (adc_window_counter++ % SCAN_CHAIN_LENGTH != 0) return;

  // For each ADC sample in the buffer, update each Goertzel filter state with this sample:
  for (size_t i = 0; i < buffer_size; i++) {
    for_each_goertzel_state(NULL, update_goertzel, adc_buffer_full_bit[i]);
  }

  for_each_goertzel_state(finalize_goertzel, NULL, -1);

  get_bit_from_top_frequency();
  parse_message();

  // Resets internal Goertzel state (not y_re/y_im):
  for_each_goertzel_state(reset_goertzel, NULL, -1);
}

/**
 * Called when DMA fills the ADC buffer. It clears the DMA interrupt, copies the buffer, re-enables DMA, and
 * triggers bit extraction from the data.
 - A bit is encoded over 10 ms of audio.
 - Each ADC buffer = 5 ms of samples (410 samples at 81.92 kHz).
 - Therefore, two buffers = 1 bit period.
 */
void adc_buffer_full_interrupt() {
  // Clears the DMA interrupt flag so it's ready for the next transfer:
  dma_ch1.clearInterrupt();

  if (buffer_half_1_is_valid) {
    // Combines prev + current into adc_buffer_full_bit:
    memcpy(adc_buffer_full_bit, adc_buffer_half_1, sizeof(adc_buffer_half_1));
    memcpy(adc_buffer_full_bit + buffer_size, (const void*)adc_buffer_half_2, sizeof(adc_buffer_half_2));
  } else {
    // Not ready to decode yet, just store current into prev and return:
    memcpy(adc_buffer_half_1, (const void*)adc_buffer_half_2, sizeof(adc_buffer_half_2));
    buffer_half_1_is_valid = true;
    dma_ch1.enable();
    return;
  }

  // Invalidates CPU cache for adc_buffer_half_2 to ensure CPU sees the latest data written by DMA (RAM2 is cacheable):
  if ((uint32_t)adc_buffer_half_2 >= 0x20200000u) {
    arm_dcache_delete((void *)adc_buffer_half_2, sizeof(adc_buffer_half_2));
  }

  // Re-enables the DMA channel for next read:
  dma_ch1.enable();

  // Uses Goertzel algorithm to analyze the frequency content of a series of ADC samples:
  decode_single_bit_from_adc_window();
  buffer_half_1_is_valid = false;
}

/**
 * Configures the system to receive data: initializes the ADC, configures Goertzel filters for frequency analysis,
 * sets the gain on the charge amplifier, sets up DMA channel for ADC to send data to buffer.
 */
void setup_receiver() {
  // Sets readPin_adc_0_pin as the input pin for ADC sampling:
  pinMode(readPin_adc_0_pin, INPUT);

  // Initializes Goertzel filters for binary 0 and 1 detection at 2.0 kHz and 2.2 kHz:
  initialize_goertzel(&gs[0], 2000, adc_sampling_rate);  // for binary 0
  initialize_goertzel(&gs[1], 2200, adc_sampling_rate);  // for binary 1

  // Sets gain on charge amplifier:
  set_charge_amplifier_gain(6);

  // Sets up ADC (for received audio signal):
  adc->adc0->setAveraging(1); // no averaging
  adc->adc0->setResolution(12); // bits
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  //adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

  // Configures DMA to transfer ADC samples into adc_buffer_half_2:
  // Note: The following line may raise a compiler warning because type-punning ADC1_R0 here violates strict aliasing rules, but can be safely ignored
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
  dma_ch1.source((volatile uint16_t &)(ADC1_R0));
  #pragma GCC diagnostic pop

  // Each time you sample from ADC you get 2 bytes, so that's why we're using buffer_size * 2:
  dma_ch1.destinationBuffer((uint16_t *)adc_buffer_half_2, buffer_size);
  dma_ch1.interruptAtCompletion();
  dma_ch1.disableOnCompletion();

  /*
    Note that given adc_sampling_rate = 81.92 kHz and that a full buffer contains 410 samples:
    Time per buffer = buffer_size / sampling_rate = 410 / 81920 = 5 ms
    -> The DMA transfer completes every (buffer_size X samples) i.e. 5 ms, triggering adc_buffer_full_interrupt().
  */
  // Triggers adc_buffer_full_interrupt every time DMA transfer completes:
  dma_ch1.attachInterrupt(&adc_buffer_full_interrupt);
  dma_ch1.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);

  // Enables DMA and start ADC with timer-based sampling at 81.92 kHz:
  dma_ch1.enable();
  adc->adc0->enableDMA();
  adc->adc0->startSingleRead(readPin_adc_0_pin);
  adc->adc0->startTimer(adc_sampling_rate);
}

// ------------------------------------------------------------------
// More Testing Accessors
// ------------------------------------------------------------------
#ifdef UNIT_TEST

uint8_t* _test_get_bitstream() {
  return bitstream;
}

int* _test_get_bit_index() {
  return &bit_index;
}

goertzel_state* _test_get_goertzel_state() {
  return gs;
}

// Buffer that stores the last delivered message during a test:
char delivered[MAX_TEXT_LENGTH] = {0};

// Allows tests to inspect what message was captured:
char* _test_get_delivered_message() {
  return delivered;
}

// When called, allows unit tests to capture or mock delivery without triggering hardware-dependent code:
void _test_set_deliver_fn(void (*fn)(const char*)) {
  deliver_fn_override = fn;
}

// Returns a pointer to adc_window_counter so tests can modify or check its value:
uint8_t* _test_get_adc_window_counter() {
  return &adc_window_counter;
}

uint16_t* _test_get_adc_buffer_half_1() {
  return adc_buffer_half_1;
}

// Returns a pointer to adc_buffer_half_2 so tests can fill it with mock ADC data - volatile because it's the DMA destination:
volatile uint16_t* _test_get_adc_buffer_half_2() {
  return adc_buffer_half_2;
}

// Returns a pointer to adc_buffer_full_bit	so tests can inspect or clear copied ADC data:
uint16_t* _test_get_adc_buffer_full_bit() {
  return adc_buffer_full_bit;
}

#endif
