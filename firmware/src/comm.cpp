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
// State
// ------------------------------------------------------------------

// ADC will sample at freq of 81.92 kHz:
static const uint32_t adc_frequency = 81920;

// Size of buffer where ADC data will be stored:
static const uint32_t buffer_size = 10240;

char tx_display_buffer[MAX_TEXT_LENGTH];
uint16_t tx_display_buffer_length = 0;

ADC *adc = new ADC();
DMAChannel dma_ch1;

// Creates dma_adc_buff1 buffer:
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff1[buffer_size];
uint16_t adc_buffer_copy[buffer_size];

static uint8_t print_ctr = 0;
static const uint8_t gs_len = 10;

// An array that will store state for the Goertzel algo - each goertzel_state obj
// holds data to compute G algo for that frequency:
goertzel_state gs[gs_len];

// Charge amplifier gain:
static const int adg728_i2c_address = 76;

// For process_bit
static const int MAX_BITS = 256;
static uint8_t bitstream[MAX_BITS];
static int bit_index = 0;


// ------------------------------------------------------------------
// Functions
// ------------------------------------------------------------------

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
 * 5. parse_message()
 *    - Converts valid bitstream to ASCII text.
 *    - Handles escape sequences, if needed.
 * - Updates chat history or state with decoded message string. (?)
 *    - May call add_message_to_chat_history(), etc. (?)
 */
void parse_message(const char* message) {

  // Optionally handle escape sequences, validation, etc. For now we assume it's valid.
  // TODO
  ChatBufferState* state = get_chat_buffer_state();
  add_message_to_chat_history(state, message, RECIPIENT_VOID, RECIPIENT_UNKEY);
  display_chat_history(state);
}

/**
 * 3. process_bit()
 *    - Converts top frequency from each interval into a bit (0 or 1).
 *    - Accumulates bits into a buffer.
 */
void process_bit() {
  float mag0 = sqrtf(powf(gs[0].y_re, 2) + powf(gs[0].y_im, 2));
  float mag1 = sqrtf(powf(gs[1].y_re, 2) + powf(gs[1].y_im, 2));

  uint8_t bit = (mag1 > mag0) ? 1 : 0;

  if (bit_index < MAX_BITS) {
    bitstream[bit_index++] = bit;
  }

  // For debug:
  Serial.printf("Bit %d: %d (mag0=%.2f, mag1=%.2f)\n", bit_index, bit, mag0, mag1);
}

/**
 * 4. attempt_message_extract()
 *    - Detects packet boundaries (e.g., SOH/STX and ETX/EOT).
 *    - Validates and extracts the message portion.
 */
void attempt_message_extract() {
  const uint8_t START1 = 0x01;
  const uint8_t START2 = 0x02;
  const uint8_t END1   = 0x03;
  const uint8_t END2   = 0x04;

  static char decoded_bytes[MAX_PACKET_SIZE];
  int byte_count = 0;

  // Convert bitstream[] into bytes
  for (int i = 0; i + 7 < bit_index; i += 8) {
    uint8_t byte = 0;
    for (int b = 0; b < 8; b++) {
      byte = (byte << 1) | bitstream[i + b];
    }

    // Store the byte
    if (byte_count < MAX_PACKET_SIZE) {
      decoded_bytes[byte_count++] = byte;
    }
  }

  // Scan for packet delimiters
  int start = -1;
  for (int i = 0; i < byte_count - 3; i++) {
    if ((uint8_t)decoded_bytes[i] == START1 &&
        (uint8_t)decoded_bytes[i + 1] == START2) {
      start = i + 2;
      break;
    }
  }

  if (start == -1) return;  // No valid header found

  int end = -1;
  for (int i = start; i < byte_count - 1; i++) {
    if ((uint8_t)decoded_bytes[i] == END1 &&
        (uint8_t)decoded_bytes[i + 1] == END2) {
      end = i;
      break;
    }
  }

  if (end == -1) return;  // No valid footer found

  // Copy message content into null-terminated string:
  char message[MAX_TEXT_LENGTH];
  int msg_len = end - start;
  if (msg_len >= MAX_TEXT_LENGTH) msg_len = MAX_TEXT_LENGTH - 1;

  strncpy(message, &decoded_bytes[start], msg_len);
  message[msg_len] = '\0';

  parse_message(message);

  // Reset bit buffer
  bit_index = 0;
}

void decode_single_bit_from_adc_window() {
  /**
   * 2. - Analyzes adc_buffer_copy[] using Goertzel for all 10 bins.
   *    - Selects the most prominent frequency at each bit interval.
   */

  // "Only run the code inside this block every Nth interrupt" (where N = SCAN_CHAIN_LENGTH) i.e. looking at one bit at a time:
  if (print_ctr++ % SCAN_CHAIN_LENGTH != 0) return;

  for (size_t i = 0; i < buffer_size; i++) {
    for (int j = 0; j < gs_len; j++) {
      goertzel_state *g = &gs[j];
      update_goertzel(g, adc_buffer_copy[i]);
    }
  }
  for (int j = 0; j < gs_len; j++) {
    goertzel_state *g = &gs[j];
    finalize_goertzel(g);
    reset_goertzel(g);
  }

  process_bit();

  attempt_message_extract();
}

/**
 * Deals with ADC data when DMA buffer is full, and processes the data with Goertzel filters and prints frequency domain data.
 * Analog signals (voltages) --> digital values that can be processed by Teensy
 *
 * 1. - adc_buffer_full_interrupt called when DMA fills the ADC buffer.
 *    - Copies buffer, clears DMA interrupt, re-enables DMA.
 */
void adc_buffer_full_interrupt() {
  // Clears the DMA interrupt flag so it's ready for the next transfer:
  dma_ch1.clearInterrupt();

  // mempcy copies a block of memory from one location to another:
  memcpy((void *)adc_buffer_copy, (void *)dma_adc_buff1, sizeof(dma_adc_buff1));

  // ? idk really
  if ((uint32_t)dma_adc_buff1 >= 0x20200000u)
  arm_dcache_delete((void *)dma_adc_buff1, sizeof(dma_adc_buff1));

  // Re-enables the DMA channel for next read:
  dma_ch1.enable();

  // Uses Goertzel algorithm to analyze the frequency content of a series of ADC samples:
  // Note: This assumes that one full ADC buffer = one bit period. Not actually sure that is what is happening
  decode_single_bit_from_adc_window();
}

/**
 * Configures the system to receive data: initializes the ADC, configures Goertzel filters for frequency analysis,
 * sets the gain on the charge amplifier, sets up DMA channel for ADC to send data to a buffer super duper efficiently.
 */
void setup_receiver() {
  Serial.println("✨ setup_receiver called");

  // Sets readPin_adc_0_pin as the input pin:
  pinMode(readPin_adc_0_pin, INPUT);

  // Initializes Goertzel filters (TODO: Hardcoded frequencies here):
  // for (int j = 0; j < gs_len; j++) {
  //   // The 2nd param sets the initial frequency for that filter: so 14000, 14200, 14400 etc
  //   initialize_goertzel(&gs[j], 15000 + (j - 5) * 200, adc_frequency);
  // }

  initialize_goertzel(&gs[0], 2000, adc_frequency);  // for binary 0
  initialize_goertzel(&gs[1], 2200, adc_frequency);  // for binary 1

  // Sets gain on charge amplifier:
  set_charge_amplifier_gain(6);

  // Sets up ADC (for received audio signal):
  adc->adc0->setAveraging(1); // no averaging
  adc->adc0->setResolution(12); // bits
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  //adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

  // Sets up DMA:
  // Note: The following line may raise a compiler warning because type-punning ADC1_R0 here violates strict aliasing rules, but can be safely ignored
  // dma_ch1.source((volatile uint16_t &)(ADC1_R0));

  // TODO: Remove these pragmas eventually. Keeping for now because the warnings clutter the compiler output
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
  dma_ch1.source((volatile uint16_t &)(ADC1_R0));
  #pragma GCC diagnostic pop

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
  adc->adc0->startSingleRead(readPin_adc_0_pin);
  // This actually determines how fast to sample the signal, and starts timer to initiate dma transfer from adc to memory once 2x buffer size bytes reached
  adc->adc0->startTimer(adc_frequency);
}

// ------------------------------------------------------------------
// Testing Accessors
// Note: Corresponding declarations are in comm_internal.h
// ------------------------------------------------------------------

// This wrapper ensures the code inside is only compiled when PlatformIO runs pio test:
#ifdef TESTING

uint8_t* _test_get_bitstream() {
  return bitstream;
}

int* _test_get_bit_index() {
  return &bit_index;
}

goertzel_state* _test_get_goertzel_state() {
  return gs;
}

#endif
