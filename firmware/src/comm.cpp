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
  adc_buffer_curr_half is filled by the DMA and declared with DMAMEM and __attribute__((aligned(32)))
  to place it in RAM2 (OCRAM), which is cacheable and requires manual cache management.

  RAM1 = DTCM (Tightly Coupled Memory) --> Not cacheable. Always in sync.

  RAM2 = OCRAM (what DMAMEM uses) --> Cacheable. Can get out of sync with the Teensy CPU’s data cache,
  which requires manual cache management: arm_dcache_flush() pushes CPU cache changes to RAM2;
  arm_dcache_delete() discards cache so the CPU reads fresh data from RAM2.
*/

// Number of ADC samples per DMA transfer into adc_buffer_curr_half.
// Samples per window (buffer_size) = adc_sampling_rate * window_duration = 81,920 * 5 ms = 410 samples.
#ifdef UNIT_TEST
const uint32_t buffer_size = 410;
#else
static const uint32_t buffer_size = 410;
#endif

// Holds one pair of Goertzel magnitudes:
// Every time G runs, it gives a magnitude for each of the 2 frequencies we're checking
struct goertzel_output_t {
  float mag_2kHz;
  float mag_2_2kHz;
};

// Stores recent Goertzel magnitudes for each bit window in a circular buffer:
static const size_t G_HISTORY_LEN = 16;
static goertzel_output_t goertzel_history_circ_buffer[G_HISTORY_LEN];
static size_t goertzel_history_circ_buffer_index = 0;

// If k = the min # of bits that need to be seen in order to identify the bit boundaries in relation to the sampling window,
// then 2k + = the min # windows (Goertzel outputs) that need to be seen
// Since k = 3, 2k + 1 = 7
static const size_t ALIGNMENT_IDENTIFYING_G_OUTPUTS = 7;
static goertzel_output_t window_history[ALIGNMENT_IDENTIFYING_G_OUTPUTS];
static size_t window_history_index = 0;
static bool bit_boundaries_are_known = false;
static uint8_t bit_alignment_phase = 0; // 0 = even windows, 1 = odd windows
static bool have_enough_windows = false;
static size_t windows_collected = 0;
// TODO: Currently an arbitrary number, might want to review/discuss these tradeoffs:
// MAX_WEAK_WINDOWS too low: Could drop a signal too early that might otherwise be fine
// MAX_WEAK_WINDOWS too high: Trying to detect a signal long after it's dropped out, or was never valid to begin with
static const uint8_t MAX_WEAK_WINDOWS = 3;
static uint8_t consecutive_weak_windows = 0;

// ADC will sample at freq of 81.92 kHz:
static const uint32_t adc_sampling_rate = 81920;

char tx_display_buffer[MAX_TEXT_LENGTH];
uint16_t tx_display_buffer_length = 0;

ADC *adc = new ADC();
DMAChannel dma_ch1;

// DMAMEM places adc_buffer_curr_half in RAM2 (OCRAM):
DMAMEM static volatile uint16_t __attribute__((aligned(32))) adc_buffer_curr_half[buffer_size];

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

// For preamble:
static const uint16_t PREAMBLE_BITS = 35; // unit is bits. 35 bits * 10 ms = 350 ms total preamble length

// Plotting amplitude values:
// Since these are updated within an interrupt, made these volatile-qualified to ensure the compiler is reading/writing these directly to/from memory instead of to/from a CPU register with potentially outdated data:
volatile float curr_mag_2kHz = 0.0;
volatile float curr_mag_2_2kHz = 0.0;
volatile bool mag_ready = false;

// ------------------------------------------------------------------
// Functions
// ------------------------------------------------------------------

static inline void update_magnitudes_to_plot(float mag_2_kHz, float mag_2_2_kHz) {
  curr_mag_2kHz = mag_2_kHz;
  curr_mag_2_2kHz = mag_2_2_kHz;
  mag_ready = true;
}

/**
 * Sends a 24‑bit SPI frame to the DAC: [addr/ctrl (8)] + [data (16)].
 * For MCP48CxDx1: top byte = 5‑bit register address + 2 command bits (must be 0 to write) + 1 don't‑care.
 * Lower 16 bits carry the 12‑bit value (left‑aligned as per device spec; unused bits are don't‑care).
 * Expects value in the 0..4095 range.
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
 * Outputs one bit as a sine wave: picks high/low frequency based on bit value,
 * then drives the DAC sample-by-sample for the full bit duration.
 */
static inline void transmit_bit(uint8_t bit, const tx_parameters_t* tx_parameters) {
  // w is the angular frequency, wherein w = 2 * pi * freq
  const float w = (bit ? (2 * PI * tx_parameters->freq_high / 1e6)
                       : (2 * PI * tx_parameters->freq_low /  1e6));
  const unsigned long bit_start_time = micros();
  unsigned long curr_bit_elapsed_useconds;

  // Generates a sine wave for the current bit:
  while ((curr_bit_elapsed_useconds = micros() - bit_start_time) < tx_parameters->usec_per_bit) {
    // The DAC sample value (integer 0–409) computed from the sine at that instant - what we actually write to the DAC:
    const uint16_t dac_sample_value = (uint16_t)(((sinf(w * curr_bit_elapsed_useconds) + 1.0f) * 0.5f) * 409);

    // Do not want an interrupt to run mid-sample, so:
    noInterrupts();
    write_to_dac(0, dac_sample_value);
    interrupts();
  }
}

/**
 * Sends the preamble sequence (alternating 1/0 bits for PREAMBLE_BITS length)
 * to help the receiver establish timing and alignment.
 */
static inline void transmit_preamble(const tx_parameters_t* tx_parameters) {
  // Arbitrarily choosing to start the preamble with a 1 instead of a 0:
  uint8_t curr_bit = 1;

  for (uint16_t i = 0; i < PREAMBLE_BITS; i++) {
    transmit_bit(curr_bit, tx_parameters);
    // Bitwise XOR assignment operator is ^=: if b is 1, 1 ^ 1 = 0 and if b is 0, 0 ^ 1 = 1
    // Alternates 1,0,1,0,...
    curr_bit ^= 1;
  }
}

/**
 * FSK transmitter: for each char, emit its 8 bits MSB‑first as tones.
 * Uses freq_low for 0 and freq_high for 1; each bit lasts usec_per_bit microseconds.
 * The sine is generated with w = 2πf and time in microseconds, so f is in Hz and time is µs.
 * Note: the current scaling multiplies by 409 (≈10‑bit). For a 12‑bit DAC, use 4095.
 */
void transmit_message(const char* message_to_transmit, const tx_parameters_t* tx_parameters) {
  transmit_preamble(tx_parameters);
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
      // Generates a sine wave for the current bit for usec_per_bit microseconds:
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
 * Enables or disables the TX power rail via tx_power_en_pin.
 */
inline void set_tx_power_enable(bool enable) {
  digitalWriteFast(tx_power_en_pin, (enable) ? HIGH : LOW);
}

/**
 * Initializes TX path: configures control pins, enables TX power, and programs DAC config/VREF.
 * Note: leaves TX power ON (set_tx_power_enable(true)); caller can disable after transmit.
 */
void setup_transmitter() {
  pinMode(tx_power_en_pin, OUTPUT);
  pinMode(xdcr_sw_pin, OUTPUT);
  pinMode(dac_cs_pin, OUTPUT);
  digitalWrite(dac_cs_pin, HIGH);

  // TODO: Conditionally enable tx power to send mesage, then disable after
  set_tx_power_enable(true);  // tested: works
  delay(100);                 // wait for power to boot
  write_to_dac(0xA, 1U << 8);  // A is address for config, 8th bit is gain. set to gain=2
  write_to_dac(8, 1);          // 8 is address for VREF, 1 means use internal ref
}

/**
 * Delivers a decoded message to the UI:
 * - In UNIT_TEST, calls an optional test override if set.
 * - Validates length and rejects protocol framing bytes.
 * - Strips simple escape chars, appends to chat history, refreshes display.
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

  size_t len = strlen(message);

  // Reject empty or overly long messages:
  if (len == 0 || len >= MAX_TEXT_LENGTH) return;
  // Rejects framing characters used by protocol:
  for (size_t i = 0; i < len; i++) {
    uint8_t c = (uint8_t)message[i];
    if (c == 0x01 || c == 0x02 || c == 0x03 || c == 0x04) return;
  }

  // Removes escape chars:
  char cleaned[MAX_TEXT_LENGTH];
  size_t j = 0;
  for (size_t i = 0; i < len && j < MAX_TEXT_LENGTH - 1; i++) {
    char c = message[i];
    if (c != '\r' && c != '\t') cleaned[j++] = c;
  }
  cleaned[j] = '\0';

  ChatBufferState* state = get_chat_buffer_state();
  add_message_to_chat_history(state, cleaned, RECIPIENT_VOID, RECIPIENT_UNKEY);
  display_chat_history(state);
}

/**
 * Computes the average combined magnitude (2.0 kHz + 2.2 kHz)
 * over the most recent `window_count` entries in window_history.
 */
static float average_window_magnitude(size_t window_count) {
  float sum = 0.0f;
  for (size_t i = 0; i < window_count; i++) {
    size_t index = (window_history_index + i) % ALIGNMENT_IDENTIFYING_G_OUTPUTS;
    sum += window_history[index].mag_2kHz + window_history[index].mag_2_2kHz;
  }
  return sum / window_count;
}

/**
 * Returns 0 if 2.0 kHz is stronger, 1 if 2.2 kHz is stronger,
 * or 255 if magnitudes are equal (ambiguous).
 */
static inline uint8_t determine_bit(const goertzel_output_t* g_ouput) {
  if (g_ouput->mag_2_2kHz > g_ouput->mag_2kHz) return 1;
  if (g_ouput->mag_2_2kHz < g_ouput->mag_2kHz) return 0;
  return 255; // out of range value (uint8_t ~ 0-255) that can be used to check for error case
}

/**
 * Uses the rolling window_history to decide bit boundaries.
 * Checks both possible phases; if at least 3 alternating 0/1 pairs
 * are found and average magnitude is above threshold,
 * sets bit_alignment_phase and marks boundaries as known.
 */
static void find_bit_boundaries(void) {
  if (!have_enough_windows || bit_boundaries_are_known) return;

  // Basically just deciding here the signal is viable if the average magnitude is above some predetermined threshold
  float avg_mag = average_window_magnitude(ALIGNMENT_IDENTIFYING_G_OUTPUTS);
  if (avg_mag < MAGNITUDE_THRESHOLD) return;

  // Outer loop tries both phases
  for (uint8_t phase = 0; phase < 2; phase++) {
    int alternating_pairs = 0;

    // Inner loop tries pairs of windows for each phase
    for (size_t i = 0; i + 1 < ALIGNMENT_IDENTIFYING_G_OUTPUTS; i += 2) {
      size_t idx0 = (window_history_index + i + phase) % ALIGNMENT_IDENTIFYING_G_OUTPUTS;
      size_t idx1 = (window_history_index + i + 1 + phase) % ALIGNMENT_IDENTIFYING_G_OUTPUTS;

      uint8_t b0 = determine_bit(&window_history[idx0]);
      uint8_t b1 = determine_bit(&window_history[idx1]);

      // Skip if either window was ambiguous (tie → 255)
      if (b0 == 255 || b1 == 255) continue;
      if (b0 != b1) {
        alternating_pairs++;
      }
    }

    // If we saw 3 alternating pairs, assume we found the correct bit boundaries
    if (alternating_pairs >= 3) {
      bit_alignment_phase = phase;
      bit_boundaries_are_known = true;
      return; // stop searching once locked
    }
  }
}

/**
 * Computes Goertzel magnitudes for 2.0 kHz and 2.2 kHz, logs them, and (if signal is strong enough)
 * appends 0/1 to bitstream based on which bin is larger.
 * Uses MAGNITUDE_THRESHOLD on (mag_2kHz + mag_2_2kHz) to suppress noise.
 */
void get_bit_from_top_frequency() {
  // Calculates the magnitude of the complex output for each frequency bin:
  float mag_2kHz = sqrtf(powf(gs[0].y_re, 2) + powf(gs[0].y_im, 2));
  float mag_2_2kHz = sqrtf(powf(gs[1].y_re, 2) + powf(gs[1].y_im, 2));

  update_magnitudes_to_plot(mag_2kHz, mag_2_2kHz);

  // Stores magnitudes in circ buffer for later analysis:
  goertzel_history_circ_buffer[goertzel_history_circ_buffer_index] = { mag_2kHz, mag_2_2kHz };
  goertzel_history_circ_buffer_index = (goertzel_history_circ_buffer_index + 1) % G_HISTORY_LEN;

  // Keeps a short rolling history used for boundary detection:
  window_history[window_history_index] = { mag_2kHz, mag_2_2kHz };
  window_history_index = (window_history_index + 1) % ALIGNMENT_IDENTIFYING_G_OUTPUTS;

  // Keep counting windows until we’ve seen enough to attempt boundary detection:
  if (!have_enough_windows) {
    windows_collected++;
    if (windows_collected >= ALIGNMENT_IDENTIFYING_G_OUTPUTS) {
      have_enough_windows = true;
    }
  }

  // Use the rolling window_history to determine where bit edges fall (sets bit_boundaries_are_known/bit_alignment_phase):
  find_bit_boundaries();

  // If boundaries aren’t established yet, stop here (only magnitudes logged this window):
  if (!bit_boundaries_are_known) return;
  else {
    // Boundaries are known → keep monitoring average strength:
    float avg_mag_lock = average_window_magnitude(ALIGNMENT_IDENTIFYING_G_OUTPUTS);
    if (avg_mag_lock < MAGNITUDE_THRESHOLD) {
      if (++consecutive_weak_windows >= MAX_WEAK_WINDOWS) {
        // Signal stayed weak too long → reset state:
        bit_boundaries_are_known = false;
        consecutive_weak_windows = 0;
        have_enough_windows = false;
        windows_collected = 0;
        return; // don't need to proceed with trying to append a bit at this point
      }
    } else {
      consecutive_weak_windows = 0;
    }
  }

  // Decide this window’s bit (1 if 2.2 kHz stronger, else 0) and append if there’s space:
  uint8_t bit = (mag_2_2kHz > mag_2kHz) ? 1 : 0;
  if (bit_index < MAX_BITS) {
    bitstream[bit_index++] = bit;
  }
}

/**
 * Reassembles bytes MSB‑first from bitstream[], searches for 2‑byte header/footer,
 * copies the payload to a C‑string, delivers it, then resets the bit buffer.
 * Header: PACKET_START1, PACKET_START2. Footer: PACKET_END1, PACKET_END2.
 */
void check_for_complete_packet() {
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
 * Applies a callback to each goertzel_state in gs[].
 * Pass exactly one of:
 *  - two_param_fn(g, sample) to process a sample, or
 *  - one_param_fn(g) for no‑sample ops (finalize/reset). In this case pass sample = -1.
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
 * Processes one ADC window at bit-aligned intervals:
 * runs Goertzel, extracts a bit, appends it to the bitstream, checks for a complete packet, then resets Goertzel state.
 */
void decode_single_bit_from_adc_window(const uint16_t* samples, size_t size) {
  uint8_t curr_phase = adc_window_counter++ % SCAN_CHAIN_LENGTH;

  // If we know for sure the current window is out of phase, don't bother
  // with processing it:
  if (bit_boundaries_are_known && curr_phase != bit_alignment_phase) return;

  // Feeds each ADC sample into the Goertzel filters:
  for (size_t i = 0; i < size; i++) {
    for_each_goertzel_state(NULL, update_goertzel, samples[i]);
  }

  // Computes this window’s frequency results:
  for_each_goertzel_state(finalize_goertzel, NULL, -1);

  get_bit_from_top_frequency();
  check_for_complete_packet();

  // Resets internal Goertzel state (not y_re/y_im):
  for_each_goertzel_state(reset_goertzel, NULL, -1);
}

/**
 * Called when DMA fills the ADC buffer. Clears the DMA interrupt, invalidates CPU cache for the DMA buffer,
 * passes the fresh samples to the decoder, then re-enables DMA.
 * DMA writes to RAM2 -> decode_single_bit_from_adc_window reads from RAM2.
 * Each ADC buffer ≈ 5 ms (410 samples @ 81.92 kHz). If SCAN_CHAIN_LENGTH == 2 and TX bit period is ~10 ms,
 * two buffers correspond to one bit period.
 */
void adc_buffer_full_interrupt() {
  // Clears the DMA interrupt flag so it's ready for the next transfer:
  dma_ch1.clearInterrupt();

  // Invalidates CPU cache for adc_buffer_curr_half to ensure CPU sees the latest data written by DMA (RAM2 is cacheable):
  arm_dcache_delete((void *)adc_buffer_curr_half, sizeof(adc_buffer_curr_half));

  // Uses Goertzel algorithm to analyze the frequency content of a full buffer of ADC samples:
  decode_single_bit_from_adc_window((const uint16_t*)adc_buffer_curr_half, buffer_size);

  // Re-enables the DMA channel for next read:
  dma_ch1.enable();
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

  // Configures DMA to transfer ADC samples into adc_buffer_curr_half:
  // Note: The following line may raise a compiler warning because type-punning ADC1_R0 here violates strict aliasing rules, but can be safely ignored
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
  dma_ch1.source((volatile uint16_t &)(ADC1_R0));
  #pragma GCC diagnostic pop

  // destinationBuffer takes a sample count here (not bytes). The library handles element width internally:
  dma_ch1.destinationBuffer((uint16_t *)adc_buffer_curr_half, buffer_size);
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

// Returns a pointer to adc_buffer_curr_half so tests can fill it with mock ADC data - volatile because it's the DMA destination:
volatile uint16_t* _test_get_adc_buffer_curr_half() {
  return adc_buffer_curr_half;
}

#endif
