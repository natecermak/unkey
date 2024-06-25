#include <ADC.h>
#include <DMAChannel.h>
#include <ILI9341_t3n.h>
#include <SPI.h>
#include <Wire.h>
#include "goertzel.h"

// ------------------ PINS ----------------------------------------------- //
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

// ------------------ ADC, DMA, and Goertzel filters -------------------- //
ADC *adc = new ADC();
DMAChannel dma_ch1;

const uint32_t adc_frequency = 81920;
const uint32_t buffer_size = 10240;
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff1[buffer_size];
uint16_t adc_buffer_copy[buffer_size];

uint8_t print_ctr = 0;
const uint8_t gs_len = 10;
goertzel_state gs[gs_len];

// ------------------ Charge amplifier gain ------------------------------ //
const int adg728_i2c_address = 76;

// ------------------  Keyboard poller timer and state-------------------- //
IntervalTimer keyboard_poller_timer;
const int keyboard_poller_period_usec = 10000; // run at 100 Hz
volatile uint64_t switch_state; 
uint32_t time_of_last_press_ms;

const uint8_t SCAN_CHAIN_BIT_INDEX_TO_KB_INDEX[] = {
  63,62,61,60,59,58,57,56,
  47,46,45,44,43,42,41,40,
  31,30,29,28,27,26,25,24,
  15,14,13,12,11,10, 9, 8,
  55,54,53,52,51,50,49,48,
  39,38,37,36,35,34,33,32,
  23,22,21,20,19,18,17,16,
   7, 6, 5, 4, 3, 2, 1, 0
};

const uint8_t CAP_KEY_INDEX = 47;
const uint8_t SYM_KEY_INDEX = 63;
const uint8_t BACK_KEY_INDEX = 15;
const char KEYBOARD_LAYOUT[3][64] = {
  "1234567890-=_+\n\nqwertyuiop(){}\n\nasdfghjkl;:\'\"   zxcvbnm,.?     ",
  "1234567890-=_+\n\nQWERTYUIOP(){}\n\nASDFGHJKL;:\'\"   ZXCVBNM,.?     ",
  "!@#$%^&*()-=_+\n\nqwertyuiop(){}\n\nasdfghjkl;:\'\"   zxcvbnm,.?     "
};

// ------------------ TFT LCD -------------------- //
bool screen_on;
const int screen_timeout_ms = 10000; // todo: this is too low, for testing only
ILI9341_t3n tft = ILI9341_t3n(tft_cs, tft_dc, tft_reset, tft_mosi, tft_sck, tft_miso);

// ------------------ Battery monitoring -------------- //
long time_of_last_battery_read_ms;
const long BATTERY_READ_PERIOD_MS = 1000;

// ------------------ Utility functions ---------------------------------- //
int ilog2(uint64_t x){
  int i = 0;
  while (x) {
    x = x >> 1;
    i++;  
  }
  return i;
}


void set_charge_amplifier_gain(uint8_t gain_index) {
  Wire.beginTransmission(adg728_i2c_address);   // Slave address
  Wire.write(1U << gain_index); // Write string to I2C Tx buffer (incl. string null at end)
  Wire.endTransmission();           // Transmit to Slave
}

inline void set_tx_power_enable(bool enable) {
  digitalWriteFast(tx_power_en, (enable) ? HIGH : LOW);
}


void adc_buffer_full_interrupt() {
  dma_ch1.clearInterrupt();
  memcpy((void *) adc_buffer_copy, (void *) dma_adc_buff1, sizeof(dma_adc_buff1));
  if ((uint32_t)dma_adc_buff1 >= 0x20200000u)
    arm_dcache_delete((void *)dma_adc_buff1, sizeof(dma_adc_buff1)); 
  dma_ch1.enable(); // Re-enable the DMA channel

  // process data
  if (print_ctr++ % 64 == 0) {
    for (size_t i = 0; i < buffer_size; i++) {
      //Serial.printf("%d\n", adc_buffer_copy[i]);
      for (int j = 0; j < gs_len; j++) {
        goertzel_state * g = &gs[j];
        update_goertzel(g, adc_buffer_copy[i]);
      }
    }
    Serial.println();
    for (int j = 0; j < gs_len; j++) {
      goertzel_state * g = &gs[j];
      finalize_goertzel(g);
      Serial.printf("GS%d (%.0f Hz): %6.1f %6.1f \t %f %d\n", 
        j, g->w0/6.28 * adc_frequency, g->y_re, g->y_im, sqrt(pow(g->y_re, 2) + pow(g->y_im, 2)), adc_buffer_copy[j]);
      reset_goertzel(g);
    }
  }
}

void setup_receiver() {
  // setup input pins
  pinMode(readPin_adc_0, INPUT);

  // Initialize Goertzel filters (TODO: Hardcoded frequencies here)
  for (int j = 0; j < gs_len; j++) {
    initialize_goertzel(&gs[j], 15000 + (j-5)*200, adc_frequency);
  }

  // set gain on charge amplifier
  set_charge_amplifier_gain(6);

  // Setup ADC
  adc->adc0->setAveraging(1);
  adc->adc0->setResolution(12);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  //adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

  // Setup DMA
  dma_ch1.source((volatile uint16_t &)(ADC1_R0));
  dma_ch1.destinationBuffer((uint16_t *)dma_adc_buff1, buffer_size * 2);
  dma_ch1.interruptAtCompletion();
  dma_ch1.disableOnCompletion();
  dma_ch1.attachInterrupt(&adc_buffer_full_interrupt);
  dma_ch1.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);

  dma_ch1.enable(); // Enable the DMA channel
  adc->adc0->enableDMA();
  adc->adc0->startSingleRead(readPin_adc_0);
  adc->adc0->startTimer(adc_frequency);
}

void setup_transmitter() {
  pinMode(tx_power_en, OUTPUT);
  pinMode(xdcr_sw, OUTPUT);
  pinMode(dac_cs, OUTPUT);
  digitalWrite(dac_cs, HIGH);
  // TODO: FOR TESTING ONLY:
  set_tx_power_enable(true); // tested: works
  delay(100); // wait for power to boot

  write_to_dac(0xA, 1U << 8); // A is address for config, 8th bit is gain. set to gain=2
  write_to_dac(8, 1);  // 8 is address for VREF, 1 means use internal ref
}

void write_to_dac(uint8_t address, uint16_t value) {
/*
  MCP48CXDX1 -- 24-bit messages.
  top byte: 5-bit address, 2 "command bits", 1 dont-care
  bottom 2 bytes: 4 dont-care, 12 data bits
*/
  uint8_t buf[3];
  buf[0] = (address << 3); // bits 1 and 2 must be 0 to write.
  buf[1] = (uint8_t) (value >> 8);
  buf[2] = (uint8_t) value;
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dac_cs, LOW);
  SPI.transfer(buf, 3);
  digitalWrite(dac_cs, HIGH);
  SPI.endTransaction();

}

void poll_keyboard() {
  static int modifier = 0;

  // latch keyboard state into shift registers
  digitalWrite(kb_load_n, LOW);
  delayNanoseconds(5);
  digitalWrite(kb_load_n, HIGH);
  delayNanoseconds(5);

  // read 64 bits of shift register, takes 2.63us, runs at 23 MHz
  // Note: bitbang to avoid taking up an SPI peripheral. Requires VERY little CPU time
  uint64_t read_buffer = 0;
  for (int bit = 0; bit < 64; bit++) {
    read_buffer |= (uint64_t) digitalReadFast(kb_data) << SCAN_CHAIN_BIT_INDEX_TO_KB_INDEX[bit];
    digitalWriteFast(kb_clock, LOW);
    digitalWriteFast(kb_clock, HIGH);
    delayNanoseconds(10);
  }
  digitalWriteFast(kb_clock, LOW);

  // compare results to previous one to detect new key presses
  uint64_t new_press = ~read_buffer & switch_state;
  switch_state = read_buffer;

  if (new_press) {
    time_of_last_press_ms = millis();
    if (!screen_on) {
      screen_on = true;
      digitalWrite(tft_led, HIGH);
    }
    else {
      uint8_t key_index = ilog2(new_press) - 1;
      switch (key_index) {
        case CAP_KEY_INDEX:
          Serial.println("You pressed CAPS");
          modifier = 1;
          break;
        case SYM_KEY_INDEX:
          Serial.println("You pressed SYM");
          modifier = 2;
          break;
        case BACK_KEY_INDEX:
          Serial.println("You pressed BACKSPACE");
          break;
        default:
          char key = KEYBOARD_LAYOUT[modifier][key_index];
          tft.printf("%c", key);
          Serial.printf("You pressed key_index=%d, key=\'%c\'\n", key_index, key);
          modifier = 0; // reset modifier keys
      }
    }
  }
  else if (screen_on && millis() - time_of_last_press_ms > screen_timeout_ms){
    screen_on = false;
    //digitalWrite(tft_led, LOW);
  }

}

void setup_keyboard_poller() {
  switch_state = 0;

  //setup SPI
  pinMode(kb_load_n, OUTPUT);
  digitalWrite(kb_load_n, HIGH);
  pinMode(kb_clock, OUTPUT);
  pinMode(kb_data, INPUT);

  // start timer
  if (!keyboard_poller_timer.begin(poll_keyboard, keyboard_poller_period_usec)){
    Serial.println("Failed setting up poller");
  }
}

void setup_screen() {
  pinMode(tft_led, OUTPUT);
  digitalWrite(tft_led, HIGH);
  screen_on = true;

  pinMode(tft_sck, OUTPUT);

  tft.begin();
  tft.setCursor(0,0);
  tft.setTextWrap(true);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.print("Hello, ocean!\n\n> ");
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

}

inline float read_battery_voltage() {
  // 2.0 for 1:1 voltage divider, 3.3V is max ADC voltage, and ADC is 12-bit (4096 values)
  return 2.0 * analogRead(battery_monitor) * 3.3 / 4096;
}

void poll_battery() {
  if (millis() - time_of_last_battery_read_ms > BATTERY_READ_PERIOD_MS) {
    time_of_last_battery_read_ms = millis();
    float battery_volts = read_battery_voltage();
    int16_t x, y;
    tft.getCursor(&x, &y);
    tft.setCursor(0,0);
    tft.printf("battery %.2fV", battery_volts);
    tft.setCursor(x, y);
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000) ;
  delay(100);
  Serial.println("Starting setup()");

  SPI.begin();
  Wire.begin();
  
  setup_screen();

//  setup_receiver();

  setup_transmitter();

//  setup_keyboard_poller();

  Serial.println("setup() complete");
}

void loop() {
  uint16_t val = 2048 + 2047 * sin(2*3.14159*micros()/1e6 * 1.5e3);
  
  write_to_dac(0, val);

  poll_battery();
}

