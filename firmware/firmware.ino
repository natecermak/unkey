#include <ADC.h>
#include <DMAChannel.h>
#include "goertzel.h"
#include <SPI.h>
#include <ILI9341_t3n.h>

// ------------------ PINS ----------------------------------------------- //
const int readPin_adc_0 = 15;
const int kb_load_n = 9;
const int kb_clock = 13;
const int kb_data = 12;

const int tft_miso = 1;
const int tft_led = 2;
const int tft_dc = 3;
const int tft_reset = 4;
const int tft_cs = 5;
const int tft_mosi = 26;
const int tft_sck = 27;

// ------------------ ADC, DMA, and Goertzel filters -------------------- //
ADC *adc = new ADC(); // adc object
DMAChannel dma_ch1;

const uint32_t adc_frequency = 81920;
const uint32_t buffer_size = 10240;
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff1[buffer_size];
uint16_t adc_buffer_copy[buffer_size];

uint8_t print_ctr = 0;
const uint8_t gs_len = 10;
goertzel_state gs[gs_len];

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

// ------------------ Utility functions ---------------------------------- //
int ilog2(uint64_t x){
  int i = 0;
  while (x) {
    x = x >> 1;
    i++;  
  }
  return i;
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
    initialize_goertzel(&gs[j], 3000 + (j-5)*200, adc_frequency);
  }

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
    digitalWrite(tft_led, LOW);
  }

};

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

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000) ;
  delay(100);
  Serial.println("Starting setup()");

  setup_screen();

  setup_receiver();

  setup_keyboard_poller();

  Serial.println("setup() complete");
  
}

void loop() {
  delay(1000);
}

