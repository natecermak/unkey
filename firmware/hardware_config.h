// ==================================================================
// hardware_config.h
// Defines Teensy pin mappings for all peripherals
// ==================================================================
#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

// ADC input:
const int readPin_adc_0 = 15;

// Keyboard shift register:
const int kb_load_n     = 22;
const int kb_clock      = 23;
const int kb_data       = 21;

// TFT display (ILI9341):
const int tft_led       = 3;
const int tft_dc        = 4;
const int tft_reset     = 5;
const int tft_cs        = 6;
const int tft_mosi      = 11;
const int tft_miso      = 12;
const int tft_sck       = 13;

// Transmission system:
const int tx_power_en   = 8;   // Enables power to the amplifier
const int xdcr_sw       = 7;   // Transducer switch
const int dac_cs        = 10;  // DAC chip select

// Battery monitor input:
const int battery_monitor = 20;

#endif