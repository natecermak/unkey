// ==================================================================
// hardware_config.h
// Defines Teensy pin mappings for all peripherals
// ==================================================================
#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

// ADC input:
const int readPin_adc_0_pin = 15;

// Keyboard shift register:
const int kb_load_n_pin     = 22;
const int kb_clock_pin      = 23;
const int kb_data_pin       = 21;

// TFT display (ILI9341):
const int tft_led_pin       = 3;
const int tft_dc_pin        = 4;
const int tft_reset_pin     = 5;
const int tft_cs_pin        = 6;
const int tft_mosi_pin      = 11;
const int tft_miso_pin      = 12;
const int tft_sck_pin       = 13;

// Transmission system:
const int tx_power_en_pin   = 8;   // Enables power to the amplifier
const int xdcr_sw_pin       = 7;   // Transducer switch
const int dac_cs_pin        = 10;  // DAC chip select

// Battery monitor input:
const int battery_monitor_pin = 20;

#endif