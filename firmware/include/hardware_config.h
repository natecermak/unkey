// ==================================================================
// hardware_config.h
// Defines Teensy pin mappings for all peripherals
// ==================================================================
#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

//----------------------------------------
// ADC Input
//----------------------------------------
#define readPin_adc_0_pin      15

//----------------------------------------
// Keyboard Shift Register
//----------------------------------------
#define kb_load_n_pin          22
#define kb_clock_pin           23
#define kb_data_pin            21

//----------------------------------------
// TFT Display (ILI9341)
//----------------------------------------
#define tft_led_pin            3
#define tft_dc_pin             4
#define tft_reset_pin          5
#define tft_cs_pin             6
#define tft_mosi_pin           11
#define tft_miso_pin           12
#define tft_sck_pin            13

//----------------------------------------
// Transmission System
//----------------------------------------
#define tx_power_en_pin        8   // Enables power to the amplifier
#define xdcr_sw_pin            7   // Transducer switch
#define dac_cs_pin             10  // DAC chip select

//----------------------------------------
// Battery Monitor Input
//----------------------------------------
#define battery_monitor_pin    20

#endif
