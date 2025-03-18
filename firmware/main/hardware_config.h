#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

//----------------------------------------
// Hardware Pin Assignments
//----------------------------------------
const int readPin_adc_0 = 15;
const int kb_load_n     = 22;
const int kb_clock      = 23;
const int kb_data       = 21;

const int tft_miso      = 12;
const int tft_led       = 3;
const int tft_dc        = 4;
const int tft_reset     = 5;
const int tft_cs        = 6;
const int tft_mosi      = 11;
const int tft_sck       = 13;

// This pin enables power to amp that in turn powers tx signal:
const int tx_power_en   = 8;
const int xdcr_sw       = 7;
const int dac_cs        = 10;

const int battery_monitor = 20;

#endif
