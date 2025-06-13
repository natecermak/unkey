// ==================================================================
// battery.cpp
// Handles battery voltage reading and display
// ==================================================================
#include <Arduino.h>
#include <ILI9341_t3n.h>

#include "config.h"
#include "comm.h"
#include "display.h"
#include "hardware_config.h"

static unsigned long time_of_last_battery_read_ms = 0;
static const unsigned long BATTERY_READ_PERIOD_MS = 1000;

/**
 * Used in poll_battery to display the current battery level on the screen.
 */
inline float read_battery_voltage() {
  // 2.0 for 1:1 voltage divider, 3.3V is max ADC voltage, and ADC is 12-bit (4096 values)
  return 2.0 * analogRead(battery_monitor_pin) * 3.3 / 4096;
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
