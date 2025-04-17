// ==================================================================
// battery.h
// Declarations for battery voltage reading and display
// ==================================================================
#ifndef BATTERY_H
#define BATTERY_H

#include <ILI9341_t3n.h>

#include "config.h"

extern unsigned long time_of_last_battery_read_ms;

extern const unsigned long BATTERY_READ_PERIOD_MS;

void poll_battery();

inline float read_battery_voltage();

#endif