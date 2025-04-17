// ==================================================================
// battery.h
// Declarations for battery voltage reading and display
// ==================================================================
#ifndef BATTERY_H
#define BATTERY_H

extern unsigned long time_of_last_battery_read_ms;

extern const unsigned long BATTERY_READ_PERIOD_MS;

void poll_battery();


#endif
