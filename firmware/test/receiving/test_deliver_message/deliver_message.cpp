// ==================================================================
// deliver_message.cpp
// Input:
// Output:
// Run just this test with $ pio test -e teensy40_test -f "receiving/test_deliver_message"
// ==================================================================
#include <Arduino.h>
#include <unity.h>

#include "comm.h"

void setUp(void) {}

void tearDown(void) {}

void test_deliver_message(void) {
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_deliver_message);
  UNITY_END();
}

void loop() {}