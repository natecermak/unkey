// ==================================================================
// parse_message.cpp
// ==================================================================
#include <Arduino.h>
#include <unity.h>

void setUp(void) {}

void tearDown(void) {}

void test_parse_message(void) {
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_parse_message);
  UNITY_END();
}

void loop() {}