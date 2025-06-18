// ==================================================================
// mock_goertzel.cpp
// Mocks Goertzel functions for unit testing to prevent altering gs[]
// ==================================================================
#ifdef UNIT_TEST
#include <Arduino.h>

#include "goertzel.h"

// Mock version of update_goertzel
// Does nothing so that gs[] is not modified during tests
void update_goertzel(goertzel_state* g, int sample) {
  (void)g;       // Suppress unused parameter warning
  (void)sample;  // Suppress unused parameter warning
  // Intentionally empty
}

// Mock version of finalize_goertzel
// Does nothing so that gs[] is not modified during tests
void finalize_goertzel(goertzel_state* g) {
  (void)g;  // Suppress unused parameter warning
  // Intentionally empty
}

// Mock version of reset_goertzel
// Does nothing so that gs[] is not modified during tests
void reset_goertzel(goertzel_state* g) {
  (void)g;  // Suppress unused parameter warning
  // Intentionally empty
}
#endif