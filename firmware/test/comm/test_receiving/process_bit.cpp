// ==================================================================
// process_bit.cpp
// Unit tests for process_bit() – ensures frequency magnitudes
// are converted to correct binary values and buffered properly
// ==================================================================
#include <unity.h>

#include "comm.h"
#include "comm_internal.h"

/*
Goal: Confirm that when frequency bin 0 or 1 has the greater magnitude, process_bit()
appends the correct 0 or 1 to the internal buffer, and increments the index.

Action:
Write a test function that:

1. Sets both Goertzel bins to zero

2. Sets gs[0] to a higher magnitude than gs[1]

3. Calls process_bit()

4. Asserts that:

    The first bit is 0

    The bit index is 1
*/

void setUp(void) {
}

void tearDown(void) {
}

void test_process_bit_appends_correct_bit(void) {
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_process_bit_appends_correct_bit);
  UNITY_END();
}
