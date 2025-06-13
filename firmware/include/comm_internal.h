// ==================================================================
// comm_internal.h
// Internal declarations for unit testing comm.cpp static state. Definitions in comm.cpp
// ==================================================================
#pragma once // alternative to #ifndef COMM_INTERNAL_H - ensures this header file is only included once per compilation unit

#include "goertzel.h"

// Declares a function that returns a pointer to the internal bitstream[] array, which is static in comm.cpp:
uint8_t* _test_get_bitstream();

// Declares a function that returns a pointer to the internal bit_index variable
int* _test_get_bit_index();

// Declares a function that returns a pointer to the internal gs[] array, which holds the Goertzel filter states (used to determine bit values):
goertzel_state* _test_get_goertzel_state();

void get_bit_from_top_frequency();
