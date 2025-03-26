// ==================================================================
// goertzel.h
// Defines Goertzel algorithm state and functions for frequency detection
// ==================================================================
#ifndef __GOERTZEL_H__
#define __GOERTZEL_H__

typedef struct {
  // Precomputed values during initialization
  float w0;
  float cos_w0;
  float sin_w0;
  float a1;        // 2*cos(w0), where w0 is normalized to 0–2π

  double s, s_z1;
  float y_re, y_im;
  int n;
} goertzel_state;

void initialize_goertzel(goertzel_state *g, float f0, float fs);

void update_goertzel(goertzel_state *g, int x);

void finalize_goertzel(goertzel_state *g);

void reset_goertzel(goertzel_state *g);

#endif