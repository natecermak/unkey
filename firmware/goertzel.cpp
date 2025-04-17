// ==================================================================
// goertzel.cpp
// Implements the Goertzel algorithm for detecting specific frequencies
// ==================================================================
#include <math.h>  // for PI

#include "goertzel.h"

void initialize_goertzel(goertzel_state *g, float f0, float fs) {
  g->w0 = 2 * M_PI * f0 / fs; // radians per sample
  g->cos_w0 = cos(g->w0);
  g->sin_w0 = sin(g->w0);
  g->a1 = 2 * g->cos_w0;
  // initialize filters to 0
  g->s = 0;
  g->s_z1 = 0;
  g->n = 0;
}

void update_goertzel(goertzel_state *g, int x) {
  float s = x + g->a1 * g->s - g->s_z1;
  g->s_z1 = g->s;
  g->s = s;
  g->n += 1;
}

void finalize_goertzel(goertzel_state *g) {
  g->y_re = (g->s - g->cos_w0 * g->s_z1) / g->n;
  g->y_im = (g->sin_w0 * g->s_z1) / g->n;
}

void reset_goertzel(goertzel_state *g) {
  g->s = 0;
  g->s_z1 = 0;
  g->n = 0;
}
