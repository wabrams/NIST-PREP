#ifndef _SIMPLE_DSP_H
#define _SIMPLE_DSP_H

// #define PERIOD_BUFFER_SIZE	256

#include "em_cmu.h"

void calculate_periods_list(int start, int stop, float pw, uint16_t *list, int *N);
void calculate_period_k(int k, float pw, int buffer_size, uint16_t *list, int *N);
void calc_chirp(int start, int stop, float pw, uint8_t *chirp, int *N);
void calc_cross(int16_t *x, int Nx, uint8_t *y, int Ny, int32_t *xy);
void cumsum(int *x, int Nx, int *ans);
#endif
