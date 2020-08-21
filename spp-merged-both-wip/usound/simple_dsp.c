#include "simple_dsp.h"
#include "usound.h"
#include "math.h"
#include "app.h"
// #include "stdio.h"
#include <stdlib.h>

void calculate_periods_list(int start, int stop, float pw, uint16_t *list, int *N)
{
  int timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (0 + 1);
  int top_start = (int) (timerFreq / start);
  *N = (int) (2 * (start * stop) / (stop + start) * pw);
  float top_step = (1.0 * (start - stop)) / (start * stop) * timerFreq / (*N - 1);
  for (int i = 0; i < *N; i++)
  {
    list[i] = top_start + (int) (i * top_step);
  }
}

void calculate_period_k(int k, float pw, int buffer_size, uint16_t *list, int *N)
{
  int timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (0 + 1);
  int top_start = (int) (32 * 6 * buffer_size / k);
  *N = (int) (timerFreq / top_start * pw);
  list[0] = top_start;
  //	for (int i=0; i<*N; i++) {
//		list[i] = top_start;
//	}
}

void calc_chirp(int start, int stop, float pw, uint8_t *chirp, int *N)
{
  int time = 0;
  int offset = 0;
  int index = 0;
  int halfway = 0;
  int period;
  int prescale = 32 * 6;
  uint16_t list[TIMER_BUFFER_SIZE];
  int N_list = 0;

  calculate_periods_list(start, stop, pw, list, &N_list);
  // printf("period_list_length: %d\r\n", N_list);
  for (int i = 0; i < N_list; i++)
  {
    period = list[i];
    // look at current period
    // if in first half of period, set 1
    halfway = period >> 1;
    while (time < (offset + halfway))
    {
      chirp[index] = 1;
      index++;
      time += prescale;
    }
    while (time < (offset + period))
    {
      chirp[index] = 0;
      index++;
      time += prescale;
    }
    offset += period;
  }
  *N = index;
}

// assumes x is longer than y
void calc_cross(int16_t *x, int Nx, uint8_t *y, int Ny, int32_t *xy)
{
  int dot = 0;
  int16_t *tempx, i, j;
  uint8_t *tempy;
  for (i = 0; i < (Nx - Ny + 1); i++)
  {
    dot = 0;
    tempx = x + i;
    tempy = y;
    for (j = 0; j < Ny; j++)
    {
      dot += ((*tempy++) > 0) ? (*tempx++) : -(*tempx++);
      /*  Two other option... not sure what will be fastest
       // dot += *tempx++ * *tempy++;
       if (*tempy++ == 0) {
       dot -= *tempx++;
       } else {
       dot += *tempx++;
       }
       */
    }
    xy[i] = dot;
  }
}

void cumsum(int *x, int Nx, int *ans)
{
  int * temp = ans;
  int prev = 0;
  int * tempx = x;

  for (int i = 0; i < Nx; i++)
  {
    *temp = prev + *tempx++;
    prev = *temp++;
  }
}

#define M_PI		3.14159265358979323846

float goertzel(int16_t *x, int32_t k)
{
  float w, cw, c, P, z0, z1, z2;

  w = 2 * M_PI * k / BUFFER_SIZE;
  cw = cosf(w);
  c = 2 * cw;
  // sw = np.sin(w);
  z1 = 0;
  z2 = 0;
  for (int idx = 0; idx < BUFFER_SIZE; idx++)
  {
    z0 = x[idx] + c * z1 - z2;
    z2 = z1;
    z1 = z0;
  }
  // I = cw*z1 -z2;
  // Q = sw*z1;

  P = z2 * z2 + z1 * z1 - c * z1 * z2;
  return P;
}

float check_float(void)
{
  return cosf(1.0);
}

void calc_chirp_v2(int k, float pw, uint8_t *chirp, int *N)
{
  int time = 0;
  int offset = 0;
  int index = 0;
  int halfway = 0;
  uint16_t period;
  int prescale = 32 * 6;
  int N_list = 0;

  calculate_period_k(k, pw, BUFFER_SIZE, &period, &N_list);
  // printLog("calc_chirp_v2, N_list: %d\r\n", N_list);
  for (int i = 0; i < N_list; i++)
  {
    // look at current period
    // if in first half of period, set 1
    halfway = period >> 1;
    while (time < (offset + halfway))
    {
      chirp[index] = 1;
      index++;
      time += prescale;
    }
    while (time < (offset + period))
    {
      chirp[index] = 0;
      index++;
      time += prescale;
    }
    offset += period;
  }
  *N = index;
}

void filter_biquad(int16_t *x, uint8_t filter)
{
  float y;
  float delay1 = 0;
  float delay2 = 0;
  float a1, a2, b0, b1, b2;

  if (filter == 0)
  {
    a1 = -1.14292982;
    a2 = 0.41273895;
    b0 = 0.63891719;
    b1 = -1.27783439;
    b2 = 0.63891719;
  }
  else
  {
    a1 = -1.91118480;
    a2 = 0.91496354;
    b0 = 9.44685778e-4;
    b1 = 0.00188937;
    b2 = 9.44685778e-4;
  }
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    y = b0 * x[i] + delay1;
    delay1 = b1 * x[i] - a1 * y + delay2;
    delay2 = b2 * x[i] - a2 * y;
    x[i] = (int) y; // put result back in place
  }

}
int shape(int32_t* x)
{
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    x[i] = fabs(x[i]);
  }
  // filter_biquad(x, 1);

  float y;
  float delay1 = 0;
  float delay2 = 0;
  float a1, a2, b0, b1, b2;
  int max = 0;
  int max_idx = 0;

  a1 = -1.91118480;
  a2 = 0.91496354;
  b0 = 9.44685778e-4;
  b1 = 0.00188937;
  b2 = 9.44685778e-4;

  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    y = b0 * x[i] + delay1;
    delay1 = b1 * x[i] - a1 * y + delay2;
    delay2 = b2 * x[i] - a2 * y;
    x[i] = (int) y; // put result back in place
    if (x[i] > max)
    {
      max = x[i];
      max_idx = i;
    }
  }
  return max_idx;
}
