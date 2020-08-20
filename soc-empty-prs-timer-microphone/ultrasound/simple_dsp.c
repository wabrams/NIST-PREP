#include "simple_dsp.h"
#include "ultrasound_define.h"

void calculate_periods_list(int start, int stop, float pw, uint16_t * list, int * N)
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

void calculate_period_k(int k, float pw, int buffer_size, uint16_t * list, int * N)
{
  int timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (0 + 1);
  int top_start = (int) (32 * 6 * buffer_size / k);
  *N = (int) (timerFreq / top_start * pw);
  for (int i = 0; i < *N; i++)
  {
    list[i] = top_start;
  }
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
    for (j = 0; j < Nx; j++)
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
