#include "spk.h"
#include "simple_dsp.h"

float pulse_width = 5e-3; /**< Input: Square Chirp Duration, in seconds **/
float dutyCycle = 0.5; /**<  **/
int numWaves; /**< Calculated: Number of waves in the Square Chirp **/

uint32_t timerFreq; /**< Constant: Calculating using TIMER_PDM's maximum frequency and prescale value **/

uint16_t pwm_list[TIMER_BUFFER_SIZE]; /**< List of COMP values for the TIMER, for PWM **/
bool speaker_on = false;
uint16_t top = 768;

int32_t k_goertzel = 256;
int32_t k_speaker = 256;

unsigned int ldma_channelTMR_COMP;

void populateBuffers(int k_value)
{
  // float pulse_width = 5e-3;
  int pw = pulse_width * 1000;
  printLog("pulse_width %d ms\r\n", pw);
  static int k = 0;  // use this to remember k from call to call... if k_value<0, use previous k
  if (k_value > 0)
  {
    k = k_value;
  }
  printLog("k: %d\r\n", k);
  calculate_period_k(k, pulse_width, BUFFER_SIZE, &top, &numWaves);
  printLog("numWaves %d top %d\r\n", numWaves, top);
//	calculate_periods_list(freq_start, freq_stop, pulse_width, list_top,
//			&numWaves);
//	printLog("numWaves %d top %d\r\n", numWaves, list_top[0]);
  for (uint32_t i = 0; i < numWaves; i++)
  {
    pwm_list[i] = top * dutyCycle;
  }
  if (numWaves > 20)
  {
    float prev = 0;
    for (int i = 0; i < 9; i++)
    {
      prev += 0.05;
      pwm_list[i] = top * prev;
      pwm_list[numWaves - 1 - i] = top * prev;
    }
  }
}

bool dma_tmr_comp_cb(unsigned int channel, unsigned int sequenceNo, void *userParam)
{
  DMADRV_StopTransfer(ldma_channelTMR_COMP);
  TIMER_Enable(TIMER1, false);
  printLog("[DMA%1d]> SPK: sequenceNo %d\r\n", channel, sequenceNo);
  speaker_on = false;

  startDMADRV_TMR();
  return 0;
}

void startDMADRV_TMR(void)
{
  TIMER1->TOPB = top;
  DMADRV_MemoryPeripheral(ldma_channelTMR_COMP, dmadrvPeripheralSignal_TIMER1_CC1, &TIMER1->CC[1].OCB, pwm_list, true, numWaves, dmadrvDataSize2, dma_tmr_comp_cb, NULL);
}

void initSpeaker(void)
{
  //TODO: cmuClock_GPIO should be enabled at this point (so we hope)
  GPIO->TIMERROUTE[1].ROUTEEN = GPIO_TIMER_ROUTEEN_CC1PEN; // | GPIO_TIMER_ROUTEEN_CC0PEN
  GPIO->TIMERROUTE[1].CC1ROUTE = (gpioPortD << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT) | (3 << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
  populateBuffers(k_speaker);

  uint32_t e = DMADRV_AllocateChannel(&ldma_channelTMR_COMP, NULL);
  printLog("\t[DMA*]> CH:%1d for SPK, EC:%lu\r\n", ldma_channelTMR_COMP, e);
}
void startSpeaker(void)
{
  startDMADRV_TMR();
}
