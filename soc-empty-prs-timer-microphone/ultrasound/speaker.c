#include "speaker.h"

//uint32_t freq_start = 20e3;
//uint32_t freq_stop  = 25e3;
float pulse_width = 5e-3; /**< Input: Square Chirp Duration, in seconds **/
int numWaves; /**< Calculated: Number of waves in the Square Chirp **/
float top_step;

uint16_t list_pwm[TIMER_BUFFER_SIZE]; /**< List of COMP values for the TIMER, for PWM **/
uint16_t list_top[TIMER_BUFFER_SIZE]; /**< List of TOP values for the TIMER, for FREQ **/

unsigned int ldma_channelTMR_TOPV, ldma_channelTMR_COMP;

void populateBuffers(int k)
{
  float pulse_width = 5e-3;
  calculate_period_k(k, pulse_width, BUFFER_SIZE, list_top, &numWaves);
  printLog("numWaves %d top %d\r\n", numWaves, list_top[0]);
//	calculate_periods_list(freq_start, freq_stop, pulse_width, list_top, &numWaves);
  printLog("numWaves %d top %d\r\n", numWaves, list_top[0]);
  for (uint32_t i = 0; i < numWaves; i++)
  {
    list_pwm[i] = list_top[i] * 0.5;
  }
  if (numWaves > 20)
  {
    float prev = 0;
    for (int i = 0; i < 9; i++)
    {
      prev += 0.05;
      list_pwm[i] = list_top[i] * prev;
      list_pwm[numWaves - 1 - i] = list_top[numWaves - 1 - i] * prev;
    }
  }
}

bool dma_tmr_comp_cb(unsigned int channel, unsigned int sequenceNo, void *userParam)
{
  printLog("tmr_comp_cb: channel %d, sequenceNo %d\r\n", channel, sequenceNo);
  TIMER_Enable(TIMER1, false);
  return 0;
}

bool dma_tmr_topv_cb(unsigned int channel, unsigned int sequenceNo, void *userParam)
{
  printLog("tmr_topv_cb: channel %d, sequenceNo %d\r\n", channel, sequenceNo);
  TIMER_Enable(TIMER1, false);
  startDMADRV_TMR();
  return 0;
}

void play_speaker(void)
{
  //TODO: Should probably check the LDMA for timer1 is actually running
  TIMER_Enable(TIMER1, true);
}

void startDMADRV_TMR(void)
{
  //TOPV
  DMADRV_MemoryPeripheral(ldma_channelTMR_TOPV, dmadrvPeripheralSignal_TIMER1_UFOF, &TIMER1->TOPB, list_top, true, numWaves, dmadrvDataSize2, dma_tmr_topv_cb, NULL);
  //COMP
  DMADRV_MemoryPeripheral(ldma_channelTMR_COMP, dmadrvPeripheralSignal_TIMER1_CC1, &TIMER1->CC[1].OCB, list_pwm, true, numWaves, dmadrvDataSize2, dma_tmr_comp_cb, NULL);
}

void init_speaker(void)
{
  CMU_ClockEnable(cmuClock_TIMER1, true);
  // these aren't needed if in a bluetooth project
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_PRS, true);
  CMU_ClockEnable(cmuClock_RTCC, true);

//  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);   // SPKR_NEG (used for differential drive, not currently configured)
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);     // SPKR_POS
  //timer
  GPIO->TIMERROUTE[1].ROUTEEN = GPIO_TIMER_ROUTEEN_CC1PEN; // | GPIO_TIMER_ROUTEEN_CC0PEN
//  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  GPIO->TIMERROUTE[1].CC1ROUTE = (gpioPortD << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT) | (3 << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
}

void initTIMER(void)
{
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
    timerInit.prescale = timerPrescale1;
    timerInit.enable = false;
    timerInit.debugRun = false;
    timerInit.riseAction = timerInputActionReloadStart;
  TIMER_Init(TIMER1, &timerInit);

  TIMER_InitCC_TypeDef timerCC0Init = TIMER_INITCC_DEFAULT;
    timerCC0Init.edge = timerEdgeBoth;
    timerCC0Init.mode = timerCCModeCapture;
    timerCC0Init.prsSel = TX_OBS_PRS_CHANNEL;
    timerCC0Init.prsInput = true;
    timerCC0Init.prsInputType = timerPrsInputAsyncPulse;
  TIMER_InitCC(TIMER1, 0, &timerCC0Init);

  TIMER_InitCC_TypeDef timerCC1Init = TIMER_INITCC_DEFAULT;
    timerCC1Init.mode = timerCCModePWM;
  TIMER_InitCC(TIMER1, 1, &timerCC1Init);
}

void init_prs(void)
{
  PRS_SourceAsyncSignalSet(RX_OBS_PRS_CHANNEL, PRS_ASYNC_CH_CTRL_SOURCESEL_MODEML, _PRS_ASYNC_CH_CTRL_SIGSEL_MODEMLFRAMEDET);
  PRS_SourceAsyncSignalSet(TX_OBS_PRS_CHANNEL, PRS_ASYNC_CH_CTRL_SOURCESEL_MODEM, _PRS_ASYNC_CH_CTRL_SIGSEL_MODEMFRAMESENT);
}

void prs_gpio(GPIO_Port_TypeDef rx_obs_port, uint8_t rx_obs_pin, GPIO_Port_TypeDef tx_obs_port, uint8_t tx_obs_pin)
{
  // Turn on the PRS and GPIO clocks to access their registers.
  // Configure pins as output.
  GPIO_PinModeSet(rx_obs_port, rx_obs_pin, gpioModePushPull, 0);
  GPIO_PinModeSet(tx_obs_port, tx_obs_pin, gpioModePushPull, 0);

  // Configure PRS Channel 0 to output RAC_RX.
//	PRS_SourceAsyncSignalSet(0, PRS_ASYNC_CH_CTRL_SOURCESEL_MODEML,
//			_PRS_ASYNC_CH_CTRL_SIGSEL_MODEMLFRAMEDET);
  PRS_PinOutput(RX_OBS_PRS_CHANNEL, prsTypeAsync, rx_obs_port, rx_obs_pin);

  /* Configure PRS Channel 0 to output RAC_RX.*/

//	PRS_SourceAsyncSignalSet(1, PRS_ASYNC_CH_CTRL_SOURCESEL_MODEM,
//			_PRS_ASYNC_CH_CTRL_SIGSEL_MODEMFRAMESENT);
  PRS_PinOutput(TX_OBS_PRS_CHANNEL, prsTypeAsync, tx_obs_port, tx_obs_pin);

}
