#include "speaker.h"

chirp_t chirps[3];
uint8_t idle_chirp;
bool usound_spk = false; /**<  **/
bool usound_lst = false; /**<  **/

uint32_t tick_pdm_start;
uint32_t start, stop, durr, measured_ms;

uint16_t ping_pwm[PPS_BUFFER_SIZE];
uint16_t pong_pwm[PPS_BUFFER_SIZE];
uint16_t ping_top[PPS_BUFFER_SIZE];
uint16_t pong_top[PPS_BUFFER_SIZE];
bool prevPingPWM, prevPingTOP;

unsigned int ldma_channelTMR_TOPV, ldma_channelTMR_COMP;

void setupChirp(chirp_t * c, int freq_start, int freq_stop, float pulse_width)
{
  int num_waves;
  calculate_periods_list(freq_start, freq_stop, pulse_width, (c->top_list), &num_waves);
  printLog("numWaves %d top %d\r\n", num_waves, c->top_list[0]);

  for (uint32_t i = 0; i < num_waves; i++)
    c->pwm_list[i] = c->top_list[i] * 0.50;

//  if (num_waves > 20)
//  {
//    float tapered_duty = 0;
//    for (int i = 0; i < 1; i++) //MODIFIED: noise has been reduced
//    {
//      tapered_duty += 0.05;
//      list_pwm[i] = list_top[i] * tapered_duty;
//      list_pwm[num_waves - 1 - i] = list_top[num_waves - 1 - i] * tapered_duty;
//    }
//  }
}

void setupChirps(void)
{
  setupChirp(&chirps[0], 25e3, 25e3, 5e-3);
  setupChirp(&chirps[1], 20e3, 25e3, 5e-3);
  setupChirp(&chirps[2], 25e3, 20e3, 5e-3);
}

static inline void copyBuffer(uint16_t * dst, uint16_t * src, int N)
{
  for (int i = 0; i < N; i++)
    dst[i] = src[i];
}

void setupBuffers()
{
  copyBuffer(ping_top, chirps[0].top_list, PPS_BUFFER_SIZE);
  copyBuffer(ping_pwm, chirps[0].pwm_list, PPS_BUFFER_SIZE);
  copyBuffer(pong_top, chirps[1].top_list, PPS_BUFFER_SIZE);
  copyBuffer(pong_pwm, chirps[1].pwm_list, PPS_BUFFER_SIZE);
  idle_chirp = 2;
}

void updatePingPongBuffers(void)
{
  //verify that buffers are in sync
  while (prevPingPWM ^ prevPingTOP)
    ;
  // copy into ping
  if (prevPingPWM)
  {
    copyBuffer(ping_top, chirps[idle_chirp].top_list, PPS_BUFFER_SIZE);
    copyBuffer(ping_pwm, chirps[idle_chirp].pwm_list, PPS_BUFFER_SIZE);
  }
  //copy into pong
  else
  {
    copyBuffer(pong_top, chirps[idle_chirp].top_list, PPS_BUFFER_SIZE);
    copyBuffer(pong_pwm, chirps[idle_chirp].pwm_list, PPS_BUFFER_SIZE);
  }

  idle_chirp = (idle_chirp + 1) % 3;
}

bool dma_tmr_topv_cb(unsigned int channel, unsigned int sequenceNo, void * userParam)
{
  TIMER_Enable(TIMER1, false);
  printLog("tmr_topv_cb: channel %d, sequenceNo %d\r\n", channel, sequenceNo);
  prevPingTOP = !prevPingTOP;
  return 0;
}

bool dma_tmr_comp_cb(unsigned int channel, unsigned int sequenceNo, void * userParam)
{
//  TIMER_Enable(TIMER1, false);
  printLog("tmr_comp_cb: channel %d, sequenceNo %d\r\n", channel, sequenceNo);
  prevPingPWM = !prevPingPWM;
  return 0;
}

void startDMADRV_TMR(void)
{
  static LDMA_Descriptor_t descLinkTOPV[2];
  static LDMA_Descriptor_t descLinkCOMP[2];
  static bool need_setup = true;
  LDMA_TransferCfg_t transfercfgTOPV = (LDMA_TransferCfg_t) LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_TIMER1_UFOF);
  LDMA_TransferCfg_t transfercfgCOMP = (LDMA_TransferCfg_t) LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_TIMER1_CC1);

  if (need_setup)
  {
    descLinkTOPV[0] = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_LINKREL_M2P_HALFWORD(ping_top, &(TIMER1 -> TOPB), PPS_BUFFER_SIZE, +1);
    descLinkTOPV[1] = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_LINKREL_M2P_HALFWORD(pong_top, &(TIMER1 -> TOPB), PPS_BUFFER_SIZE, -1);
    descLinkCOMP[0] = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_LINKREL_M2P_HALFWORD(ping_pwm, &(TIMER1 -> CC[1].OCB), PPS_BUFFER_SIZE, +1);
    descLinkCOMP[1] = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_LINKREL_M2P_HALFWORD(pong_pwm, &(TIMER1 -> CC[1].OCB), PPS_BUFFER_SIZE, -1);
    need_setup = false;
  }

  prevPingPWM = prevPingTOP = false;
  DMADRV_LdmaStartTransfer(ldma_channelTMR_TOPV, &transfercfgTOPV, descLinkTOPV, dma_tmr_topv_cb, NULL);
  DMADRV_LdmaStartTransfer(ldma_channelTMR_COMP, &transfercfgCOMP, descLinkCOMP, dma_tmr_comp_cb, NULL);
}

//void initPRS(void)
//{
//  // Select GPIO as source and button 0 GPIO pin as signal for PRS channel 0
//  PRS_SourceAsyncSignalSet(TX_OBS_PRS_CHANNEL, PRS_ASYNC_CH_CTRL_SOURCESEL_GPIO, BSP_GPIO_PB0_PIN);
//  // Do not apply any logic on the PRS Channel
//  PRS_Combine (TX_OBS_PRS_CHANNEL, TX_OBS_PRS_CHANNEL, prsLogic_A);
//  // Select PRS channel for Timer 0
//  PRS_ConnectConsumer(TX_OBS_PRS_CHANNEL, prsTypeAsync, prsConsumerTIMER0_CC0);
//}

void init_speaker(void)
{
  // CMU
  CMU_ClockEnable(cmuClock_TIMER1, true);
  // these aren't needed if in a bluetooth project
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_PRS, true);
  CMU_ClockEnable(cmuClock_RTCC, true);
  //CMU_ClockEnable(cmuClock_PDM, true);

  //gpio
//  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);     // SPKR_NEG (used for differential drive, not currently configured)
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);     // SPKR_POS
  //timer
  GPIO->TIMERROUTE[1].ROUTEEN = GPIO_TIMER_ROUTEEN_CC1PEN; // | GPIO_TIMER_ROUTEEN_CC0PEN
//  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  GPIO->TIMERROUTE[1].CC1ROUTE = (gpioPortD << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT)
      | (3 << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);

  setupChirps();
  setupBuffers();
}

void initTIMER(void)
{
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  timerInit.prescale = timerPrescale1;
  timerInit.enable = false;
  timerInit.debugRun = false;
//  timerInit.riseAction = timerInputActionReloadStart;
  TIMER_Init(TIMER1, &timerInit);

//  TIMER_InitCC_TypeDef timerCC0Init = TIMER_INITCC_DEFAULT;
//  timerCC0Init.edge = timerEdgeBoth;
//  timerCC0Init.mode = timerCCModeCapture;
//  timerCC0Init.prsSel = TX_OBS_PRS_CHANNEL;
//  timerCC0Init.prsInput = true;
//  timerCC0Init.prsInputType = timerPrsInputAsyncPulse;
//  TIMER_InitCC(TIMER1, 0, &timerCC0Init);

  TIMER_InitCC_TypeDef timerCC1Init = TIMER_INITCC_DEFAULT;
  timerCC1Init.mode = timerCCModePWM;
  TIMER_InitCC(TIMER1, 1, &timerCC1Init);

  // timerFreq = CMU_ClockFreqGet(cmuClock_TIMER1) / (timerPrescale1 + 1);
}

void init_prs(void)
{
  PRS_SourceAsyncSignalSet(RX_OBS_PRS_CHANNEL, PRS_ASYNC_CH_CTRL_SOURCESEL_MODEML,
  _PRS_ASYNC_CH_CTRL_SIGSEL_MODEMLFRAMEDET);
  PRS_SourceAsyncSignalSet(TX_OBS_PRS_CHANNEL, PRS_ASYNC_CH_CTRL_SOURCESEL_MODEM,
  _PRS_ASYNC_CH_CTRL_SIGSEL_MODEMFRAMESENT);
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
