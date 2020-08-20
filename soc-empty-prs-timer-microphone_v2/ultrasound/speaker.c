#include "speaker.h"

float pulse_width = 5e-3; /**< Input: Square Chirp Duration, in seconds **/
float dutyCycle = 0.5;    /**<  **/
int numWaves;             /**< Calculated: Number of waves in the Square Chirp **/

uint32_t timerFreq; /**< Constant: Calculating using TIMER_PDM's maximum frequency and prescale value **/

uint16_t list_pwm[TIMER_BUFFER_SIZE]; /**< List of COMP values for the TIMER, for PWM **/
// uint16_t list_top[TIMER_BUFFER_SIZE]; /**< List of TOP values for the TIMER, for FREQ **/
uint16_t top = 768;

unsigned int ldma_channelTMR_TOPV;    /**< LDMA Channel for TOPV (FREQ of signal) **/
unsigned int ldma_channelTMR_COMP;    /**< LDMA Channel for COMP (PWM output) **/
#ifdef DIFFERENTIAL_DRIVE
unsigned int ldma_channelTMR_CINV;    /**< LDMA Channel for CINV (PWM output, inverted) **/
#endif

void populateBuffers(int k_value)
{
  // float pulse_width = 5e-3;
	int pw = pulse_width*1000;
	printLog("pulse_width %d ms\r\n", pw);
	static int k=0;  // use this to remember k from call to call... if k_value<0, use previous k
	if (k_value > 0) {
		k = k_value;
	}
	printLog("k: %d\r\n", k);
	calculate_period_k(k, pulse_width, BUFFER_SIZE, &top,
			&numWaves);
	printLog("numWaves %d top %d\r\n", numWaves, top);
//	calculate_periods_list(freq_start, freq_stop, pulse_width, list_top,
//			&numWaves);
//	printLog("numWaves %d top %d\r\n", numWaves, list_top[0]);
	for (uint32_t i = 0; i < numWaves; i++) {
		list_pwm[i] = top * dutyCycle;
	}
	if (numWaves > 20) {
		float prev = 0;
		for (int i = 0; i < 9; i++) {
			prev += 0.05;
			list_pwm[i] = top * prev;
			list_pwm[numWaves - 1 - i] = top * prev;
		}
	}
}
//bool dma_tmr_topv_cb(unsigned int channel, unsigned int sequenceNo,
//		void *userParam) {
//	printLog("tmr_topv_cb: channel %d, sequenceNo %d\r\n", channel, sequenceNo);
//	TIMER_Enable(TIMER1, false);
//	return 0;
//}

bool dma_tmr_comp_cb(unsigned int channel, unsigned int sequenceNo, void * userParam)
{
//	printLog("tmr_comp_cb: channel %d, sequenceNo %d\r\n", channel, sequenceNo);
  printLog("done speaking\r\n");
	TIMER_Enable(TIMER1, false);
	startDMADRV_TMR();
	return 0;
}

void play_speaker(void)
{
	// Should probably check the LDMA for timer1 is actually running
	TIMER_Enable(TIMER1, true);
}

void startDMADRV_TMR(void)
{
	//TOPV
//	DMADRV_MemoryPeripheral(ldma_channelTMR_TOPV,
//			dmadrvPeripheralSignal_TIMER1_UFOF, &TIMER1->TOPB, list_top, true,
//			numWaves, dmadrvDataSize2, dma_tmr_topv_cb, NULL);
	TIMER1->TOPB = top;

	//COMP
	DMADRV_MemoryPeripheral(ldma_channelTMR_COMP, dmadrvPeripheralSignal_TIMER1_CC1, &TIMER1->CC[1].OCB, list_pwm, true, numWaves, dmadrvDataSize2, dma_tmr_comp_cb, NULL);
#ifdef DIFFERENTIAL_DRIVE
	//CINV
	DMADRV_MemoryPeripheral(ldma_channelTMR_CINV, dmadrvPeripheralSignal_TIMER1_CC2, &TIMER1->CC[2].OCB, list_pwm, true, numWaves, dmadrvDataSize2, NULL, NULL);
#endif
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

	//gpio
#ifdef DIFFERENTIAL_DRIVE
  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);     // SPKR_NEG (used for differential drive, not currently configured)
#endif
	GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);     // SPKR_POS
	//button
//  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPullFilter, 1);
//  GPIO_IntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, false, false, false);
	//timer
	GPIO->TIMERROUTE[1].ROUTEEN = GPIO_TIMER_ROUTEEN_CC1PEN; // | GPIO_TIMER_ROUTEEN_CC0PEN
//  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
	GPIO->TIMERROUTE[1].CC1ROUTE =
			(gpioPortD << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT)
					| (3 << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
}

void initTIMER(void)
{
  // set up TIMER1 for speaker
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
    timerInit.prescale = timerPrescale1;
    timerInit.enable = false;
    timerInit.debugRun = false;
    timerInit.riseAction = timerInputActionReloadStart;
	TIMER_Init(TIMER1, &timerInit);

	// set up TIMER1's CC0 for listening to PRS
	TIMER_InitCC_TypeDef inpCCInit = TIMER_INITCC_DEFAULT;
    inpCCInit.edge = timerEdgeBoth;
    inpCCInit.mode = timerCCModeCapture;
    inpCCInit.prsSel = TX_OBS_PRS_CHANNEL;
    inpCCInit.prsInput = true;
    inpCCInit.prsInputType = timerPrsInputAsyncPulse;
	TIMER_InitCC(TIMER1, 0, &inpCCInit);

	// set up TIMER1's CC1 for outputting PWM
	TIMER_InitCC_TypeDef pwmCCInit = TIMER_INITCC_DEFAULT;
	  pwmCCInit.mode = timerCCModePWM;
	TIMER_InitCC(TIMER1, 1, &pwmCCInit);
	// set up TIMER1's CC2 for outputting PWM, inverted
#ifdef DIFFERENTIAL_DRIVE
	  pwmCCInit.outInvert = true;
	TIMER_InitCC(TIMER1, 2, &pwmCCInit);
#endif
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
